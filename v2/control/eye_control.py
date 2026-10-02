"""
Project Eye v2 - seri surucu + kalibrasyon + davranis.

Katmanlar (alttan uste):

1. **Kalibrasyon** (saf matematik, SPEC §6): ``eye_angle``, ``lid_angle``,
   ``lid_follow``, ``compute_angles``; ``load_settings`` / ``save_settings``.
2. **Baglanti** (``EyeLink``): satir tabanli protokol (SPEC §5). Gercek port
   ``find_eye_port`` ile bulunur ve ``EYE v2 READY`` el sikismasiyla
   DOGRULANIR - Lunar gimbal'i (``READY`` / ``P090T095`` protokolu) ya da
   baska bir cihaz asla surulmez. Donanim yoksa ``sim.SimSerial``'e duser.
3. **Davranis** (``EyeController``): ``look(u, v)``, ``lids(...)``,
   ``blink()``, ``wink(side)``; ``update(dt)`` icinde sakkad hiz siniri,
   kapaklarin pitch'i takibi, rastgele kirpma, bostayken yavas gezinme,
   S komutlarinin hiz/degisim sinirli gonderimi ve firmware idle'ina
   dusmemek icin canli tutma (keepalive).

Bagimlilik: yalnizca pyserial (+stdlib). pyserial yoksa sim moduna duser.

Hizli deneme:
    python eye_control.py            # donanimi bul, yoksa sim; demo hareketi
    python eye_control.py --sim      # dogrudan sim
    python eye_control.py --port COM5
"""

from __future__ import annotations

import copy
import json
import math
import os
import random
import sys
import time
from typing import Callable, Iterable

import sim

# --------------------------------------------------------------------------
# Sabitler
# --------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
V2_DIR = os.path.dirname(HERE)
SETTINGS_PATH = os.path.join(V2_DIR, "settings.json")

CHANNELS = ("EYE_YAW", "EYE_PITCH", "LID_UL", "LID_LL", "LID_UR", "LID_LR")
EYE_CHANNELS = ("EYE_YAW", "EYE_PITCH")
LID_CHANNELS = ("LID_UL", "LID_LL", "LID_UR", "LID_LR")
# robotun kendi solu/sagi (SPEC §1: +X = robotun sagi)
SIDE_LIDS = {"left": ("LID_UL", "LID_LL"), "right": ("LID_UR", "LID_LR")}

BAUD = sim.BAUD
FW_BANNER = sim.FW_BANNER
LUNAR_GIMBAL_BANNER = "READY"   # lunar-tracker/firmware/pan_tilt - DOKUNMA
HANDSHAKE_TIMEOUT_S = 3.0       # Uno DTR reset + bootloader ~1.6 sn
HANDSHAKE_QUERY_AFTER_S = 2.0   # banner gelmezse (reset olmadiysa) '?' ile sor

SEND_HZ = 50.0                  # S komutu ust hizi
SEND_MIN_DELTA_DEG = 0.1        # bundan kucuk degisimi tekrar gonderme
KEEPALIVE_S = 0.5               # firmware'in 2 sn idle'ina dusmemesi icin

BLINK_S = 0.25                  # kapanma %30, kapali %20, acilma %50
WINK_S = 0.45

# Varsayilanlar = mekanik modelin onerisi (cad/out/kinematics.json -> settings_suggestion).
# SPEC §6'daki ornek degerler (kapak 60/120...) yeni firmware limitlerinin disina
# tasiyordu; eksik anahtar tamamlanirken model degerleri daha guvenli.
DEFAULT_SETTINGS = {
    "serial_port": "auto",
    "channels": {
        "EYE_YAW": {"center": 90, "min": 60, "max": 120, "invert": False, "deg_per_unit": 30},
        "EYE_PITCH": {"center": 90, "min": 64, "max": 121, "invert": False, "deg_per_unit": 34.43},
        "LID_UL": {"closed": 64.38, "open": 96.15, "invert": False},
        "LID_LL": {"closed": 113.77, "open": 86.85, "invert": False},
        "LID_UR": {"closed": 115.62, "open": 83.85, "invert": False},
        "LID_LR": {"closed": 66.23, "open": 93.15, "invert": False},
    },
    "behaviour": {
        "blink_interval_s": [3, 6],
        "lid_follow_pitch": 0.6,
        "saccade_speed_deg_s": 500,
        "idle_after_s": 1.5,
    },
}

# Mekanik ekibin kinematik tablolari (yoksa/okunamazsa SPEC §6 dogrusal formulu)
KINEMATICS_PATH = os.path.join(V2_DIR, "cad", "out", "kinematics.json")


class SettingsError(ValueError):
    pass


def clamp(v: float, lo: float, hi: float) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return lo if v < lo else hi if v > hi else v


# --------------------------------------------------------------------------
# Ayarlar
# --------------------------------------------------------------------------
def _deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def validate_settings(s: dict) -> None:
    """Sema ve deger kontrolu; sorun varsa SettingsError (hepsini listeler)."""
    problems = []
    ch = s.get("channels", {})
    for name in CHANNELS:
        c = ch.get(name)
        if not isinstance(c, dict):
            problems.append(f"{name}: eksik")
            continue
        keys = ("center", "min", "max", "deg_per_unit") if name in EYE_CHANNELS else ("closed", "open")
        for k in keys:
            if not isinstance(c.get(k), (int, float)) or isinstance(c.get(k), bool):
                problems.append(f"{name}.{k}: sayi olmali")
            elif not 0 <= c[k] <= 180 and k != "deg_per_unit":
                problems.append(f"{name}.{k}: 0-180 disinda ({c[k]})")
        if not isinstance(c.get("invert"), bool):
            problems.append(f"{name}.invert: true/false olmali")
        nums_ok = all(isinstance(c.get(k), (int, float)) for k in keys)
        if name in EYE_CHANNELS and nums_ok:
            if not c["min"] <= c["center"] <= c["max"]:
                problems.append(f"{name}: min <= center <= max olmali")
    b = s.get("behaviour", {})
    bi = b.get("blink_interval_s")
    if not (isinstance(bi, (list, tuple)) and len(bi) == 2 and 0 < bi[0] <= bi[1]):
        problems.append("behaviour.blink_interval_s: [min, max] ve 0 < min <= max olmali")
    for k in ("lid_follow_pitch", "saccade_speed_deg_s", "idle_after_s"):
        if not isinstance(b.get(k), (int, float)) or b[k] < 0:
            problems.append(f"behaviour.{k}: >= 0 sayi olmali")
    if not isinstance(s.get("serial_port"), str):
        problems.append("serial_port: metin olmali ('auto' ya da 'COM5')")
    if problems:
        raise SettingsError("; ".join(problems))


def load_settings(path: str | None = None) -> dict:
    """settings.json'u okur, eksik anahtarlari varsayilanlarla tamamlar, dogrular."""
    path = path or SETTINGS_PATH
    data = {}
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    s = _deep_merge(DEFAULT_SETTINGS, data)
    validate_settings(s)
    return s


def dumps_settings(s: dict) -> str:
    """SPEC §6'daki okunur bicim: her kanal tek satir."""
    lines = ["{", f'  "serial_port": {json.dumps(s["serial_port"])},', '  "channels": {']
    names = list(s["channels"].keys())
    for i, name in enumerate(names):
        pad = " " * (len("EYE_PITCH") - len(name))
        body = json.dumps(s["channels"][name], separators=(", ", ": "))
        lines.append(f'    "{name}": {pad}{body}{"," if i < len(names) - 1 else ""}')
    lines.append("  },")
    lines.append('  "behaviour": {')
    bkeys = list(s["behaviour"].keys())
    for i, k in enumerate(bkeys):
        lines.append(f'    "{k}": {json.dumps(s["behaviour"][k])}{"," if i < len(bkeys) - 1 else ""}')
    lines.append("  }")
    lines.append("}")
    extra = {k: v for k, v in s.items() if k not in ("serial_port", "channels", "behaviour")}
    text = "\n".join(lines) + "\n"
    if extra:  # bilinmeyen ust-duzey anahtarlari kaybetme
        merged = json.loads(text)
        merged.update(extra)
        text = json.dumps(merged, indent=2) + "\n"
    return text


def save_settings(s: dict, path: str | None = None) -> None:
    """Dogrular, sonra atomik yazar (yarim kalmis dosya olmaz)."""
    validate_settings(s)
    path = path or SETTINGS_PATH
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(dumps_settings(s))
    os.replace(tmp, path)


# --------------------------------------------------------------------------
# Kinematik tablolar (cad/out/kinematics.json)
# --------------------------------------------------------------------------
def _interp1(xs, ys, x: float) -> float:
    """Parcali dogrusal enterpolasyon; xs artan. Tablo disinda uc degere kirpar."""
    if x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return float(ys[i - 1] + t * (ys[i] - ys[i - 1]))
    return float(ys[-1])


def _strictly_increasing(xs) -> bool:
    return len(xs) >= 2 and all(b > a for a, b in zip(xs, xs[1:]))


def _segment(axis, x: float) -> int:
    """x'i iceren [axis[i-1], axis[i]] araliginin i'si (1..len-1)."""
    for i in range(1, len(axis)):
        if x <= axis[i]:
            return i
    return len(axis) - 1


class Kinematics:
    """Mekanik modelden gelen servo tablolari.

    * ``yaw_servo(yaw_deg)``              EYE_YAW: 1B tablo (modelde birebir 90 + yaw)
    * ``pitch_servo(yaw_deg, pitch_deg)`` EYE_PITCH: 2B tablo, cift dogrusal
      enterpolasyon (koselerdeki yaw-pitch capraz etkisi telafi edilir)
    * ``lid_servo(name, openness)``       kapaklar: 1B tablo (4-cubuk, dogrusal degil)

    Acilar goz derecesi (SPEC §1); servo degerleri modelin notr=90 konvansiyonunda.
    """

    def __init__(self, yaw_tab, pitch_grid, lid_tabs, source: str = "?"):
        self.yaw_x, self.yaw_s = yaw_tab
        self.grid_yaw, self.grid_pitch, self.grid = pitch_grid  # grid[pitch_i][yaw_j]
        self.lid_tabs = lid_tabs                                  # name -> (openness, servo)
        self.source = source
        self.yaw_range = min(abs(self.yaw_x[0]), abs(self.yaw_x[-1]),
                             abs(self.grid_yaw[0]), abs(self.grid_yaw[-1]))
        self.pitch_range = min(abs(self.grid_pitch[0]), abs(self.grid_pitch[-1]))

    @classmethod
    def from_dict(cls, data: dict, source: str = "?") -> "Kinematics":
        """Sema/deger kontrolu yapar; sorun varsa ValueError/KeyError/TypeError."""
        ch = data["channels"]
        yt = sorted((float(r["eye_yaw_deg"]), float(r["servo_deg"])) for r in ch["EYE_YAW"]["table"])
        yaw_tab = ([a for a, _ in yt], [b for _, b in yt])
        if not _strictly_increasing(yaw_tab[0]):
            raise ValueError("EYE_YAW tablosu artan degil")
        g = data["cross_coupling"]["EYE_PITCH_servo_deg_grid"]
        gy = [float(x) for x in g["yaw_deg"]]
        gp = [float(x) for x in g["pitch_deg"]]
        grid = [[float(x) for x in row] for row in g["servo_deg"]]
        if not (_strictly_increasing(gy) and _strictly_increasing(gp)):
            raise ValueError("pitch izgarasi eksenleri artan degil")
        if len(grid) != len(gp) or any(len(r) != len(gy) for r in grid):
            raise ValueError("pitch izgarasi boyutu eksenlerle uyusmuyor")
        lids = {}
        for name in LID_CHANNELS:
            rows = sorted((float(r["openness"]) + 0.0, float(r["servo_deg"])) for r in ch[name]["table"])
            xs = [a for a, _ in rows]
            if not _strictly_increasing(xs) or xs[0] > 0.0 or xs[-1] < 1.0:
                raise ValueError(f"{name} tablosu 0..1 acikligi kapsamiyor")
            lids[name] = (xs, [b for _, b in rows])
        values = [*yaw_tab[1], *(x for r in grid for x in r), *(x for t in lids.values() for x in t[1])]
        for v in values:
            if not (math.isfinite(v) and 0.0 <= v <= 180.0):
                raise ValueError(f"tabloda 0-180 disi servo acisi: {v}")
        return cls(yaw_tab, (gy, gp, grid), lids, source)

    # -- sorgular ---------------------------------------------------------
    def yaw_servo(self, yaw_deg: float) -> float:
        return _interp1(self.yaw_x, self.yaw_s, yaw_deg)

    def pitch_servo(self, yaw_deg: float, pitch_deg: float) -> float:
        """Cift dogrusal enterpolasyon; izgara disi girdiler kenara kirpilir."""
        gy, gp, g = self.grid_yaw, self.grid_pitch, self.grid
        y = clamp(yaw_deg, gy[0], gy[-1])
        p = clamp(pitch_deg, gp[0], gp[-1])
        j, i = _segment(gy, y), _segment(gp, p)
        ty = (y - gy[j - 1]) / (gy[j] - gy[j - 1])
        tp = (p - gp[i - 1]) / (gp[i] - gp[i - 1])
        lo = g[i - 1][j - 1] + ty * (g[i - 1][j] - g[i - 1][j - 1])
        hi = g[i][j - 1] + ty * (g[i][j] - g[i][j - 1])
        return lo + tp * (hi - lo)

    def eye_pitch_at(self, yaw_deg: float, servo_deg: float) -> float:
        """Ters yon: bu yaw'da bu pitch servo acisi gozu kac dereceye getirir.

        Izgara disindaki servo acisi icin uc araliktan dogrusal disdegerleme
        (yalnizca yaklasik; telemetri ve testler icin).
        """
        xs = [self.pitch_servo(yaw_deg, p) for p in self.grid_pitch]
        ps = self.grid_pitch
        if servo_deg < xs[0]:
            return ps[0] + (servo_deg - xs[0]) * (ps[1] - ps[0]) / (xs[1] - xs[0])
        if servo_deg > xs[-1]:
            return ps[-1] + (servo_deg - xs[-1]) * (ps[-1] - ps[-2]) / (xs[-1] - xs[-2])
        return _interp1(xs, ps, servo_deg)

    def lid_servo(self, name: str, openness: float) -> float:
        xs, ys = self.lid_tabs[name]
        return _interp1(xs, ys, openness)

    def lid_max_open(self, name: str) -> float:
        return self.lid_tabs[name][0][-1]


def load_kinematics(path: str | None = None, log: Callable[[str], None] = print) -> Kinematics | None:
    """kinematics.json'u okur; yoksa ya da bozuksa None (dogrusal formule dusulur)."""
    path = path or KINEMATICS_PATH
    if not os.path.isfile(path):
        log(f"[eye] kinematik tablo yok ({path}) - SPEC §6 dogrusal formulu kullaniliyor")
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return Kinematics.from_dict(json.load(f), source=path)
    except Exception as e:  # bozuk tablo asla cokertmemeli
        log(f"[eye] kinematik tablo okunamadi ({e.__class__.__name__}: {e}) - dogrusal formul")
        return None


def fw_clamp(ch: int, deg: float) -> float:
    """Firmware'in sabit mekanik limitleri (sim.py = .ino). PC asla bunun disini gondermez."""
    return clamp(deg, sim.LIMIT_MIN[ch], sim.LIMIT_MAX[ch])


# --------------------------------------------------------------------------
# Kalibrasyon formulleri (SPEC §6)
# --------------------------------------------------------------------------
def eye_angle(cfg: dict, u: float) -> float:
    """angle = center + clamp(u,-1,1) * deg_per_unit * (invert ? -1 : 1), sonra [min,max]."""
    sign = -1.0 if cfg.get("invert") else 1.0
    a = cfg["center"] + clamp(u, -1.0, 1.0) * cfg["deg_per_unit"] * sign
    return clamp(a, cfg["min"], cfg["max"])


def lid_angle(cfg: dict, openness: float, kin: Kinematics | None = None,
              name: str | None = None) -> float:
    """Kapak acikligi -> ham servo acisi.

    Dogrusal (SPEC §6): angle = closed + openness * (open - closed), openness [0,1].
    Tablo varken modelin 4-cubuk egrisi T(o) kalibrasyona oturtulur:
        angle = closed + (T(o) - T(0)) * (open - closed) / (T(1) - T(0))
    o=0 ve o=1'de kalibre edilen deger AYNEN korunur, aradaki egri modelden
    gelir; openness tablonun ust ucuna (>1, bakis takibi payi) kadar izinlidir.

    ``invert``: servo ters takildiginda aci 180 - aci olarak aynalanir.
    """
    if kin is not None and name in kin.lid_tabs:
        t0, t1 = kin.lid_servo(name, 0.0), kin.lid_servo(name, 1.0)
        if abs(t1 - t0) > 1e-6:
            o = clamp(openness, 0.0, kin.lid_max_open(name))
            a = cfg["closed"] + (kin.lid_servo(name, o) - t0) * (cfg["open"] - cfg["closed"]) / (t1 - t0)
            return 180.0 - a if cfg.get("invert") else a
    o = clamp(openness, 0.0, 1.0)
    a = cfg["closed"] + o * (cfg["open"] - cfg["closed"])
    return 180.0 - a if cfg.get("invert") else a


def eye_angles(ch: dict, u: float, v: float, kin: Kinematics | None = None) -> tuple[float, float]:
    """Bakis (u, v) -> (EYE_YAW, EYE_PITCH) ham servo acilari.

    Tablo varken goz acisi yaw = u * 30, pitch = v * 25 (tablo kapsami); yaw
    servosu 1B tablodan, pitch servosu (yaw, pitch) 2B tablosundan. ``center``
    kalibrasyon trimi olarak eklenir (model notru = tablo(0)); ``invert`` servo
    tepkisini center etrafinda aynalar. Sonuc settings [min,max]'a kirpilir
    (compute_angles ayrica firmware limitine kirpar): ulasilamayan poz en yakin
    guvenli poza iner. Tablo yoksa SPEC §6 dogrusal formulu.
    """
    cy, cp = ch["EYE_YAW"], ch["EYE_PITCH"]
    if kin is None:
        return eye_angle(cy, u), eye_angle(cp, v)
    yaw_deg = clamp(u, -1.0, 1.0) * kin.yaw_range
    pitch_deg = clamp(v, -1.0, 1.0) * kin.pitch_range
    sy = -1.0 if cy.get("invert") else 1.0
    sp = -1.0 if cp.get("invert") else 1.0
    yaw = cy["center"] + sy * (kin.yaw_servo(yaw_deg) - kin.yaw_servo(0.0))
    pitch = cp["center"] + sp * (kin.pitch_servo(yaw_deg, pitch_deg) - kin.pitch_servo(0.0, 0.0))
    return clamp(yaw, cy["min"], cy["max"]), clamp(pitch, cp["min"], cp["max"])


def lid_follow(openness: float, pitch_v: float, k: float,
               max_upper: float = 1.0, max_lower: float = 1.0) -> tuple[float, float]:
    """Kapaklarin bakisin pitch'ini takibi -> (ust, alt) aciklik.

    Yukari bakinca (v>0) ust kapak kalkar (daha acik), alt kapak da yukari
    gelir (daha kapali); asagida tersi. Kayma aciklikla orantili: kapali bir
    goz (kirpma) bakis yuzunden aralanmaz. firmware idle'i ayni formulu kullanir.
    max_*: kinematik tablo varken takip 1'in ustune (modelin payina) cikabilir.
    """
    shift = k * 0.5 * clamp(pitch_v, -1.0, 1.0) * openness
    return clamp(openness + shift, 0.0, max_upper), clamp(openness - shift, 0.0, max_lower)


def compute_angles(settings: dict, u: float, v: float,
                   left_open: float, right_open: float,
                   follow_pitch: bool = True, kin: Kinematics | None = None) -> list[float]:
    """Bakis (u, v) ve goz basina aciklik -> kanal 0..5 ham servo acilari.

    Donen her aci firmware'in sabit mekanik limitleri icindedir (fw_clamp).
    """
    ch = settings["channels"]
    k = settings["behaviour"]["lid_follow_pitch"] if follow_pitch else 0.0
    mu = kin.lid_max_open("LID_UL") if kin is not None else 1.0
    ml = kin.lid_max_open("LID_LL") if kin is not None else 1.0
    lu, ll = lid_follow(clamp(left_open, 0.0, 1.0), v, k, mu, ml)
    ru, rl = lid_follow(clamp(right_open, 0.0, 1.0), v, k, mu, ml)
    yaw, pitch = eye_angles(ch, u, v, kin)
    raw = [
        yaw,
        pitch,
        lid_angle(ch["LID_UL"], lu, kin, "LID_UL"),
        lid_angle(ch["LID_LL"], ll, kin, "LID_LL"),
        lid_angle(ch["LID_UR"], ru, kin, "LID_UR"),
        lid_angle(ch["LID_LR"], rl, kin, "LID_LR"),
    ]
    return [fw_clamp(i, a) for i, a in enumerate(raw)]


def format_s_command(angles: Iterable[float]) -> str:
    """S satiri. 2 ondalik: limitler 2 ondaliklidir, 1 ondalik yuvarlama
    (113.77 -> 113.8) limitin disina tasabiliyordu. Satir <= 47 bayt."""
    vals = [clamp(float(a), 0.0, 180.0) for a in angles]
    if len(vals) != 6:
        raise ValueError("S komutu 6 aci ister")
    return "S " + " ".join(f"{a:.2f}" for a in vals)


def parse_state(line: str):
    """'STATE a0..a5 idle=0|1' -> (acilar listesi, idle) ya da None."""
    parts = line.strip().split()
    if len(parts) != 8 or parts[0] != "STATE" or not parts[7].startswith("idle="):
        return None
    try:
        angles = [float(p) for p in parts[1:7]]
    except ValueError:
        return None
    flag = parts[7][5:]
    if flag not in ("0", "1"):
        return None
    return angles, flag == "1"


# --------------------------------------------------------------------------
# Port bulma + el sikisma
# --------------------------------------------------------------------------
def _serial_module():
    try:
        import serial  # noqa: F401
        from serial.tools import list_ports  # noqa: F401
        return serial
    except ImportError:
        return None


def candidate_ports(exclude: Iterable[str] = ()) -> list[str]:
    """Arduino benzeri portlar once; Bluetooth sanal portlari atlanir (acilisi uzun suruyor)."""
    serial = _serial_module()
    if serial is None:
        return []
    from serial.tools import list_ports
    excl = {e.strip().upper() for e in exclude if e.strip()}
    likely, other = [], []
    for p in list_ports.comports():
        if p.device.upper() in excl:
            continue
        desc = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
        if "bluetooth" in desc or "bthenum" in desc:
            continue
        if any(k in desc for k in ("arduino", "ch340", "ch341", "wch", "usb-serial", "usb serial", "2341:", "1a86:")):
            likely.append(p.device)
        else:
            other.append(p.device)
    return likely + other


def handshake(ser, timeout_s: float = HANDSHAKE_TIMEOUT_S,
              query_after_s: float = HANDSHAKE_QUERY_AFTER_S,
              clock: Callable[[], float] = time.monotonic) -> str:
    """Acik bir portun Project Eye v2 olup olmadigini dogrular.

    Donen: 'eye' (dogrulandi), 'lunar' (Lunar gimbal'i - dokunma), 'unknown'.
    Banner'i bekler; gelmezse (kart resetlenmediyse) '?' sorup STATE bekler.
    Lunar gimbal'i '?' satirini zaten yok sayar (P...T... bicimi degil).
    """
    start = clock()
    queried = False
    while clock() - start < timeout_s:
        raw = ser.readline()
        if raw:
            line = raw.decode("ascii", "replace").strip()
            if line == FW_BANNER or parse_state(line) is not None:
                return "eye"
            if line == LUNAR_GIMBAL_BANNER:
                return "lunar"
        elif not queried and clock() - start >= query_after_s:
            ser.write(b"?\n")
            queried = True
        # gercek portta readline() kendi timeout'u (0.1 sn) kadar bekler
    return "unknown"


def _open_real(device: str):
    import serial
    return serial.Serial(device, BAUD, timeout=0.1, write_timeout=0.5)


def find_eye_port(port: str = "auto", exclude: Iterable[str] = (),
                  opener: Callable[[str], object] | None = None,
                  ports: list[str] | None = None, log: Callable[[str], None] = print,
                  handshake_timeout_s: float = HANDSHAKE_TIMEOUT_S,
                  query_after_s: float = HANDSHAKE_QUERY_AFTER_S):
    """Project Eye v2 firmware'ini calistiran portu bulur -> (ser, device) ya da (None, None).

    ``port`` 'auto' degilse yalnizca o denenir. ``EYE_EXCLUDE_PORTS`` ortam
    degiskeni (virgulle) otomatik taramadan port cikarir - ornegin Lunar
    gimbal'i COM3'te ve acilinca resetlenmesini istemiyorsan: EYE_EXCLUDE_PORTS=COM3.
    """
    opener = opener or _open_real
    env_excl = os.environ.get("EYE_EXCLUDE_PORTS", "").split(",")
    if port and port.lower() != "auto":
        devices = [port]
    else:
        devices = ports if ports is not None else candidate_ports(list(exclude) + env_excl)
    for dev in devices:
        try:
            ser = opener(dev)
        except Exception as e:  # mesgul port (Lunar calisiyor), izin yok, vb.
            log(f"[eye] {dev}: acilamadi ({e.__class__.__name__}) - atlandi")
            continue
        try:
            kind = handshake(ser, handshake_timeout_s, query_after_s)
        except Exception as e:
            kind = "unknown"
            log(f"[eye] {dev}: el sikisma hatasi ({e})")
        if kind == "eye":
            log(f"[eye] {dev}: Project Eye v2 dogrulandi")
            return ser, dev
        log(f"[eye] {dev}: {'Lunar gimbal' if kind == 'lunar' else 'tanimsiz cihaz'} - kullanilmiyor")
        try:
            ser.close()
        except Exception:
            pass
    return None, None


# --------------------------------------------------------------------------
# Baglanti
# --------------------------------------------------------------------------
class EyeLink:
    """Satir tabanli protokol ucu. Gercek pyserial portu ya da SimSerial sarar."""

    def __init__(self, ser, name: str, simulated: bool):
        self.ser = ser
        self.name = name
        self.simulated = simulated
        self.ok_count = 0
        self.err_count = 0
        self.last_error: str | None = None
        self.last_state = None
        self.alive = True
        self._rx = bytearray()

    @classmethod
    def open(cls, port: str = "auto", allow_sim: bool = True, sim_clock=time.monotonic,
             log: Callable[[str], None] = print) -> "EyeLink":
        if port and port.lower() == "sim":
            return cls.simulator(sim_clock)
        if _serial_module() is None:
            log("[eye] pyserial kurulu degil - sim moduna geciliyor")
        else:
            ser, dev = find_eye_port(port, log=log)
            if ser is not None:
                return cls(ser, dev, simulated=False)
            log("[eye] Project Eye v2 bulunamadi")
        if not allow_sim:
            raise ConnectionError("Project Eye v2 bulunamadi ve sim kapali")
        log("[eye] SIM modu: komutlar firmware simulatorune gidiyor")
        return cls.simulator(sim_clock)

    @classmethod
    def simulator(cls, clock=time.monotonic, seed=None) -> "EyeLink":
        s = sim.SimSerial(clock=clock, seed=seed)
        link = cls(s, "SIM", simulated=True)
        link.poll()  # banner'i tuket
        return link

    def send(self, line: str) -> bool:
        if not self.alive:
            return False
        try:
            self.ser.write((line + "\n").encode("ascii"))
            return True
        except Exception as e:
            self.alive = False
            self.last_error = f"yazma hatasi: {e}"
            return False

    def poll(self) -> list[str]:
        """Bekleyen cevaplari bloklamadan okur, sayaclari gunceller."""
        lines: list[str] = []
        if not self.alive:
            return lines
        try:
            n = self.ser.in_waiting
            if n:
                self._rx += self.ser.read(n)
        except Exception as e:
            self.alive = False
            self.last_error = f"okuma hatasi: {e}"
            return lines
        while True:
            idx = self._rx.find(b"\n")
            if idx < 0:
                break
            line = self._rx[:idx].decode("ascii", "replace").strip()
            del self._rx[:idx + 1]
            if not line:
                continue
            lines.append(line)
            if line == "OK":
                self.ok_count += 1
            elif line.startswith("ERR"):
                self.err_count += 1
                self.last_error = line
            else:
                st = parse_state(line)
                if st is not None:
                    self.last_state = st
        if len(self._rx) > 4096:  # anlamsiz veri birikmesin
            self._rx.clear()
        return lines

    def query_state(self, timeout_s: float = 0.5, clock=time.monotonic):
        """'?' gonderir, STATE cevabini bekler -> (acilar, idle) ya da None."""
        self.last_state = None
        if not self.send("?"):
            return None
        start = clock()
        while True:
            self.poll()
            if self.last_state is not None:
                return self.last_state
            if clock() - start >= timeout_s:
                return None
            if not self.simulated:
                time.sleep(0.005)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
        self.alive = False


# --------------------------------------------------------------------------
# Davranis
# --------------------------------------------------------------------------
def _blink_envelope(p: float) -> float:
    """Kirpma zarfi (0..1 ilerleme -> aciklik carpani): hizli kapan, kisa dur, yavas ac."""
    if p < 0.3:
        return 1.0 - p / 0.3
    if p < 0.5:
        return 0.0
    if p < 1.0:
        return (p - 0.5) / 0.5
    return 1.0


class EyeController:
    """Yuksek seviye goz API'si.

    Kullanim::

        eye = EyeController()          # donanimi bul, yoksa sim
        while True:
            eye.look(u, v)             # -1..1; +u robotun sagi, +v yukari
            eye.update()               # her karede cagir (dt otomatik)

    ``look`` hedefi belirler; gercek bakis ``saccade_speed_deg_s`` hiziyla
    oraya gider. ``lids`` taban acikligi belirler; kirpma/wink bunun uzerine
    carpan olarak gelir, pitch takibi en son uygulanir.
    """

    def __init__(self, settings: dict | None = None, link: EyeLink | None = None,
                 port: str | None = None, allow_sim: bool = True,
                 clock: Callable[[], float] = time.monotonic,
                 rng: random.Random | None = None, log: Callable[[str], None] = print,
                 kinematics="auto"):
        """kinematics: "auto" = cad/out/kinematics.json (yoksa dogrusal), None = dogrusal
        formul, ya da hazir bir Kinematics nesnesi."""
        self.settings = settings if settings is not None else load_settings()
        validate_settings(self.settings)
        self.kin = load_kinematics(log=log) if kinematics == "auto" else kinematics
        self.clock = clock
        self.rng = rng or random.Random()
        self.log = log
        if link is None:
            link = EyeLink.open(port or self.settings["serial_port"], allow_sim=allow_sim,
                                sim_clock=clock, log=log)
        self.link = link

        now = clock()
        self.gaze_u = 0.0          # anlik (hiz sinirli) bakis
        self.gaze_v = 0.0
        self.target_u = 0.0
        self.target_v = 0.0
        self.open_left = 0.85      # taban aciklik
        self.open_right = 0.85
        self.follow_pitch = True
        self.auto_blink = True
        self.idle_wander = True    # look() gelmezse yavasca etrafa bakin
        self.enabled = True        # False: S gonderme (firmware idle'a birakir)
        self._last_look = now
        self._last_update = now
        self._blink_start: float | None = None
        self._blink_len = BLINK_S
        self._wink: tuple[str, float, float] | None = None   # (side, start, len)
        self._next_blink = now + self._blink_interval()
        self._last_sent: list[float] | None = None
        self._last_send_t = -1e9
        self.effective_open = (self.open_left, self.open_right)  # kirpma dahil (sol, sag)
        self.angles = compute_angles(self.settings, 0.0, 0.0, self.open_left, self.open_right,
                                     kin=self.kin)
        self.sent_count = 0
        self._lost_reported = False

    # -- durum --------------------------------------------------------------
    @property
    def simulated(self) -> bool:
        return self.link.simulated

    @property
    def mapping(self) -> str:
        return "TABLE" if self.kin is not None else "LINEAR"

    @property
    def mode(self) -> str:
        if not self.link.alive:
            return "OFFLINE"
        return "SIM" if self.link.simulated else f"HW {self.link.name}"

    @property
    def idle(self) -> bool:
        """look() idle_after_s'den uzun suredir cagrilmadi mi."""
        return self.clock() - self._last_look > self.settings["behaviour"]["idle_after_s"]

    def _blink_interval(self) -> float:
        lo, hi = self.settings["behaviour"]["blink_interval_s"]
        return self.rng.uniform(lo, hi)

    # -- API ----------------------------------------------------------------
    def look(self, u: float, v: float) -> None:
        """Bakis hedefi, -1..1 (+u robotun sagi, +v yukari)."""
        if not (math.isfinite(u) and math.isfinite(v)):
            return
        self.target_u = clamp(u, -1.0, 1.0)
        self.target_v = clamp(v, -1.0, 1.0)
        self._last_look = self.clock()

    def lids(self, openness: float | None = None, left: float | None = None,
             right: float | None = None) -> None:
        """Taban kapak acikligi (0 kapali .. 1 acik). left/right goz bazinda ezer."""
        if openness is not None:
            self.open_left = self.open_right = clamp(openness, 0.0, 1.0)
        if left is not None:
            self.open_left = clamp(left, 0.0, 1.0)
        if right is not None:
            self.open_right = clamp(right, 0.0, 1.0)

    def blink(self, duration: float = BLINK_S) -> None:
        now = self.clock()
        self._blink_start = now
        self._blink_len = max(0.05, duration)
        self._next_blink = now + self._blink_interval()

    def wink(self, side: str, duration: float = WINK_S) -> None:
        """Tek goz kirpma; side = 'left' | 'right' (robotun kendi solu/sagi)."""
        if side not in SIDE_LIDS:
            raise ValueError("side 'left' ya da 'right' olmali")
        self._wink = (side, self.clock(), max(0.05, duration))

    def release(self) -> None:
        """PC kontrolunu birak: S gonderimi durur, 2 sn sonra firmware kendi yasar."""
        self.enabled = False
        self.link.send("A 1")

    def resume(self) -> None:
        self.enabled = True
        self._last_sent = None

    def detach(self) -> None:
        """Tum servolari birak (guc kesilmis gibi). Sonraki update tekrar baglar."""
        self.link.send("D")
        self._last_sent = None

    # -- dongu --------------------------------------------------------------
    def _envelopes(self, now: float) -> tuple[float, float]:
        left = right = 1.0
        if self._blink_start is not None:
            p = (now - self._blink_start) / self._blink_len
            if p >= 1.0:
                self._blink_start = None
            else:
                left = right = _blink_envelope(p)
        if self._wink is not None:
            side, start, length = self._wink
            p = (now - start) / length
            if p >= 1.0:
                self._wink = None
            else:
                e = _blink_envelope(p)
                if side == "left":
                    left = min(left, e)
                else:
                    right = min(right, e)
        return left, right

    def _step_gaze(self, dt: float, tu: float, tv: float) -> None:
        """Hedefe derece uzayinda hiz sinirli (sakkad) yaklasma."""
        # birim basina goz derecesi: tabloda +-30/+-25, dogrusal modda deg_per_unit
        if self.kin is not None:
            dpu_y, dpu_p = self.kin.yaw_range, self.kin.pitch_range
        else:
            ch = self.settings["channels"]
            dpu_y = max(1e-6, float(ch["EYE_YAW"]["deg_per_unit"]))
            dpu_p = max(1e-6, float(ch["EYE_PITCH"]["deg_per_unit"]))
        dy = (tu - self.gaze_u) * dpu_y
        dp = (tv - self.gaze_v) * dpu_p
        dist = math.hypot(dy, dp)
        max_step = self.settings["behaviour"]["saccade_speed_deg_s"] * dt
        if dist <= max_step or dist < 1e-9:
            self.gaze_u, self.gaze_v = tu, tv
        else:
            f = max_step / dist
            self.gaze_u += (tu - self.gaze_u) * f
            self.gaze_v += (tv - self.gaze_v) * f

    def update(self, dt: float | None = None) -> list[float]:
        """Bir adim: bakis, kapaklar, kirpma, gonderim. Donen: 6 ham aci."""
        now = self.clock()
        if dt is None:
            dt = now - self._last_update
        dt = clamp(dt, 0.0, 0.25)
        self._last_update = now

        tu, tv = self.target_u, self.target_v
        if self.idle_wander and self.idle:
            # ultimatum EyeRig'den: yavas, organik etrafi kolacan etme
            tu = 0.45 * math.sin(now * 0.37) + 0.12 * math.sin(now * 1.3)
            tv = 0.18 * math.sin(now * 0.51 + 1.0)
        self._step_gaze(dt, tu, tv)

        if self.auto_blink and self._blink_start is None and now >= self._next_blink:
            self.blink()
        env_l, env_r = self._envelopes(now)

        self.effective_open = (self.open_left * env_l, self.open_right * env_r)
        self.angles = compute_angles(self.settings, self.gaze_u, self.gaze_v,
                                     self.effective_open[0], self.effective_open[1],
                                     follow_pitch=self.follow_pitch, kin=self.kin)
        self._maybe_send(now)
        self.link.poll()
        if not self.link.alive and not self._lost_reported:
            self._lost_reported = True
            self.log(f"[eye] baglanti koptu: {self.link.last_error} - komutlar gonderilmiyor")
        return self.angles

    def _maybe_send(self, now: float) -> None:
        if not self.enabled or not self.link.alive:
            return
        if now - self._last_send_t < 1.0 / SEND_HZ:
            return
        changed = (self._last_sent is None or
                   max(abs(a - b) for a, b in zip(self.angles, self._last_sent)) >= SEND_MIN_DELTA_DEG)
        if not changed and now - self._last_send_t < KEEPALIVE_S:
            return
        if self.link.send(format_s_command(self.angles)):
            self._last_sent = list(self.angles)
            self._last_send_t = now
            self.sent_count += 1

    def close(self, detach: bool = False) -> None:
        """Kapat. Varsayilan: servolar bagli kalir, firmware 2 sn sonra idle'a gecer."""
        try:
            if detach:
                self.link.send("D")
            else:
                self.link.send("A 1")
            if not self.link.simulated:
                time.sleep(0.05)  # son baytlar ciksin
        finally:
            self.link.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# --------------------------------------------------------------------------
def _demo(argv: list[str]) -> int:
    """Kucuk demo: sag-sol bak, yukari-asagi bak, kirp, wink."""
    import argparse
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Project Eye v2 hizli demo")
    ap.add_argument("--port", default=None, help="'auto', 'sim' ya da COMx")
    ap.add_argument("--sim", action="store_true", help="donanimi arama, dogrudan sim")
    args = ap.parse_args(argv)
    eye = EyeController(port="sim" if args.sim else args.port)
    print(f"mod: {eye.mode}  esleme: {eye.mapping}")
    script = [(0.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0), (0.0, 0.0)]
    try:
        for u, v in script:
            eye.look(u, v)
            t_end = time.monotonic() + 0.8
            while time.monotonic() < t_end:
                a = eye.update()
                time.sleep(0.02)
            print(f"look({u:+.1f},{v:+.1f}) -> " + " ".join(f"{x:5.1f}" for x in a))
        eye.blink()
        eye.wink("left")
        t_end = time.monotonic() + 0.6
        while time.monotonic() < t_end:
            eye.update()
            time.sleep(0.02)
        st = eye.link.query_state()
        print(f"firmware STATE: {st}  gonderilen S: {eye.sent_count}  ERR: {eye.link.err_count}")
    finally:
        eye.close()
    return 0


if __name__ == "__main__":
    sys.exit(_demo(sys.argv[1:]))
