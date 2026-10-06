"""
Project Eye v5 - seri surucu + kalibrasyon + davranis. 3 kanal:
0 EYE_YAW, 1 EYE_PITCH, 2 LIDS (dort kapak - 2 ust + 2 alt - birlikte). v3'un
kopyasi; guvenlik kurallari ayni.

1. **Kalibrasyon**: ``eye_angles`` (kinematics.json tablosu ya da dogrusal
   formul), ``lid_angle``, ``lid_follow``, ``compute_angles``; settings.json.
2. **Baglanti** (``EyeLink``): ``EYE v5 READY`` el sikismasiyla DOGRULANAN
   port (yalnizca banner); Lunar gimbal'i (``READY``), v2 ve v3 kartlari reddedilir.
   Donanim yoksa ``sim.SimSerial``.
3. **Davranis** (``EyeController``): ``look(u, v)``, ``lids(openness)``,
   ``blink()`` (wink YOK: dort kapak tek servoda); ``update(dt)``: sakkad hiz
   siniri, kapagin pitch takibi, rastgele kirpma, bosta gezinme, keepalive.

Guvenlik: hicbir yoldan firmware limiti disinda aci gonderilmez (fw_clamp),
S acilari 2 ondalik.

    python eye_control.py            # donanimi bul, yoksa sim; demo hareketi
    python eye_control.py --sim
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

import human_motion
import sim

# --------------------------------------------------------------------------
# Sabitler
# --------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
V2_DIR = os.path.dirname(HERE)
SETTINGS_PATH = os.path.join(V2_DIR, "settings.json")

CHANNELS = ("EYE_YAW", "EYE_PITCH", "LIDS")
EYE_CHANNELS = ("EYE_YAW", "EYE_PITCH")
LID_CHANNELS = ("LIDS",)

BAUD = sim.BAUD
FW_BANNER = sim.FW_BANNER
LUNAR_GIMBAL_BANNER = "READY"   # lunar-tracker/firmware/pan_tilt - DOKUNMA
HANDSHAKE_TIMEOUT_S = 3.0       # Uno DTR reset + bootloader ~1.6 sn
HANDSHAKE_QUERY_AFTER_S = 2.0   # banner gelmezse (reset olmadiysa) '?' ile sor

SEND_HZ = 50.0                  # S komutu ust hizi
SEND_MIN_DELTA_DEG = 0.1        # bundan kucuk degisimi tekrar gonderme
KEEPALIVE_S = 0.5               # firmware'in 2 sn idle'ina dusmemesi icin

BLINK_S = 0.25                  # kapanma %30, kapali %20, acilma %50

# Varsayilanlar = v4 mekanik modelinin onerisi (cad/out/kinematics.json -> settings_suggestion)
DEFAULT_SETTINGS = {
    "serial_port": "auto",
    "channels": {
        "EYE_YAW": {"center": 90, "min": 65, "max": 115, "invert": True, "deg_per_unit": 25.0},
        "EYE_PITCH": {"center": 90, "min": 70, "max": 110, "invert": False, "deg_per_unit": 20.0},
        "LIDS": {"closed": 102.29, "open": 77.71, "min": 77.71, "max": 102.29, "invert": False},
    },
    "behaviour": {
        "blink_interval_s": [3, 6],
        "lid_follow_pitch": 0.0,
        "saccade_speed_deg_s": 500,
        "idle_after_s": 1.5,
        "pitch_dead": False,
        "human_motion": False,  # True: kirpma/bakinma insan istatistiklerinden (human_motion.py)   # True: pitch servosu merkezde sabit; istenen aci yalniz hesaplanir (pitch_wanted)
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
        if name in LID_CHANNELS and ("min" in c or "max" in c):
            mm = (c.get("min"), c.get("max"))
            if not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in mm):
                problems.append(f"{name}: min ve max birlikte sayi olmali")
            elif not 0 <= mm[0] <= mm[1] <= 180:
                problems.append(f"{name}: 0 <= min <= max <= 180 olmali")
    b = s.get("behaviour", {})
    bi = b.get("blink_interval_s")
    if not (isinstance(bi, (list, tuple)) and len(bi) == 2 and 0 < bi[0] <= bi[1]):
        problems.append("behaviour.blink_interval_s: [min, max] ve 0 < min <= max olmali")
    for k in ("lid_follow_pitch", "saccade_speed_deg_s", "idle_after_s"):
        if not isinstance(b.get(k), (int, float)) or b[k] < 0:
            problems.append(f"behaviour.{k}: >= 0 sayi olmali")
    if not isinstance(b.get("human_motion"), bool):
        problems.append("behaviour.human_motion: true/false olmali")
    if not isinstance(b.get("pitch_dead"), bool):
        problems.append("behaviour.pitch_dead: true/false olmali")
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


def _table(rows, xkey: str):
    pts = sorted((float(r[xkey]) + 0.0, float(r["servo_deg"])) for r in rows)
    xs, ys = [a for a, _ in pts], [b for _, b in pts]
    if not _strictly_increasing(xs):
        raise ValueError(f"'{xkey}' tablosu artan degil")
    for v in ys:
        if not (math.isfinite(v) and 0.0 <= v <= 180.0):
            raise ValueError(f"tabloda 0-180 disi servo acisi: {v}")
    return xs, ys


class Kinematics:
    """v4 mekanik modelinin servo tablolari; uc kanal da 1B (servo acilari arasinda capraz etki yok).

    * ``yaw_servo(yaw_deg)``     EYE_YAW   (modelde servo = 90 - yaw)
    * ``pitch_servo(pitch_deg)`` EYE_PITCH (modelde servo = 90 + pitch)
    * ``lid_servo(openness)``    LIDS      (0 kapali 102.29 .. 1 acik 77.71; dort-cubuk, hafif egri)

    ``*_dir``: tablonun kendi yonu (+1: aci artinca servo artar, -1: azalir).
    """

    def __init__(self, yaw_tab, pitch_tab, lid_tab, source: str = "?"):
        self.yaw_x, self.yaw_s = yaw_tab
        self.pitch_x, self.pitch_s = pitch_tab
        self.lid_x, self.lid_s = lid_tab
        self.source = source
        self.yaw_range = min(abs(self.yaw_x[0]), abs(self.yaw_x[-1]))
        self.pitch_range = min(abs(self.pitch_x[0]), abs(self.pitch_x[-1]))
        if self.yaw_range <= 0 or self.pitch_range <= 0:
            raise ValueError("goz tablolari 0'in iki yanini kapsamali")
        self.yaw_dir = 1.0 if self.yaw_servo(self.yaw_range) >= self.yaw_servo(0.0) else -1.0
        self.pitch_dir = 1.0 if self.pitch_servo(self.pitch_range) >= self.pitch_servo(0.0) else -1.0

    @classmethod
    def from_dict(cls, data: dict, source: str = "?") -> "Kinematics":
        """Sema/deger kontrolu yapar; sorun varsa ValueError/KeyError/TypeError."""
        ch = data["channels"]
        yaw = _table(ch["EYE_YAW"]["table"], "eye_yaw_deg")
        pitch = _table(ch["EYE_PITCH"]["table"], "eye_pitch_deg")
        lid = _table(ch["LIDS"]["table"], "lid")
        if lid[0][0] > 0.0 or lid[0][-1] < 1.0:
            raise ValueError("LIDS tablosu 0..1 acikligi kapsamiyor")
        return cls(yaw, pitch, lid, source)

    def yaw_servo(self, yaw_deg: float) -> float:
        return _interp1(self.yaw_x, self.yaw_s, yaw_deg)

    def pitch_servo(self, pitch_deg: float) -> float:
        return _interp1(self.pitch_x, self.pitch_s, pitch_deg)

    def lid_servo(self, openness: float) -> float:
        return _interp1(self.lid_x, self.lid_s, openness)

    def lid_max_open(self) -> float:
        return self.lid_x[-1]


def load_kinematics(path: str | None = None, log: Callable[[str], None] = print) -> Kinematics | None:
    """kinematics.json'u okur; yoksa ya da bozuksa None (dogrusal formule dusulur)."""
    path = path or KINEMATICS_PATH
    if not os.path.isfile(path):
        log(f"[eye] kinematik tablo yok ({path}) - dogrusal formul kullaniliyor")
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
# Kalibrasyon formulleri (v2 SPEC §6'nin 3 kanalli hali)
# --------------------------------------------------------------------------
def eye_angle(cfg: dict, u: float) -> float:
    """angle = center + clamp(u,-1,1) * deg_per_unit * (invert ? -1 : 1), sonra [min,max]."""
    sign = -1.0 if cfg.get("invert") else 1.0
    a = cfg["center"] + clamp(u, -1.0, 1.0) * cfg["deg_per_unit"] * sign
    return clamp(a, cfg["min"], cfg["max"])


def lid_angle(cfg: dict, openness: float, kin: Kinematics | None = None) -> float:
    """Kapak acikligi -> ham servo acisi.

    Dogrusal: angle = closed + openness * (open - closed), openness [0,1].
    Tablo varken T(o) kalibrasyona oturtulur (v2 ile ayni):
        angle = closed + (T(o) - T(0)) * (open - closed) / (T(1) - T(0))
    ``invert``: servo ters takildiysa 180 - aci. Sonra varsa [min,max] (ham aci).
    """
    a = None
    if kin is not None:
        t0, t1 = kin.lid_servo(0.0), kin.lid_servo(1.0)
        if abs(t1 - t0) > 1e-6:
            o = clamp(openness, 0.0, kin.lid_max_open())
            a = cfg["closed"] + (kin.lid_servo(o) - t0) * (cfg["open"] - cfg["closed"]) / (t1 - t0)
    if a is None:
        o = clamp(openness, 0.0, 1.0)
        a = cfg["closed"] + o * (cfg["open"] - cfg["closed"])
    if cfg.get("invert"):
        a = 180.0 - a
    if "min" in cfg and "max" in cfg:
        a = clamp(a, cfg["min"], cfg["max"])
    return a


def eye_angles(ch: dict, u: float, v: float, kin: Kinematics | None = None) -> tuple[float, float]:
    """Bakis (u, v) -> (EYE_YAW, EYE_PITCH) ham servo acilari.

    Tablo varken goz acisi yaw = u * 25, pitch = v * 20 (tablo kapsami):
        servo = center + yon * (T(aci) - T(0)),  yon = settings_yonu * tablo_yonu
    ve bu sapma her yonde +-1 = settings min/max (kalibre donanim siniri) olacak sekilde olceklenir.
    ``invert`` dogrusal formuldeki anlamini korur (true: +u -> servo azalir); tablo
    yalnizca egrinin SEKLINI verir. Boylece settings_suggestion (yaw invert=true)
    tabloyla (servo = 90 - yaw) birebir ayni sonucu verir, donanimda yon ters
    cikarsa tek bayrakla iki modda da duzelir. Sonra [min,max]'a kirpilir.
    """
    cy, cp = ch["EYE_YAW"], ch["EYE_PITCH"]
    if kin is None:
        return eye_angle(cy, u), eye_angle(cp, v)
    yaw_deg = clamp(u, -1.0, 1.0) * kin.yaw_range
    pitch_deg = clamp(v, -1.0, 1.0) * kin.pitch_range
    wy = (-1.0 if cy.get("invert") else 1.0) * kin.yaw_dir
    wp = (-1.0 if cp.get("invert") else 1.0) * kin.pitch_dir
    yaw = cy["center"] + _to_hw_limits(cy, u, wy * (kin.yaw_servo(yaw_deg) - kin.yaw_servo(0.0)),
                                       wy * (kin.yaw_servo(math.copysign(kin.yaw_range, u)) - kin.yaw_servo(0.0)))
    pitch = cp["center"] + _to_hw_limits(cp, v, wp * (kin.pitch_servo(pitch_deg) - kin.pitch_servo(0.0)),
                                         wp * (kin.pitch_servo(math.copysign(kin.pitch_range, v)) - kin.pitch_servo(0.0)))
    return clamp(yaw, cy["min"], cy["max"]), clamp(pitch, cp["min"], cp["max"])


def _to_hw_limits(c: dict, x: float, off: float, off_full: float) -> float:
    """Tablo egrisinin SEKLI korunur, ama +-1 = kalibre edilmis donanim siniri (min/max).
    Kalibrasyonda bulunan gercek aralik (orn. yaw 45-111) CAD tablosunun +-25 derecesinden
    genis olabilir; olcek her yon icin ayri (merkez ortada olmayabilir)."""
    if abs(off_full) < 1e-9:
        return off
    limit = c["max"] if off_full > 0 else c["min"]
    return off * abs(limit - c["center"]) / abs(off_full)


def lid_follow(openness: float, pitch_v: float, k: float, max_open: float = 1.0) -> float:
    """Ust kapaklar bakisin pitch'ini takip eder: yukari bakinca kalkar, asagi bakinca iner.

    v4'te kapaklar pitch cercevesinde: pitch'i ZATEN mekanik olarak izler. Bu
    formul onun USTUNE eklenir; bu yuzden lid_follow_pitch varsayilani 0.0. Kayma aciklikla orantili: kapali goz aralanmaz. firmware idle ayni formul.
    """
    return clamp(openness + k * 0.5 * clamp(pitch_v, -1.0, 1.0) * openness, 0.0, max_open)


def compute_angles(settings: dict, u: float, v: float, openness: float,
                   follow_pitch: bool = True, kin: Kinematics | None = None) -> list[float]:
    """Bakis (u, v) ve kapak acikligi -> kanal 0..2 ham servo acilari.

    Donen her aci firmware'in sabit mekanik limitleri icindedir (fw_clamp).
    """
    ch = settings["channels"]
    k = settings["behaviour"]["lid_follow_pitch"] if follow_pitch else 0.0
    lid_o = lid_follow(clamp(openness, 0.0, 1.0), v, k, kin.lid_max_open() if kin else 1.0)
    yaw, pitch = eye_angles(ch, u, v, kin)
    raw = [yaw, pitch, lid_angle(ch["LIDS"], lid_o, kin)]
    return [fw_clamp(i, a) for i, a in enumerate(raw)]


def format_s_command(angles: Iterable[float]) -> str:
    """S satiri. 2 ondalik: limitler 2 ondaliklidir, 1 ondalik yuvarlama
    (113.77 -> 113.8) limitin disina tasabiliyordu. Satir <= 23 bayt."""
    vals = [clamp(float(a), 0.0, 180.0) for a in angles]
    if len(vals) != 3:
        raise ValueError("S komutu 3 aci ister")
    return "S " + " ".join(f"{a:.2f}" for a in vals)


def parse_state(line: str):
    """'STATE a0 a1 a2 idle=0|1' -> (acilar listesi, idle) ya da None."""
    parts = line.strip().split()
    if len(parts) != 5 or parts[0] != "STATE" or not parts[4].startswith("idle="):
        return None
    try:
        angles = [float(p) for p in parts[1:4]]
    except ValueError:
        return None
    flag = parts[4][5:]
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
    """Acik bir portun Project Eye v5 olup olmadigini dogrular.

    Donen: 'eye' (v4 dogrulandi), 'lunar' (Lunar gimbal'i - dokunma), 'other-eye'
    (v2/v3 karti), 'unknown'.

    v4'te YALNIZCA acilis banner'i (``EYE v5 READY``) kabul edilir. v3 karti da
    3 acili ``STATE`` dondurdugu icin '?' cevabi v3 ile v4'u ayiramaz; bu yuzden
    v2/v3'teki "banner gelmezse '?' sor, STATE'i kabul et" yolu KALDIRILDI.
    Uno portu acilinca DTR ile resetlenip banner'i yeniden basar; resetlenmeyen
    kart 'unknown' olur (bkz. DECISIONS.md). query_after_s geriye uyumluluk icin
    duruyor, kullanilmiyor. Hicbir karta hicbir sey YAZILMAZ.
    """
    start = clock()
    while clock() - start < timeout_s:
        raw = ser.readline()
        if raw:
            line = raw.decode("ascii", "replace").strip()
            if line == FW_BANNER:
                return "eye"
            if line == LUNAR_GIMBAL_BANNER:
                return "lunar"
            if line.startswith("EYE v") and line.endswith("READY"):
                return "other-eye"   # v2 (6 kanal) / v3 (farkli kapak limitleri)
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
    """Project Eye v5 firmware'ini calistiran portu bulur -> (ser, device) ya da (None, None).

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
            log(f"[eye] {dev}: Project Eye v5 dogrulandi")
            return ser, dev
        what = {"lunar": "Lunar gimbal", "other-eye": "baska surum Project Eye"}.get(kind, "tanimsiz cihaz")
        log(f"[eye] {dev}: {what} - kullanilmiyor")
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

    def __init__(self, ser, name: str, simulated: bool, clock: Callable[[], float] = time.monotonic):
        self.ser = ser
        self.name = name
        self.simulated = simulated
        self.clock = clock
        self.last_rx_t = clock()    # karttan son satir (saglik: cevap kesildi mi)
        self.resets = 0             # acilistan sonra gelen banner = kart yeniden basladi (brown-out?)
        self.ok_count = 0
        self.err_count = 0
        self.button_presses = 0     # firmware "BTN" (PC surerken D2 butonuna basildi)
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
            log("[eye] Project Eye v5 bulunamadi")
        if not allow_sim:
            raise ConnectionError("Project Eye v5 bulunamadi ve sim kapali")
        log("[eye] SIM modu: komutlar firmware simulatorune gidiyor")
        return cls.simulator(sim_clock)

    @classmethod
    def simulator(cls, clock=time.monotonic, seed=None) -> "EyeLink":
        s = sim.SimSerial(clock=clock, seed=seed)
        link = cls(s, "SIM", simulated=True, clock=clock)
        link.poll()  # banner'i tuket
        link.resets = 0
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
            self.last_rx_t = self.clock()
            if line.startswith("EYE v") and line.endswith("READY"):
                self.resets += 1
            if line == "OK":
                self.ok_count += 1
            elif line == "BTN":
                self.button_presses += 1
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
    oraya gider. ``lids`` taban acikligi belirler; kirpma bunun uzerine
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
        self.openness = 0.85       # taban kapak acikligi
        self.follow_pitch = True
        self.auto_blink = True
        self.idle_wander = True    # look() gelmezse yavasca etrafa bakin
        self.enabled = True        # False: S gonderme (firmware idle'a birakir)
        self._last_look = now
        self._last_update = now
        self.blink_context = "rest"   # human_motion: "rest" | "conversation" (yuz varken)
        self._wander = human_motion.GazeWander(self.rng)
        self._blink_start: float | None = None
        self._blink_len = BLINK_S
        self._next_blink = now + self._blink_interval()
        self._last_sent: list[float] | None = None
        self._last_send_t = -1e9
        self.effective_open = self.openness      # kirpma dahil
        self.angles = compute_angles(self.settings, 0.0, 0.0, self.openness, kin=self.kin)
        self.pitch_wanted = self.angles[1]   # pitch_dead iken servo yerine gostermek icin
        self.sent_count = 0
        self._lost_reported = False

    # -- durum --------------------------------------------------------------
    @property
    def simulated(self) -> bool:
        return self.link.simulated

    @property
    def pitch_dead(self) -> bool:
        return self.settings["behaviour"]["pitch_dead"]

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

    @property
    def human_motion(self) -> bool:
        return self.settings["behaviour"]["human_motion"]

    def _blink_interval(self) -> float:
        if self.human_motion:
            return human_motion.blink_interval(self.rng, self.blink_context)
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

    def lids(self, openness: float) -> None:
        """Taban kapak acikligi (0 kapali .. 1 acik); dort kapak birlikte."""
        if math.isfinite(openness):
            self.openness = clamp(openness, 0.0, 1.0)

    def blink(self, duration: float = BLINK_S) -> None:
        now = self.clock()
        self._blink_start = now
        self._blink_len = max(0.05, duration)
        self._next_blink = now + self._blink_interval()

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
    def _envelope(self, now: float) -> float:
        if self._blink_start is None:
            return 1.0
        p = (now - self._blink_start) / self._blink_len
        if p >= 1.0:
            self._blink_start = None
            return 1.0
        return _blink_envelope(p)

    def _step_gaze(self, dt: float, tu: float, tv: float) -> None:
        """Hedefe derece uzayinda hiz sinirli (sakkad) yaklasma."""
        # birim basina goz derecesi: tabloda +-25/+-20, dogrusal modda deg_per_unit
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
        """Bir adim: bakis, kapak, kirpma, gonderim. Donen: 3 ham aci."""
        now = self.clock()
        if dt is None:
            dt = now - self._last_update
        dt = clamp(dt, 0.0, 0.25)
        self._last_update = now

        tu, tv = self.target_u, self.target_v
        if self.idle_wander and self.idle:
            # ultimatum EyeRig'den: yavas, organik etrafi kolacan etme
            if self.human_motion:
                # fiksasyon -> sakkad, nokta periyotsuz gurultuden (human_motion.py)
                tu, tv = self._wander.step(now)
            else:
                tu = 0.45 * math.sin(now * 0.37) + 0.12 * math.sin(now * 1.3)
                tv = 0.18 * math.sin(now * 0.51 + 1.0)
        self._step_gaze(dt, tu, tv)

        if self.auto_blink and self._blink_start is None and now >= self._next_blink:
            self.blink(human_motion.blink_duration(self.rng) if self.human_motion else BLINK_S)
        self.effective_open = self.openness * self._envelope(now)
        wanted = compute_angles(self.settings, self.gaze_u, self.gaze_v, self.effective_open,
                                follow_pitch=self.follow_pitch, kin=self.kin)
        self.pitch_wanted = wanted[1]
        if self.pitch_dead:
            # pitch "olu": donanim v=0 pozunda kalir (yaw/kapak da o poza gore), bakis hesabi surer
            wanted = compute_angles(self.settings, self.gaze_u, 0.0, self.effective_open,
                                    follow_pitch=self.follow_pitch, kin=self.kin)
        self.angles = wanted
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
    """Kucuk demo: sag-sol bak, yukari-asagi bak, kirp."""
    import argparse
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Project Eye v5 hizli demo")
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
