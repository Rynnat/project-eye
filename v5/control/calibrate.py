"""
Project Eye v5 - kanal kanal etkilesimli kalibrasyon (settings.json'a yazar).

Montajdan sonra her servonun gercek acilarini bulmak icin. Secili kanal
klavyeyle oynatilir, digerleri kalibre notr konumda (gozler center, kapaklar
open) bekler. Firmware'in 2 sn idle'ina dusmemek icin S komutu surekli
tazelenir.

    python calibrate.py              # donanimi bul (yoksa sim)
    python calibrate.py --port COM5
    python calibrate.py --sim        # donanimsiz prova

Tuslar:
  1..3 / Tab      kanal sec (0 EYE_YAW, 1 EYE_PITCH, 2 LIDS)
  <- / ->         -1 / +1 derece       ,  /  .   -0.2 / +0.2 derece
  asagi / yukari  -5 / +5 derece
  Goz kanali:     c = center kaydet   n = min kaydet   x = max kaydet
  Kapak kanali:   k = closed kaydet   o = open kaydet  (LIDS min/max bunlari kapsayacak
                  sekilde kendiliginden guncellenir)
  i               invert ac/kapa
  g               kayitli degerler arasinda gez (center/min/max ya da closed/open)
  t               test: goz min->max->center, kapak open->closed->open
  z               tum servolari birak (detach); herhangi bir hareket tekrar baglar
  s               kaydet (settings.json)        h  yardim
  q               cik (kaydedilmemis degisiklik varsa sorar)

Guvenlik: firmware'in sabit mekanik limitleri (sim.LIMIT_MIN/MAX = .ino,
cad/out/kinematics.json'dan) disina hic aci gonderilmez; jog limite dayaninca
ekranda [FW LIMIT] yazar.
Ilk denemede servo kollarini takmadan (ya da itme cubuklarini sokmeden)
yon ve orta noktayi gormek en guvenlisidir.
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import sim  # noqa: E402
from eye_control import (CHANNELS, EYE_CHANNELS, SETTINGS_PATH, EyeLink, SettingsError,  # noqa: E402
                         clamp, eye_angle, format_s_command, lid_angle, load_settings,
                         save_settings, validate_settings)

KEEPALIVE_S = 0.5
EYE_KEYS = {"c": "center", "n": "min", "x": "max"}
LID_KEYS = {"k": "closed", "o": "open"}


class Calibrator:
    """Klavye olaylarini isleyen, konsoldan bagimsiz kalibrasyon durumu (test edilebilir)."""

    def __init__(self, settings: dict, link: EyeLink, settings_path: str = SETTINGS_PATH):
        self.settings = copy.deepcopy(settings)
        self.link = link
        self.path = settings_path
        self.sel = 0
        self.dirty = False
        self.angles = self.neutral_angles()
        self._goto_idx = 0
        self._last_send = 0.0

    # -- yardimcilar --------------------------------------------------------
    @property
    def name(self) -> str:
        return CHANNELS[self.sel]

    @property
    def cfg(self) -> dict:
        return self.settings["channels"][self.name]

    def is_eye(self) -> bool:
        return self.name in EYE_CHANNELS

    def neutral_angle(self, ch: int) -> float:
        name = CHANNELS[ch]
        cfg = self.settings["channels"][name]
        a = eye_angle(cfg, 0.0) if name in EYE_CHANNELS else lid_angle(cfg, 1.0)
        return clamp(a, sim.LIMIT_MIN[ch], sim.LIMIT_MAX[ch])

    def neutral_angles(self) -> list[float]:
        return [self.neutral_angle(i) for i in range(len(CHANNELS))]

    def to_stored(self, raw: float) -> float:
        """Ham aciyi settings degerine cevirir (kapakta invert = 180 - aci aynasi)."""
        if not self.is_eye() and self.cfg.get("invert"):
            return round(180.0 - raw, 2)
        return round(raw, 2)

    def from_stored(self, val: float) -> float:
        if not self.is_eye() and self.cfg.get("invert"):
            return 180.0 - val
        return float(val)

    def send(self) -> None:
        self.link.send(format_s_command(self.angles))
        self._last_send = time.monotonic()

    def keepalive(self) -> None:
        if time.monotonic() - self._last_send >= KEEPALIVE_S:
            self.send()
        self.link.poll()

    def status_line(self) -> str:
        a = self.angles[self.sel]
        at_limit = a <= sim.LIMIT_MIN[self.sel] or a >= sim.LIMIT_MAX[self.sel]
        fw = " [FW LIMIT]" if at_limit else ""
        stored = ", ".join(f"{k}={v}" for k, v in self.cfg.items())
        return f"[{self.sel}] {self.name:<9} aci={a:6.1f}{fw}   {{{stored}}}{' *' if self.dirty else ''}"

    # -- olaylar ------------------------------------------------------------
    def select(self, ch: int) -> str:
        # onceki kanali notr konumuna al (yeni kayitli degerle)
        self.angles[self.sel] = self.neutral_angle(self.sel)
        self.sel = ch % len(CHANNELS)
        self._goto_idx = 0
        self.send()
        return f"secildi: {self.name}"

    def jog(self, delta: float) -> str:
        a = round(self.angles[self.sel] + delta, 2)
        self.angles[self.sel] = clamp(a, sim.LIMIT_MIN[self.sel], sim.LIMIT_MAX[self.sel])
        self.send()
        return ""

    def record(self, field: str) -> str:
        self.cfg[field] = self.to_stored(self.angles[self.sel])
        self.dirty = True
        msg = f"{self.name}.{field} = {self.cfg[field]}"
        if not self.is_eye() and "min" in self.cfg and "max" in self.cfg:
            # kapak min/max = closed..open (ham aci; invert aynasi dahil)
            raw = [self.from_stored(self.cfg["closed"]), self.from_stored(self.cfg["open"])]
            self.cfg["min"], self.cfg["max"] = round(min(raw), 2), round(max(raw), 2)
            msg += f"  (min/max -> {self.cfg['min']}/{self.cfg['max']})"
        return msg

    def toggle_invert(self) -> str:
        self.cfg["invert"] = not self.cfg["invert"]
        self.dirty = True
        return f"{self.name}.invert = {self.cfg['invert']}"

    def goto_next(self) -> str:
        fields = ("center", "min", "max") if self.is_eye() else ("closed", "open")
        f = fields[self._goto_idx % len(fields)]
        self._goto_idx += 1
        self.angles[self.sel] = clamp(self.from_stored(self.cfg[f]),
                                      sim.LIMIT_MIN[self.sel], sim.LIMIT_MAX[self.sel])
        self.send()
        return f"-> {f}"

    def test_sequence(self, sleep=time.sleep, hz: float = 50.0) -> str:
        """Kayitli degerlerle kanali oynat (firmware hiz siniri zaten yumusatir)."""
        if self.is_eye():
            points = [self.cfg["min"], self.cfg["max"], self.cfg["center"]]
        else:
            points = [self.from_stored(self.cfg["open"]), self.from_stored(self.cfg["closed"]),
                      self.from_stored(self.cfg["open"])]
        start = self.angles[self.sel]
        for p in points:
            steps = max(1, int(0.6 * hz))  # her bacak 0.6 sn
            for i in range(1, steps + 1):
                a = start + (p - start) * i / steps
                self.angles[self.sel] = clamp(a, sim.LIMIT_MIN[self.sel], sim.LIMIT_MAX[self.sel])
                self.send()
                self.link.poll()
                sleep(1.0 / hz)
            start = p
        self.angles[self.sel] = round(self.angles[self.sel], 2)
        return "test bitti"

    def save(self) -> str:
        try:
            validate_settings(self.settings)
        except SettingsError as e:
            return f"KAYDEDILMEDI: {e}"
        save_settings(self.settings, self.path)
        self.dirty = False
        return f"kaydedildi: {self.path}"

    def handle_key(self, key: str) -> str | None:
        """Tek tus -> mesaj. 'QUIT' donerse cikis istenmis demektir."""
        if key in "123" and len(key) == 1:
            return self.select(int(key) - 1)
        if key == "\t":
            return self.select(self.sel + 1)
        if key in ("LEFT", "RIGHT", "UP", "DOWN", ",", "."):
            step = {"LEFT": -1.0, "RIGHT": 1.0, "UP": 5.0, "DOWN": -5.0, ",": -0.2, ".": 0.2}[key]
            return self.jog(step)
        k = key.lower()
        if self.is_eye() and k in EYE_KEYS:
            return self.record(EYE_KEYS[k])
        if not self.is_eye() and k in LID_KEYS:
            return self.record(LID_KEYS[k])
        if k == "i":
            return self.toggle_invert()
        if k == "g":
            return self.goto_next()
        if k == "t":
            return self.test_sequence()
        if k == "z":
            self.link.send("D")
            return "servolar birakildi (detach)"
        if k == "s":
            return self.save()
        if k == "h":
            return __doc__
        if k == "q":
            return "QUIT"
        return None


def _read_key(msvcrt) -> str | None:
    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):
        code = msvcrt.getch()
        return {b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT"}.get(code)
    try:
        return ch.decode("ascii")
    except UnicodeDecodeError:
        return None


def main(argv=None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Project Eye v5 kalibrasyon")
    ap.add_argument("--port", default=None)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--settings", default=SETTINGS_PATH)
    args = ap.parse_args(argv)
    try:
        import msvcrt
    except ImportError:
        print("calibrate.py su an yalnizca Windows konsolunda calisir (msvcrt).")
        return 1

    settings = load_settings(args.settings)
    link = EyeLink.open("sim" if args.sim else (args.port or settings["serial_port"]))
    cal = Calibrator(settings, link, args.settings)
    print(f"baglanti: {'SIM' if link.simulated else link.name}   (h = yardim, q = cik)")
    cal.send()
    print(cal.status_line())
    try:
        while True:
            key = _read_key(msvcrt)
            if key is None:
                cal.keepalive()
                time.sleep(0.02)
                continue
            msg = cal.handle_key(key)
            if msg == "QUIT":
                if cal.dirty:
                    print("\nKaydedilmemis degisiklik var. Kaydedilsin mi? (e/h) ", end="", flush=True)
                    ans = msvcrt.getwch().lower()
                    print(ans)
                    if ans in ("e", "y"):
                        print(cal.save())
                break
            if msg:
                print("\n" + msg)
            print("\r" + cal.status_line() + " " * 8, end="", flush=True)
            if link.last_error:
                print(f"\n[firmware] {link.last_error}")
                link.last_error = None
    except KeyboardInterrupt:
        pass
    finally:
        link.send("A 1")  # cikinca firmware kendi yasasin
        link.close()
        print("\ncikildi")
    return 0


if __name__ == "__main__":
    sys.exit(main())
