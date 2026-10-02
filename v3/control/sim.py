"""
Project Eye v3 (POC) - firmware simulatoru (donanim yokken sahte seri port).

Iki parca:

* ``FirmwareModel`` - ``firmware/project_eye_v3/project_eye_v3.ino``'nun
  satir satir Python esdegeri: ayni sabitler, ayni satir tamponu/ayristirma,
  ayni hiz sinirli hareket (5 ms tik), ayni 2 sn idle kurali ve idle
  davranisi (sakkad, kirpma, kapak nefesi, pitch takibi).
* ``SimSerial`` - pyserial ``Serial``'in bu projede kullanilan alt kumesini
  taklit eden sahte port. ``eye_control`` bunu gercek portla ayni sekilde
  kullanir; testler ve "donanim yok" modu bununla calisir.

Sabitler .ino ile SENKRON tutulmalidir; ``tests/test_firmware_sync.py``
.ino dosyasini okuyup karsilastirir.

Yalnizca standart kutuphane kullanir.
"""

from __future__ import annotations

import math
import random
import time
from typing import Callable

# --------------------------------------------------------------------------
# Firmware sabitleri (project_eye_v3.ino ile AYNI)
# --------------------------------------------------------------------------
FW_BANNER = "EYE v3 READY"
BAUD = 115200
NUM_CH = 3
PINS = (3, 5, 6)
LIMIT_MIN = (65.0, 70.0, 67.5)     # kinematics.json limits_checked, pay yok
LIMIT_MAX = (115.0, 110.0, 112.5)
MAX_SPEED_DEG_S = 600.0
SERVO_US_MIN = 544
SERVO_US_MAX = 2400
TICK_MS = 5
IDLE_TIMEOUT_MS = 2000
LINE_MAX = 64

IDLE_EYE_CENTER = (90.0, 90.0)
IDLE_EYE_DEG_PER_UNIT = (25.0, 20.0)
IDLE_LID_CLOSED = 112.5
IDLE_LID_OPEN = 67.5
IDLE_LID_FOLLOW_PITCH = 0.6
IDLE_GAZE_U_MAX = 0.6
IDLE_GAZE_V_MAX = 0.45
IDLE_BREATH_BASE = 0.72
IDLE_BREATH_AMP = 0.12
IDLE_BREATH_PERIOD_S = 5.0
IDLE_SACCADE_MIN_MS = 400
IDLE_SACCADE_MAX_MS = 2500
IDLE_BLINK_MIN_MS = 3000
IDLE_BLINK_MAX_MS = 6000
IDLE_BLINK_CLOSED_MS = 120
BOOT_LID_OPENNESS = 0.8


class ProtocolError(ValueError):
    """Firmware'in ``ERR <mesaj>`` ile reddedecegi bir satir."""


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def clamp_to_limit(ch: int, deg: float) -> float:
    return _clamp(deg, LIMIT_MIN[ch], LIMIT_MAX[ch])


def _parse_number(tok: str) -> float:
    """strtod + 'tamami tuketildi mi' kontrolunun esdegeri."""
    if "_" in tok:  # Python float() '1_0' kabul eder, strtod etmez
        raise ProtocolError("bad number")
    try:
        v = float(tok)
    except ValueError:
        raise ProtocolError("bad number") from None
    if math.isnan(v) or math.isinf(v):
        raise ProtocolError("bad number")
    return v


def parse_command(line: str):
    """Tek bir (satir sonu atilmis) komut satirini ayristirir.

    Donen: ``None`` (bos satir, sessizce yok sayilir) ya da ``(cmd, args)``:
      ('S', (a0, a1, a2))  ('D', ())  ('A', (True|False,))  ('?', ())
    Gecersizse ``ProtocolError(<firmware'in ERR mesaji>)`` firlatir.
    Firmware'deki handleLine() ile birebir ayni kurallar.
    """
    toks = line.replace("\t", " ").split()
    if not toks:
        return None
    cmd = toks[0]
    if len(cmd) != 1:
        raise ProtocolError("unknown command")
    c = cmd.upper()
    args = toks[1:]
    if c == "S":
        vals = []
        for i in range(NUM_CH):
            if i >= len(args):
                raise ProtocolError("S needs 3 angles")
            v = _parse_number(args[i])
            if v < 0.0 or v > 180.0:
                raise ProtocolError("angle out of 0-180")
            vals.append(v)
        if len(args) > NUM_CH:
            raise ProtocolError("S needs 3 angles")
        return "S", tuple(vals)
    if c == "D":
        if args:
            raise ProtocolError("D takes no args")
        return "D", ()
    if c == "A":
        if len(args) != 1 or args[0] not in ("0", "1"):
            raise ProtocolError("A needs 1 or 0")
        return "A", (args[0] == "1",)
    if c == "?":
        if args:
            raise ProtocolError("? takes no args")
        return "?", ()
    raise ProtocolError("unknown command")


def _lid_angle(openness: float) -> float:
    openness = _clamp(openness, 0.0, 1.0)
    return IDLE_LID_CLOSED + openness * (IDLE_LID_OPEN - IDLE_LID_CLOSED)


def _fmt1(v: float) -> str:
    """Arduino Serial.print(float, 1) ile ayni: yarim yukari yuvarlama."""
    return f"{math.floor(v * 10.0 + 0.5) / 10.0:.1f}"


class FirmwareModel:
    """project_eye_v3.ino'nun Python esdegeri (zaman milisaniye, disaridan verilir)."""

    def __init__(self, now_ms: int = 0, seed: int | None = None):
        self.rng = random.Random(seed)
        self.pos = [clamp_to_limit(0, IDLE_EYE_CENTER[0]),
                    clamp_to_limit(1, IDLE_EYE_CENTER[1]),
                    clamp_to_limit(2, _lid_angle(BOOT_LID_OPENNESS))]
        self.target = list(self.pos)
        self.attached = True
        self.idle_enabled = True
        self.idle_active = False
        self.last_s_ms = now_ms
        self.last_tick_ms = now_ms
        self.now_ms = now_ms
        # idle alt durumu
        self.idle_u = 0.0
        self.idle_v = 0.0
        self.idle_next_saccade_ms = 0
        self.idle_next_blink_ms = 0
        self.idle_blink_start_ms = 0
        self.idle_blinking = False
        self.blink_count = 0      # test/telemetri icin: idle'da kac kirpma basladi
        # satir tamponu
        self._buf: list[str] = []
        self._overflow = False
        self._bad_char = False
        # gozlem icin: gonderilen tum darbe genislikleri (kanal -> son us)
        self.last_us = [self.deg_to_us(p) for p in self.pos]

    # -- yardimcilar --------------------------------------------------------
    @staticmethod
    def deg_to_us(deg: float) -> int:
        return int(SERVO_US_MIN + (deg / 180.0) * (SERVO_US_MAX - SERVO_US_MIN) + 0.5)

    def _random(self, lo: int, hi: int) -> int:
        """Arduino random(lo, hi): [lo, hi) araliginda tamsayi."""
        return self.rng.randrange(lo, hi)

    # -- zaman --------------------------------------------------------------
    def advance(self, now_ms: int) -> None:
        """loop()'un idle-giris kontrolu ve 5 ms tiklerini now_ms'e kadar calistirir."""
        now_ms = int(now_ms)
        t = self.last_tick_ms + TICK_MS
        while t <= now_ms:
            self._check_idle_entry(t)
            if self.idle_active:
                self._idle_update(t)
            self._step_motion(TICK_MS / 1000.0)
            self.last_tick_ms = t
            t += TICK_MS
        self._check_idle_entry(now_ms)
        self.now_ms = max(self.now_ms, now_ms)

    def _check_idle_entry(self, now_ms: int) -> None:
        if (self.idle_enabled and self.attached and not self.idle_active
                and now_ms - self.last_s_ms >= IDLE_TIMEOUT_MS):
            self._idle_enter(now_ms)

    def _step_motion(self, dt: float) -> None:
        max_step = MAX_SPEED_DEG_S * dt
        for i in range(NUM_CH):
            d = _clamp(self.target[i] - self.pos[i], -max_step, max_step)
            self.pos[i] += d
            if self.attached:
                self.last_us[i] = self.deg_to_us(self.pos[i])

    # -- idle ---------------------------------------------------------------
    def _idle_enter(self, now_ms: int) -> None:
        self.idle_active = True
        self.idle_next_saccade_ms = now_ms
        self.idle_next_blink_ms = now_ms + self._random(IDLE_BLINK_MIN_MS, IDLE_BLINK_MAX_MS + 1)
        self.idle_blinking = False

    def _idle_update(self, now_ms: int) -> None:
        if now_ms - self.idle_next_saccade_ms >= 0:
            if self._random(0, 100) < 30:
                self.idle_u = self._random(-15, 16) / 100.0
                self.idle_v = self._random(-10, 11) / 100.0
            else:
                um, vm = int(IDLE_GAZE_U_MAX * 100), int(IDLE_GAZE_V_MAX * 100)
                self.idle_u = self._random(-um, um + 1) / 100.0
                self.idle_v = self._random(-vm, vm + 1) / 100.0
            self.idle_next_saccade_ms = now_ms + self._random(IDLE_SACCADE_MIN_MS, IDLE_SACCADE_MAX_MS + 1)

        if not self.idle_blinking and now_ms - self.idle_next_blink_ms >= 0:
            self.idle_blinking = True
            self.idle_blink_start_ms = now_ms
            self.blink_count += 1
        if self.idle_blinking and now_ms - self.idle_blink_start_ms >= IDLE_BLINK_CLOSED_MS:
            self.idle_blinking = False
            self.idle_next_blink_ms = now_ms + self._random(IDLE_BLINK_MIN_MS, IDLE_BLINK_MAX_MS + 1)

        period_ms = int(IDLE_BREATH_PERIOD_S * 1000)
        phase = (now_ms % period_ms) / period_ms
        openness = IDLE_BREATH_BASE + IDLE_BREATH_AMP * math.sin(phase * 2.0 * math.pi)
        if self.idle_blinking:
            openness = 0.0
        upper = openness + IDLE_LID_FOLLOW_PITCH * 0.5 * self.idle_v * openness

        self.target[0] = clamp_to_limit(0, IDLE_EYE_CENTER[0] + self.idle_u * IDLE_EYE_DEG_PER_UNIT[0])
        self.target[1] = clamp_to_limit(1, IDLE_EYE_CENTER[1] + self.idle_v * IDLE_EYE_DEG_PER_UNIT[1])
        self.target[2] = clamp_to_limit(2, _lid_angle(upper))

    def _idle_stop(self) -> None:
        self.idle_active = False
        self.idle_blinking = False
        self.target = list(self.pos)

    # -- komutlar -----------------------------------------------------------
    def state_line(self) -> str:
        return "STATE " + " ".join(_fmt1(p) for p in self.pos) + f" idle={1 if self.idle_active else 0}"

    def handle_line(self, line: str, now_ms: int) -> str | None:
        try:
            parsed = parse_command(line)
        except ProtocolError as e:
            return f"ERR {e}"
        if parsed is None:
            return None
        cmd, args = parsed
        if cmd == "S":
            self.target = [clamp_to_limit(i, a) for i, a in enumerate(args)]
            self.idle_active = False
            self.idle_blinking = False
            self.last_s_ms = now_ms
            self.attached = True
            return "OK"
        if cmd == "D":
            self.idle_active = False
            self.attached = False
            self.target = list(self.pos)
            return "OK"
        if cmd == "A":
            self.idle_enabled = args[0]
            if not self.idle_enabled and self.idle_active:
                self._idle_stop()
            return "OK"
        return self.state_line()  # '?'

    def feed(self, data: bytes, now_ms: int) -> list[str]:
        """Gelen baytlari pollSerial() gibi isler; uretilen cevap satirlarini dondurur."""
        self.advance(now_ms)
        out: list[str] = []
        for b in data:
            ch = b
            if ch == 13:          # '\r'
                continue
            if ch == 10:          # '\n'
                if self._overflow:
                    out.append("ERR line too long")
                elif self._bad_char:
                    out.append("ERR bad character")
                else:
                    resp = self.handle_line("".join(self._buf), now_ms)
                    if resp is not None:
                        out.append(resp)
                self._buf.clear()
                self._overflow = False
                self._bad_char = False
                continue
            if self._overflow:
                continue
            if (ch < 32 or ch > 126) and ch != 9:
                self._bad_char = True
            if len(self._buf) >= LINE_MAX:
                self._overflow = True
                continue
            self._buf.append(chr(ch))
        return out


class SimSerial:
    """pyserial ``Serial`` alt kumesi: write / readline / read / in_waiting / close.

    Her okuma-yazmada ``clock()`` ile firmware modeli zamanda ilerletilir.
    ``clock`` saniye donduren bir fonksiyon (varsayilan time.monotonic);
    testler sahte saat vererek deterministik kosar.
    """

    def __init__(self, port: str = "SIM", baudrate: int = BAUD, timeout: float | None = 0.0,
                 clock: Callable[[], float] = time.monotonic, seed: int | None = None):
        self.port = port
        self.name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._clock = clock
        self.model = FirmwareModel(now_ms=self._now_ms(), seed=seed)
        self._rx = bytearray((FW_BANNER + "\r\n").encode("ascii"))  # Serial.println -> \r\n
        self.is_open = True
        self.written: list[str] = []   # gozlem: gonderilen satirlar

    def _now_ms(self) -> int:
        return int(self._clock() * 1000.0)

    def _sync(self) -> None:
        self.model.advance(self._now_ms())

    # -- pyserial API -------------------------------------------------------
    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("SimSerial kapali")
        text = bytes(data).decode("ascii", "replace")
        self.written.extend(l for l in text.split("\n") if l)
        for resp in self.model.feed(bytes(data), self._now_ms()):
            self._rx += (resp + "\r\n").encode("ascii")
        return len(data)

    @property
    def in_waiting(self) -> int:
        self._sync()
        return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        self._sync()
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def readline(self) -> bytes:
        self._sync()
        idx = self._rx.find(b"\n")
        if idx < 0:
            return b""  # gercek portta timeout'a denk
        line = bytes(self._rx[:idx + 1])
        del self._rx[:idx + 1]
        return line

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


if __name__ == "__main__":
    # Elle deneme: satir yaz, firmware simulatorunun cevabini gor.
    import sys
    s = SimSerial()
    print(s.readline().decode().strip())
    print("Komut yaz (S/D/A/?), cikis icin Ctrl+Z/Ctrl+D.")
    for raw in sys.stdin:
        s.write(raw.encode("ascii", "replace") if raw.endswith("\n") else (raw + "\n").encode())
        while True:
            line = s.readline()
            if not line:
                break
            print(line.decode().strip())
