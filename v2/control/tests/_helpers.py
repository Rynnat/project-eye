"""Testler icin ortak yardimcilar: control/ klasorunu import yoluna ekler, sahte saatler."""

import os
import sys

CONTROL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V2_DIR = os.path.dirname(CONTROL_DIR)
INO_PATH = os.path.join(V2_DIR, "firmware", "project_eye_v2", "project_eye_v2.ino")
if CONTROL_DIR not in sys.path:
    sys.path.insert(0, CONTROL_DIR)


class FakeClock:
    """Elle ilerletilen saat (saniye)."""

    def __init__(self, t: float = 100.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class AutoClock(FakeClock):
    """Her okumada kendiliginden ilerleyen saat (bekleme donguleri icin)."""

    def __init__(self, t: float = 100.0, step: float = 0.01):
        super().__init__(t)
        self.step = step

    def __call__(self) -> float:
        self.t += self.step
        return self.t


def quiet(*_a, **_k):
    """log= icin sessiz yazici."""
