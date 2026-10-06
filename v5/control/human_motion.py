"""
Insan goz hareketi istatistikleri + tekrar etmeyen gurultu (Project Eye v5).

Kaynaklar (parametrelerin geldigi yer; dagilim BICIMLERI bizim modelleme secimimiz):

* Kirpma sikligi - Bentivoglio ve ark. 1997, "Analysis of blink rate patterns in
  normal subjects", Movement Disorders 12(6): dinlenirken ~17/dk, konusurken ~26/dk.
  Kirpmalar arasi sure saga carpik oldugu icin log-normal ile modellenir; ortalama
  60/oran olacak sekilde.
* Fiksasyon (bir noktaya bakma) suresi - Henderson 2003, "Human gaze control during
  real-world scene perception", Trends in Cognitive Sciences 7(11): sahne izlerken
  ortalama ~330 ms. Yine log-normal.
* Kirpma suresi - insan tam kirpmasi ~100-400 ms; servo kapaklarin yetisebilmesi icin
  0.20-0.32 sn araliginda.

Bakinma iki katmanli (insanlarda da boyle: buyuk sakkadla bir bolgeye gec, orada
kucuk sakkadlarla dolas):
* bolge: DONANIMIN mekanik araliginin tamamindan rastgele (v5 artifact'teki "Canli" ile
  ayni: yaw +-23/25, pitch +-17/20); 0.7-2.3 sn'de bir degisir (artifact ile ayni)
* bolge icinde: her fiksasyonda (Henderson, ort. ~330 ms) gradyan (Perlin) gurultusunden
  kucuk sapma. Kafes gradyanlari ihtiyac duyuldukca uretilir (tablo yok) -> periyot yok.
-1..1 birimi donanima baglidir: +-1 = eklemin mekanik siniri (kinematik tablo/kalibrasyon).
"""

from __future__ import annotations

import math
import random

BLINK_RATE_PER_MIN = {"rest": 17.0, "conversation": 26.0}   # Bentivoglio 1997
BLINK_IBI_SIGMA = 0.6            # log-normal sekil (saga carpik; modelleme secimi)
BLINK_IBI_RANGE_S = (0.4, 15.0)
BLINK_DURATION_S = (0.20, 0.32)

FIXATION_MEAN_S = 0.33           # Henderson 2003
FIXATION_SIGMA = 0.4
FIXATION_RANGE_S = (0.12, 1.5)

WANDER_FREQ_HZ = 0.12            # gurultu kafesinin zaman olcegi (yavas, organik kayma)
WANDER_U, WANDER_V = 23 / 25, 17 / 20   # bolge genligi: donanim araliginin payi (v5 artifact)
REGION_DWELL_S = (0.7, 2.3)      # bolge degisim araligi (v5 artifact)
MICRO_U, MICRO_V = 0.08, 0.06    # bolge icindeki kucuk sakkadlarin genligi


def _lognormal_with_mean(rng: random.Random, mean: float, sigma: float) -> float:
    """Ortalamasi `mean` olan log-normal ornek (mu = ln(mean) - sigma^2/2)."""
    return rng.lognormvariate(math.log(mean) - sigma * sigma / 2.0, sigma)


def blink_interval(rng: random.Random, context: str = "rest") -> float:
    rate = BLINK_RATE_PER_MIN.get(context, BLINK_RATE_PER_MIN["rest"])
    lo, hi = BLINK_IBI_RANGE_S
    return min(hi, max(lo, _lognormal_with_mean(rng, 60.0 / rate, BLINK_IBI_SIGMA)))


def blink_duration(rng: random.Random) -> float:
    return rng.uniform(*BLINK_DURATION_S)


def fixation_duration(rng: random.Random) -> float:
    lo, hi = FIXATION_RANGE_S
    return min(hi, max(lo, _lognormal_with_mean(rng, FIXATION_MEAN_S, FIXATION_SIGMA)))


class Noise1D:
    """Periyotsuz 1B gradyan (Perlin) gurultusu, cikti yaklasik -1..1."""

    def __init__(self, rng: random.Random):
        self._rng = rng
        self._grad: dict[int, float] = {}

    def _g(self, i: int) -> float:
        g = self._grad.get(i)
        if g is None:
            g = self._grad[i] = self._rng.uniform(-1.0, 1.0)
        return g

    def __call__(self, x: float) -> float:
        i = math.floor(x)
        f = x - i
        fade = f * f * f * (f * (f * 6 - 15) + 10)
        a, b = self._g(i) * f, self._g(i + 1) * (f - 1)
        return 2.0 * (a + (b - a) * fade)     # 1B Perlin ~ +-0.5 -> +-1


class GazeWander:
    """Insan benzeri bakinma: bolge (tam donanim araligi) + bolge icinde fiksasyon -> kucuk sakkad."""

    def __init__(self, rng: random.Random):
        self.rng = rng
        self._nu, self._nv = Noise1D(rng), Noise1D(rng)
        self._phase = rng.uniform(0, 1000)
        self._next_fix = -1.0
        self._next_region = -1.0
        self.region = (0.0, 0.0)
        self.target = (0.0, 0.0)

    def step(self, now: float):
        if now >= self._next_region:
            self.region = (self.rng.uniform(-WANDER_U, WANDER_U), self.rng.uniform(-WANDER_V, WANDER_V))
            self._next_region = now + self.rng.uniform(*REGION_DWELL_S)
            self._next_fix = now
        if now >= self._next_fix:
            x = self._phase + now * WANDER_FREQ_HZ * 8
            u = self.region[0] + MICRO_U * self._nu(x)
            v = self.region[1] + MICRO_V * self._nv(x + 57.3)
            self.target = (max(-1.0, min(1.0, u)), max(-1.0, min(1.0, v)))
            self._next_fix = now + fixation_duration(self.rng)
        return self.target
