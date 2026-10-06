"""human_motion: insan istatistiklerine uyum + bakinmanin tekrar etmemesi."""

import copy
import random
import statistics
import unittest

import human_motion as hm
from _helpers import FakeClock, quiet
from eye_control import DEFAULT_SETTINGS, EyeController, EyeLink


class StatsTests(unittest.TestCase):
    def test_blink_rates_match_bentivoglio(self):
        rng = random.Random(1)
        for ctx, rate in hm.BLINK_RATE_PER_MIN.items():
            mean = statistics.fmean(hm.blink_interval(rng, ctx) for _ in range(20000))
            self.assertAlmostEqual(60.0 / mean, rate, delta=rate * 0.05, msg=ctx)

    def test_fixation_mean_matches_henderson(self):
        rng = random.Random(2)
        mean = statistics.fmean(hm.fixation_duration(rng) for _ in range(20000))
        self.assertAlmostEqual(mean, hm.FIXATION_MEAN_S, delta=0.02)

    def test_intervals_are_not_constant(self):
        rng = random.Random(3)
        xs = [hm.blink_interval(rng) for _ in range(500)]
        self.assertGreater(statistics.pstdev(xs), 1.0)


class WanderTests(unittest.TestCase):
    def test_noise_is_continuous_and_bounded(self):
        n = hm.Noise1D(random.Random(4))
        prev = n(0.0)
        for i in range(1, 5000):
            x = i * 0.01
            y = n(x)
            self.assertLessEqual(abs(y), 1.0)
            self.assertLess(abs(y - prev), 0.1)
            prev = y

    def test_wander_never_repeats_a_window(self):
        """10 dakikalik bakinmada, 20 sn'lik ilk pencere hicbir kaymada yeniden olusmaz."""
        w = hm.GazeWander(random.Random(5))
        dt = 0.05
        seq = [w.step(i * dt) for i in range(int(600 / dt))]
        win = int(20 / dt)
        first = seq[:win]
        for shift in range(int(2 / dt), len(seq) - win, 7):
            same = sum(1 for a, b in zip(first, seq[shift:shift + win])
                       if abs(a[0] - b[0]) < 0.02 and abs(a[1] - b[1]) < 0.02)
            self.assertLess(same / win, 0.5, msg=f"kayma {shift * dt:.1f} sn")

    def test_wander_covers_hardware_range(self):
        w = hm.GazeWander(random.Random(7))
        seq = [w.step(i * 0.05) for i in range(int(300 / 0.05))]
        us = [p[0] for p in seq]
        vs = [p[1] for p in seq]
        self.assertGreater(max(us), 0.8)
        self.assertLess(min(us), -0.8)
        self.assertGreater(max(vs), 0.7)
        self.assertLess(min(vs), -0.7)
        self.assertTrue(all(abs(u) <= 1 and abs(v) <= 1 for u, v in seq))

    def test_wander_uses_fixations(self):
        w = hm.GazeWander(random.Random(6))
        dt = 0.01
        seq = [w.step(i * dt) for i in range(int(120 / dt))]
        jumps = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
        per_s = jumps / 120
        self.assertAlmostEqual(per_s, 1 / hm.FIXATION_MEAN_S, delta=0.6)


class ControllerTests(unittest.TestCase):
    def test_controller_blinks_at_human_rate(self):
        clock = FakeClock()
        s = copy.deepcopy(DEFAULT_SETTINGS)
        s["behaviour"]["human_motion"] = True
        eye = EyeController(settings=s, link=EyeLink.simulator(clock, seed=3), clock=clock,
                            rng=random.Random(9), log=quiet, kinematics="auto")
        eye.look(0.0, 0.0)
        blinks, was = 0, False
        dt = 0.02
        for _ in range(int(600 / dt)):
            clock.advance(dt)
            eye.look(0.0, 0.0)
            eye.update(dt)
            now = eye._blink_start is not None
            blinks += now and not was
            was = now
        self.assertAlmostEqual(blinks / 10, hm.BLINK_RATE_PER_MIN["rest"], delta=3)


if __name__ == "__main__":
    unittest.main()
