"""EyeController davranisi (sakkad hiz siniri, kirpma, wink, pitch takibi, idle) + FollowBrain."""

import copy
import random
import unittest

from _helpers import FakeClock, quiet
from eye_control import BLINK_S, DEFAULT_SETTINGS, EyeController, EyeLink


def make(clock=None, **beh):
    clock = clock or FakeClock()
    s = copy.deepcopy(DEFAULT_SETTINGS)
    s["behaviour"].update(beh)
    eye = EyeController(settings=s, link=EyeLink.simulator(clock, seed=3), clock=clock,
                        rng=random.Random(7), log=quiet)
    return eye, clock


def run(eye, clock, seconds, dt=0.02):
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        eye.update(dt)


class GazeTests(unittest.TestCase):
    def test_saccade_speed_limit(self):
        eye, clock = make()
        eye.auto_blink = False
        eye.look(1.0, 0.0)
        clock.advance(0.01)
        eye.update(0.01)
        self.assertAlmostEqual(eye.gaze_u, 500 * 0.01 / 30, places=6)   # 5 derece / 30 derece-birim
        run(eye, clock, 0.1)
        self.assertEqual(eye.gaze_u, 1.0)
        self.assertEqual(eye.angles[0], 120.0)

    def test_diagonal_limit_in_degrees(self):
        eye, clock = make()
        eye.look(1.0, 1.0)
        eye.update(0.01)
        moved = ((eye.gaze_u * 30) ** 2 + (eye.gaze_v * 25) ** 2) ** 0.5
        self.assertAlmostEqual(moved, 5.0, places=6)

    def test_look_rejects_nan_and_clamps(self):
        eye, _ = make()
        eye.look(float("nan"), 0.3)
        self.assertEqual((eye.target_u, eye.target_v), (0.0, 0.0))
        eye.look(7, -7)
        self.assertEqual((eye.target_u, eye.target_v), (1.0, -1.0))

    def test_idle_wander(self):
        eye, clock = make(idle_after_s=1.5)
        eye.look(0.0, 0.0)
        run(eye, clock, 1.0)
        self.assertFalse(eye.idle)
        self.assertAlmostEqual(eye.gaze_u, 0.0)
        run(eye, clock, 3.0)
        self.assertTrue(eye.idle)
        self.assertNotAlmostEqual(eye.gaze_u, 0.0, places=3)
        eye.idle_wander = False
        eye.look(0.2, 0.0)
        run(eye, clock, 0.5)
        self.assertAlmostEqual(eye.gaze_u, 0.2)


class LidTests(unittest.TestCase):
    def test_blink_closes_then_reopens(self):
        eye, clock = make()
        eye.auto_blink = False
        eye.lids(0.9)
        eye.update(0.0)
        eye.blink()
        clock.advance(BLINK_S * 0.4)
        eye.update(0.0)
        self.assertEqual(eye.effective_open, (0.0, 0.0))
        ch = eye.settings["channels"]
        self.assertAlmostEqual(eye.angles[2], ch["LID_UL"]["closed"])   # kapali
        self.assertAlmostEqual(eye.angles[3], ch["LID_LL"]["closed"])
        clock.advance(BLINK_S)
        eye.update(0.0)
        self.assertEqual(eye.effective_open, (0.9, 0.9))

    def test_wink_one_side(self):
        eye, clock = make()
        eye.auto_blink = False
        eye.lids(1.0)
        eye.wink("left")
        clock.advance(0.45 * 0.4)
        eye.update(0.0)
        self.assertEqual(eye.effective_open, (0.0, 1.0))
        ch = eye.settings["channels"]
        for got, want in zip(eye.angles[2:6], (ch["LID_UL"]["closed"], ch["LID_LL"]["closed"],   # sol kapali
                                               ch["LID_UR"]["open"], ch["LID_LR"]["open"])):      # sag acik
            self.assertAlmostEqual(got, want)
        with self.assertRaises(ValueError):
            eye.wink("middle")

    def test_per_side_openness(self):
        eye, _ = make()
        eye.auto_blink = False
        eye.lids(0.5, right=1.0)
        eye.update(0.0)
        self.assertEqual(eye.effective_open, (0.5, 1.0))

    def test_auto_blink_interval(self):
        eye, clock = make(blink_interval_s=[3, 6])
        blinks, was_closed = 0, False
        for _ in range(int(30 / 0.01)):
            clock.advance(0.01)
            eye.update(0.01)
            closed = eye.effective_open[0] < 0.05
            if closed and not was_closed:
                blinks += 1
            was_closed = closed
        self.assertTrue(4 <= blinks <= 11, blinks)
        eye2, clock2 = make()
        eye2.auto_blink = False
        run(eye2, clock2, 10.0)
        self.assertEqual(eye2.effective_open, (0.85, 0.85))

    def test_lids_follow_pitch(self):
        eye, clock = make()
        eye.auto_blink = False
        eye.lids(0.8)
        eye.look(0.0, 0.0)
        run(eye, clock, 0.2)
        ul0, ll0 = eye.angles[2], eye.angles[3]
        eye.look(0.0, 1.0)
        run(eye, clock, 0.2)
        # UL: open=120 > closed -> daha acik = daha buyuk aci; LL: open=70 -> yukari = kapanma = buyuk aci
        self.assertGreater(eye.angles[2], ul0)
        self.assertGreater(eye.angles[3], ll0)
        eye.follow_pitch = False
        run(eye, clock, 0.05)
        self.assertAlmostEqual(eye.angles[2], ul0)


class FollowBrainTests(unittest.TestCase):
    def setUp(self):
        import face_follow
        self.ff = face_follow
        self.eye, self.clock = make()
        self.eye.auto_blink = False
        self.brain = face_follow.FollowBrain(self.eye)

    def frames(self, n, target, dt=1 / 30):
        for _ in range(n):
            self.clock.advance(dt)
            self.brain.step(self.clock(), dt, target)
            self.eye.update(dt)

    def test_lock_startle_track_lose_sleep(self):
        ff = self.ff
        self.frames(ff.TARGET_DEBOUNCE_FRAMES + 3, (0.5, 0.2))
        self.assertEqual(self.brain.status, "LOCK!")
        self.assertGreater(self.eye.open_left, 0.9)
        self.frames(30, (0.5, 0.2))                       # 1 sn sabit yuz
        self.assertEqual(self.brain.status, "TRACKING")
        self.assertAlmostEqual(self.eye.open_left, ff.LID_OPEN_TRACK, places=2)
        self.assertAlmostEqual(self.eye.gaze_u, 0.5, places=2)
        self.frames(ff.TARGET_DEBOUNCE_FRAMES + 1, None)   # yuz kayboldu -> kirpma
        self.assertIsNotNone(self.eye._blink_start)
        self.frames(90, None)                              # 3 sn: uykulu
        self.assertEqual(self.brain.status, "DORMANT")
        self.assertAlmostEqual(self.eye.open_left, ff.LID_OPEN_IDLE, places=2)

    def test_fast_motion_squints(self):
        ff = self.ff
        self.frames(40, (0.0, 0.0))
        u = 0.0
        for _ in range(20):                                # ~1.5 birim/sn hareket
            u += 0.05
            self.frames(1, (min(u, 1.0) - 0.5, 0.0))
        self.assertGreater(self.brain.focus, 0.2)
        self.assertLess(self.eye.open_left, ff.LID_OPEN_TRACK - 0.03)


if __name__ == "__main__":
    unittest.main()
