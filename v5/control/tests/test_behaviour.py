"""v4 EyeController davranisi (sakkad siniri, kirpma, kapak pitch takibi, idle) + FollowBrain."""

import copy
import random
import unittest

from _helpers import FakeClock, quiet
from eye_control import BLINK_S, DEFAULT_SETTINGS, EyeController, EyeLink


def make(kinematics="auto", **beh):
    clock = FakeClock()
    s = copy.deepcopy(DEFAULT_SETTINGS)
    s["behaviour"].update(beh)
    eye = EyeController(settings=s, link=EyeLink.simulator(clock, seed=3), clock=clock,
                        rng=random.Random(7), log=quiet, kinematics=kinematics)
    return eye, clock


def run(eye, clock, seconds, dt=0.02):
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        eye.update(dt)


class GazeTests(unittest.TestCase):
    def test_saccade_speed_limit(self):
        for kin in ("auto", None):
            eye, clock = make(kin)
            eye.auto_blink = False
            eye.look(1.0, 0.0)
            eye.update(0.01)
            self.assertAlmostEqual(eye.gaze_u, 500 * 0.01 / 25, places=6)
            run(eye, clock, 0.2)
            self.assertEqual(eye.angles[0], 65.0)       # +u = robotun sagi -> servo 65 (invert)

    def test_idle_wander_and_nan(self):
        eye, clock = make()
        eye.look(float("nan"), 0.2)
        self.assertEqual((eye.target_u, eye.target_v), (0.0, 0.0))
        eye.look(0, 0)
        run(eye, clock, 3.0)
        self.assertTrue(eye.idle)
        self.assertNotAlmostEqual(eye.gaze_u, 0.0, places=3)


class LidTests(unittest.TestCase):
    def test_blink(self):
        eye, clock = make()
        eye.auto_blink = False
        eye.lids(0.9)
        eye.blink()
        clock.advance(BLINK_S * 0.4)
        eye.update(0.0)
        self.assertEqual(eye.effective_open, 0.0)
        self.assertEqual(eye.angles[2], 102.29)
        clock.advance(BLINK_S)
        eye.update(0.0)
        self.assertAlmostEqual(eye.effective_open, 0.9)
        self.assertFalse(hasattr(eye, "wink"))

    def test_auto_blink_interval(self):
        eye, clock = make()
        blinks, was = 0, False
        for _ in range(3000):
            clock.advance(0.01)
            eye.update(0.01)
            closed = eye.effective_open < 0.05
            blinks += closed and not was
            was = closed
        self.assertTrue(4 <= blinks <= 11, blinks)

    def test_default_no_software_lid_follow(self):
        # v4: kapaklar pitch'i mekanik izler -> varsayilan k=0, servo acisi pitch'ten bagimsiz
        eye, clock = make()
        self.assertEqual(eye.settings["behaviour"]["lid_follow_pitch"], 0.0)
        eye.auto_blink = False
        eye.lids(0.7)
        run(eye, clock, 0.1)
        flat = eye.angles[2]
        eye.look(0, 1)
        run(eye, clock, 0.2)
        self.assertEqual(eye.angles[2], flat)

    def test_lid_follows_pitch_when_enabled(self):
        eye, clock = make(lid_follow_pitch=0.6)
        eye.auto_blink = False
        eye.lids(0.7)
        run(eye, clock, 0.1)
        flat = eye.angles[2]
        eye.look(0, 1)
        run(eye, clock, 0.2)
        self.assertLess(eye.angles[2], flat)             # daha acik = daha kucuk servo acisi
        eye.follow_pitch = False
        run(eye, clock, 0.05)
        self.assertAlmostEqual(eye.angles[2], flat)


class FollowBrainTests(unittest.TestCase):
    def test_lock_track_sleep(self):
        import face_follow as ff
        eye, clock = make()
        eye.auto_blink = False
        brain = ff.FollowBrain(eye)

        def frames(n, target, dt=1 / 30):
            for _ in range(n):
                clock.advance(dt)
                brain.step(clock(), dt, target)
                eye.update(dt)

        frames(ff.TARGET_DEBOUNCE_FRAMES + 3, (0.5, 0.2))
        self.assertEqual(brain.status, "LOCK!")
        frames(30, (0.5, 0.2))
        self.assertEqual(brain.status, "TRACKING")
        self.assertAlmostEqual(eye.openness, ff.LID_OPEN_TRACK, places=2)
        self.assertAlmostEqual(eye.gaze_u, 0.5, places=2)
        frames(ff.TARGET_DEBOUNCE_FRAMES + 1, None)
        self.assertIsNotNone(eye._blink_start)
        frames(90, None)
        self.assertEqual(brain.status, "DORMANT")
        self.assertAlmostEqual(eye.openness, ff.LID_OPEN_IDLE, places=2)


class PitchDeadTests(unittest.TestCase):
    def test_pitch_servo_stays_centered_but_wanted_moves(self):
        live, c1 = make(pitch_dead=False)
        dead, c2 = make(pitch_dead=True)
        for eye, clock in ((live, c1), (dead, c2)):
            eye.auto_blink = False
            eye.look(0.4, 0.8)
            run(eye, clock, 0.5)
        centre, _ = make(pitch_dead=True)
        self.assertNotAlmostEqual(live.angles[1], centre.angles[1], places=1)
        self.assertAlmostEqual(dead.angles[1], centre.angles[1], places=2)   # servo hep v=0 pozunda
        self.assertAlmostEqual(dead.pitch_wanted, live.angles[1], places=2)  # ama hedef aci hesaplaniyor
        self.assertEqual(dead.link.query_state()[0][1], dead.angles[1])     # karta da merkez gitti


class ModeBrainTests(unittest.TestCase):
    def setUp(self):
        import face_follow as ff
        self.ff = ff
        self.eye, self.clock = make()
        self.brain = ff.ModeBrain(self.eye)

    def frames(self, n, target, dt=1 / 30):
        for _ in range(n):
            self.clock.advance(dt)
            self.brain.step(self.clock(), dt, target)
            self.eye.update(dt)

    def test_neutral_ignores_face(self):
        self.brain.set_mode("NOTR")
        self.frames(60, (0.8, 0.5))
        self.assertAlmostEqual(self.eye.gaze_u, 0.0, places=3)
        self.assertAlmostEqual(self.eye.gaze_v, 0.0, places=3)
        self.assertFalse(self.eye.auto_blink)
        self.assertAlmostEqual(self.eye.openness, self.ff.LID_OPEN_TRACK, places=2)

    def test_tracking_follows_then_holds(self):
        self.brain.set_mode("TRACKING")
        self.frames(60, (0.6, -0.3))
        self.assertAlmostEqual(self.eye.gaze_u, 0.6, places=2)
        self.assertEqual(self.brain.status, "TRACKING")
        self.frames(120, None)           # yuz kayboldu: son noktada bekler, kolacan etmez
        self.assertAlmostEqual(self.eye.gaze_u, 0.6, places=2)
        self.assertAlmostEqual(self.eye.openness, self.ff.LID_OPEN_TRACK, places=2)

    def test_idle_wanders_with_face_present(self):
        self.brain.set_mode("IDLE")
        self.assertTrue(self.eye.idle)
        self.frames(90, (0.9, 0.0))
        self.assertAlmostEqual(self.eye.openness, self.ff.LID_OPEN_IDLE, places=2)
        self.assertNotAlmostEqual(self.eye.gaze_u, 0.9, places=1)

    def test_switch_back_to_live(self):
        self.brain.set_mode("NOTR")
        self.brain.set_mode("CANLI")
        self.assertTrue(self.eye.auto_blink and self.eye.idle_wander)
        self.frames(self.ff.TARGET_DEBOUNCE_FRAMES + 3, (0.5, 0.2))
        self.assertEqual(self.brain.status, "LOCK!")

    def test_buttons_hit(self):
        for m, x, y in self.ff.mode_button_rects():
            self.assertEqual(self.ff.hit_mode_button(x + 5, y + 5), m)
        self.assertIsNone(self.ff.hit_mode_button(2, 2))


if __name__ == "__main__":
    unittest.main()
