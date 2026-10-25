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

    def test_free_live_ignores_face_but_moves(self):
        self.brain.set_mode("SERBEST")
        self.assertTrue(self.eye.idle and self.eye.auto_blink)
        us = []
        for _ in range(10):
            self.frames(30, (0.9, 0.0))
            us.append(round(self.eye.gaze_u, 3))
        self.assertEqual(self.brain.status, "CANLI")
        self.assertGreater(len(set(us)), 1)                        # bakiniyor
        self.assertNotAlmostEqual(self.eye.gaze_u, 0.9, places=1)  # yuze kilitlenmiyor
        self.assertAlmostEqual(self.eye.openness, self.ff.LID_OPEN_TRACK, places=2)

    def test_tracking_live_locks_on(self):
        self.brain.set_mode("NOTR")
        self.brain.set_mode("TAKIP")
        self.assertTrue(self.eye.auto_blink and self.eye.idle_wander)
        self.frames(self.ff.TARGET_DEBOUNCE_FRAMES + 3, (0.5, 0.2))
        self.assertEqual(self.brain.status, "LOCK!")
        self.frames(30, (0.5, 0.2))
        self.assertAlmostEqual(self.eye.gaze_u, 0.5, places=2)

    def test_tracking_live_wanders_without_face(self):
        self.brain.set_mode("TAKIP")
        us = []
        for _ in range(10):                       # hic yuz yok: hemen canli bakinma
            self.frames(30, None)
            us.append(round(self.eye.gaze_u, 3))
        self.assertGreater(len(set(us)), 1)
        self.assertTrue(self.brain.status.startswith("CANLI"))
        self.assertAlmostEqual(self.eye.openness, self.ff.LID_OPEN_TRACK, places=2)  # uykulu degil
        self.frames(self.ff.TARGET_DEBOUNCE_FRAMES + 3, (0.5, 0.2))                   # yuz geldi
        self.assertEqual(self.brain.status, "LOCK!")
        self.frames(30, (0.5, 0.2))
        self.assertAlmostEqual(self.eye.gaze_u, 0.5, places=2)
        self.frames(self.ff.TARGET_DEBOUNCE_FRAMES + 1, None)                         # yuz gitti
        self.assertTrue(self.brain.status.startswith("CANLI"))
        self.assertTrue(self.eye.idle)

    def test_lifeless_tracking_follows_then_holds(self):
        self.brain.set_mode("CANSIZ")
        self.assertFalse(self.eye.auto_blink or self.eye.idle_wander)
        self.frames(60, (0.6, -0.3))
        self.assertAlmostEqual(self.eye.gaze_u, 0.6, places=2)
        self.assertEqual(self.brain.status, "TRACKING")
        self.frames(120, None)            # yuz gitti: son noktada bekler, bakinmaz
        self.assertAlmostEqual(self.eye.gaze_u, 0.6, places=2)
        self.assertIsNone(self.eye._blink_start)
        self.assertAlmostEqual(self.eye.openness, self.ff.LID_OPEN_TRACK, places=2)

    def test_hardware_button_cycles_modes(self):
        """PC surerken D2'ye basilinca firmware "BTN" yazar, idle'i degistirmez; tracker sonraki moda gecer."""
        self.brain.set_mode(self.ff.BUTTON_CYCLE[0])
        seen = self.eye.link.button_presses
        order = []
        for _ in range(len(self.ff.BUTTON_CYCLE)):
            self.frames(5, None)                      # S komutlari gidiyor -> PC kontrolde
            self.eye.link.ser.press_button()
            self.frames(1, None)
            while seen < self.eye.link.button_presses:
                seen += 1
                self.brain.next_mode()
            order.append(self.brain.mode)
            self.assertFalse(self.eye.link.ser.model.idle_enabled)
        self.assertEqual(order, list(self.ff.BUTTON_CYCLE[1:]) + [self.ff.BUTTON_CYCLE[0]])

    def test_manual_mode(self):
        self.brain.set_mode("TAKIP")
        self.frames(20, (0.4, 0.0))
        self.brain.set_mode("MANUEL")
        self.assertFalse(self.eye.auto_blink or self.eye.idle_wander)
        self.assertAlmostEqual(self.brain.manual[0], self.eye.gaze_u, places=6)   # sicramadan devralir
        self.brain.manual_set(u=-0.7, v=0.3, lid=0.4)
        self.frames(30, (0.9, 0.9))                       # yuz olsa da elle kontrol
        self.assertAlmostEqual(self.eye.gaze_u, -0.7, places=2)
        self.assertAlmostEqual(self.eye.openness, 0.4, places=2)
        self.assertEqual(self.brain.status, "MANUEL")
        self.brain.manual_set(du=-0.5, dlid=1.0)          # sinirlara kirpilir
        self.assertEqual(self.brain.manual[0], -1.0)
        self.assertEqual(self.brain.manual[2], 1.0)

    def test_screen_to_gaze_inverts_reticle(self):
        for mirror in (False, True):
            for u, v in ((0.5, -0.25), (-0.8, 0.6), (0.0, 0.0)):
                gu = -u if not mirror else u
                x, y = 960 / 2 * (1 + gu), 540 / 2 * (1 - v)
                ru, rv = self.ff.screen_to_gaze(x, y, 960, 540, mirror)
                self.assertAlmostEqual(ru, u, places=6)
                self.assertAlmostEqual(rv, v, places=6)

    def test_wheel_delta_decoding(self):
        # Windows'ta olculen bayraklar (WM_MOUSEWHEEL, OpenCV 5)
        self.assertEqual(self.ff.wheel_delta(7864320), 120)
        self.assertEqual(self.ff.wheel_delta(-7864320), -120)
        self.assertEqual(self.ff.wheel_delta(0), 0)

    def test_buttons_hit(self):
        import hud
        labels = [self.ff.MODE_LABELS[m] for m in self.ff.MODES]
        rects = hud.pill_rects(labels, 960, 540)
        self.assertEqual(len(rects), len(self.ff.MODES))
        for i, x, y, bw, bh in rects:
            self.assertEqual(hud.hit_pill(labels, 960, 540, x + 3, y + 3), i)
            self.assertLessEqual(x + bw, 960)
        self.assertIsNone(hud.hit_pill(labels, 960, 540, 2, 2))


class LinkMonitorTests(unittest.TestCase):
    def setUp(self):
        import face_follow as ff
        self.ff = ff
        self.eye, self.clock = make()
        self.eye.link.simulated = False          # sim karti "donanim" gibi izle
        self.found = []                          # Reconnector'un "bulacagi" kart
        self.scans = 0

        def finder(port):
            self.scans += 1
            return self.found.pop(0) if self.found else (None, None)

        self.ports = ["COM3"]
        rc = ff.Reconnector("auto", finder=finder, lister=lambda: list(self.ports))
        self.mon = ff.LinkMonitor(self.eye, rc, io_thread=False)   # sahte saatle esli kalsin

    def tick(self, seconds, dt=0.05):
        st = None
        for _ in range(int(round(seconds / dt))):
            self.clock.advance(dt)
            self.eye.update(dt)
            st = self.mon.update(self.clock())
            self.eye.link.poll()
        return st

    def test_ok_while_board_answers(self):
        self.assertEqual(self.tick(3.0), "ok")

    def test_silent_when_board_stops_answering(self):
        self.tick(1.0)
        self.eye.link.ser.write = lambda data: len(data)    # kart artik cevap vermiyor
        self.assertEqual(self.tick(2.0), "silent")

    def test_lost_when_port_dies(self):
        self.eye.link.alive = False
        self.assertEqual(self.tick(0.1), "lost")

    def test_reset_banner_shown(self):
        self.tick(1.0)
        self.eye.link.ser._rx += b"EYE v5 READY\r\n"
        self.assertEqual(self.tick(0.2), "reset")
        self.assertEqual(self.tick(self.ff.LinkMonitor.RESET_SHOW_S + 0.5), "ok")

    def new_board(self):
        import sim
        ser = sim.SimSerial(clock=self.clock, seed=1)
        ser.reset_input_buffer()                 # banner'i el sikisma tuketmis gibi
        return ser, "COM3"

    def wait_scan(self):
        t = self.mon.reconnector._thread
        if t is not None:
            t.join(2)

    def test_reconnects_after_cable_back(self):
        self.tick(1.0)
        self.eye.link.alive = False              # kablo cekildi
        self.ports = []
        self.assertEqual(self.tick(0.2), "lost")
        self.wait_scan()
        self.found.append(self.new_board())      # kablo takildi
        self.ports = ["COM3"]
        self.tick(0.05)
        self.wait_scan()
        self.assertEqual(self.tick(0.5), "ok")
        self.assertFalse(self.eye.simulated)
        self.assertGreater(self.eye.link.ser.model.last_s_ms, 0)   # yeni karta komut gidiyor

    def test_no_hardware_recovers_when_board_plugged(self):
        self.eye.link.simulated = True
        self.assertEqual(self.tick(0.1), "sim")
        self.wait_scan()
        self.found.append(self.new_board())
        self.ports = ["COM3", "COM4"]            # port listesi degisti -> hemen tara
        self.tick(0.05)
        self.wait_scan()
        self.assertEqual(self.tick(0.5), "ok")
        self.assertFalse(self.eye.simulated)

    def test_scan_not_repeated_while_ports_unchanged(self):
        self.eye.link.simulated = True
        for _ in range(20):
            self.tick(0.1)
            self.wait_scan()
        self.assertEqual(self.scans, 1)          # Lunar gimbal'i surekli resetleme

    def test_long_silence_reopens(self):
        self.tick(1.0)
        self.eye.link.ser.write = lambda data: len(data)
        self.assertEqual(self.tick(2.0), "silent")
        self.wait_scan()
        self.found.append(self.new_board())
        st = self.tick(4.0)
        self.wait_scan()
        self.assertEqual(self.tick(0.5), "ok")

    def test_sim_reported(self):
        self.eye.link.simulated = True
        self.assertEqual(self.tick(0.1), "sim")



class IoThreadTests(unittest.TestCase):
    """Gercek saatle: G/C thread'i komutlari iletir, cevaplari toplar, kapanista kuyrugu bosaltir."""

    def test_threaded_link_round_trip_and_close_flush(self):
        import time as _t
        import sim
        ser = sim.SimSerial(seed=2)
        ser.reset_input_buffer()
        link = EyeLink(ser, "COM9", simulated=False)
        link.start_io_thread()
        self.assertTrue(link.send("S 78 90 87"))
        self.assertTrue(link.send("?"))
        deadline = _t.monotonic() + 2.0
        while link.last_state is None and _t.monotonic() < deadline:
            link.poll()
            _t.sleep(0.01)
        self.assertIsNotNone(link.last_state)
        self.assertEqual(link.last_state[0], [78.0, 90.0, 87.0])
        link.send("A 1")
        link.close()
        deadline = _t.monotonic() + 2.0
        while ser.is_open and _t.monotonic() < deadline:
            _t.sleep(0.01)
        self.assertFalse(ser.is_open)
        self.assertIn("A 1", ser.written)
        self.assertFalse(link.send("?"))


class WanderFromGazeTests(unittest.TestCase):
    def _lose_face(self, from_gaze):
        import face_follow as ff
        old = ff.WANDER_FROM_GAZE
        ff.WANDER_FROM_GAZE = from_gaze
        try:
            eye, clock = make(human_motion=True)
            eye.auto_blink = False
            eye._wander.start_at(-0.8, 0.0, clock())   # bakinma bolgesi uzakta (sol)
            brain = ff.ModeBrain(eye, "TAKIP")
            dt = 1 / 30
            for _ in range(60):                          # yuz sagda
                clock.advance(dt)
                brain.step(clock(), dt, (0.8, 0.0))
                eye.update(dt)
            u0 = eye.gaze_u
            for _ in range(int(0.5 / dt)):               # yuz kayboldu, ilk 0.5 sn
                clock.advance(dt)
                brain.step(clock(), dt, None)
                eye.update(dt)
            return u0, eye.gaze_u
        finally:
            ff.WANDER_FROM_GAZE = old

    def test_v1_jumps_to_far_region(self):
        u0, u1 = self._lose_face(False)
        self.assertGreater(abs(u1 - u0), 1.0)

    def test_v2_starts_where_eye_is(self):
        u0, u1 = self._lose_face(True)
        self.assertLess(abs(u1 - u0), 0.2)


class TargetPickerTests(unittest.TestCase):
    A = (100, 100, 200, 200, 0.9)    # sol yuz (alan 10000)
    B = (500, 100, 601, 201, 0.9)    # sag yuz (alan 10201)

    def test_hold_zero_is_largest(self):
        import face_follow as ff
        p = ff.TargetPicker(0.0)
        for i in range(10):
            a = (100, 100, 200 + (i % 2) * 5, 205, 0.9)   # boyutlar her kare yer degistirir
            boxes = [a, self.B]
            self.assertIs(p.pick(boxes, i / 30), max(boxes, key=ff._box_area))

    def test_hold_stops_flicker_then_switches_to_larger(self):
        import face_follow as ff
        p = ff.TargetPicker(1.0)
        self.assertIs(p.pick([self.A], 0.0), self.A)          # once yalniz A
        t, picked = 0.0, []
        while t < 0.95:                                       # B biraz buyuk ama 1 sn dolmadi
            t += 1 / 30
            picked.append(p.pick([self.A, self.B], t))
        self.assertTrue(all(b is self.A for b in picked))
        self.assertIs(p.pick([self.A, self.B], 1.01), self.B)  # sure doldu -> en buyuk
        self.assertIs(p.pick([self.A, self.B], 1.05), self.B)  # yeni hedefte yine 1 sn kalir

    def test_lost_target_switches_immediately(self):
        import face_follow as ff
        p = ff.TargetPicker(1.0)
        p.pick([self.A], 0.0)
        self.assertIs(p.pick([self.B], 0.1), self.B)
        self.assertIsNone(p.pick([], 0.2))


if __name__ == "__main__":
    unittest.main()
