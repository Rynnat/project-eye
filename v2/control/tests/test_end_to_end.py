"""Uctan uca: EyeController -> seri satirlar -> firmware simulatoru; port bulma; kalibrasyon araci."""

import copy
import os
import random
import tempfile
import unittest

from _helpers import AutoClock, FakeClock, quiet
import sim
from eye_control import (DEFAULT_SETTINGS, KEEPALIVE_S, SEND_HZ, EyeController, EyeLink, find_eye_port,
                         handshake, load_settings)


def make(clock):
    link = EyeLink.simulator(clock, seed=11)
    eye = EyeController(settings=copy.deepcopy(DEFAULT_SETTINGS), link=link, clock=clock,
                        rng=random.Random(5), log=quiet)
    return eye, link, link.ser.model


def run(eye, clock, seconds, dt=0.01):
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        eye.update(dt)


class EndToEndTests(unittest.TestCase):
    def test_banner_consumed_and_commands_ok(self):
        clock = FakeClock()
        eye, link, model = make(clock)
        eye.auto_blink = False
        eye.look(1.0, 0.5)
        eye.lids(0.6)
        run(eye, clock, 1.0)
        self.assertEqual(link.err_count, 0)
        self.assertGreater(link.ok_count, 0)
        self.assertLessEqual(eye.sent_count, SEND_HZ * 1.0 + 1)
        # firmware konumu = kontrolcunun hesapladigi acilar (hiz siniri coktan tamamlandi)
        for fw, pc in zip(model.pos, eye.angles):
            self.assertAlmostEqual(fw, pc, delta=0.06)
        angles, idle = link.query_state()
        self.assertFalse(idle)
        self.assertAlmostEqual(angles[0], 120.0, delta=0.05)

    def test_keepalive_blocks_firmware_idle_release_allows_it(self):
        clock = FakeClock()
        eye, link, model = make(clock)
        eye.auto_blink = False
        eye.idle_wander = False
        eye.look(0.0, 0.0)
        run(eye, clock, 5.0)                 # hareket yok ama keepalive var
        self.assertFalse(model.idle_active)
        gaps = [b for b in link.ser.written if b.startswith("S")]
        self.assertGreaterEqual(len(gaps), int(5.0 / KEEPALIVE_S) - 1)
        eye.release()
        run(eye, clock, 2.1)
        self.assertTrue(model.idle_active)
        eye.resume()
        run(eye, clock, 0.05)
        self.assertFalse(model.idle_active)

    def test_detach_and_reattach(self):
        clock = FakeClock()
        eye, link, model = make(clock)
        run(eye, clock, 0.1)
        eye.detach()
        link.poll()
        self.assertFalse(model.attached)
        run(eye, clock, 0.05)
        self.assertTrue(model.attached)

    def test_close_hands_over_to_firmware(self):
        clock = FakeClock()
        eye, link, model = make(clock)
        run(eye, clock, 0.1)
        eye.close()
        self.assertEqual(link.ser.written[-1], "A 1")
        self.assertFalse(link.alive)
        clock.advance(2.5)
        model.advance(int(clock() * 1000))
        self.assertTrue(model.idle_active)

    def test_err_is_counted(self):
        clock = FakeClock()
        _, link, _ = make(clock)
        link.send("S 1 2")
        link.send("Q")
        link.poll()
        self.assertEqual(link.err_count, 2)
        self.assertEqual(link.last_error, "ERR unknown command")

    def test_open_sim(self):
        link = EyeLink.open("sim", log=quiet)
        self.assertTrue(link.simulated)
        self.assertIsNotNone(link.query_state())


class _FakePort:
    """Gercek porta benzer: satir listesi verir, sonra bos doner; yazilanlari saklar."""

    def __init__(self, lines=(), answer_query=None):
        self.lines = [l.encode() + b"\r\n" for l in lines]
        self.answer_query = answer_query
        self.written = []
        self.closed = False

    def readline(self):
        return self.lines.pop(0) if self.lines else b""

    def write(self, data):
        self.written.append(data)
        if data == b"?\n" and self.answer_query:
            self.lines.append(self.answer_query.encode() + b"\r\n")
        return len(data)

    def close(self):
        self.closed = True


class PortDiscoveryTests(unittest.TestCase):
    def test_handshake_kinds(self):
        c = AutoClock(step=0.05)
        self.assertEqual(handshake(_FakePort(["", "EYE v2 READY"]), 3, 2, clock=c), "eye")
        self.assertEqual(handshake(_FakePort(["READY"]), 3, 2, clock=c), "lunar")
        silent = _FakePort()
        self.assertEqual(handshake(silent, 3, 2, clock=c), "unknown")
        self.assertEqual(silent.written, [b"?\n"])      # yalnizca zararsiz sorgu gitti
        noreset = _FakePort(answer_query="STATE 90.0 90.0 108.0 80.0 72.0 100.0 idle=1")
        self.assertEqual(handshake(noreset, 3, 2, clock=c), "eye")

    def test_find_skips_busy_lunar_and_unknown(self):
        ports = {
            "COM3": _FakePort(["READY"]),               # Lunar gimbal
            "COM4": _FakePort(["garbage", "hello"]),    # baska cihaz
            "COM7": sim.SimSerial(port="COM7"),         # Project Eye v2
        }

        def opener(dev):
            if dev == "COM2":
                raise OSError("Access is denied")        # mesgul port
            return ports[dev]

        ser, dev = find_eye_port("auto", opener=opener, ports=["COM2", "COM3", "COM4", "COM7"],
                                 log=quiet, handshake_timeout_s=0.3, query_after_s=0.1)
        self.assertEqual(dev, "COM7")
        self.assertIs(ser, ports["COM7"])
        self.assertTrue(ports["COM3"].closed)
        self.assertTrue(ports["COM4"].closed)
        # Lunar gimbal'ine hicbir S/D/A komutu gitmedi
        self.assertFalse(any(w.startswith((b"S", b"D", b"A")) for w in ports["COM3"].written))

    def test_find_nothing(self):
        ser, dev = find_eye_port("auto", opener=lambda d: _FakePort(["READY"]), ports=["COM3"],
                                 log=quiet, handshake_timeout_s=0.2, query_after_s=0.1)
        self.assertIsNone(ser)
        self.assertIsNone(dev)

    def test_explicit_port_only(self):
        tried = []

        def opener(dev):
            tried.append(dev)
            return sim.SimSerial(port=dev)

        ser, dev = find_eye_port("COM9", opener=opener, ports=["COM1", "COM2"], log=quiet)
        self.assertEqual((tried, dev), (["COM9"], "COM9"))


class CalibratorTests(unittest.TestCase):
    def setUp(self):
        import calibrate
        self.cal_mod = calibrate
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "settings.json")
        self.link = EyeLink.simulator(seed=2)
        self.cal = calibrate.Calibrator(load_settings(), self.link, self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_jog_record_save(self):
        cal = self.cal
        self.assertEqual(cal.angles, [90, 90, 96.15, 86.85, 83.85, 93.15])   # notr: gozler center, kapaklar open
        cal.handle_key("3")                                                    # LID_UL
        for _ in range(3):
            cal.handle_key("RIGHT")
        cal.handle_key(",")
        self.assertAlmostEqual(cal.angles[2], 98.95)
        self.assertEqual(self.link.ser.written[-1], "S 90.00 90.00 98.95 86.85 83.85 93.15")
        cal.handle_key("o")
        for _ in range(6):
            cal.handle_key("DOWN")    # 98.95 - 30 = 68.95
        cal.handle_key("k")
        self.assertEqual(cal.settings["channels"]["LID_UL"], {"closed": 68.95, "open": 98.95, "invert": False})
        self.assertTrue(cal.dirty)
        self.assertIn("kaydedildi", cal.handle_key("s"))
        self.assertEqual(load_settings(self.path)["channels"]["LID_UL"]["open"], 98.95)
        self.link.poll()
        self.assertEqual(self.link.err_count, 0)

    def test_invalid_eye_not_saved(self):
        cal = self.cal
        cal.handle_key("1")
        cal.handle_key("UP")          # 95
        cal.handle_key("n")           # min 95 > center 90
        self.assertIn("KAYDEDILMEDI", cal.handle_key("s"))
        self.assertFalse(os.path.exists(self.path))

    def test_lid_invert_stores_logical_value(self):
        cal = self.cal
        cal.handle_key("4")           # LID_LL
        cal.handle_key("i")
        cal.handle_key("RIGHT")       # ham 86.85 + 1 = 87.85
        cal.handle_key("k")
        self.assertEqual(cal.settings["channels"]["LID_LL"]["closed"], 92.15)   # 180 - 87.85

    def test_fw_limit_warning_and_quit(self):
        cal = self.cal
        cal.handle_key("1")
        for _ in range(10):
            cal.handle_key("DOWN")    # 90 - 50 = 40 -> firmware limiti 60'ta durur
        self.assertEqual(cal.angles[0], sim.LIMIT_MIN[0])
        self.assertIn("[FW LIMIT]", cal.status_line())
        self.assertTrue(all(float(x) >= sim.LIMIT_MIN[0] for x in
                            (w.split()[1] for w in self.link.ser.written if w.startswith("S"))))
        self.assertEqual(cal.handle_key("q"), "QUIT")

    def test_test_sequence_runs(self):
        cal = self.cal
        cal.handle_key("2")
        self.assertEqual(cal.test_sequence(sleep=lambda s: None), "test bitti")
        self.assertEqual(cal.angles[1], 90.0)


if __name__ == "__main__":
    unittest.main()
