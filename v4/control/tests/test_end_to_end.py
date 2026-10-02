"""v4 uctan uca: kontrolcu -> sim firmware; port bulma (Lunar + v2 reddi); kalibrasyon araci."""

import copy
import os
import random
import tempfile
import unittest

from _helpers import AutoClock, FakeClock, quiet
import sim
from eye_control import (DEFAULT_SETTINGS, KEEPALIVE_S, EyeController, EyeLink, find_eye_port, handshake,
                         load_settings)


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
    def test_commands_reach_firmware(self):
        clock = FakeClock()
        eye, link, model = make(clock)
        eye.auto_blink = False
        eye.look(-1.0, 0.5)
        eye.lids(0.6)
        run(eye, clock, 1.0)
        self.assertEqual(link.err_count, 0)
        for fw, pc in zip(model.pos, eye.angles):
            self.assertAlmostEqual(fw, pc, delta=0.01)
        angles, idle = link.query_state()
        self.assertFalse(idle)
        self.assertAlmostEqual(angles[0], 115.0, delta=0.05)
        self.assertTrue(all(len(w.split()) == 4 for w in link.ser.written if w.startswith("S")))

    def test_keepalive_release_resume_close(self):
        clock = FakeClock()
        eye, link, model = make(clock)
        eye.idle_wander = False
        run(eye, clock, 5.0)
        self.assertFalse(model.idle_active)
        self.assertGreaterEqual(sum(w.startswith("S") for w in link.ser.written), int(5 / KEEPALIVE_S) - 1)
        eye.release()
        run(eye, clock, 2.1)
        self.assertTrue(model.idle_active)
        eye.resume()
        run(eye, clock, 0.05)
        self.assertFalse(model.idle_active)
        eye.close()
        self.assertEqual(link.ser.written[-1], "A 1")


class _FakePort:
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
        self.assertEqual(handshake(_FakePort(["", "EYE v4 READY"]), 3, 2, clock=c), "eye")
        self.assertEqual(handshake(_FakePort(["READY"]), 3, 2, clock=c), "lunar")
        self.assertEqual(handshake(_FakePort(["EYE v2 READY"]), 3, 2, clock=c), "other-eye")
        self.assertEqual(handshake(_FakePort(["EYE v3 READY"]), 3, 2, clock=c), "other-eye")
        # resetlenmeyen kart: banner yok. v3 de 3 acili STATE verdigi icin STATE ile kabul YOK
        silent = _FakePort(answer_query="STATE 90.0 90.0 80.0 idle=0")
        self.assertEqual(handshake(silent, 3, 2, clock=c), "unknown")
        self.assertEqual(silent.written, [])                                # hicbir sey yazilmaz

    def test_find_skips_lunar_v2_busy(self):
        ports = {"COM3": _FakePort(["READY"]), "COM4": _FakePort(["EYE v2 READY"]),
                 "COM5": _FakePort(["EYE v3 READY"]), "COM7": sim.SimSerial(port="COM7")}

        def opener(dev):
            if dev == "COM2":
                raise OSError("Access is denied")
            return ports[dev]

        ser, dev = find_eye_port("auto", opener=opener, ports=["COM2", "COM3", "COM4", "COM5", "COM7"], log=quiet,
                                 handshake_timeout_s=0.3, query_after_s=0.1)
        self.assertEqual(dev, "COM7")
        self.assertTrue(ports["COM3"].closed and ports["COM4"].closed and ports["COM5"].closed)
        for p in ("COM3", "COM4", "COM5"):
            self.assertFalse(any(w.startswith((b"S", b"D", b"A")) for w in ports[p].written))


class CalibratorTests(unittest.TestCase):
    def setUp(self):
        import calibrate
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "settings.json")
        self.link = EyeLink.simulator(seed=2)
        self.cal = calibrate.Calibrator(load_settings(), self.link, self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_jog_record_save(self):
        cal = self.cal
        self.assertEqual(cal.angles, [90, 90, 77.71])       # notr: gozler center, kapak acik
        self.assertIsNone(cal.handle_key("4"))              # 4. kanal yok
        cal.handle_key("3")
        cal.handle_key("RIGHT")
        cal.handle_key(".")
        self.assertAlmostEqual(cal.angles[2], 78.91)
        self.assertEqual(self.link.ser.written[-1], "S 90.00 90.00 78.91")
        cal.handle_key("o")
        for _ in range(4):
            cal.handle_key("UP")                           # 78.91 + 20 = 98.91
        msg = cal.handle_key("k")
        self.assertIn("min/max", msg)
        self.assertEqual(cal.settings["channels"]["LIDS"],
                         {"closed": 98.91, "open": 78.91, "min": 78.91, "max": 98.91, "invert": False})
        self.assertIn("kaydedildi", cal.handle_key("s"))
        self.assertEqual(load_settings(self.path)["channels"]["LIDS"]["closed"], 98.91)

    def test_never_beyond_firmware_limit(self):
        cal = self.cal
        cal.handle_key("2")
        for _ in range(10):
            cal.handle_key("UP")
        self.assertEqual(cal.angles[1], sim.LIMIT_MAX[1])
        self.assertIn("[FW LIMIT]", cal.status_line())
        cal.handle_key("3")
        for _ in range(20):
            cal.handle_key("DOWN")
        self.assertEqual(cal.angles[2], sim.LIMIT_MIN[2])
        for w in self.link.ser.written:
            vals = [float(x) for x in w.split()[1:]]
            for i, x in enumerate(vals):
                self.assertTrue(sim.LIMIT_MIN[i] <= x <= sim.LIMIT_MAX[i], w)
        self.assertEqual(cal.test_sequence(sleep=lambda s: None), "test bitti")


if __name__ == "__main__":
    unittest.main()
