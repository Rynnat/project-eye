"""v4 firmware davranisi (sim.FirmwareModel): limitler, hiz siniri, idle."""

import unittest

from _helpers import CONTROL_DIR  # noqa: F401
import sim
from sim import FirmwareModel


def cmd(m, text, t):
    return m.feed((text + "\n").encode("ascii"), t)


class LimitsAndSpeedTests(unittest.TestCase):
    def test_clamped_to_mechanical_limits(self):
        m = FirmwareModel(seed=0)
        self.assertEqual(cmd(m, "S 0 180 0", 0), ["OK"])
        m.advance(1000)
        self.assertEqual(m.pos, [sim.LIMIT_MIN[0], sim.LIMIT_MAX[1], sim.LIMIT_MIN[2]])
        cmd(m, "S 180 0 180", 1000)
        m.advance(2000)
        self.assertEqual(m.pos, [sim.LIMIT_MAX[0], sim.LIMIT_MIN[1], sim.LIMIT_MAX[2]])

    def test_speed_limit(self):
        m = FirmwareModel(seed=0)
        c = sim.IDLE_EYE_CENTER[0]       # acilis: kalibre merkez
        cmd(m, "S %.1f 90 90" % (c + 25), 0)
        m.advance(25)                    # 25 ms * 600 deg/s = 15 derece
        self.assertAlmostEqual(m.pos[0], c + 15, places=4)
        m.advance(50)
        self.assertAlmostEqual(m.pos[0], c + 25, places=4)

    def test_boot_pose(self):
        m = FirmwareModel(seed=0)
        self.assertEqual(m.pos[:2], list(sim.IDLE_EYE_CENTER))
        self.assertAlmostEqual(m.pos[2], sim.IDLE_LID_CLOSED + 0.8 * (sim.IDLE_LID_OPEN - sim.IDLE_LID_CLOSED))


class IdleTests(unittest.TestCase):
    def setUp(self):
        self.m = FirmwareModel(now_ms=0, seed=42)
        cmd(self.m, "A 1", 0)                          # acilista idle kapali (buton / A 1 ile acilir)
        cmd(self.m, "S 90 90 80", 0)

    def idle_flag(self, t):
        return cmd(self.m, "?", t)[0].endswith("idle=1")

    def test_idle_rules(self):
        self.assertFalse(self.idle_flag(1990))
        self.assertTrue(self.idle_flag(2000))
        cmd(self.m, "S 90 90 90", 2500)
        self.assertFalse(self.idle_flag(2501))
        cmd(self.m, "D", 2600)
        self.m.advance(9000)
        self.assertFalse(self.m.idle_active)          # detach sonrasi idle yok
        cmd(self.m, "S 90 90 90", 9000)
        cmd(self.m, "A 0", 9000)
        self.m.advance(20000)
        self.assertFalse(self.m.idle_active)

    def test_idle_life(self):
        m = self.m
        prev, yaws, lid_max, t = list(m.pos), set(), 0.0, 0
        while t < 30000:
            t += sim.TICK_MS
            m.advance(t)
            for i, (a, b) in enumerate(zip(prev, m.pos)):
                self.assertLessEqual(abs(a - b), 600 * sim.TICK_MS / 1000 + 1e-6)
                self.assertTrue(sim.LIMIT_MIN[i] <= b <= sim.LIMIT_MAX[i])
            prev = list(m.pos)
            yaws.add(round(m.target[0], 1))
            lid_max = max(lid_max, m.pos[2])
        self.assertGreater(len(yaws), 8)
        self.assertTrue(4 <= m.blink_count <= 11, m.blink_count)
        self.assertGreaterEqual(lid_max, sim.IDLE_LID_CLOSED - 0.5)   # kirpmada kapak kapandi


if __name__ == "__main__":
    unittest.main()


class ButtonTests(unittest.TestCase):
    """D2 butonu: acilista notr (idle yok); bas -> idle hemen; tekrar bas -> notr poza doner, idle kalici kapali."""

    def test_boot_neutral_then_toggle(self):
        m = FirmwareModel(now_ms=0, seed=7)
        m.advance(10000)
        self.assertFalse(m.idle_active)                # acilista kendiliginden idle YOK
        m.press_button(10000)
        self.assertTrue(m.idle_active)                 # bekleme olmadan baslar
        m.advance(15000)
        m.press_button(15000)
        self.assertFalse(m.idle_active)
        m.advance(20000)
        self.assertEqual(m.pos[:2], list(sim.IDLE_EYE_CENTER))      # notr poz
        self.assertAlmostEqual(m.pos[2], sim.IDLE_LID_CLOSED + 0.8 * (sim.IDLE_LID_OPEN - sim.IDLE_LID_CLOSED), places=3)
        m.advance(40000)
        self.assertFalse(m.idle_active)                # tekrar kendiliginden baslamaz

