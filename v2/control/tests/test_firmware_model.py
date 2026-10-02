"""Firmware davranisi (sim.FirmwareModel uzerinden): limitler, hiz siniri, idle kurallari."""

import unittest

from _helpers import CONTROL_DIR  # noqa: F401
import sim
from sim import FirmwareModel


def cmd(m, text, t):
    return m.feed((text + "\n").encode("ascii"), t)


class LimitsAndSpeedTests(unittest.TestCase):
    def test_clamped_to_mechanical_limits(self):
        m = FirmwareModel(seed=0)
        self.assertEqual(cmd(m, "S 0 180 0 180 0 180", 0), ["OK"])
        m.advance(1000)
        for ch in range(sim.NUM_CH):
            expected = sim.LIMIT_MIN[ch] if ch % 2 == 0 else sim.LIMIT_MAX[ch]
            self.assertAlmostEqual(m.pos[ch], expected, places=6, msg=f"kanal {ch}")

    def test_speed_limit_600_deg_s(self):
        m = FirmwareModel(seed=0)
        cmd(m, "S 120 90 90 90 90 90", 0)
        m.advance(25)                      # 25 ms -> en fazla 15 derece
        self.assertAlmostEqual(m.pos[0], 90.0 + 15.0, places=4)
        m.advance(55)                      # 30 derece 50 ms'de biter
        self.assertAlmostEqual(m.pos[0], 120.0, places=4)

    def test_microseconds_mapping(self):
        self.assertEqual(FirmwareModel.deg_to_us(0), 544)
        self.assertEqual(FirmwareModel.deg_to_us(180), 2400)
        self.assertEqual(FirmwareModel.deg_to_us(90), 1472)


class IdleTests(unittest.TestCase):
    def setUp(self):
        self.m = FirmwareModel(now_ms=0, seed=42)
        cmd(self.m, "S 90 90 110 80 70 100", 0)

    def idle_flag(self, t):
        return cmd(self.m, "?", t)[0].endswith("idle=1")

    def test_enters_idle_after_2s(self):
        self.assertFalse(self.idle_flag(1990))
        self.assertTrue(self.idle_flag(2000))

    def test_query_does_not_reset_timer(self):
        for t in range(0, 2000, 200):
            cmd(self.m, "?", t)
        self.assertTrue(self.idle_flag(2005))

    def test_s_takes_back_control(self):
        self.m.advance(3000)
        self.assertTrue(self.m.idle_active)
        self.assertEqual(cmd(self.m, "S 90 90 90 90 90 90", 3000), ["OK"])
        self.assertFalse(self.idle_flag(3001))
        self.assertTrue(self.idle_flag(5000))

    def test_a0_disables_and_holds(self):
        self.m.advance(2600)
        self.assertTrue(self.m.idle_active)
        cmd(self.m, "A 0", 2600)
        held = list(self.m.pos)
        self.m.advance(10000)
        self.assertFalse(self.m.idle_active)
        for a, b in zip(held, self.m.pos):
            self.assertAlmostEqual(a, b, places=6)
        cmd(self.m, "A 1", 10000)
        self.assertTrue(self.idle_flag(10001))   # timer S'ten beri coktan doldu

    def test_detach_prevents_idle_until_s(self):
        self.assertEqual(cmd(self.m, "D", 100), ["OK"])
        self.m.advance(10000)
        self.assertFalse(self.m.attached)
        self.assertFalse(self.m.idle_active)
        cmd(self.m, "S 90 90 90 90 90 90", 10000)
        self.assertTrue(self.m.attached)

    def test_idle_life(self):
        """Idle'da: hiz siniri korunur, sakkadlar olur, 3-6 sn'de bir kapak tam kapanir."""
        m = self.m
        m.advance(2000)
        prev = list(m.pos)
        yaws, ul_min, t = set(), 999.0, 2000
        end = 2000 + 30000
        while t < end:
            t += sim.TICK_MS
            m.advance(t)
            for a, b in zip(prev, m.pos):
                self.assertLessEqual(abs(a - b), sim.MAX_SPEED_DEG_S * sim.TICK_MS / 1000 + 1e-6)
            prev = list(m.pos)
            yaws.add(round(m.target[0], 1))
            ul_min = min(ul_min, m.pos[2])
            u = (m.pos[0] - 90.0) / 30.0
            self.assertLessEqual(abs(u), sim.IDLE_GAZE_U_MAX + 1e-6)
        self.assertGreater(len(yaws), 8, "sakkad yok gibi")
        self.assertTrue(30000 / 6000 - 1 <= m.blink_count <= 30000 / 3000 + 1, m.blink_count)
        self.assertLessEqual(ul_min, sim.IDLE_LID_CLOSED[0] + 0.5, "kirpmada kapak kapanmadi")

    def test_breathing_moves_lids(self):
        m = self.m
        m.advance(2000)
        seen = []
        for t in range(2000, 7000, 50):
            m.advance(t)
            if not m.idle_blinking:
                seen.append(m.target[2])
        self.assertGreater(max(seen) - min(seen), 5.0)


if __name__ == "__main__":
    unittest.main()
