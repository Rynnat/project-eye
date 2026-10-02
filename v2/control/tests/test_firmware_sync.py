""".ino ile sim.py sabitleri ayni mi? settings.json ornek degerleri firmware zarfinin icinde mi?"""

import re
import unittest

from _helpers import INO_PATH
import sim
from eye_control import CHANNELS, lid_angle, load_settings


def _ino():
    with open(INO_PATH, encoding="utf-8") as f:
        return f.read()


def _array(src, name):
    m = re.search(r"\b" + name + r"\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}", src)
    assert m, f"{name} .ino'da yok"
    return tuple(float(x.strip().rstrip("f")) for x in m.group(1).split(","))


def _scalar(src, name):
    m = re.search(r"\b" + name + r"\s*=\s*([0-9.]+)f?\s*;", src)
    assert m, f"{name} .ino'da yok"
    return float(m.group(1))


class FirmwareSyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.src = _ino()

    def test_banner(self):
        self.assertIn(f'#define FW_BANNER "{sim.FW_BANNER}"', self.src)

    def test_arrays(self):
        for name in ("PINS", "LIMIT_MIN", "LIMIT_MAX", "IDLE_EYE_CENTER", "IDLE_EYE_DEG_PER_UNIT",
                     "IDLE_LID_CLOSED", "IDLE_LID_OPEN"):
            with self.subTest(name=name):
                self.assertEqual(_array(self.src, name), tuple(float(x) for x in getattr(sim, name)))
        self.assertEqual(sim.PINS, (3, 5, 6, 9, 10, 11))  # SPEC §4

    def test_scalars(self):
        for name in ("MAX_SPEED_DEG_S", "SERVO_US_MIN", "SERVO_US_MAX", "TICK_MS", "IDLE_TIMEOUT_MS",
                     "LINE_MAX", "IDLE_LID_FOLLOW_PITCH", "IDLE_GAZE_U_MAX", "IDLE_GAZE_V_MAX",
                     "IDLE_BREATH_BASE", "IDLE_BREATH_AMP", "IDLE_BREATH_PERIOD_S",
                     "IDLE_SACCADE_MIN_MS", "IDLE_SACCADE_MAX_MS", "IDLE_BLINK_MIN_MS",
                     "IDLE_BLINK_MAX_MS", "IDLE_BLINK_CLOSED_MS", "BOOT_LID_OPENNESS"):
            with self.subTest(name=name):
                self.assertEqual(_scalar(self.src, name), float(getattr(sim, name)))
        self.assertLessEqual(sim.MAX_SPEED_DEG_S, 600)          # SPEC §5
        self.assertEqual(sim.IDLE_TIMEOUT_MS, 2000)
        self.assertEqual((sim.IDLE_BLINK_MIN_MS, sim.IDLE_BLINK_MAX_MS), (3000, 6000))
        self.assertIn("Serial.begin(115200)", self.src)

    def test_settings_inside_firmware_envelope(self):
        s = load_settings()
        for i, name in enumerate(CHANNELS):
            cfg = s["channels"][name]
            if "center" in cfg:
                vals = [cfg["center"], cfg["min"], cfg["max"]]
            else:  # ham servo acisi (invert aynasi dahil)
                vals = [lid_angle(cfg, 0.0), lid_angle(cfg, 1.0)]
            for v in vals:
                self.assertTrue(sim.LIMIT_MIN[i] <= v <= sim.LIMIT_MAX[i], f"{name}={v}")

    def test_idle_defaults_match_settings_example(self):
        # Firmware idle'i PC'siz calistigi icin kapak/goz kalibrasyonu derlemeye gomulu.
        # settings.json kalibrasyonla degisirse bu test HATIRLATIR: .ino + sim.py guncellenmeli.
        s = load_settings()["channels"]
        lids = ("LID_UL", "LID_LL", "LID_UR", "LID_LR")
        self.assertEqual(sim.IDLE_LID_CLOSED, tuple(float(s[n]["closed"]) for n in lids))
        self.assertEqual(sim.IDLE_LID_OPEN, tuple(float(s[n]["open"]) for n in lids))
        self.assertEqual(sim.IDLE_EYE_DEG_PER_UNIT,
                         (float(s["EYE_YAW"]["deg_per_unit"]), float(s["EYE_PITCH"]["deg_per_unit"])))


if __name__ == "__main__":
    unittest.main()
