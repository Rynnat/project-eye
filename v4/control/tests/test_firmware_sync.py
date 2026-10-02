"""v4: .ino <-> sim.py sabitleri; limitler <-> cad/out/kinematics.json; idle <-> settings.json."""

import json
import os
import re
import unittest

from _helpers import INO_PATH, V4_DIR
import sim
from eye_control import KINEMATICS_PATH, load_settings


def _src():
    with open(INO_PATH, encoding="utf-8") as f:
        return f.read()


def _array(src, name):
    m = re.search(r"\b" + name + r"\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}", src)
    assert m, name
    return tuple(float(x.strip().rstrip("f")) for x in m.group(1).split(","))


def _scalar(src, name):
    m = re.search(r"\b" + name + r"\s*=\s*([0-9.]+)f?\s*;", src)
    assert m, name
    return float(m.group(1))


class FirmwareSyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.src = _src()

    def test_ino_path_is_v4(self):
        self.assertTrue(INO_PATH.endswith(os.path.join("project_eye_v4", "project_eye_v4.ino")), INO_PATH)
        self.assertIn(f'#define FW_BANNER "{sim.FW_BANNER}"', self.src)
        self.assertEqual(sim.FW_BANNER, "EYE v4 READY")
        self.assertEqual(sim.IDLE_LID_FOLLOW_PITCH, load_settings()["behaviour"]["lid_follow_pitch"])

    def test_arrays_and_scalars(self):
        for name in ("PINS", "LIMIT_MIN", "LIMIT_MAX", "IDLE_EYE_CENTER", "IDLE_EYE_DEG_PER_UNIT"):
            with self.subTest(name=name):
                self.assertEqual(_array(self.src, name), tuple(float(x) for x in getattr(sim, name)))
        self.assertEqual(sim.PINS, (3, 5, 6))
        for name in ("NUM_CH", "MAX_SPEED_DEG_S", "TICK_MS", "IDLE_TIMEOUT_MS", "LINE_MAX", "IDLE_LID_CLOSED",
                     "IDLE_LID_OPEN", "IDLE_LID_FOLLOW_PITCH", "IDLE_GAZE_U_MAX", "IDLE_GAZE_V_MAX",
                     "IDLE_BREATH_BASE", "IDLE_BREATH_AMP", "IDLE_BREATH_PERIOD_S", "IDLE_SACCADE_MIN_MS",
                     "IDLE_SACCADE_MAX_MS", "IDLE_BLINK_MIN_MS", "IDLE_BLINK_MAX_MS", "IDLE_BLINK_CLOSED_MS",
                     "BOOT_LID_OPENNESS", "SERVO_US_MIN", "SERVO_US_MAX"):
            with self.subTest(name=name):
                self.assertEqual(_scalar(self.src, name), float(getattr(sim, name)))
        self.assertLessEqual(sim.MAX_SPEED_DEG_S, 600)
        self.assertEqual((sim.IDLE_BLINK_MIN_MS, sim.IDLE_BLINK_MAX_MS), (3000, 6000))

    @unittest.skipUnless(os.path.isfile(KINEMATICS_PATH), "kinematics.json yok")
    def test_limits_equal_mechanical_no_margin(self):
        with open(KINEMATICS_PATH, encoding="utf-8") as f:
            lc = json.load(f)["limits_checked"]
        for i, n in enumerate(("EYE_YAW", "EYE_PITCH", "LIDS")):
            self.assertEqual((sim.LIMIT_MIN[i], sim.LIMIT_MAX[i]), tuple(float(x) for x in lc[n]), n)

    def test_idle_defaults_match_settings(self):
        s = load_settings()["channels"]
        self.assertEqual(sim.IDLE_LID_CLOSED, s["LIDS"]["closed"])
        self.assertEqual(sim.IDLE_LID_OPEN, s["LIDS"]["open"])
        self.assertEqual(sim.IDLE_EYE_DEG_PER_UNIT, (s["EYE_YAW"]["deg_per_unit"], s["EYE_PITCH"]["deg_per_unit"]))

    def test_settings_inside_envelope(self):
        s = load_settings()["channels"]
        for i, name in enumerate(("EYE_YAW", "EYE_PITCH")):
            for k in ("center", "min", "max"):
                self.assertTrue(sim.LIMIT_MIN[i] <= s[name][k] <= sim.LIMIT_MAX[i], (name, k))
        for k in ("closed", "open", "min", "max"):
            self.assertTrue(sim.LIMIT_MIN[2] <= s["LIDS"][k] <= sim.LIMIT_MAX[2], k)
        self.assertTrue(os.path.isfile(os.path.join(V4_DIR, "settings.json")))


if __name__ == "__main__":
    unittest.main()
