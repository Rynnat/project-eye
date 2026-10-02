"""SPEC §6 kalibrasyon formulleri, ayar dosyasi okuma/yazma, S/STATE bicimi."""

import copy
import json
import os
import tempfile
import unittest

from _helpers import V2_DIR
import sim
from eye_control import (DEFAULT_SETTINGS, SettingsError, fw_clamp, compute_angles, eye_angle, format_s_command,
                         lid_angle, lid_follow, load_settings, parse_state, save_settings, validate_settings)

YAW = {"center": 90, "min": 60, "max": 120, "invert": False, "deg_per_unit": 30}


class EyeFormulaTests(unittest.TestCase):
    def test_linear(self):
        self.assertEqual(eye_angle(YAW, 0), 90)
        self.assertEqual(eye_angle(YAW, 0.5), 105)
        self.assertEqual(eye_angle(YAW, -1), 60)

    def test_u_clamped_to_unit(self):
        self.assertEqual(eye_angle(YAW, 5), 120)
        self.assertEqual(eye_angle(YAW, -5), 60)

    def test_invert(self):
        inv = dict(YAW, invert=True)
        self.assertEqual(eye_angle(inv, 0.5), 75)
        self.assertEqual(eye_angle(inv, -1), 120)

    def test_min_max_clamp_after_scale(self):
        narrow = dict(YAW, min=80, max=95)
        self.assertEqual(eye_angle(narrow, 1), 95)
        self.assertEqual(eye_angle(narrow, -1), 80)
        off_center = dict(YAW, center=100, deg_per_unit=40, max=130)
        self.assertEqual(eye_angle(off_center, 1), 130)


class LidFormulaTests(unittest.TestCase):
    def test_linear_both_directions(self):
        ul = {"closed": 60, "open": 120, "invert": False}
        ll = {"closed": 120, "open": 70, "invert": False}
        self.assertEqual(lid_angle(ul, 0), 60)
        self.assertEqual(lid_angle(ul, 1), 120)
        self.assertEqual(lid_angle(ul, 0.5), 90)
        self.assertEqual(lid_angle(ll, 0), 120)
        self.assertEqual(lid_angle(ll, 1), 70)
        self.assertEqual(lid_angle(ll, 0.25), 107.5)

    def test_openness_clamped(self):
        ul = {"closed": 60, "open": 120, "invert": False}
        self.assertEqual(lid_angle(ul, -3), 60)
        self.assertEqual(lid_angle(ul, 3), 120)

    def test_invert_mirrors(self):
        ul = {"closed": 60, "open": 120, "invert": True}
        self.assertEqual(lid_angle(ul, 0), 120)
        self.assertEqual(lid_angle(ul, 1), 60)

    def test_follow_pitch(self):
        self.assertEqual(lid_follow(0.8, 0.0, 0.6), (0.8, 0.8))
        up, low = lid_follow(0.8, 1.0, 0.6)
        self.assertGreater(up, 0.8)      # yukari bakinca ust kapak kalkar
        self.assertLess(low, 0.8)        # alt kapak da yukari gelir
        up, low = lid_follow(0.8, -1.0, 0.6)
        self.assertLess(up, 0.8)
        self.assertGreater(low, 0.8)
        self.assertEqual(lid_follow(0.0, 1.0, 0.6), (0.0, 0.0))   # kapali goz aralanmaz
        self.assertEqual(lid_follow(1.0, 1.0, 1.0)[0], 1.0)       # kirpilir
        self.assertEqual(lid_follow(0.8, 1.0, 0.0), (0.8, 0.8))   # k=0 takip yok

    def test_compute_angles_linear(self):
        s = copy.deepcopy(DEFAULT_SETTINGS)
        a = compute_angles(s, 0, 0, 1.0, 0.0)
        self.assertEqual(a, [90, 90, 96.15, 86.85, 115.62, 66.23])
        a = compute_angles(s, 1, -1, 0.5, 0.5, follow_pitch=False)
        self.assertAlmostEqual(a[0], 120)
        self.assertAlmostEqual(a[1], 64)                 # 90 - 34.43 = 55.57 -> settings min 64
        self.assertAlmostEqual(a[2], (64.38 + 96.15) / 2)

    def test_compute_angles_never_outside_firmware_limits(self):
        # SPEC §6 ornek degerleri yeni mekanik limitlerin disinda: kirpilmali
        s = copy.deepcopy(DEFAULT_SETTINGS)
        s["channels"].update({
            "EYE_PITCH": {"center": 90, "min": 40, "max": 140, "invert": False, "deg_per_unit": 50},
            "LID_UL": {"closed": 60, "open": 120, "invert": False},
            "LID_LL": {"closed": 120, "open": 70, "invert": False},
            "LID_UR": {"closed": 120, "open": 60, "invert": False},
            "LID_LR": {"closed": 60, "open": 110, "invert": False}})
        for u in (-1, 0, 1):
            for v in (-1, 0, 1):
                for o in (0.0, 0.5, 1.0):
                    for i, a in enumerate(compute_angles(s, u, v, o, o)):
                        self.assertTrue(sim.LIMIT_MIN[i] <= a <= sim.LIMIT_MAX[i], (u, v, o, i, a))
                        self.assertEqual(a, fw_clamp(i, a))


class SettingsFileTests(unittest.TestCase):
    def test_repo_settings_follows_schema(self):
        # degerler kalibrasyonla degisebilir; sema (anahtarlar) SPEC §6 ile ayni kalmali
        load_settings()  # dogrulama hatasi firlatmamali
        with open(os.path.join(V2_DIR, "settings.json"), encoding="utf-8") as f:
            raw = json.load(f)
        self.assertEqual(set(raw), set(DEFAULT_SETTINGS))
        self.assertEqual(set(raw["behaviour"]), set(DEFAULT_SETTINGS["behaviour"]))
        for name, cfg in DEFAULT_SETTINGS["channels"].items():
            self.assertEqual(set(raw["channels"][name]), set(cfg), name)

    def test_partial_file_merges_defaults(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "s.json")
            with open(p, "w", encoding="utf-8") as f:
                json.dump({"channels": {"EYE_YAW": {"center": 95}}}, f)
            s = load_settings(p)
            self.assertEqual(s["channels"]["EYE_YAW"]["center"], 95)
            self.assertEqual(s["channels"]["EYE_YAW"]["deg_per_unit"], 30)
            self.assertEqual(s["behaviour"], DEFAULT_SETTINGS["behaviour"])

    def test_validation(self):
        bad = copy.deepcopy(DEFAULT_SETTINGS)
        bad["channels"]["EYE_YAW"]["min"] = 100      # min > center
        bad["channels"]["LID_UL"]["open"] = 200
        bad["channels"]["LID_LL"]["invert"] = "no"
        bad["behaviour"]["blink_interval_s"] = [6, 3]
        with self.assertRaises(SettingsError) as cm:
            validate_settings(bad)
        msg = str(cm.exception)
        for part in ("EYE_YAW", "LID_UL.open", "LID_LL.invert", "blink_interval_s"):
            self.assertIn(part, msg)

    def test_save_roundtrip_atomic(self):
        s = copy.deepcopy(DEFAULT_SETTINGS)
        s["channels"]["LID_UR"]["closed"] = 118.5
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "settings.json")
            save_settings(s, p)
            self.assertEqual(load_settings(p), s)
            self.assertFalse(os.path.exists(p + ".tmp"))
            with open(p, encoding="utf-8") as f:
                text = f.read()
            self.assertIn('"LID_UR":    {"closed": 118.5', text)   # SPEC bicimi: kanal basina tek satir
            bad = copy.deepcopy(s)
            bad["channels"]["EYE_PITCH"]["max"] = 10
            with self.assertRaises(SettingsError):
                save_settings(bad, p)
            self.assertEqual(load_settings(p), s)   # bozuk ayar dosyayi ezmedi


class WireFormatTests(unittest.TestCase):
    def test_s_command_roundtrip(self):
        line = format_s_command([90, 90.04, 180, 0, 123.456, 113.77])
        self.assertEqual(line, "S 90.00 90.04 180.00 0.00 123.46 113.77")   # limit 113.77 yuvarlanip asilmaz
        self.assertLessEqual(len(line), sim.LINE_MAX)
        self.assertEqual(sim.parse_command(line)[0], "S")
        self.assertEqual(format_s_command([-5, 200, 0, 0, 0, 0])[:14], "S 0.00 180.00 ")
        self.assertLessEqual(len(format_s_command([180] * 6)), sim.LINE_MAX)
        with self.assertRaises(ValueError):
            format_s_command([1, 2, 3])

    def test_parse_state(self):
        self.assertEqual(parse_state("STATE 90.0 90.0 60.0 120.0 120.0 60.0 idle=1"),
                         ([90.0, 90.0, 60.0, 120.0, 120.0, 60.0], True))
        for bad in ("STATE 90 90 idle=1", "STATE a b c d e f idle=0", "OK", "STATE 1 2 3 4 5 6 idle=2", ""):
            self.assertIsNone(parse_state(bad), bad)


if __name__ == "__main__":
    unittest.main()
