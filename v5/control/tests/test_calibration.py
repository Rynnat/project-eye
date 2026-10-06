"""v4 formulleri (dogrusal + tablo), kinematik tablo, ayar dosyasi, S/STATE bicimi, limit guvenligi."""

import copy
import json
import os
import tempfile
import unittest

from _helpers import V4_DIR, quiet

KALIBRE = os.path.isfile(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "kalibrasyon.json"))
import sim
from eye_control import (DEFAULT_SETTINGS, KINEMATICS_PATH, Kinematics, SettingsError, compute_angles,
                         eye_angle, eye_angles, format_s_command, fw_clamp, lid_angle, lid_follow,
                         load_kinematics, load_settings, parse_state, save_settings, validate_settings)

HAS_CAD = os.path.isfile(KINEMATICS_PATH)
YAW = {"center": 90, "min": 65, "max": 115, "invert": True, "deg_per_unit": 25}
LIDS = {"closed": 112.5, "open": 67.5, "min": 67.5, "max": 112.5, "invert": False}


def synthetic(pitch_curve=False) -> dict:
    """Elle hesaplanabilir tablo; pitch_curve=True ise pitch dogrusal DEGIL."""
    ps = [70, 90, 115] if pitch_curve else [70, 90, 110]
    return {"channels": {
        "EYE_YAW": {"table": [{"eye_yaw_deg": -25, "servo_deg": 115}, {"eye_yaw_deg": 0, "servo_deg": 90},
                              {"eye_yaw_deg": 25, "servo_deg": 65}]},
        "EYE_PITCH": {"table": [{"eye_pitch_deg": -20, "servo_deg": ps[0]},
                                {"eye_pitch_deg": 0, "servo_deg": ps[1]},
                                {"eye_pitch_deg": 20, "servo_deg": ps[2]}]},
        "LIDS": {"table": [{"lid": 0, "servo_deg": 112.5}, {"lid": 0.5, "servo_deg": 95},
                           {"lid": 1, "servo_deg": 67.5}]},
    }}


class LinearFormulaTests(unittest.TestCase):
    def test_eye_invert_and_clamp(self):
        self.assertEqual(eye_angle(YAW, 0), 90)
        self.assertEqual(eye_angle(YAW, 1), 65)          # invert: +u -> servo azalir
        self.assertEqual(eye_angle(YAW, -0.5), 102.5)
        self.assertEqual(eye_angle(YAW, -9), 115)
        self.assertEqual(eye_angle(dict(YAW, invert=False, max=100), 1), 100)

    def test_lid(self):
        self.assertEqual(lid_angle(LIDS, 0), 112.5)
        self.assertEqual(lid_angle(LIDS, 1), 67.5)
        self.assertEqual(lid_angle(LIDS, 0.5), 90)
        self.assertEqual(lid_angle(LIDS, 7), 67.5)
        self.assertEqual(lid_angle(dict(LIDS, min=80), 1), 80)                  # min/max kirpar
        self.assertEqual(lid_angle(dict(LIDS, invert=True, min=0, max=180), 0), 67.5)   # 180 - 112.5

    def test_lid_follow(self):
        self.assertEqual(lid_follow(0.8, 0, 0.6), 0.8)
        self.assertGreater(lid_follow(0.8, 1, 0.6), 0.8)   # yukari bakinca kapak kalkar
        self.assertLess(lid_follow(0.8, -1, 0.6), 0.8)
        self.assertEqual(lid_follow(0.0, 1, 0.6), 0.0)     # kapali goz aralanmaz
        self.assertEqual(lid_follow(1.0, 1, 1.0), 1.0)


class TableTests(unittest.TestCase):
    def test_linear_table_equals_linear_formula(self):
        k = Kinematics.from_dict(synthetic())
        ch = copy.deepcopy(DEFAULT_SETTINGS)["channels"]
        for u in (-1, -0.3, 0, 0.6, 1):
            for v in (-1, 0.25, 1):
                t = eye_angles(ch, u, v, k)
                lin = eye_angles(ch, u, v, None)
                self.assertAlmostEqual(t[0], lin[0])
                self.assertAlmostEqual(t[1], lin[1])

    def test_invert_semantics_same_in_both_modes(self):
        k = Kinematics.from_dict(synthetic())
        ch = copy.deepcopy(DEFAULT_SETTINGS)["channels"]
        ch["EYE_YAW"]["invert"] = False       # donanimda yon ters cikti
        self.assertAlmostEqual(eye_angles(ch, 1, 0, k)[0], 115)
        self.assertAlmostEqual(eye_angles(ch, 1, 0, None)[0], 115)

    def test_curved_table_and_lid_fit(self):
        k = Kinematics.from_dict(synthetic(pitch_curve=True))
        ch = copy.deepcopy(DEFAULT_SETTINGS)["channels"]
        ch["EYE_PITCH"]["max"] = 120
        # +-1 = kalibre donanim siniri (120); egrinin sekli korunur: tabloda 0.5 -> sapmanin yarisi
        self.assertAlmostEqual(eye_angles(ch, 0, 0.5, k)[1], 105)     # 90 + 0.5*(120-90)
        self.assertAlmostEqual(eye_angles(ch, 0, 1, k)[1], 120)
        self.assertEqual(lid_angle(LIDS, 0.5, k), 95)                 # tablonun egrisi
        shifted = dict(LIDS, closed=110, open=70, min=60, max=120)
        self.assertEqual(lid_angle(shifted, 0, k), 110)               # uclar kalibrasyona oturur
        self.assertEqual(lid_angle(shifted, 1, k), 70)

    def test_bad_tables_rejected_and_fallback(self):
        for mutate in (lambda d: d["channels"].pop("LIDS"),
                       lambda d: d["channels"]["EYE_PITCH"]["table"].reverse() or
                       d["channels"]["EYE_PITCH"]["table"].append({"eye_pitch_deg": 0, "servo_deg": 1}),
                       lambda d: d["channels"]["LIDS"]["table"].pop(),
                       lambda d: d["channels"]["EYE_YAW"]["table"][0].__setitem__("servo_deg", 181),
                       lambda d: d["channels"]["EYE_YAW"]["table"].pop(0)):
            d = synthetic()
            mutate(d)
            with self.subTest(mutate=mutate), self.assertRaises((ValueError, KeyError, TypeError)):
                Kinematics.from_dict(d)
        self.assertIsNone(load_kinematics(os.path.join(V4_DIR, "yok.json"), log=quiet))
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, "k.json")
            with open(p, "w", encoding="utf-8") as f:
                f.write("{bozuk")
            self.assertIsNone(load_kinematics(p, log=quiet))

    @unittest.skipUnless(HAS_CAD, "kinematics.json yok")
    @unittest.skipIf(KALIBRE, "settings.json elde kalibre edildi (kalibrasyon.json); model tablosuyla birebir olmasi beklenmez")
    def test_real_table_matches_model(self):
        k = load_kinematics(log=quiet)
        self.assertIsNotNone(k)
        self.assertEqual((k.yaw_range, k.pitch_range, k.yaw_dir, k.pitch_dir), (25, 20, -1, 1))
        s = load_settings()
        self.assertEqual(compute_angles(s, 1, -1, 1.0, follow_pitch=False, kin=k), [65, 70, 77.71])
        self.assertEqual(compute_angles(s, -1, 1, 0.0, kin=k), [115, 110, 102.29])
        # LIDS dort-cubuk tablosu (dogrusal degil): tablo satirlari aynen verilir
        with open(KINEMATICS_PATH, encoding="utf-8") as f:
            rows = json.load(f)["channels"]["LIDS"]["table"]
        self.assertGreaterEqual(len(rows), 5)
        for r in rows:
            self.assertAlmostEqual(compute_angles(s, 0, 0, r["lid"], kin=k)[2], r["servo_deg"], places=6)
        lin_mid = (102.29 + 77.71) / 2
        self.assertNotAlmostEqual(lid_angle(s["channels"]["LIDS"], 0.5, k), lin_mid, places=2)

    @unittest.skipUnless(HAS_CAD, "kinematics.json yok")
    @unittest.skipIf(KALIBRE, "settings.json elde kalibre edildi; model onerisinden sapmasi beklenir")
    def test_settings_follow_suggestion(self):
        with open(KINEMATICS_PATH, encoding="utf-8") as f:
            sug = json.load(f)["settings_suggestion"]["channels"]
        s = load_settings()["channels"]
        self.assertEqual(set(s), set(sug))
        for name, cfg in sug.items():
            for key, val in cfg.items():
                if not key.startswith("_"):
                    self.assertEqual(s[name][key], val, f"{name}.{key}")


class SafetyTests(unittest.TestCase):
    def test_never_outside_firmware_limits(self):
        wild = copy.deepcopy(DEFAULT_SETTINGS)
        wild["channels"] = {
            "EYE_YAW": {"center": 90, "min": 0, "max": 180, "invert": False, "deg_per_unit": 80},
            "EYE_PITCH": {"center": 100, "min": 0, "max": 180, "invert": True, "deg_per_unit": 60},
            "LIDS": {"closed": 150, "open": 20, "invert": False}}
        kins = [None, Kinematics.from_dict(synthetic()), Kinematics.from_dict(synthetic(True))]
        if HAS_CAD:
            kins.append(load_kinematics(log=quiet))
        for s in (wild, copy.deepcopy(DEFAULT_SETTINGS)):
            for k in kins:
                for u in [x / 5 for x in range(-7, 8)]:
                    for v in [x / 5 for x in range(-7, 8)]:
                        for o in (-1, 0, 0.5, 1, 2):
                            a = compute_angles(s, u, v, o, kin=k)
                            for i, x in enumerate(a):
                                self.assertTrue(sim.LIMIT_MIN[i] <= x <= sim.LIMIT_MAX[i], (u, v, o, i, x))
                                self.assertEqual(x, fw_clamp(i, x))
                            # gonderilen metin de limit icinde (2 ondalik yuvarlama asmaz)
                            vals = [float(t) for t in format_s_command(a).split()[1:]]
                            for i, x in enumerate(vals):
                                self.assertTrue(sim.LIMIT_MIN[i] <= x <= sim.LIMIT_MAX[i])


class SettingsFileTests(unittest.TestCase):
    def test_repo_settings_schema(self):
        with open(os.path.join(V4_DIR, "settings.json"), encoding="utf-8") as f:
            raw = json.load(f)
        self.assertEqual(set(raw), {"serial_port", "channels", "behaviour"})
        self.assertEqual(set(raw["channels"]), {"EYE_YAW", "EYE_PITCH", "LIDS"})
        self.assertEqual(set(raw["channels"]["LIDS"]), {"closed", "open", "min", "max", "invert"})
        self.assertEqual(raw["behaviour"]["lid_follow_pitch"], 0.0)          # v4: mekanik takip
        load_settings()

    def test_validation(self):
        bad = copy.deepcopy(DEFAULT_SETTINGS)
        bad["channels"]["EYE_YAW"]["min"] = 100
        bad["channels"]["LIDS"]["min"] = 120
        bad["channels"]["LIDS"]["invert"] = "no"
        bad["behaviour"]["blink_interval_s"] = [6, 3]
        with self.assertRaises(SettingsError) as cm:
            validate_settings(bad)
        for part in ("EYE_YAW", "LIDS:", "LIDS.invert", "blink_interval_s"):
            self.assertIn(part, str(cm.exception))
        no_mm = copy.deepcopy(DEFAULT_SETTINGS)
        del no_mm["channels"]["LIDS"]["min"], no_mm["channels"]["LIDS"]["max"]
        validate_settings(no_mm)                                  # min/max opsiyonel

    def test_save_roundtrip(self):
        s = copy.deepcopy(DEFAULT_SETTINGS)
        s["channels"]["LIDS"]["closed"] = 110.25
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "settings.json")
            save_settings(s, p)
            self.assertEqual(load_settings(p), s)
            with open(p, encoding="utf-8") as f:
                self.assertIn('"LIDS":      {"closed": 110.25', f.read())


class WireFormatTests(unittest.TestCase):
    def test_s_and_state(self):
        self.assertEqual(format_s_command([90, 67.5, 112.499]), "S 90.00 67.50 112.50")
        self.assertEqual(sim.parse_command(format_s_command([180, 180, 180]))[0], "S")
        with self.assertRaises(ValueError):
            format_s_command([1, 2, 3, 4, 5, 6])
        self.assertEqual(parse_state("STATE 90.0 70.0 112.5 idle=1"), ([90.0, 70.0, 112.5], True))
        for bad in ("STATE 90.0 90.0 60.0 120.0 120.0 60.0 idle=1", "STATE 1 2 idle=0", "OK", ""):
            self.assertIsNone(parse_state(bad), bad)


if __name__ == "__main__":
    unittest.main()
