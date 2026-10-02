"""cad/out/kinematics.json tablolari: enterpolasyon, kose pozlari, limit guvenligi, geri dusus."""

import copy
import json
import os
import random
import tempfile
import unittest

from _helpers import FakeClock, V2_DIR, quiet
import sim
from eye_control import (DEFAULT_SETTINGS, KINEMATICS_PATH, EyeController, EyeLink, Kinematics,
                         compute_angles, eye_angles, lid_angle, load_kinematics, load_settings)

HAS_CAD = os.path.isfile(KINEMATICS_PATH)


def synthetic() -> dict:
    """Kucuk, elle hesaplanabilir tablo (gercek dosyadan bagimsiz testler icin)."""
    lid = lambda s0, s1, s2: [{"openness": 0.0, "servo_deg": s0}, {"openness": 1.0, "servo_deg": s1},
                              {"openness": 1.5, "servo_deg": s2}]
    return {
        "channels": {
            "EYE_YAW": {"table": [{"eye_yaw_deg": -30, "servo_deg": 60}, {"eye_yaw_deg": 0, "servo_deg": 90},
                                  {"eye_yaw_deg": 30, "servo_deg": 120}]},
            "LID_UL": {"table": lid(70, 100, 110)}, "LID_LL": {"table": lid(110, 80, 70)},
            "LID_UR": {"table": lid(110, 80, 70)}, "LID_LR": {"table": lid(70, 100, 110)},
        },
        "cross_coupling": {"EYE_PITCH_servo_deg_grid": {
            "yaw_deg": [-30, 0, 30], "pitch_deg": [-25, 0, 25],
            "servo_deg": [[66, 60, 66], [90, 90, 90], [118, 120, 118]]}},
    }


class InterpolationTests(unittest.TestCase):
    def setUp(self):
        self.k = Kinematics.from_dict(synthetic())

    def test_grid_nodes_exact(self):
        self.assertEqual(self.k.pitch_servo(0, -25), 60)
        self.assertEqual(self.k.pitch_servo(-30, -25), 66)
        self.assertEqual(self.k.pitch_servo(30, 25), 118)

    def test_bilinear(self):
        # (yaw 15, pitch -12.5): alt satir 63, orta satir 90 -> 76.5
        self.assertAlmostEqual(self.k.pitch_servo(15, -12.5), 76.5)
        self.assertAlmostEqual(self.k.pitch_servo(-15, 12.5), (119 + 90) / 2)

    def test_outside_grid_clamped(self):
        self.assertEqual(self.k.pitch_servo(99, -99), 66)
        self.assertEqual(self.k.yaw_servo(45), 120)

    def test_inverse(self):
        self.assertAlmostEqual(self.k.eye_pitch_at(0, 60), -25)
        self.assertAlmostEqual(self.k.eye_pitch_at(30, 66), -25)
        self.assertLess(self.k.eye_pitch_at(30, 60), -25)       # yaw 30'da 60 derece -> -25'in altina

    def test_lid_table_with_calibration_fit(self):
        cfg = {"closed": 70, "open": 100, "invert": False}
        self.assertEqual(lid_angle(cfg, 0.0, self.k, "LID_UL"), 70)
        self.assertEqual(lid_angle(cfg, 1.0, self.k, "LID_UL"), 100)
        self.assertEqual(lid_angle(cfg, 1.5, self.k, "LID_UL"), 110)   # takip payi
        self.assertEqual(lid_angle(cfg, 9.0, self.k, "LID_UL"), 110)   # tablo ucuna kirpilir
        # kalibrasyon farkli (servo kolu 2 derece kaymis): uclar kalibrasyona oturur
        shifted = {"closed": 72, "open": 102, "invert": False}
        self.assertEqual(lid_angle(shifted, 0.0, self.k, "LID_UL"), 72)
        self.assertEqual(lid_angle(shifted, 1.0, self.k, "LID_UL"), 102)

    def test_bad_tables_rejected(self):
        for mutate in (lambda d: d["cross_coupling"]["EYE_PITCH_servo_deg_grid"]["servo_deg"].pop(),
                       lambda d: d["cross_coupling"]["EYE_PITCH_servo_deg_grid"]["yaw_deg"].reverse(),
                       lambda d: d["channels"]["LID_UL"]["table"].pop(),         # 1.0'dan sonrasi var ama...
                       lambda d: d["channels"]["LID_LL"]["table"].__setitem__(0, {"openness": 0.5, "servo_deg": 1}),
                       lambda d: d["channels"]["EYE_YAW"]["table"][0].__setitem__("servo_deg", 200),
                       lambda d: d["channels"].pop("LID_LR")):
            d = synthetic()
            mutate(d)
            with self.subTest(mutate=mutate):
                try:
                    Kinematics.from_dict(d)
                except (ValueError, KeyError, TypeError):
                    continue
                # LID_UL pop: 0 ve 1 hala kapsaniyor -> gecerli kalabilir
                self.assertEqual(len(d["channels"]["LID_UL"]["table"]), 2)


class FallbackTests(unittest.TestCase):
    def test_missing_or_broken_file_falls_back(self):
        self.assertIsNone(load_kinematics(os.path.join(V2_DIR, "yok", "kinematics.json"), log=quiet))
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "k.json")
            with open(p, "w", encoding="utf-8") as f:
                f.write("{ bozuk json")
            self.assertIsNone(load_kinematics(p, log=quiet))
            with open(p, "w", encoding="utf-8") as f:
                json.dump({"channels": {}}, f)
            self.assertIsNone(load_kinematics(p, log=quiet))

    def test_linear_when_no_table(self):
        s = copy.deepcopy(DEFAULT_SETTINGS)
        self.assertEqual(eye_angles(s["channels"], 0.5, 0.0, None), (105.0, 90.0))
        clock = FakeClock()
        eye = EyeController(settings=s, link=EyeLink.simulator(clock), clock=clock, log=quiet,
                            kinematics=None)
        self.assertEqual(eye.mapping, "LINEAR")
        eye.update(0.0)


@unittest.skipUnless(HAS_CAD, "cad/out/kinematics.json yok")
class RealTableTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k = load_kinematics(log=quiet)
        with open(KINEMATICS_PATH, encoding="utf-8") as f:
            cls.raw = json.load(f)

    def test_loaded(self):
        self.assertIsNotNone(self.k)
        self.assertEqual((self.k.yaw_range, self.k.pitch_range), (30.0, 25.0))

    def test_firmware_limits_match_mechanical(self):
        lc = self.raw["limits_checked"]
        names = ("EYE_YAW", "EYE_PITCH_firmware_safe", "LID_UL", "LID_LL", "LID_UR", "LID_LR")
        for i, n in enumerate(names):
            with self.subTest(n=n):
                self.assertEqual((sim.LIMIT_MIN[i], sim.LIMIT_MAX[i]), tuple(lc[n]))

    def test_settings_follow_suggestion(self):
        sug = self.raw["settings_suggestion"]["channels"]
        s = load_settings()["channels"]
        for name, cfg in sug.items():
            for key, val in cfg.items():
                if not key.startswith("_"):
                    self.assertEqual(s[name][key], val, f"{name}.{key}")

    def test_corner_poses_compensated(self):
        # yaw +-30 kosesinde tablo, pitch -25 icin firmware_safe alt sinirini verir
        self.assertAlmostEqual(self.k.pitch_servo(30, -25), sim.LIMIT_MIN[1])
        self.assertAlmostEqual(self.k.pitch_servo(-30, 25), sim.LIMIT_MAX[1])
        self.assertAlmostEqual(self.k.pitch_servo(0, 0), 90.0)

    def test_no_pose_leaves_limits_or_25_deg(self):
        """Tum bakis/aciklik izgarasinda: firmware limiti asilmaz, gercek goz pitch'i +-25'i gecmez."""
        s = load_settings()
        steps = [x / 10 for x in range(-12, 13)]          # -1.2..1.2 (dis girdiler dahil)
        for u in steps:
            for v in steps:
                for o in (0.0, 0.5, 1.0):
                    a = compute_angles(s, u, v, o, 1 - o, kin=self.k)
                    for i, x in enumerate(a):
                        self.assertTrue(sim.LIMIT_MIN[i] <= x <= sim.LIMIT_MAX[i], (u, v, i, x))
                    yaw_deg = a[0] - 90.0
                    real = self.k.eye_pitch_at(yaw_deg, a[1])
                    self.assertLessEqual(abs(real), 25.0 + 1e-6, (u, v, a[1], real))

    def test_unreachable_clamped_to_nearest_safe(self):
        s = load_settings()
        yaw, pitch = eye_angles(s["channels"], 0.0, -1.0, self.k)
        self.assertEqual(pitch, s["channels"]["EYE_PITCH"]["min"])     # tablo 56.73 isterdi
        self.assertLess(self.k.eye_pitch_at(0, pitch), -19.0)           # ulasilabilen en asagi
        yaw, pitch = eye_angles(s["channels"], 1.0, 0.0, self.k)
        self.assertEqual((yaw, pitch), (120.0, 90.0))                   # pitch=0'da capraz etki yok

    def test_linear_would_have_been_unsafe(self):
        # SPEC dogrusal + yaw=0 sinirlari (56.73) -> yaw 30'da goz -31.8: tablo + limit bunu onler
        self.assertLess(self.k.eye_pitch_at(30, 56.73), -30)

    def test_controller_uses_table_end_to_end(self):
        clock = FakeClock()
        link = EyeLink.simulator(clock, seed=1)
        eye = EyeController(settings=load_settings(), link=link, clock=clock, rng=random.Random(1), log=quiet)
        self.assertEqual(eye.mapping, "TABLE")
        eye.auto_blink = False
        eye.look(1.0, -1.0)
        for _ in range(100):
            clock.advance(0.01)
            eye.update(0.01)
        link.poll()
        self.assertEqual(link.err_count, 0)
        model = link.ser.model
        for i, p in enumerate(model.pos):
            self.assertTrue(sim.LIMIT_MIN[i] <= p <= sim.LIMIT_MAX[i])
        self.assertAlmostEqual(model.pos[1], 64.0, places=2)             # settings min (koseden guvenli)

    def test_lid_follow_uses_table_margin(self):
        s = load_settings()
        up = compute_angles(s, 0, 1.0, 1.0, 1.0, kin=self.k)
        flat = compute_angles(s, 0, 0.0, 1.0, 1.0, kin=self.k)
        # yukari bakinca ust kapak "open"in otesine (tablo payi) kalkar, ama limit icinde
        self.assertGreater(up[2], flat[2])
        self.assertLessEqual(up[2], sim.LIMIT_MAX[2])
        self.assertLess(up[4], flat[4])                                  # UR ters yonlu servo


if __name__ == "__main__":
    unittest.main()
