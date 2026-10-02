"""out/kinematics.json (v4): servo acisi <-> goz/kapak acisi (v3 ile ayni format). Model degerleri; montajdan sonra kalibre edin.

  PYTHONUTF8=1 python kinematics_export.py
"""
import json
import os

import numpy as np

import kin
from params import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def r1(x):
    return float(round(x, 2))


def build_json():
    sa = kin.servo_angles
    J = {
        "_about": "Project Eye v4 (v3 revizyonu) mekanik kinematik tablolari (v4/cad/kinematics_export.py). Model degerleri; montaj sonrasi kalibre edin.",
        "_servo_convention": ("servo_deg = 90 + (servo kolunun, mil ucundan DISARI bakan eksen etrafinda sag-el donusu). "
                              "SG90'da darbe genisligi artinca hangi yone dondugu ureticiye gore degisir: ters ise 'invert'."),
        "_eye_convention": ("yaw + = robot sagina (+X), pitch + = yukari. Pitch cercevesi X ekseni etrafinda doner, gozler cercevede "
                            "dikey pimlerle yaw yapar: R_goz = Rx(-pitch) * Ry(yaw)."),
        "_lid_convention": (f"lid 0 = kapali (ust kenar {UP_EDGE_CLOSED:+.1f}, dudak {UP_EDGE_CLOSED - LIP_DROP:+.1f}; alt kenar {LO_EDGE_CLOSED:+.1f} deg), "
                            f"1 = tam acik (ust +{UP_OPEN:.0f}, alt -{kin.LO_OPEN_ACT:.1f} deg). Ust kapak acisi = lid*{UP_OPEN} (dogrusal), "
                            "alt kapak ve servo acisi dort-cubuktan (hafif dogrusal degil). Kapaklar CERCEVEDE: pitch'i mekanik olarak izler."),
        "channels": {}, "limits_checked": {}, "cross_coupling": {}, "transmission": {}, "settings_suggestion": {},
    }
    ch = J["channels"]
    ch["EYE_YAW"] = {"channel": 0, "mechanism": f"paralelkenar: iki goz kolu + servo krank (r={LEVER_R}), duz lama; servo pitch cercevesinde",
                     "input": "eye_yaw_deg",
                     "table": [{"eye_yaw_deg": y, "servo_deg": r1(sa(y, 0, 0)["EYE_YAW"])} for y in (-25, -12.5, 0, 12.5, 25)],
                     "note": "birebir ve dogrusal: servo_deg = 90 - yaw (servo mili asagi bakar). pitch'ten bagimsiz."}
    ch["EYE_PITCH"] = {"channel": 1, "mechanism": "direkt: pitch cercevesinin sag yan plakasi servo koluna vidali (mil = pitch ekseni)",
                       "input": "eye_pitch_deg",
                       "table": [{"eye_pitch_deg": p, "servo_deg": r1(sa(0, p, 0)["EYE_PITCH"])} for p in (-20, -10, 0, 10, 20)],
                       "note": "birebir: servo_deg = 90 + pitch"}
    lids = [i / 10 for i in range(11)]
    ch["LIDS"] = {"channel": 2, "mechanism": ("LIDS servosu cercevede (sag gozun arkasi, mil -X); krank r=20 ucundaki tek pime iki lama: "
                                              "ust lama -> ust kapaklar (ayni yon), alt lama capraz -> alt kapaklar (TERS yon). 4 kapak tek kanal."),
                  "input": "lid (0 kapali .. 1 acik)",
                  "table": [{"lid": l, "upper_edge_deg": r1(UP_EDGE_CLOSED + kin.lid_deltas(l)[0]),
                             "lower_edge_deg": r1(LO_EDGE_CLOSED - kin.lid_deltas(l)[1]), "servo_deg": r1(sa(0, 0, l)["LIDS"])}
                            for l in lids],
                  "note": (f"servo_deg = 90 + (b - b_orta), b = krank acisi (dort-cubuk). Kapali -> {r1(sa(0,0,0)['LIDS'])}, "
                           f"acik -> {r1(sa(0,0,1)['LIDS'])}. v3'e gore ARALIK DEGISTI (v3: 112.5 kapali .. 67.5 acik).")}
    J["cross_coupling"] = {"EYE_YAW": "yok", "EYE_PITCH": "yok",
                           "LIDS": ("servo acisi pitch'ten bagimsiz; ama kapaklar cercevede oldugu icin pitch'i MEKANIK olarak izler. "
                                    "Yazilimdaki lid_follow_pitch bunun USTUNE eklenir -> v4'te 0'a yakin baslayin.")}
    J["limits_checked"] = {
        "EYE_YAW": sorted([r1(sa(-YAW_RANGE, 0, 0)["EYE_YAW"]), r1(sa(YAW_RANGE, 0, 0)["EYE_YAW"])]),
        "EYE_PITCH": [r1(90 - PITCH_RANGE), r1(90 + PITCH_RANGE)],
        "LIDS": sorted([r1(sa(0, 0, 0)["LIDS"]), r1(sa(0, 0, 1)["LIDS"])]),
        "_note": ("check.py bu araliklarin koselerinde ve ara pozlarda carpisma taradi. Firmware sabit limitleri bu araliklarin "
                  "disina CIKMAMALI; disi DOGRULANMADI (kapak > acik: ust kapak arka kenari ust pim gobegine, alt kapak alt kopruye "
                  "yaklasir; kapali < 0: kapak kenarlari birbirine dayanir)."),
    }
    J["transmission"] = {"_note": "kol ile lama arasi aci; 90 ideal, olu nokta 0/180.",
                         "EYE_YAW": {"neutral": 90.0, f"at_+-{YAW_RANGE:.0f}": r1(90 - YAW_RANGE)},
                         "EYE_PITCH": "direkt (lama yok)",
                         "LIDS": {f"lid_{l}": [r1(x) for x in kin.transmission(l)] for l in (0, 0.5, 1.0)},
                         "_LIDS_note": "LIDS: [ust pim, alt pim, krank-ust lama, krank-alt lama] teget sapmasi (0 ideal = 90 deg iletim)"}
    lo, hi = J["limits_checked"]["LIDS"]
    J["settings_suggestion"] = {"channels": {
        "EYE_YAW": {"center": 90, "min": int(90 - YAW_RANGE), "max": int(90 + YAW_RANGE), "invert": True, "deg_per_unit": YAW_RANGE,
                    "_note": "invert=True: model servo_deg = 90 - yaw. Donanimda yonu dogrulayin."},
        "EYE_PITCH": {"center": 90, "min": int(90 - PITCH_RANGE), "max": int(90 + PITCH_RANGE), "invert": False,
                      "deg_per_unit": PITCH_RANGE},
        "LIDS": {"closed": r1(sa(0, 0, 0)["LIDS"]), "open": r1(sa(0, 0, 1)["LIDS"]), "min": lo, "max": hi, "invert": False},
    }, "_note": "Model tabanli baslangic. Servo kolunu takmadan once servoyu notr (90) aciya getirin; yonler donanimda dogrulanmali."}
    return J


def write():
    J = build_json()
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "kinematics.json"), "w", encoding="utf-8") as f:
        json.dump(J, f, indent=1, ensure_ascii=False)
    return J


if __name__ == "__main__":
    print(json.dumps(write()["settings_suggestion"], indent=1, ensure_ascii=False))
