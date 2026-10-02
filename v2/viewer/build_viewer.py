"""Project Eye v2 - 3D goruntuleyiciyi (../model.html) uretir.

Hareket JS'te YENIDEN YAZILMAZ: mekanik modelin kendi kinematigi (cad/eye_v2.pose_transforms)
bir poz izgarasinda calistirilir, gruplarin 4x4 donusumleri kaydedilir; goruntuleyici izgara
noktalari arasinda enterpole eder. Boylece ekranda gorulen hareket, carpisma taramasinin
kullandigi modelle birebir aynidir.

cad/ klasorune YAZMAZ, sadece okur. Model degisince yeniden calistirin:
  C:\\Users\\LENOVO\\epic-project\\.venv-cad\\Scripts\\python.exe build_viewer.py
"""
import base64
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
sys.path.insert(0, str(CAD))
os.chdir(CAD)                      # cad modulleri goreli yollarla calisiyor olabilir
import eye_v2 as E                 # noqa: E402
import kin                         # noqa: E402

EYE_GROUPS = ["eye_L", "eye_R", "gobek_L", "gobek_R", "yawcoupler", "yawhorn", "rocker", "rod_pitch_L", "rod_pitch_R"]
LID_KEYS = ["UL", "LL", "UR", "LR"]
YAWS = list(range(-30, 31, 5))                 # 13
PITCHES = list(range(-25, 26, 5))              # 11
LID_STEPS = 21                                 # kapali (0) .. mekanik tam acik (1)


def m16(M):
    return [round(float(x), 5) for x in np.asarray(M).flatten()]


def rel(M, M0):
    """GLB'deki mesh'ler NOTR pozda dunya koordinatinda. pose_transforms ise bazi gruplari
    (or. cubuklar) kendi yerel cercevesinden dunyaya tasiyor. Her iki durumda da dogru olan:
    notrdeki donusumun tersiyle carpip 'notr -> poz' goreli donusumu saklamak."""
    return np.asarray(M) @ np.linalg.inv(np.asarray(M0))


def main():
    M0 = E.pose_transforms(0, 0, "closed")
    # --- goz izgarasi: (yaw, pitch) -> goza bagli gruplarin donusumleri + servo acilari ---
    eye_grid, servo_eye = [], []
    for y in YAWS:
        row_m, row_s = [], []
        for p in PITCHES:
            Ms = E.pose_transforms(y, p, "closed")
            row_m.append({g: m16(rel(Ms[g], M0[g])) for g in EYE_GROUPS})
            s = E.servo_angles(y, p, "closed")
            row_s.append([round(s["EYE_YAW"], 2), round(s["EYE_PITCH"], 2)])
        eye_grid.append(row_m)
        servo_eye.append(row_s)

    # --- kapak izgarasi: her kapak kendi acikligiyla (gozden bagimsiz) ---
    lim = {"U": kin.LID_UP_DMAX, "L": kin.LID_LO_DMIN}
    nominal = {"U": kin.LID_UP_DNOM / kin.LID_UP_DMAX, "L": kin.LID_LO_DNOM / kin.LID_LO_DMIN}
    lid_grid = {k: [] for k in LID_KEYS}
    lid_servo = {k: [] for k in LID_KEYS}
    for i in range(LID_STEPS):
        f = i / (LID_STEPS - 1)
        deltas = {k: f * lim[k[0]] for k in LID_KEYS}
        Ms = E.pose_transforms(0, 0, deltas)
        s = E.servo_angles(0, 0, deltas)
        for k in LID_KEYS:
            lid_grid[k].append({g: m16(rel(Ms[g], M0[g])) for g in (f"lid_{k}", f"link_{k}", f"lidhorn_{k}")})
            lid_servo[k].append(round(s[f"LID_{k}"], 2))

    glb = (CAD / "out" / "glb" / "assembly.glb").read_bytes()
    report = (CAD / "out" / "check_report.txt").read_text(encoding="utf-8")
    sonuc = next((l for l in report.splitlines() if l.startswith("SONUC")), "")

    data = {
        "yaws": YAWS, "pitches": PITCHES, "eye": eye_grid, "servo_eye": servo_eye,
        "lid_steps": LID_STEPS, "lid": lid_grid, "lid_servo": lid_servo,
        "lid_nominal": nominal, "sonuc": sonuc,
    }
    tpl = (HERE / "template.html").read_text(encoding="utf-8")
    html = (tpl.replace("/*__POSE_DATA__*/null", json.dumps(data, separators=(",", ":")))
               .replace("__GLB_BASE64__", base64.b64encode(glb).decode("ascii")))
    out = HERE.parent / "model.html"
    out.write_text(html, encoding="utf-8")
    print(f"model.html yazildi: {out.stat().st_size // 1024} KB (GLB {len(glb) // 1024} KB)")
    print("kontrol:", sonuc)


if __name__ == "__main__":
    main()
