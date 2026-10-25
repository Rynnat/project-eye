r"""Project Eye v6 - 3B goruntuleyici (../model.html). v5/viewer/build_viewer.py'den uyarlandi.

Hareket JS'te yeniden yazilmaz: kin_v6.pose (yaw, pitch, kapak) 3B izgarada calistirilir, notre goreli 4x4'ler kaydedilir;
goruntuleyici uc dogrusal enterpolasyon yapar. GLB koku 'G_masa' (egim + masa), gruplar onun cocugu -> butun montaj egik masada.
check_report.txt 'SONUC: TEMIZ' degilse model.html URETILMEZ.
  PYTHONUTF8=1 ..\..\..\..\.venv-cad\Scripts\python.exe -B build_viewer.py
  node test_harness\run.js ..\model.html
"""
import base64
import json
import os
import sys
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
sys.path.insert(0, str(CAD))
import numpy as np  # noqa: E402
import eye_v6 as E6  # noqa: E402
import kin_v6 as K6  # noqa: E402
from params_v6 import TILT_DEG, PITCH_MIN, PITCH_MAX  # noqa: E402

YAWS = list(range(-25, 26, 5))                       # 11
PITCHES = list(range(int(PITCH_MIN), int(PITCH_MAX) + 1, 4))    # -10..18 (8)
LIDS = [i / 4 for i in range(5)]


def m16(M):
    return [round(float(x), 5) for x in np.asarray(M).flatten()]


def main():
    rep = (CAD / "out" / "check_report.txt").read_text(encoding="utf-8")
    sonuc = rep.splitlines()[0]
    if sonuc.strip() != "SONUC: TEMIZ":
        print("check_report TEMIZ degil -> model.html uretilmedi:", sonuc)
        sys.exit(1)
    assert PITCHES[0] == PITCH_MIN and PITCHES[-1] == PITCH_MAX, PITCHES
    M0 = K6.pose(0, 0, 0.0)
    groups = [g for g in M0 if g != "static"]
    grid, servo = [], []
    for y in YAWS:
        ry, sy = [], []
        for p in PITCHES:
            rp, sp = [], []
            for l in LIDS:
                Ms = K6.pose(y, p, l)
                rp.append({g: m16(np.asarray(Ms[g]) @ np.linalg.inv(np.asarray(M0[g]))) for g in groups})
                s = K6.servo_angles(y, p, l)
                sp.append([round(s["EYE_YAW"], 2), round(s["EYE_PITCH"], 2), round(s["LIDS"], 2), round(s["EYE_PITCH_L"], 2)])
            ry.append(rp); sy.append(sp)
        grid.append(ry); servo.append(sy)
    P = E6.build(verbose=False)
    parcalar, bom = {}, {}
    for n, prt in P.items():
        if prt.kind == "printed":
            bom[n] = prt.bom_key or n
            if prt.bom_key in E6.YENI:
                parcalar.setdefault(prt.bom_key, []).append(n)
    tum = {n: prt.kind for n, prt in P.items()}
    import trimesh
    hacim = {k: round(float(trimesh.load(CAD / "out" / "stl" / f"{k}.stl").volume) / 1000.0, 2) for k in parcalar}
    oz = json.loads((CAD / "out" / "check_ozet.json").read_text(encoding="utf-8"))
    rozet = f"pitch {PITCH_MIN:+.0f}..{PITCH_MAX:+.0f} · tork payı (B) {oz['tork']['0.75'][0]:.2f} · masa {TILT_DEG:.0f}°"
    glb = (CAD / "out" / "glb" / "assembly.glb").read_bytes()
    data = {"yaws": YAWS, "pitches": PITCHES, "lids": LIDS, "grid": grid, "servo": servo, "sonuc": sonuc, "rozet": rozet,
            "parcalar": parcalar, "bom": bom, "tum": tum, "hacim": hacim, "tilt": TILT_DEG,
            "eksen_y": round(K6.pitch_ekseni_yukseklik(), 2)}
    tpl = (HERE / "template.html").read_text(encoding="utf-8")
    html = (tpl.replace("/*__POSE_DATA__*/null", json.dumps(data, separators=(",", ":")))
               .replace("__GLB_BASE64__", base64.b64encode(glb).decode("ascii")))
    (HERE.parent / "model.html").write_text(html, encoding="utf-8")
    print("model.html:", (HERE.parent / "model.html").stat().st_size // 1024, "KB; gruplar:", groups)
    print(sonuc, "|", rozet)


if __name__ == "__main__":
    main()
