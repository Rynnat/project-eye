"""Project Eye v5 - 3D goruntuleyiciyi (../model.html) uretir.

v2'deki gibi: hareket JS'te yeniden yazilmaz; cad/eye_v5.pose_transforms (yaw, pitch, kapak) 3B izgarada
calistirilir, notre goreli 4x4 donusumler kaydedilir, goruntuleyici uc dogrusal enterpolasyon yapar.
cad/ klasorune YAZMAZ. Model degisince:  ..\..\..\..\.venv-cad\Scripts\python.exe build_viewer.py
"""
import base64, json, os, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
sys.path.insert(0, str(CAD)); os.chdir(CAD)
import eye_v5 as E  # noqa: E402

YAWS = list(range(-25, 26, 5))            # 11
PITCHES = list(range(-20, 21, 5))         # 9
LIDS = [i / 4 for i in range(5)]          # 0 kapali .. 1 acik


def m16(M):
    return [round(float(x), 5) for x in np.asarray(M).flatten()]


def main():
    M0 = E.pose_transforms(0, 0, 0.0)
    groups = [g for g in M0 if g != "static"]
    grid, servo = [], []
    for y in YAWS:
        ry, sy = [], []
        for p in PITCHES:
            rp, sp = [], []
            for l in LIDS:
                Ms = E.pose_transforms(y, p, l)
                rp.append({g: m16(np.asarray(Ms[g]) @ np.linalg.inv(np.asarray(M0[g]))) for g in groups})
                s = E.servo_angles(y, p, l)
                sp.append([round(s["EYE_YAW"], 2), round(s["EYE_PITCH"], 2), round(s["LIDS"], 2)])
            ry.append(rp); sy.append(sp)
        grid.append(ry); servo.append(sy)
    # basilan parcalar (STL anahtari -> parca adlari): goruntuleyicide filament rengi secilebilsin
    P = E.build(verbose=False)
    parcalar = {}
    for n, prt in P.items():
        if prt.kind == "printed":
            parcalar.setdefault(prt.bom_key or n, []).append(n)
    tum = {n: prt.kind for n, prt in P.items()}          # agac (montaj) paneli icin tum parcalar
    # STL basina kati hacim (cm3): proje ilerlemesi hacme gore (iris ile taban ayni agirlikta sayilmaz)
    import trimesh
    hacim = {k: round(float(trimesh.load(CAD / "out" / "stl" / f"{k}.stl").volume) / 1000.0, 2) for k in parcalar}
    glb = (CAD / "out" / "glb" / "assembly.glb").read_bytes()
    rep = (CAD / "out" / "check_report.txt").read_text(encoding="utf-8")
    sonuc = next((l for l in rep.splitlines() if l.startswith("SONUC")), "")
    data = {"yaws": YAWS, "pitches": PITCHES, "lids": LIDS, "grid": grid, "servo": servo, "sonuc": sonuc, "parcalar": parcalar, "tum": tum, "hacim": hacim}
    tpl = (HERE / "template.html").read_text(encoding="utf-8")
    html = (tpl.replace("/*__POSE_DATA__*/null", json.dumps(data, separators=(",", ":")))
               .replace("__GLB_BASE64__", base64.b64encode(glb).decode("ascii")))
    (HERE.parent / "model.html").write_text(html, encoding="utf-8")
    print("model.html:", (HERE.parent / "model.html").stat().st_size // 1024, "KB; gruplar:", groups)
    print(sonuc)


if __name__ == "__main__":
    main()
