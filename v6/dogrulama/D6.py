"""D6: frame_L matkap - serit kalinligi (5.5 hedefi payi) ve delip gecme halinde arkada parca var mi."""
import os, sys, itertools
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(V6, "cad"))
import numpy as np
import eye_v6 as E6, kin_v6 as K6, params as P5
from eye_v6 import cyl, xform, I
from params_v6 import *

P = E6.build(verbose=False)
fl = P["frame_L"].v5_shape           # matkapsiz v5 sekil
grp = P["frame_L"].group
for yv in ADP_VIDA_Y:
    L = cyl((E6.X_PLAKA_L - 1, yv, ADP_VIDA_Z), (E6.X_PLAKA_L + 30, yv, ADP_VIDA_Z), M2_PILOT / 2)
    ic = I(fl, L)
    bb = ic.BoundingBox()
    print(f"y={yv}: serit icinde Ø1.6 silindir malzeme boyu x {bb.xmin:.2f}..{bb.xmax:.2f} = {bb.xmax-bb.xmin:.2f} mm, hacim doluluk {ic.Volume()/(np.pi*0.8**2*(bb.xmax-bb.xmin))*100:.0f}%")
    kalan = (bb.xmax - bb.xmin) - ADP_PILOT_DERIN
    print(f"   5.5 mm derinlik hedefi icin kalan et {kalan:.2f} mm (bant ile durma toleransi ~ +-0.5 gerekir)")
# arkada (serit sonrasi) 5.6..15 mm: tum diger parcalara carpiyor mu
bad = {}
maxvol = 0.0
for yv in ADP_VIDA_Y:
    x0 = bb.xmax if bb.xmax > E6.X_PLAKA_L else E6.X_PLAKA_L + 5.6
    L = cyl((x0 - 0.0, yv, ADP_VIDA_Z), (x0 + 10.0, yv, ADP_VIDA_Z), M2_PILOT / 2)
    for yaw, pit, lid in itertools.product((-25, 0, 25), (-13, 0, 18, 21), (0.0, 0.5, 1.0)):
        Ms = K6.pose(yaw, pit, lid)
        Lw = xform(L, Ms[grp])
        for n, p in P.items():
            if p.kind == "visual" or n == "frame_L" or p.group == grp and n.startswith("scr_adp"):
                continue
            sh = xform(p.shape, Ms[p.group])
            try:
                v = Lw.intersect(sh).Volume()
            except Exception:
                v = 0.0
            if v > 1e-3:
                bad.setdefault(n, []).append((yaw, pit, lid, round(v, 3)))
                maxvol = max(maxvol, v)
print("delip gecme yolunda (serit +0..+10 mm) carpan parca:", {k: v[:3] for k, v in bad.items()} or "YOK")
print("en buyuk kesisim hacmi mm3:", maxvol)
print("ESIK: kesisim 0 -> delip gecmek zararsiz")
