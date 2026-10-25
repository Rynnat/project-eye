"""D7: v5 krank ucu somun cebi - kose eti (radyal) ve cep derinligi, geometriden."""
import os, sys, math
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(V6, "cad"))
import numpy as np, cadquery as cq
import eye_v6 as E6, params as P5
E5 = E6.E5
cr = E5.crank_local()
tip = (0.0, -P5.LEVER_R)
ymid = E5.CRANK_NUT_Y + P5.M4_NUT_T / 2          # cep ortasi (yerel y)
print(f"cep: y {E5.CRANK_NUT_Y:.2f}..{E5.CRANK_NUT_Y + P5.M4_NUT_T + P5.NUT_POCKET_DEPTH_CLR:.2f} (derinlik {P5.M4_NUT_T + P5.NUT_POCKET_DEPTH_CLR:.2f}), AF {P5.M4_NUT_AF}+{P5.NUT_POCKET_CLR}")
solid = cr
best = None
for deg in np.arange(0, 360, 1.0):
    # arka kolun (z>0 yonu: merkeze dogru) icinden gecen isinlari at: kol malzemesi kalinligi ayri; yalniz disk disi yon (z<tip) ve yanlar
    d = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
    # x=d0, z=tip_z + d1  ; z yerel = -20 + d1 ; kolun oldugu yon d1>0 (merkeze)
    r_in = None; r_out = None
    for r in np.arange(0.0, 9.0, 0.02):
        p = cq.Vector(d[0] * r, ymid, tip[1] + d[1] * r)
        inside = solid.isInside(p)
        if r_in is None and inside is False and r > 0.5 and False: pass
        if r_in is None and r > 0.5 and inside:
            r_in = r
        if r_in is not None and r_out is None and not inside:
            r_out = r; break
    if r_in is not None and r_out is not None:
        t = r_out - r_in
        if d[1] <= 0.2 and (best is None or t < best[0]):      # kol tarafi (d1>0) haric
            best = (t, deg, r_in, r_out)
print(f"en ince radyal et (cep ortasi duzleminde, kol yonu haric): {best[0]:.2f} mm @ {best[1]:.0f} deg (ic {best[2]:.2f}, dis {best[3]:.2f})")
print(f"hesap: 5.9 - 7.3/sqrt3 = {5.9 - 7.3/math.sqrt(3):.2f} mm; esik MIN_ET 1.6")
print("tork/beyazlama: donanim testi (OLCULEMEZ model ile)")
