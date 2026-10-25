"""D4: kama hafifletme - hacim/kutle + devrilme (R5 alfa30 yontemiyle, ayni fonksiyonlar)."""
import os, sys
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.normpath(os.path.join(HERE, "..", "cad"))
sys.path.insert(0, CAD)
import numpy as np
import eye_v6 as E6, kin_v6 as K6, analiz as AN, params as P5
from params_v6 import *
from checks.devrilme import kontrol_devrilme, destek_disk

def vol(sh): return sh.Volume()

def hacim(duvar, deri, nerv):
    E6.KAMA_DUVAR, E6.KAMA_DIS_DERI, E6.KAMA_NERVUR_Z = duvar, deri, nerv
    return E6.make_kama()

orig = (KAMA_DUVAR, KAMA_DIS_DERI, KAMA_NERVUR_Z)
print("== hacim (sol yari)")
res = {}
for ad, args in (("mevcut 2.4/2.0/3nerv", orig), ("1.6/1.6/1nerv", (1.6, 1.6, (-15.0,))), ("1.2/1.2/1nerv", (1.2, 1.2, (-15.0,)))):
    v = vol(hacim(*args)); res[ad] = v
    print(f"{ad}: {v:.0f} mm3 = {v*PLA_RHO:.1f} g (dolgu 1.0)")
print("== devrilme alfa30")
E6.KAMA_DUVAR, E6.KAMA_DIS_DERI, E6.KAMA_NERVUR_Z = orig
P = E6.build(verbose=False)
allp = {n: p for n, p in P.items() if p.kind != "visual"}

def devril(kL, etiket):
    PP = dict(allp)
    PP["kama_L"] = E6.Part("kama_L", kL, "static", "#000", bom_key="kama", print_R=None)
    PP["kama_R"] = E6.Part("kama_R", E6.ayna_x(kL), "static", "#000", bom_key="kama", print_R=None)
    a = TILT_DEG; Mma = K6.masa_M(a); pz = {}
    for p in (PITCH_MIN - PITCH_ASIM, PITCH_MIN, 0.0, PITCH_MAX, PITCH_MAX + PITCH_ASIM):
        for l in (0.0, 1.0):
            Ms = K6.pose(0, p, l)
            kl = AN.kutleler(PP, DOLGU_ESAS, M={g: Mma @ M for g, M in Ms.items()})
            pz[f"p{p:+.0f}_k{l:.0f}"] = [(m, c) for _, m, c, _ in kl]
    kp = K6.kama_profil(a); x_dis = P5.BASE_X[0] - RAY_PAY - RAY_T; dest = []
    for (yy, zz) in kp["ayak"]:
        for xa in (x_dis + AYAK_X_KENAR, 2 * KAMA_ORTA_X - (x_dis + AYAK_X_KENAR)):
            c = Mma @ np.array([xa, yy, zz, 1.0]); dest += destek_disk(c[:2], AYAK_D / 2)
    s = kontrol_devrilme(pz, dest, DEVRILME_MESAFE, DEVRILME_ACI, ad="x", etiket=etiket)
    en = [l for l in s.notlar if "EN KOTU" in l] if hasattr(s, "notlar") else []
    print(etiket, en or s.notlar[:1], "gecti" if s.gecti else "KALDI")
    return s

devril(E6.make_kama(), "mevcut ")
for ad, args in (("1.6/1.6/1", (1.6, 1.6, (-15.0,))), ("1.2/1.2/1", (1.2, 1.2, (-15.0,)))):
    kL = hacim(*args)
    devril(kL, ad + " ")
print("ESIK: hafif kama <=55 g/yari ve devrilme >=25 mm / >=24 deg (denetim beklentisi)")
