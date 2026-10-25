"""Project Eye v6 - v5 montajini IMPORT eder (v5/cad/eye_v5.build), yalniz yeni parcalari ve yeni vida yerlesimini ekler.

  PYTHONUTF8=1 python -B eye_v6.py                         -> out/stl (YALNIZ yeni basilacaklar), out/step, out/glb, rig.json
  PYTHONUTF8=1 python -B eye_v6.py --only kama              -> yalniz o parca(lar) + parca kontrolleri (out/parca_<ad>.txt)
     parcalar: kama, sol_servo_braketi, sol_kol_adaptoru  (virgulle birden fazla)

v5 klasorune YAZMAZ (sys.dont_write_bytecode; disa aktarma v6/cad/out). v5 parcalari degismez (check: dondurma).
Degisenler (PLAN §2, §11 SECENEK B):
  - pivot_bracket + sol pitch pimi (scr_fpivot/nut_fpivot) CIKAR  -> sol_servo_braketi + SG90 (D9) + sol_kol_adaptoru
  - krank ucu vidasi: v5 M4x10 yuvarlak ALTTAN -> M4x16 havsa USTTEN, kranktaki somuna kafa-somun kilidi + alt emniyet somunu
  - frame_L: montaj modelinde 2x O1.6 MATKAP deligi, seritten delip gecer (STL ayni; MONTAJ adim A0)
  - pitch servosu (sag) MG90S (govde modeli SG90 ile ayni [TAHMIN], PLAN R12)
  - kama x2 (ayna), kaymaz ayak x4, masa (gorsel)
"""
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
V5CAD = os.path.normpath(os.path.join(HERE, "..", "..", "v5", "cad"))
AGENT_FLOW = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "agent-flow"))
for p in (AGENT_FLOW, V5CAD, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import cadquery as cq

import eye_v5 as E5                      # v5 geometri (DONDURULMUS)
from eye_v5 import Part, box, rbox, cyl, U, D, I, xform, frame_M, unit, poly_yz, hole
import params as P5                      # v5 olculeri
from params_v6 import *
import kin_v6 as K6
from sablonlar import sg90_cebi, somun_tuzagi, serbest_delik, pahli_delik
from sablonlar.ayar import ayarli_somun_yuvasi

OUT = os.path.join(HERE, "out")
YENI = ("kama", "sol_servo_braketi", "sol_kol_adaptoru")      # v6'da basilacak STL'ler
CIKAN = ("pivot_bracket", "scr_fpivot", "nut_fpivot", "scr_ctip")
C6 = dict(kama="#3b4a3f", braket="#2e3438", adaptor="#5b646b", ayak="#202020", masa="#8b6b4a", metal="#8a9096",
          servo="#3a64c8", mg="#24408f", horn="#f4f4f4", nut="#b8bcc0")

FASTENERS6 = []        # (vida, somun, tip, uc tasmasi mm) - v5 FASTENERS'in v6 hali


# =============================================================================================== yardimcilar
def wp_shape(wp):
    """cq.Workplane -> tek Shape (birlesik)."""
    vals = [v for v in wp.vals() if isinstance(v, cq.Shape)]
    return vals[0] if len(vals) == 1 else cq.Compound.makeCompound(vals)


def yerlestir(wp, origin, zdir, xdir):
    """Sablon kesicisi (yerel +Z eksen, z=0 tabla yuzu) -> dunya: yerel x -> xdir, z -> zdir, origin."""
    z = unit(zdir); x = unit(np.asarray(xdir, float) - z * np.dot(xdir, z)); y = np.cross(z, x)
    M = np.eye(4); M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z; M[:3, 3] = origin
    return xform(wp_shape(wp), M)


def screw_csk_iso(nut_face, axis, L=16.0, k=M4_CSK_K):
    """M4 havsa, KAFA ALTI (O4 noktasi) nut_face'te: kafa 90 deg koni O4 -> O(4+2k) (k = ISO 10642 maks), axis = kafadan uca.
    L toplam boy (kafa dahil). Govde kok capta (v5 M4_SHANK_MODEL)."""
    a = unit(axis); p = np.asarray(nut_face, float)
    top = p - a * k
    head = E5.cone(top, p, 2.0 + k, 2.0)
    return U(head, cyl(p - a * 0.01, top + a * L, P5.M4_SHANK_MODEL / 2)), top + a * L


def nut_m2(p0, axis, vdir):
    a = unit(axis); p0 = np.asarray(p0, float)
    return D(E5.hex_prism(p0, a, vdir, M2_NUT_AF, M2_NUT_T), cyl(p0 - a * 0.1, p0 + a * (M2_NUT_T + 0.1), 1.0))


def screw_m2(head_face, axis, L=8.0):
    return E5.screw_round(head_face, axis, L, M2_SHANK_MODEL, M2_HEAD_D, M2_HEAD_H)


# =============================================================================================== geometri sabitleri (v5'ten turetilir)
X_PLAKA_L = -(P5.EYE_X + P5.PLATE_X[1])                 # -80.4 frame_L dis plaka yuzu (serit)
X_GOBEK_UC = -(P5.EYE_X + P5.PITCH_PIN_BOSS_X[1])        # -88.5 sol gobek ucu
X_ADP_FLANS = X_PLAKA_L - ADP_FLANS_T                    # -83.4
X_ADP_IC = X_GOBEK_UC - ADP_EKSENEL                      # -88.9 uc duvari ic yuzu
X_ADP_UC = X_ADP_IC - ADP_UC_T                           # -92.9 kol oturma yuzu
X_SOL_MIL = X_ADP_UC - P5.HORN_T                         # -94.4 sol SG90 mil ucu
SOL_SRV = E5.Servo((X_SOL_MIL, 0.0, 0.0), (1, 0, 0), (0, -1, 0))   # mil +X (cerceveye), govde boyu -Y (sag servo aynasi)
X_KULAK_ALT = SOL_SRV.base[0] + P5.SG_EAR_Z0             # -108.3 kulak alt yuzu (plakaya oturur)
X_PLAKA_DIS = X_KULAK_ALT - BRK_PLAKA_T                  # -111.8


def sol_servo_M():
    """sablonlar SG90 yerel (x boy, z mil, govde alti z=0) -> dunya."""
    ld = SOL_SRV.ld; sd = SOL_SRV.sd; n = np.cross(sd, ld)
    M = np.eye(4); M[:3, 0] = ld; M[:3, 1] = n; M[:3, 2] = sd; M[:3, 3] = SOL_SRV.base
    return M


# =============================================================================================== PARCA: kama
def make_kama(alfa=TILT_DEG, duvar=None, deri=None, nervur=None, nervur_z=None):
    """Sol yari (x dis -> orta), mekanizma koordinatinda. Sag yari = x = KAMA_ORTA_X'e gore ayna (ayni STL).
    duvar/deri/nervur/nervur_z: None -> params_v6 (hafiflik kontrolu MIN_ET varyantini bununla kurar)."""
    duvar = KAMA_DUVAR if duvar is None else duvar
    deri = KAMA_DIS_DERI if deri is None else deri
    nervur = KAMA_NERVUR if nervur is None else nervur
    nervur_z = KAMA_NERVUR_Z if nervur_z is None else nervur_z
    kp = K6.kama_profil(alfa)
    x_dis = P5.BASE_X[0] - RAY_PAY - RAY_T                       # -104.3
    x_ic = KAMA_ORTA_X - KAMA_ORTA_BOSLUK / 2                    # 5.25
    body = poly_yz(kp["profil"], x_dis, x_ic)
    # cepler: govde profilinin KAMA_DUVAR ice ofseti - nervurler; dis deri KAMA_DIS_DERI, ortaya (ic yuze) acik
    w = cq.Workplane("YZ", origin=(x_dis + deri, 0, 0)).polyline(kp["govde"]).close().offset2D(-duvar)
    cep = w.extrude(x_ic - x_dis - deri + 1.0).val()
    nerv = [box(x_dis, x_ic + 2, -200, 0, zr - nervur / 2, zr + nervur / 2) for zr in nervur_z]
    cep = D(cep, *nerv)
    body = D(body, cep)
    # ray: dis duvar (taban yan yuzunden RAY_PAY) + dudak (taban ustunden RAY_PAY), arka bolge
    z0 = kp["z_arka"]; z1 = RAY_Z[1]
    ray = U(box(x_dis, P5.BASE_X[0] - RAY_PAY, K6.BASE_ALT_Y - 1.0, RAY_DUDAK_UST, z0, z1),
            box(P5.BASE_X[0] - RAY_PAY - 0.01, P5.BASE_X[0] + RAY_DUDAK_IC, P5.BASE_Y[1] + RAY_PAY, RAY_DUDAK_UST, z0, z1))
    body = U(body, ray)
    # kablo kertigi: arka dudak orta bolgede kaldirilir (y -60'a kadar)
    kx0 = KAMA_ORTA_X - KERTIK_X
    body = D(body, box(kx0, x_ic + 1, K6.BASE_ALT_Y, KAMA_DUDAK_UST + 1, z0 - 1, K6.BASE_ARKA_Z))
    # kaymaz ayak izleri (alt yuzde O10 x 0.5)
    n3 = np.array([0.0, kp["n"][0], kp["n"][1]])
    izler = []
    for (yy, zz) in kp["ayak"]:
        for xa in (x_dis + AYAK_X_KENAR,):
            c = np.array([xa, yy, zz])
            izler.append(cyl(c - n3 * 0.2, c + n3 * AYAK_IZ, AYAK_D / 2 + 0.2))
    body = D(body, *izler)
    return body


def kama_ayaklari(alfa=TILT_DEG):
    """4 kaymaz ayak (satin alinan) - sol yarinin 2'si + aynasi. Doner [(ad, shape)]."""
    kp = K6.kama_profil(alfa)
    n3 = np.array([0.0, kp["n"][0], kp["n"][1]])
    x_dis = P5.BASE_X[0] - RAY_PAY - RAY_T
    out = []
    for i, (yy, zz) in enumerate(kp["ayak"]):
        for side, xa in (("L", x_dis + AYAK_X_KENAR), ("R", 2 * KAMA_ORTA_X - (x_dis + AYAK_X_KENAR))):
            c = np.array([xa, yy, zz])
            out.append((f"ayak_{side}{'arka' if i == 0 else 'on'}", cyl(c + n3 * AYAK_IZ, c - n3 * (AYAK_T - AYAK_IZ), AYAK_D / 2)))
    return out


def ayna_x(shape, x0=KAMA_ORTA_X):
    M = np.eye(4); M[0, 0] = -1; M[0, 3] = 2 * x0
    return xform(shape, M)


# =============================================================================================== PARCA: sol_kol_adaptoru
def make_sol_kol_adaptoru():
    yb, yt = ADP_FLANS_Y; zs0, zs1 = P5.STRIP_Z
    flans = rbox(X_ADP_FLANS, X_PLAKA_L, yb, yt, zs0, zs1, r=1.0)
    r_ic = P5.PLATE_DISK_R + GECME_PAY                   # 6.7
    kap = cyl((X_PLAKA_L, 0, 0), (X_ADP_IC, 0, 0), r_ic + ADP_KAP_DUVAR)        # temas yuzunden baslar (tablada)
    uc = cyl((X_ADP_IC + 0.01, 0, 0), (X_ADP_UC, 0, 0), ADP_UC_R)
    body = U(flans, kap, uc)
    # D-delik: gobek (r 6.5, z <= FRONT_Z) + 0.2 pay; uc duvarina kadar
    bore = I(cyl((X_PLAKA_L + 1.0, 0, 0), (X_ADP_IC, 0, 0), r_ic),
             box(X_PLAKA_L + 1.0, X_ADP_IC - 0.01, -20, 20, -20, P5.FRONT_Z + GECME_PAY))
    kes = [bore]
    for yv in ADP_VIDA_Y:      # M2 gecis (baskida dikey: flans yuzu tablada) - sablon serbest_delik("M2")
        kes.append(yerlestir(serbest_delik("M2", ADP_FLANS_T + 0.2), (X_PLAKA_L, yv, ADP_VIDA_Z), (-1, 0, 0), (0, 1, 0)))
    for sg in (-1, 1):         # kol sac vidasi pilotlari r = 6 (v5 HORN_SCREW_R, kol y yonunde)
        kes.append(hole((X_ADP_UC, sg * P5.HORN_SCREW_R, 0), (1, 0, 0), KOL_VIDA_DERIN, M2_PILOT))
    return D(body, *kes)


# =============================================================================================== PARCA: sol_servo_braketi
BRK_SOMUN_Y = BRK_AYAK_Y[1] - (P5.M4_NUT_T + P5.NUT_POCKET_DEPTH_CLR)    # -50.4 somun yuk yuzu (cep tabani)


def make_sol_servo_braketi():
    ay0, ay1 = BRK_AYAK_Y
    plaka = rbox(X_PLAKA_DIS, X_KULAK_ALT, ay0, BRK_PLAKA_Y1, -BRK_PLAKA_Z, BRK_PLAKA_Z, r=1.0)
    ayak = rbox(X_PLAKA_DIS, BRK_AYAK_X[1], ay0, ay1, *BRK_AYAK_Z, r=1.0)
    nerv = []
    for z0, z1 in ((BRK_NERVUR_Z[0], BRK_NERVUR_Z[1]), (-BRK_NERVUR_Z[1], -BRK_NERVUR_Z[0])):
        tri = cq.Workplane("XY", origin=(0, 0, z0)).polyline(
            [(X_KULAK_ALT - 0.5, ay1 - 0.5), (X_KULAK_ALT - 0.5, BRK_NERVUR_UST_Y), (BRK_NERVUR_X1, ay1 - 0.5)]).close().extrude(z1 - z0).val()
        nerv.append(tri)
    body = U(plaka, ayak, *nerv)
    kes = []
    # servo cebi (sablonlar.sg90_cebi: 22.9 x 12.7, kablo yarigi MILE YAKIN uc) - plakanin icinden gecer
    kes.append(xform(wp_shape(sg90_cebi(derinlik=P5.SG_H + 0.01, kulak_delikleri=None)), sol_servo_M()))
    # uzak kulak: M2 gecis (baskida YATAY +0.2) - plaka boyunca
    e_lo, e_hi = SOL_SRV.ear_holes()[1]
    kes.append(yerlestir(serbest_delik("M2", BRK_PLAKA_T + 0.4, yatay=True), (X_KULAK_ALT + 0.2, e_lo[1], e_lo[2]), (-1, 0, 0), (0, 0, 1)))
    # ayak: alttan M4 havsa (tabandaki v5 havsalari) -> x yonunde +-BRK_AYAR yarik, somun USTTEN acik cepte
    for zc in BRK_VIDA_Z:      # sablonlar.ayar.ayarli_somun_yuvasi [TASLAK]: +-BRK_AYAR yarik + ustten acik uzun somun cebi
        kes.append(yerlestir(ayarli_somun_yuvasi("M4", BRK_AYAR, derinlik=BRK_SOMUN_Y - ay0), (BRK_VIDA_X, ay0, zc), (0, 1, 0), (1, 0, 0)))
    return D(body, *kes)




# =============================================================================================== build
def yeni_parca(ad, alfa=TILT_DEG):
    if ad == "kama":
        return make_kama(alfa)
    if ad == "sol_servo_braketi":
        return make_sol_servo_braketi()
    if ad == "sol_kol_adaptoru":
        return make_sol_kol_adaptoru()
    raise KeyError(ad)


PR = dict(kama=E5.PR_XUP, sol_servo_braketi=E5.PR_YUP, sol_kol_adaptoru=E5.PR_XDOWN)
NOT = dict(kama="DIS yuz (x -104.3) tablada; profil x boyunca prizmatik (desteksiz); SAG yari icin dilimleyicide X AYNALA",
           sol_servo_braketi="ayak alti (y -54) tablada; plaka dik; servo cebi penceresi kopru; kulak deligi yatay (+0.2)",
           sol_kol_adaptoru="flans temas yuzu (x -80.4) tablada; D-delik ust duvarla kopru")
RENK = dict(kama=C6["kama"], sol_servo_braketi=C6["braket"], sol_kol_adaptoru=C6["adaptor"])


def build(verbose=True, alfa=TILT_DEG):
    """{ad: Part}: v5 montaji (CIKAN'lar haric) + v6 yenileri. Parcalar NOTR pozda, MEKANIZMA koordinatinda."""
    def log(*a):
        if verbose:
            print(*a, flush=True)
    log("v5 import (eye_v5.build)...")
    P = E5.build(verbose=False)
    FASTENERS6.clear()
    for scr, nut, typ, tip in E5.FASTENERS:
        if scr not in CIKAN and nut not in ("nut_fpivot", "nut_brk_-4", "nut_brk_24"):
            FASTENERS6.append((scr, nut, typ, tip))
    v5_orijinal = {n: P[n] for n in P}
    for n in CIKAN:
        P.pop(n)

    def add(name, shape, group, color, **kw):
        P[name] = Part(name, shape, group, color, **kw)

    def add_fast(scr, nut, typ, tip, nut_p0, axis, nut_t):
        a = unit(axis)
        FASTENERS6.append((scr, nut, typ, float(np.dot(np.asarray(tip) - (np.asarray(nut_p0) + a * nut_t), a))))

    # ---------------------------------------------------------- krank ucu: M4x16 havsa USTTEN + alt emniyet somunu
    log("krank ucu vidasi...")
    ytop = P5.LEVER_Y[1]; cy0 = P5.COUPLER_Y[0]
    ctip = (0.0, ytop + E5.CRANK_NUT_Y + P5.M4_NUT_T, P5.YAW_SERVO_Z - P5.LEVER_R)   # kranktaki somunun UST yuzu (y -12.1)
    scr, tip = screw_csk_iso(ctip, (0, -1, 0), 16.0)
    add("scr_ctip6", scr, "yawcrank", C6["metal"], kind="hardware", bom_key="M4x16 havsa",
        note="USTTEN; kafa kranktaki somuna oturur ve sikilir (kafa-somun kilidi); vida sabitleyici")
    nut_lo_y = cy0 - M4_JAM_GAP                                          # -21.0 alt somunun ust (yuk) yuzu
    add("nut_ctip_alt", E5.nut_shape((0, nut_lo_y, ctip[2]), (0, -1, 0), (1, 0, 0), "M4"), "yawcrank", C6["nut"],
        kind="hardware", bom_key="M4 somun", note="emniyet somunu: coupler'a 0.2 (1/4 tur geri), vida sabitleyici")
    add_fast("scr_ctip6", "nut_ctip_alt", "M4", tip, (0, nut_lo_y, ctip[2]), (0, -1, 0), P5.M4_NUT_T)
    add_fast("scr_ctip6", "nut_ctip", "M4", tip, (0, ctip[1], ctip[2]), (0, -1, 0), P5.M4_NUT_T)

    # ---------------------------------------------------------- frame_L: montaj modelinde 2x O1.6 matkap deligi
    pil = [hole((X_PLAKA_L, yv, ADP_VIDA_Z), (1, 0, 0), ADP_PILOT_DERIN, M2_PILOT) for yv in ADP_VIDA_Y]
    fl = v5_orijinal["frame_L"]
    add("frame_L", D(fl.shape, *pil), fl.group, fl.color, kind="printed", material=fl.material, bom_key="frame_L",
        print_R=fl.print_R, note="v5 STL AYNEN; montajda 2x O1.6 matkap, adaptor kilavuzuyla seritten DELIP GECER (MONTAJ A0)", horizontal_holes=fl.horizontal_holes)
    P["frame_L"].v5_shape = fl.shape                                    # dondurma kontrolu matkapsiz sekille karsilastirir

    # ---------------------------------------------------------- sag pitch servosu MG90S (govde SG90 modeli)
    P["servo_pitch"].bom_key = "MG90S"
    P["servo_pitch"].color = C6["mg"]
    P["servo_pitch"].note = "MG90S [TAHMIN: govde SG90 modeli, PLAN R12 kumpas]"

    # ---------------------------------------------------------- yeni basilan parcalar
    log("sol_kol_adaptoru...")
    add("sol_kol_adaptoru", make_sol_kol_adaptoru(), "frame", RENK["sol_kol_adaptoru"], material="PLA",
        bom_key="sol_kol_adaptoru", print_R=PR["sol_kol_adaptoru"], note=NOT["sol_kol_adaptoru"])
    for i, yv in enumerate(ADP_VIDA_Y):
        hf = (X_ADP_FLANS, yv, ADP_VIDA_Z)
        add(f"scr_adp{i}", screw_m2(hf, (1, 0, 0), 8.0), "frame", C6["metal"], kind="hardware", bom_key="M2x8 mercimek",
            note="frame_L seridindeki O1.6 matkap pilotuna kilavuzsuz")
    add("horn_pitch_L", SOL_SRV.horn((0, 1, 0)), "frame", C6["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    f = SOL_SRV.horn_face()
    for i, sg in enumerate((-1, 1)):
        p = f + np.array([0, 1.0, 0]) * sg * P5.HORN_SCREW_R
        add(f"scr_horn_pitchL{i}", E5.screw_round(p - SOL_SRV.sd * P5.HORN_T, SOL_SRV.sd, 5.0, 1.2, 3.0, 1.2), "frame",
            C6["metal"], kind="hardware", bom_key="SG90 kol vidasi (servo ile gelir)")

    log("sol_servo_braketi + SG90 (D9)...")
    add("sol_servo_braketi", make_sol_servo_braketi(), "static", RENK["sol_servo_braketi"], material="PLA/PETG",
        bom_key="sol_servo_braketi", print_R=PR["sol_servo_braketi"], note=NOT["sol_servo_braketi"],
        horizontal_holes=["sol servo uzak kulak M2 (X)"])
    add("servo_pitch_L", SOL_SRV.body(), "static", C6["servo"], kind="purchased", bom_key="SG90")
    add("cable_pitch_L", SOL_SRV.cable(), "static", "#8a5a2b", kind="purchased", bom_key="SG90 kablo cikisi")
    e_lo, e_hi = SOL_SRV.ear_holes()[1]
    add("scr_m2_pitchL", screw_m2(e_hi, (-1, 0, 0), 8.0), "static", C6["metal"], kind="hardware", bom_key="M2x8 mercimek")
    nut_p0 = (X_PLAKA_DIS, e_lo[1], e_lo[2])
    add("nut_m2_pitchL", nut_m2(nut_p0, (-1, 0, 0), (0, 0, 1)), "static", C6["nut"], kind="hardware", bom_key="M2 somun")
    add_fast("scr_m2_pitchL", "nut_m2_pitchL", "M2", np.asarray(e_hi) + np.array([-8.0, 0, 0]), nut_p0, (-1, 0, 0), M2_NUT_T)
    # braket ayagi somunlari (v5 scr_brk_* AYNI vidalar, tabandaki v5 havsalari); somun ustten acik cepte
    for sz in BRK_VIDA_Z:
        nm = f"nut_brk_{int(sz) + 10}"
        p0 = (BRK_VIDA_X, BRK_SOMUN_Y, sz)
        add(nm, E5.nut_shape(p0, (0, 1, 0), (1, 0, 0), "M4"), "static", C6["nut"], kind="hardware", bom_key="M4 somun")
        scr_tip = (BRK_VIDA_X, P5.BASE_Y[0] + (P5.M4_CSK_D - P5.M4_CSK_HEAD_D) / 2 + 16.0, sz)
        add_fast(f"scr_brk_{int(sz) + 10}", nm, "M4", scr_tip, p0, (0, 1, 0), P5.M4_NUT_T)

    log("kama x2 + ayaklar...")
    kL = make_kama(alfa)
    add("kama_L", kL, "static", RENK["kama"], material="PLA", bom_key="kama", print_R=PR["kama"], note=NOT["kama"])
    add("kama_R", ayna_x(kL), "static", RENK["kama"], material="PLA", bom_key="kama", print_R=PR["kama"],
        note="kama STL'i X'te AYNALANMIS (dilimleyici)")
    for nm, sh in kama_ayaklari(alfa):
        add(nm, sh, "static", C6["ayak"], kind="purchased", bom_key="kaymaz ayak O10")
    # masa (gorsel): masa yuzu z = 0 alti 4 mm levha, mekanizma koordinatina geri tasinir
    Mm = K6.masa_M(alfa)
    masa = xform(box(-180, 180, -170, 140, -4.0, 0.0), np.linalg.inv(Mm))
    add("masa", masa, "static", C6["masa"], kind="visual", bom_key="masa (gorsel)")
    return P


def v5_orijinal_parcalar():
    """dondurma kontrolu icin: v5 build'in DEGISTIRILMEMIS hali."""
    return E5.build(verbose=False)


# =============================================================================================== disa aktarma
def parca_kontrol(ad, part, yaz=True):
    """Parca biter bitmez: tek govde, havada ada, et, tabla, sarkma (baski yonunde)."""
    from checks import rapor_yaz
    from checks.havada import kontrol_tek_govde, kontrol_ada
    from checks.tabla import kontrol_tabla, kontrol_et
    from checks.sarkma import kontrol_sarkma
    m = E5.print_mesh(part)
    g = {ad: m}
    S = [kontrol_tek_govde(g), kontrol_ada(g), kontrol_et(g, min_et=MIN_ET), kontrol_tabla(g), kontrol_sarkma(g)]
    if yaz:
        os.makedirs(OUT, exist_ok=True)
        rapor_yaz(S, os.path.join(OUT, f"parca_{ad}.txt"), baslik=f"PARCA KONTROLU: {ad} (baski yonunde)")
    return S


def export_yeni(P, adlar=YENI):
    import trimesh  # noqa
    for d_ in ("stl", "step"):
        os.makedirs(os.path.join(OUT, d_), exist_ok=True)
    out = {}
    for key in adlar:
        p = next(pp for pp in P.values() if pp.kind == "printed" and pp.bom_key == key)
        p.shape.exportStep(os.path.join(OUT, "step", f"{key}.step"))
        m = E5.print_mesh(p)
        fn = os.path.join(OUT, "stl", f"{key}.stl")
        m.export(fn)
        out[key] = fn
    return out


def export_glb(P, alfa=TILT_DEG, tol=0.03):
    """assembly.glb: kok 'G_masa' (egim + masa), altinda v5 ile ayni 'G_<grup>' dugumleri (mekanizma koordinati)."""
    import trimesh
    os.makedirs(os.path.join(OUT, "glb"), exist_ok=True)
    scene = trimesh.Scene()
    Rt = K6.K5.R(K6.EX, -alfa)                     # Y-yukari goruntuleyici: yalniz egim (+ masa y=0)
    Tm = Rt.copy(); Tm[1, 3] = -float(K6.kama_profil(alfa)["d_masa"])
    scene.graph.update(frame_to="G_masa", frame_from=scene.graph.base_frame, matrix=Tm)
    groups = {}
    for name, p in P.items():
        groups.setdefault(p.group, []).append(name)
    for g, names in groups.items():
        scene.graph.update(frame_to="G_" + g, frame_from="G_masa", matrix=np.eye(4))
        for n in names:
            p = P[n]
            v, t = E5.tess(p.shape, tol, 0.25)
            m = trimesh.Trimesh(v, t, process=False)
            rgb = [int(p.color[i:i + 2], 16) for i in (1, 3, 5)]
            m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[*rgb, 255], metallicFactor=0.6 if p.kind == "hardware" else 0.05, roughnessFactor=0.5, name=n))
            scene.add_geometry(m, node_name=n, geom_name=n, parent_node_name="G_" + g)
    scene.export(os.path.join(OUT, "glb", "assembly.glb"))
    rig = E5.rig_json({n: p for n, p in P.items()})
    rig["_v6"] = {"masa": "G_masa = R(X, -TILT_DEG) + masa y=0; tum G_<grup> dugumleri onun cocugu (mekanizma koordinati)",
                  "TILT_DEG": alfa, "pitch_aralik": [PITCH_MIN, PITCH_MAX], "sol_pitch_servo": "EYE_PITCH_L = 180 - EYE_PITCH (D9)"}
    rig["frame"]["range"] = [PITCH_MIN, PITCH_MAX]
    with open(os.path.join(OUT, "glb", "rig.json"), "w", encoding="utf-8") as f:
        json.dump(rig, f, indent=1, ensure_ascii=False)


def main():
    import time
    t0 = time.time()
    only = None
    if "--only" in sys.argv:
        only = sys.argv[sys.argv.index("--only") + 1].split(",")
    if only:
        for ad in only:
            sh = yeni_parca(ad)
            part = Part(ad, sh, "static", RENK[ad], print_R=PR[ad], bom_key=ad)
            S = parca_kontrol(ad, part)
            export_yeni({ad: part}, (ad,))
            print(f"{ad}: " + ", ".join(f"{s.ad} {'GECTI' if s.gecti else 'SORUN'}" for s in S))
            for s in S:
                for b in s.bulgular:
                    print("   -", b)
                for t in s.notlar[:3]:
                    print("   .", t)
        print(f"bitti {time.time() - t0:.1f}s")
        return
    P = build()
    bad = [n for n, p in P.items() if p.kind != "visual" and not p.shape.isValid()]
    multi = [n for n, p in P.items() if p.kind == "printed" and len(p.shape.Solids()) != 1]
    print(f"{len(P)} parca; gecersiz {bad}; tek kati olmayan basili {multi}")
    for ad in YENI:
        S = parca_kontrol(ad, next(p for p in P.values() if p.bom_key == ad and p.kind == "printed"))
        print(f"{ad}: " + ", ".join(f"{s.ad} {'GECTI' if s.gecti else 'SORUN'}" for s in S))
    export_yeni(P)
    export_glb(P)
    print(f"bitti {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
