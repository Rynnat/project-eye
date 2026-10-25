"""Project Eye v6 - tum kontroller (agent-flow/checks) -> out/check_report.txt (1. satir SONUC:, sonra KAPSAM:).

  PYTHONUTF8=1 python -B check.py            # tam (54 poz)
  PYTHONUTF8=1 python -B check.py --quick    # 12 poz (gelistirme)
Kontroller (PLAN §7 matrisi):
  R16 dondurma (v5 STL md5 + v6'daki v5 parcalari = v5 STL) | R9 yeni parcalar (tek govde, ada, et, tabla, sarkma) |
  R3 cakisma: rijit (notr) + hareketli gruplar yaw x pitch x kapak, MASA EGIK (TILT_DEG) | R4 zemin (masa) | R8 havada donanim |
  R6 montaj-sokme (cerceve alt montaji + ana montaj) + giris yolu (alet) | R10 tolerans Z1-B, Z2, Z3, Z5..Z8 |
  R7 vida boyu/envanter/adet | R1/R2 yercekimi torku (SECENEK B) + dolgu taramasi | R5 devrilme (alfa 20/30/40) |
  R17 kablo | tutma testi esigi (R11) | PLAN sapmasi etiketi | KAPSAM notlari (olculemeyenler -> kullanici)
  Duzeltme turu 1 (denetim D2/D4/D6/D7/D8): kama hafiflik | MONTAJ kalibrasyon sirasi | sikma talimati sayisal |
  frame_L matkap (kor delik payi / delip gecme arkasi) | BOM_FARK <-> envanter
"""
import json
import math
import os
import sys
import time

import numpy as np

import eye_v6 as E6
import kin_v6 as K6
import analiz as AN
import params as P5
from params_v6 import *
from checks import rapor_yaz, geo
from checks.core import Sonuc
from checks.cakisma import kontrol_cakisma
from checks.havada import kontrol_tek_govde, kontrol_ada, kontrol_donanim_temas
from checks.tabla import kontrol_tabla, kontrol_et
from checks.sarkma import kontrol_sarkma
from checks.zemin import kontrol_zemin
from checks.montaj_sokme import kontrol_montaj_sokme
from checks.giris_yolu import kontrol_giris_yolu, alet_silindiri
from checks.tolerans import kontrol_tolerans
from checks.vida_envanter import kontrol_vida_envanter
from checks.vida_adet import kontrol_vida_adet
from checks.kablo import kontrol_kablo
from checks.test_esik import kontrol_tutma_testleri
from checks.plan_sapma import kontrol_plan_sapmasi
from checks.yercekimi_tork import kontrol_yercekimi_tork, eylemsizlik, dinamik_tork
from checks.devrilme import kontrol_devrilme, destek_disk
from checks.dondurma import kontrol_md5, kontrol_parca_ayni
from checks.hafiflik import kontrol_hafiflik
from checks.belge import kontrol_adim_sirasi, kontrol_sikma_talimati, kontrol_kor_delik, kontrol_bom_belge_envanter

QUICK = "--quick" in sys.argv
OUT = E6.OUT
V5_STL = os.path.join(E6.V5CAD, "out", "stl")
ESIK_V = 0.05          # mm3: havsa kafa konisi yuvasiyla AYNI yuzey -> ucgenleme 0.012-0.014 mm3 (v5 raporu §2)


def log(*a):
    print(*a, flush=True)


# ============================================================================================ muaflar (gerekceli)
def eklem_ciftleri(P):
    """Tasarim geregi TEMAS / 0.4 eksenel bosluk / pim-delik ciftleri (farkli hareket gruplari). Bunlarda min bosluk OLCULMEZ,
    kesisim AYRI kontrolde olculur (cakisma_eklem)."""
    out = {}
    for a, b in E6.E5.JOINTS:
        if a in P and b in P and P[a].group != P[b].group:
            out[(a, b)] = "v5 eklemi (eye_v5.JOINTS: pim-delik / AXIAL_GAP / kol-servo mili); v5 check §4b + donanimda calisiyor"
    out[("horn_pitch_L", "servo_pitch_L")] = "sol SG90 mili - kol kavramasi (tasarim geregi temas, satin alinan)"
    out[("scr_ctip6", "coupler")] = "krank vidasi govdesi coupler deliginde doner (pim-delik, 4.3 / 3.4 model)"
    out[("nut_ctip_alt", "coupler")] = "alt emniyet somunu coupler'a 0.2 (PLAN §3/T3, 1/4 tur geri) - Z5 zinciri olcer"
    return {k: v for k, v in out.items() if v}


RIJIT_MUAF = {
    ("servo_yaw", "cable_yaw"): "satin alinan servo + kendi kablo cikis modeli (ayni parca; v5 §2 2.34 mm3)",
    ("servo_lid", "cable_lid"): "ayni (v5)",
    ("servo_pitch", "cable_pitch"): "ayni (v5; MG90S govdesi SG90 modeli)",
    ("servo_pitch_L", "cable_pitch_L"): "ayni (yeni sol SG90)",
    ("servo_yaw", "scr_m3_yaw0"): "v5 BILINEN: SG90 uzak kulak deligi govdeye 2.65 mm, M3 kafa govdeye ~0.15 mm biner (v5 DECISIONS, donanimda calisti)",
    ("servo_yaw", "nut_m3_yaw0"): "v5 BILINEN (ayni)",
    ("servo_lid", "scr_m3_lid0"): "v5 BILINEN (ayni)",
    ("servo_lid", "nut_m3_lid0"): "v5 BILINEN (ayni)",
    ("servo_pitch", "scr_m3_pitch0"): "v5 BILINEN (ayni; MG90S'te kulak olcusu PLAN R12 kumpas)",
    ("servo_pitch", "nut_m3_pitch0"): "v5 BILINEN (ayni)",
    ("scr_U", "lid_up_L"): "v5 BILINEN: U pimi M4 plastige kilavuzsuz (pilot 3.5) - dis modeli",
    ("lid_lo_L", "lid_lo_R"): "v5 BILINEN: alt kapak orta bindirmesi (v5 §2 2.17 mm3, basildi, donanimda calisiyor)",
}


# ============================================================================================ pozlar
def pozlar(P, alfa=TILT_DEG):
    Mm = K6.masa_M(alfa)
    lids = (0.0, 1.0) if QUICK else CHECK_LIDS
    yaws = CHECK_YAWS
    pits = (PITCH_MIN - PITCH_ASIM, PITCH_MAX, PITCH_MAX + PITCH_ASIM, 0.0) if QUICK else CHECK_PITCHES
    out = {}
    for y in yaws:
        for p in pits:
            for l in lids:
                Ms = K6.pose(y, p, l)
                out[f"y{y:+.0f}_p{p:+.0f}_k{l:.1f}"] = {n: (pt.shape, Mm @ Ms[pt.group]) for n, pt in P.items() if pt.kind != "visual"}
    return out


def sert_durma(P):
    """Pitch yukari mekanik durma acisi (cerceve x taban ilk temas) - ikiye bolme; bilgi."""
    stat = [n for n, p in P.items() if p.group == "static" and p.kind != "visual"]
    mov = [n for n, p in P.items() if p.group != "static" and p.kind != "visual" and not n.startswith("horn_pitch")]
    M = {n: geo.to_manifold(P[n].shape) for n in stat + mov}

    def temas(q):
        Ms = K6.pose(0, q, 0.0)
        for a in mov:
            A = geo.transform(M[a], Ms[P[a].group]); ab = geo.bbox(A)
            for b in stat:
                if b.startswith("servo_pitch") and a.startswith(("frame_R", "sol_kol", "scr_horn")):
                    continue
                if geo.bb_ayrik(ab, geo.bbox(M[b]), 0.0):
                    continue
                if geo.kesisim_hacmi(A, M[b]) > 0.01:
                    return (a, b)
        return None
    lo, hi = PITCH_MAX, PITCH_MAX + 8
    if not temas(hi):
        return None, None
    while hi - lo > 0.05:
        m = (lo + hi) / 2
        if temas(m):
            hi = m
        else:
            lo = m
    return hi, temas(hi)


# ============================================================================================ montaj
CRANK = ["yaw_crank", "horn_yaw", "scr_horn_yaw0", "scr_horn_yaw1", "nut_ctip", "scr_ctip6"]
YAWM = ["yaw_mount", "servo_yaw", "cable_yaw", "scr_m3_yaw0", "nut_m3_yaw0", "nut_ym_L63", "nut_ym_R63"]
SOLSRV = ["sol_servo_braketi", "servo_pitch_L", "cable_pitch_L", "scr_m2_pitchL", "nut_m2_pitchL", "nut_brk_-4", "nut_brk_24"]
TABAN = ["base_L", "base_R", "servo_pitch", "cable_pitch", "scr_m3_pitch0", "nut_m3_pitch0"]
MASKE = ["mask", "nut_mask_40", "nut_mask_160"]
KAMA = ["kama_L", "kama_R", "ayak_Larka", "ayak_Lon", "ayak_Rarka", "ayak_Ron"]


def montaj_siralari(P):
    allp = [n for n, p in P.items() if p.kind != "visual"]
    adim1 = [(CRANK, (0, -1, 0)), (["nut_ctip_alt"], (0, 1, 0)), (YAWM, (0, -1, 0)), (["scr_ym_L63"], (0, 1, 0)),
             (["scr_ym_R63"], (0, 1, 0)), (["sol_kol_adaptoru"], (1, 0, 0)), (["scr_adp0"], (1, 0, 0)), (["scr_adp1"], (1, 0, 0)),
             (["horn_pitch_L", "scr_horn_pitchL0", "scr_horn_pitchL1"], (1, 0, 0)), (SOLSRV, (1, 0, 0))]
    cerceve = [n for n in allp if P[n].group != "static"] + SOLSRV
    kok1 = [n for n in cerceve if not any(n in g for g, _ in adim1)]
    sira1 = [(kok1, None)] + adim1
    sira2 = [(TABAN, None),
             (cerceve, [((0, -1, 0), None), ((1, 0, 0), 5.0)]),
             (["scr_brk_-4"], (0, 1, 0)), (["scr_brk_24"], (0, 1, 0)),
             (MASKE, [((0, -1, 0), None), ((0, 0, -1), 30.0)]),     # onde indir, sonra 30 mm geri kaydir (PLAN T11 '-Y' tek basina imkansiz)
             (["scr_mask_40"], (0, 1, 0)), (["scr_mask_160"], (0, 1, 0)),
             (KAMA, (0, 0, 1))]
    return sira1, cerceve, sira2


def aletler(P, sira1, cerceve):
    kok1 = sira1[0][0]
    yt = P5.YAW_SERVO_Z - P5.LEVER_R
    h = E6.SOL_SRV
    L = []
    ctip_y = P5.LEVER_Y[1] + E6.E5.CRANK_NUT_Y + P5.M4_NUT_T
    L.append(dict(ad="T1 krank vidasi (ustten, servo yok)", vida="scr_ctip6",
                  geo=alet_silindiri((0, ctip_y + M4_CSK_K, yt), (0, -1, 0), 6.0, 60.0), mevcut=CRANK))
    L.append(dict(ad="T3 alt emniyet somunu (alttan lokma 7)", vida="nut_ctip_alt",
                  geo=alet_silindiri((0, P5.COUPLER_Y[0] - M4_JAM_GAP - P5.M4_NUT_T, yt), (0, 1, 0), 10.0, 40.0),
                  mevcut=kok1 + CRANK))
    for k, sg in (("L", -1), ("R", 1)):
        L.append(dict(ad=f"T5 yaw_mount vidasi {k} (alttan)", vida=f"scr_ym_{k}63",
                      geo=alet_silindiri((sg * P5.YM_SCREW_X, P5.RB_Y[0], -63.5), (0, 1, 0), 7.0, 30.0),
                      mevcut=kok1 + CRANK + ["nut_ctip_alt"] + YAWM))
    for i, yv in enumerate(ADP_VIDA_Y):
        L.append(dict(ad=f"A2 adaptor vidasi {i} (-X tarafindan)", vida=f"scr_adp{i}",
                      geo=alet_silindiri((E6.X_ADP_FLANS - M2_HEAD_H, yv, ADP_VIDA_Z), (1, 0, 0), 4.0, 40.0),
                      mevcut=kok1 + CRANK + ["nut_ctip_alt"] + YAWM + ["scr_ym_L63", "scr_ym_R63", "sol_kol_adaptoru"]))
    f = h.horn_face()
    for i, sg in enumerate((-1, 1)):
        p = f + np.array([0, 1.0, 0]) * sg * P5.HORN_SCREW_R - h.sd * (P5.HORN_T + 1.2)
        L.append(dict(ad=f"A3 sol kol vidasi {i} (servo yokken)", vida=f"scr_horn_pitchL{i}",
                      geo=alet_silindiri(p, h.sd, 3.0, 40.0),
                      mevcut=kok1 + CRANK + ["nut_ctip_alt"] + YAWM + ["scr_ym_L63", "scr_ym_R63", "sol_kol_adaptoru",
                                                                       "scr_adp0", "scr_adp1", "horn_pitch_L"]))
    e_lo, e_hi = h.ear_holes()[1]
    L.append(dict(ad="B1 sol servo kulak M2 (braket alt montaji)", vida="scr_m2_pitchL",
                  geo=alet_silindiri(np.asarray(e_hi) + np.array([M2_HEAD_H, 0, 0]), (-1, 0, 0), 3.5, 40.0),
                  mevcut=["sol_servo_braketi", "servo_pitch_L", "cable_pitch_L", "nut_m2_pitchL"]))
    tab = TABAN + cerceve
    for sz in BRK_VIDA_Z:
        L.append(dict(ad=f"T10 braket vidasi z{sz:+.0f} (taban altindan, kama yokken)", vida=f"scr_brk_{int(sz) + 10}",
                      geo=alet_silindiri((BRK_VIDA_X, P5.BASE_Y[0], sz), (0, 1, 0), 7.0, 40.0), mevcut=tab))
    for mx in P5.MASK_SCREW_X:
        L.append(dict(ad=f"T11 maske vidasi x{mx:+.0f}", vida=f"scr_mask_{int(mx) + 100}",
                      geo=alet_silindiri((mx, P5.BASE_Y[0], 10.0), (0, 1, 0), 7.0, 40.0),
                      mevcut=tab + ["scr_brk_-4", "scr_brk_24"] + MASKE))
    return L


# ============================================================================================ tolerans
def tolerans_zincirleri(P, z2, z3):
    return {
        "Z1-B sol mil kavrama (eksenel; braket yarigi ayar payi tum sapmalari karsilamali)": [
            ("braket ayak yarigi ayar payi (model, +-)", BRK_AYAR, 0, +1),
            ("MG90S mil ucu - SG90 (kulak altindan) [TAHMIN, PLAN Z1]", 0, 1.0, -1),
            ("MG90S kol kalinligi farki", 0, 0.3, -1),
            ("MG90S cepte oturma", 0, 0.2, -1),
            ("sol SG90 kulak->mil ucu (servo farki) [TAHMIN]", 0, 0.3, -1),
            ("sol_servo_braketi plaka konumu (baski)", 0, 0.2, -1),
            ("sol_kol_adaptoru uc duvari + D-delik derinligi (baski)", 0, 0.2, -1),
            ("frame_L gobek ucu (v5 baski)", 0, 0.2, -1),
            ("taban yarilari yapistirma x kaymasi [TAHMIN]", 0, 0.3, -1)],
        "Z2 krank vida kafasi <-> yaw servo govdesi (y)": {"esik": MIN_BOSLUK, "halkalar": [
            (f"model boslugu (yaw -25..25, en kucuk; kafa ISO maks {M4_CSK_K})", z2, 0, +1),
            ("servo govde ust yuzu [TAHMIN SG90]", 0, 0.3, -1),
            ("kol kalinligi (krank-servo arasi) [VARSAYIM 1.5]", 0, 0.2, -1),
            ("krank somun cebi tabani (baski)", 0, 0.1, -1)]},
        "Z3a arka en alcak <-> kama dudagi (pitch asim pozu)": {"esik": MIN_BOSLUK, "halkalar": [
            (f"model boslugu @ pitch {PITCH_MAX + PITCH_ASIM:+.0f} (asim dahil)", z3["kama"], 0, +1),
            ("vida ucu / somun konumu (T3)", 0, 0.5, -1),
            ("kama baski yuksekligi / aci (+-0.5 deg x 80 mm)", 0, 0.7, -1),
            ("taban kalinligi", 0, 0.2, -1)]},
        "Z3b arka en alcak <-> masa (pitch asim pozu)": {"esik": MIN_BOSLUK, "halkalar": [
            (f"model boslugu @ pitch {PITCH_MAX + PITCH_ASIM:+.0f}", z3["masa"], 0, +1),
            ("vida ucu / somun konumu (T3)", 0, 0.5, -1),
            ("kama baski yuksekligi / aci", 0, 0.7, -1),
            ("kaymaz ayak kalinligi [TAHMIN 1.5]", 0, 0.5, -1),
            ("taban kalinligi", 0, 0.2, -1)]},
        "Z5 alt emniyet somunu <-> coupler (eksenel, sikistirmamali)": [
            ("1/4 tur geri (0.7/4)", 0.175, 0, +1),
            ("kullanici ayari", 0, 0.1, -1)],
        "Z6 ray dudagi alti <-> taban ustu (taban kayarak girmeli)": [
            ("tasarim payi", RAY_PAY, 0, +1),
            ("taban kalinligi (baski Z) [TAHMIN]", 0, 0.1, -1),
            ("kama ray dudak konumu (baski XY) [TAHMIN]", 0, 0.1, -1),
            ("taban sehimi / yapistirma [TAHMIN]", 0, 0.05, -1)],
        "Z7 adaptor D-deligi <-> frame_L gobegi (radyal)": [
            ("tasarim payi (radyal)", GECME_PAY, 0, +1),
            ("adaptor dikey delik baskida kuculur (cap -0.3) [TOL]", 0, 0.15, -1),
            ("gobek (v5, yatay basildi) [TAHMIN]", 0, 0.1, -1)],
        "Z8 iki pitch servosu es eksenlilik (radyal kaciklik <= izin) [RSS]": {"olcut": "rss", "halkalar": [
            ("izin verilen kaciklik [TAHMIN: cerceve + plastik kol esner]", 0.5, 0, +1),
            ("braket ayak z (4.3 delik / M4)", 0, 0.15, -1),
            ("sol servo cep boslugu (12.7 / 12.5)", 0, 0.1, -1),
            ("MG90S cep boslugu", 0, 0.1, -1),
            ("adaptor merkezlenmesi (Z7 payi)", 0, GECME_PAY, -1),
            ("frame_L gobek / frame_R kol plakasi es eksenliligi (v5 baski)", 0, 0.1, -1),
            ("kol gobegi - mil boslugu x2", 0, 0.07, -1)]},
    }


# ============================================================================================ ana
# ------------------------------------------------------------------ duzeltme turu 1 yardimcilari
KALIBRASYON_SIRASI = [                       # denetim D2: MONTAJ adim tablosunda bu sirayla
    ("sag pitch kalibrasyonu (D5, D9 fisi cikik)", r"kalibr.*(D5|sa[gğ] pitch)|(D5|sa[gğ] pitch).*kalibr"),
    ("D9 ofseti", r"D9 ofset"),
    ("daraltilmis ilk tarama", r"daralt"),
    ("tam tarama (R14)", r"tam tarama|R14")]
BOM_ENVANTER_KALEM = [                       # denetim D8 (+ D3 kondansator)
    dict(kalem="SG90", bom_desen=r"SG90", env_desen=r"SG90 servo \(bo[sş]ta\)"),
    dict(kalem="MG90S", bom_desen=r"MG90S", env_desen=r"MG90S"),
    dict(kalem="1000 uF kondansator", bom_desen=r".*1000 µF.*", env_desen=r"kondansat[oö]r 1000")]
ENVANTER_MD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dogrulama", "envanter_plan-v2.md")


def matkap_olc(P, PZ, arka=10.0):
    """frame_L seridi: O1.6 delik ekseninde malzeme boyu (matkapsiz v5 sekli) + seridin arkasinda 0..arka mm silindirin
    tum pozlarda diger parcalarla (frame_L dahil, matkapsiz) en buyuk kesisimi."""
    from eye_v6 import cyl, I
    fl = P["frame_L"].v5_shape
    boy, arka_sil = [], []
    for yv in ADP_VIDA_Y:
        L = cyl((E6.X_PLAKA_L - 1, yv, ADP_VIDA_Z), (E6.X_PLAKA_L + 30, yv, ADP_VIDA_Z), M2_PILOT / 2)
        bb = I(fl, L).BoundingBox()
        boy.append(bb.xmax - bb.xmin)
        arka_sil.append(geo.to_manifold(cyl((bb.xmax + 0.05, yv, ADP_VIDA_Z), (bb.xmax + arka, yv, ADP_VIDA_Z), M2_PILOT / 2)))
    grp = P["frame_L"].group
    mf = {n: geo.to_manifold(p.v5_shape if n == "frame_L" else p.shape) for n, p in P.items()
          if p.kind != "visual" and not n.startswith("scr_adp")}
    enb, yer = 0.0, None
    for pk, poz in PZ.items():
        Mg = poz["frame_L"][1]
        for c in arka_sil:
            cw = geo.transform(c, Mg)
            for n, m in mf.items():
                mw = geo.transform(m, poz[n][1])
                if geo.bb_ayrik(geo.bbox(cw), geo.bbox(mw)):
                    continue
                v = geo.kesisim_hacmi(cw, mw)
                if v > enb:
                    enb, yer = v, (pk, n)
    return min(boy), enb, yer, len(PZ)


def main():
    t0 = time.time()
    S = []
    log("build...")
    P = E6.build(verbose=False)
    allp = {n: p for n, p in P.items() if p.kind != "visual"}
    gruplar = {n: p.group for n, p in allp.items()}

    # ---------------------------------------------------------------- R16 dondurma
    log("dondurma...")
    beklenen = json.load(open(os.path.join(E6.HERE, "v5_dondurulmus_md5.json"), encoding="utf-8"))["md5"]
    S.append(kontrol_md5(V5_STL, beklenen, ad="R16_v5_stl_md5"))
    v6v5, ref = {}, {}
    goruldu = set()
    for n, p in P.items():
        if p.kind != "printed" or p.bom_key in E6.YENI or p.bom_key in goruldu:
            continue
        goruldu.add(p.bom_key)
        part = p
        if hasattr(p, "v5_shape"):
            part = E6.Part(n, p.v5_shape, p.group, p.color, print_R=p.print_R, bom_key=p.bom_key)
        v6v5[p.bom_key] = E6.E5.print_mesh(part)
        ref[p.bom_key] = os.path.join(V5_STL, f"{p.bom_key}.stl")
    s = kontrol_parca_ayni(v6v5, ref, ad="R16_v5_parca_geometri")
    s.notlar.append("frame_L: montaj modelinde 2x O1.6 x 5.5 MATKAP deligi var (MONTAJ A0); karsilastirma matkapsiz v5 sekliyle")
    s.notlar.append(f"v6 montajindan CIKAN v5 parcalari: {list(E6.CIKAN)} (pivot_bracket STL'i kullanilmaz, saklanir)")
    S.append(s)

    # ---------------------------------------------------------------- R9 yeni parcalar
    log("yeni parcalar...")
    stl = geo.yukle_stl_klasoru(os.path.join(OUT, "stl"))
    S += [kontrol_tek_govde(stl), kontrol_ada(stl), kontrol_et(stl, min_et=MIN_ET), kontrol_tabla(stl), kontrol_sarkma(stl)]

    # ---------------------------------------------------------------- R3 cakisma
    log("cakisma (rijit, notr)...")
    eklem = eklem_ciftleri(allp)
    Mm = K6.masa_M()
    notr = {n: (p.shape, Mm @ K6.pose(0, 0, 0.0)[p.group]) for n, p in allp.items()}
    muaf_r = dict(RIJIT_MUAF); muaf_r.update(eklem)
    s = kontrol_cakisma({"notr_egik": notr}, muaf=muaf_r, min_bosluk=MIN_BOSLUK_OLCUM, kesisim_esigi=ESIK_V, gruplar=gruplar,
                        ad="R3_cakisma_rijit_notr")
    S.append(s)
    PZ = pozlar(allp)
    log(f"cakisma ({len(PZ)} poz, egik)...")
    s = kontrol_cakisma(PZ, muaf=eklem, min_bosluk=MIN_BOSLUK_OLCUM, kesisim_esigi=ESIK_V, gruplar=gruplar, sadece_capraz=True,
                        ad="R3_cakisma_pozlar")
    s.olculdu.append(f"pozlar: yaw {CHECK_YAWS} x pitch {CHECK_PITCHES if not QUICK else 'quick'} x kapak {CHECK_LIDS}; masa egik {TILT_DEG} deg; "
                     f"min bosluk {MIN_BOSLUK} (olcum {MIN_BOSLUK_OLCUM}: 0.4 AXIAL_GAP duz yuzeyde ucgenleme payi 0.005)")
    S.append(s)
    log("cakisma eklem kesisimi...")
    s = kontrol_cakisma(PZ, yalniz=list(eklem), min_bosluk=0.0, kesisim_esigi=ESIK_V, ad="R3_cakisma_eklem_kesisim")
    s.olculmedi.append("eklem ciftlerinde (temas / 0.4 eksenel / pim-delik) min bosluk ESIGI uygulanmadi (tasarim geregi); kesisim olculdu")
    S.append(s)
    q_dur, cift = sert_durma(allp)
    s = Sonuc("R3_pitch_mekanik_durma")
    if q_dur is None:
        s.notlar.append(f"pitch {PITCH_MAX}..{PITCH_MAX + 8}: cerceve tabana degmiyor")
    else:
        s.notlar.append(f"pitch YUKARI mekanik durma ~{q_dur:.1f} deg ({cift[0]} x {cift[1]}; v5 geometrisi) -> ust sinir {PITCH_MAX:+.0f} + asim {PITCH_ASIM:.0f} = "
                        f"{PITCH_MAX + PITCH_ASIM:+.0f} < durma: pay {q_dur - PITCH_MAX - PITCH_ASIM:.1f} deg")
        if q_dur < PITCH_MAX + PITCH_ASIM:
            s.bulgu(f"pitch ust sinir + asim ({PITCH_MAX + PITCH_ASIM:+.0f}) mekanik durmayi ({q_dur:.1f}) gecer")
    s.olculdu.append("pitch ust mekanik durma acisi (ikiye bolme, 0.05 deg; sabit x hareketli kesisim > 0.01 mm3)")
    S.append(s)

    # ---------------------------------------------------------------- R4 zemin + Z3 olcumleri
    log("zemin...")
    temas = [n for n in allp if n.startswith("ayak_")]
    s = kontrol_zemin(PZ, temas, min_bosluk=MIN_BOSLUK, ad="R4_zemin_masa")
    S.append(s)
    asim = PITCH_MAX + PITCH_ASIM
    z3 = {"kama": 99.0, "masa": 99.0}
    kama_M = {n: geo.to_manifold(allp[n].shape) for n in ("kama_L", "kama_R")}
    for y in CHECK_YAWS:
        Ms = K6.pose(y, asim, 1.0)
        for n, p in allp.items():
            if p.group == "static":
                continue
            A = geo.transform(geo.to_manifold(p.shape), Ms[p.group])
            for kn, K in kama_M.items():
                if not geo.bb_ayrik(geo.bbox(A), geo.bbox(K), 6.0):
                    z3["kama"] = min(z3["kama"], geo.min_bosluk(A, K, 6.0))
            zmin = float(geo.bbox(geo.transform(A, Mm))[0][2])
            z3["masa"] = min(z3["masa"], zmin)

    # ---------------------------------------------------------------- R8 havada donanim
    yeni_hw = ["scr_ctip6", "nut_ctip_alt", "scr_adp0", "scr_adp1", "scr_horn_pitchL0", "scr_horn_pitchL1", "scr_m2_pitchL",
               "nut_m2_pitchL", "nut_brk_-4", "nut_brk_24", "scr_brk_-4", "scr_brk_24"]
    basili = {n: notr[n] for n, p in allp.items() if p.kind == "printed"}
    s = kontrol_donanim_temas({n: notr[n] for n in yeni_hw}, basili, ad="R8_havada_donanim_yeni")
    s.olculmedi.append("v5 donanimi (degismeyen 80+ vida/somun) bu kontrolde yok: v5 check_havada.py ile olculdu, yeri degismedi")
    S.append(s)

    # ---------------------------------------------------------------- R6 montaj
    log("montaj-sokme...")
    sira1, cerceve, sira2 = montaj_siralari(allp)
    notr_m = {n: (p.shape, K6.pose(0, 0, 0.0)[p.group]) for n, p in allp.items()}
    s = kontrol_montaj_sokme(sira1, notr_m, ad="R6_montaj_cerceve_alt")
    S.append(s)
    s = kontrol_montaj_sokme(sira2, notr_m, ad="R6_montaj_ana")
    S.append(s)
    s = kontrol_giris_yolu(aletler(allp, sira1, cerceve), notr_m, ad="R6_giris_yolu",
                           muaf={("T3*", "scr_ctip6"): "lokma ici bos: vida ucu (somundan 1.4 mm tasar) lokmanin icine girer; silindir dolu modellendi"})
    s.olculmedi.append("somunlarin cebe konma yolu, elle tutma, kablo yonlendirme olculmedi (kullanici)")
    S.append(s)

    # ---------------------------------------------------------------- R10 tolerans
    log("tolerans...")
    MZ = {n: geo.to_manifold(allp[n].shape) for n in ("scr_ctip6", "servo_yaw")}
    z2 = 99.0
    for y in range(-25, 26, 5):
        Ms = K6.pose(y, 0, 0.0)
        z2 = min(z2, geo.min_bosluk(geo.transform(MZ["scr_ctip6"], Ms["yawcrank"]), geo.transform(MZ["servo_yaw"], Ms["frame"]), 10.0))
    s = kontrol_tolerans(tolerans_zincirleri(allp, z2, z3), olculmeyen=[
        "Z1-B/Z2/Z8 [TAHMIN] halkalari kumpasla (PLAN R12): MG90S ve sol SG90 kulak alti -> mil ucu, kol kalinligi",
        "Z8 izin verilen es eksen kacikligi (0.5) TAHMIN: ilk calistirmada titreme/akim gozlemi (R14)"])
    s.notlar.append(f"Z2 PLAN kotumser varyanti (SG90 ustu HER YERDE gobek seviyesinde, y -9.1 varsayimi): nominal 0.52, en kotu -0.08 -> "
                    f"modelde gobek yalniz mil cevresinde (r 5.9), kafa milden >= 16 mm'de govde seviyesinin ({z2:.2f} mm) ustunde")
    S.append(s)

    # ---------------------------------------------------------------- R7 vidalar
    vidalar = [
        dict(ad="scr_ctip6", tip="M4", L=16.0, sikisan=M4_CSK_K + P5.M4_NUT_T + 1.6 + P5.AXIAL_GAP + 3.5 + M4_JAM_GAP),
        dict(ad="scr_brk_-4", tip="M4", L=16.0, sikisan=(P5.BASE_Y[1] - P5.BASE_Y[0]) + (BRK_SOMUN_Y_() - P5.BASE_Y[1])),
        dict(ad="scr_brk_24", tip="M4", L=16.0, sikisan=(P5.BASE_Y[1] - P5.BASE_Y[0]) + (BRK_SOMUN_Y_() - P5.BASE_Y[1])),
        dict(ad="scr_m2_pitchL", tip="M2", L=8.0, sikisan=P5.SG_EAR_T + BRK_PLAKA_T),
        dict(ad="scr_adp0", tip="M2", L=8.0, sikisan=None, somunsuz_dis=8.0 - ADP_FLANS_T),
        dict(ad="scr_adp1", tip="M2", L=8.0, sikisan=None, somunsuz_dis=8.0 - ADP_FLANS_T)]
    s = kontrol_vida_envanter(vidalar, {"M2": (8.0,), "M4": (16.0,)}, ad="R7_vida_boyu_envanter")
    s.olculmedi.append("M4x16 HAVSA: envanterde M4 havsa yalniz 2x M4x10 -> 2 adet v5 pivot_bracket'tan sokulen + 1 SATIN ALINACAK (BOM_FARK)")
    s.olculmedi.append("degismeyen v5 vidalari bu listede yok (v5 check §8b ile olculdu)")
    S.append(s)
    tas = {sc: t for sc, nut, typ, t in E6.FASTENERS6}
    bag = [dict(vida="scr_ctip6", tip="M4", boy=16, tasma=tas.get("scr_ctip6")),
           dict(vida="scr_brk_-4", tip="M4", boy=16, tasma=tas.get("scr_brk_-4")),
           dict(vida="scr_brk_24", tip="M4", boy=16, tasma=tas.get("scr_brk_24")),
           dict(vida="scr_m2_pitchL", tip="M2", boy=8, tasma=tas.get("scr_m2_pitchL")),
           dict(vida="scr_adp0", tip="M2", boy=8, tasma=None), dict(vida="scr_adp1", tip="M2", boy=8, tasma=None)]
    s = kontrol_vida_adet(bag, {"M4x16": 3, "M2x8": 30}, ad="R7_vida_adet")
    s.notlar.append("M4x16 'elde' 3 = 2 sokulen (pivot_bracket) + 1 SATIN (BOM_FARK, ~10 TL); satin alinmadan T1 yapilamaz")
    S.append(s)

    # ---------------------------------------------------------------- R1/R2 tork
    log("tork...")
    yuk = K6.masa_yukari_mek()
    acilar = list(np.arange(PITCH_MIN, PITCH_MAX + 0.01, 1.0))
    capB = (KAP_MG90S + KAP_SG90) * IKI_SERVO_VERIM
    tork_ozet = {}
    for dolgu in DOLGU_TARAMA:
        kl = AN.kutleler(allp, dolgu, AN.donen)
        I = eylemsizlik(kl, (0, 0, 0), (1, 0, 0))
        dyn = dinamik_tork(I, HAREKET_DEG, HAREKET_S)
        s = kontrol_yercekimi_tork(kl, (0, 0, 0), (-1, 0, 0), yuk, acilar, capB, hedef=TORK_HEDEF,
                                   ek={f"dinamik ({HAREKET_DEG:.0f} deg / {HAREKET_S} s, I {I:.2e})": dyn, "surtunme+kablo [TAHMIN]": SURTUNME_KGCM},
                                   bilgi_kapasite=(1.8 * RAY_MIN_V / KAP_V) * KAP_BILGI_ORAN * 2 * IKI_SERVO_VERIM,
                                   ad=f"R2_pitch_tork_B_dolgu{dolgu}",
                                   girdi_notu=f"SECENEK B kapasite = (MG90S {KAP_MG90S:.3f} + SG90 {KAP_SG90:.3f}) x verim {IKI_SERVO_VERIM} @ {RAY_MIN_V} V")
        s.notlar.append(f"karsilastirma: tek MG90S (B olmasaydi, agirliksiz) pay {KAP_MG90S / s.gereken:.2f}")
        if dolgu != DOLGU_ESAS:
            # tarama: yalniz esas dolgu bulgu uretir; digerleri bilgi (PLAN §5.0 'dolgu 0.75 esas')
            if not s.gecti:
                s.olculmedi.append(f"dolgu {dolgu}: pay {s.pay:.2f} < {TORK_HEDEF} (yalniz bilgi; esas dolgu {DOLGU_ESAS})")
            s.bulgular = []; s.gecti = True
        tork_ozet[dolgu] = (s.pay, s.gereken, s.kutle, s.am.tolist(), dyn)
        S.append(s)
    for a in (20.0, 40.0):
        kl = AN.kutleler(allp, DOLGU_ESAS, AN.donen)
        s = kontrol_yercekimi_tork(kl, (0, 0, 0), (-1, 0, 0), K6.masa_yukari_mek(a), acilar, capB, hedef=TORK_HEDEF,
                                   ek={"dinamik": tork_ozet[DOLGU_ESAS][4], "surtunme+kablo": SURTUNME_KGCM}, ad=f"R2_bilgi_alfa{a:.0f}")
        s.olculmedi.append(f"alfa {a:.0f} yalniz bilgi (TILT_DEG = {TILT_DEG})"); s.gecti = True; s.bulgular = []
        S.append(s)
    for nm, srv in (("EYE_YAW", "SG90"), ("LIDS", "SG90")):
        pass
    s = Sonuc("R2_diger_eksenler")
    s.notlar.append("yaw ve kapak eksenleri v5 ile ayni (PLAN §5.4: yerçekimi <= 0.01 kg*cm, pay > 5; v5 donanimda calisiyor); egim bunlari degistirmez")
    s.olculmedi.append("yaw/kapak yerçekimi torku bu raporda yeniden olculmedi (PLAN §5.4 hesabi + v5 donanim)")
    S.append(s)

    # ---------------------------------------------------------------- tutma testi esigi (R11)
    gB = tork_ozet[DOLGU_ESAS][1]
    pay_servo = gB / (2 * IKI_SERVO_VERIM)
    testler = [dict(ad=f"T-R11 {srv}", servo=srv, kutle_kg=TEST_G / 1000, kol_cm=TEST_KOL_CM, gerilim_v=KAP_V, kapasite_kgcm=cap,
                    gereken_kgcm=pay_servo, ray_min_v=RAY_MIN_V, kapasite_gerilim_v=RAY_MIN_V, olcekli_esdeger=True)
               for srv, cap in (("MG90S", KAP_MG90S), ("SG90 sol", KAP_SG90))]
    s = kontrol_tutma_testleri(testler, hedef=TORK_HEDEF, ad="R11_tutma_testi_esik")
    s.notlar.append(f"servo basina pay: gereken {gB:.3f} / (2 x {IKI_SERVO_VERIM}) = {pay_servo:.3f} kg*cm")
    S.append(s)

    # ---------------------------------------------------------------- R5 devrilme
    log("devrilme...")
    for a in (20.0, 30.0, 40.0):
        PP = allp
        if a != TILT_DEG:
            PP = dict(allp)
            kL = E6.make_kama(a)
            PP["kama_L"] = E6.Part("kama_L", kL, "static", "#000000", bom_key="kama", print_R=None)
            PP["kama_R"] = E6.Part("kama_R", E6.ayna_x(kL), "static", "#000000", bom_key="kama", print_R=None)
            for nm, sh in E6.kama_ayaklari(a):
                PP[nm] = E6.Part(nm, sh, "static", "#000000", kind="purchased", bom_key="kaymaz ayak O10")
        Mma = K6.masa_M(a)
        pz = {}
        for p in (PITCH_MIN - PITCH_ASIM, PITCH_MIN, 0.0, PITCH_MAX, PITCH_MAX + PITCH_ASIM):
            for l in (0.0, 1.0):
                Ms = K6.pose(0, p, l)
                kl = AN.kutleler(PP, DOLGU_ESAS, M={g: Mma @ M for g, M in Ms.items()})
                pz[f"p{p:+.0f}_k{l:.0f}"] = [(m, c) for _, m, c, _ in kl]
        kp = K6.kama_profil(a)
        x_dis = P5.BASE_X[0] - RAY_PAY - RAY_T
        dest = []
        for (yy, zz) in kp["ayak"]:
            for xa in (x_dis + AYAK_X_KENAR, 2 * KAMA_ORTA_X - (x_dis + AYAK_X_KENAR)):
                c = Mma @ np.array([xa, yy, zz, 1.0])
                dest += destek_disk(c[:2], AYAK_D / 2)
        s = kontrol_devrilme(pz, dest, DEVRILME_MESAFE, DEVRILME_ACI, ad=f"R5_devrilme_alfa{a:.0f}", etiket=f"alfa{a:.0f} ")
        if a != TILT_DEG:
            if not s.gecti:
                s.olculmedi.append(f"alfa {a:.0f}: {len(s.bulgular)} esik alti (yalniz bilgi, secilen alfa {TILT_DEG}): {s.bulgular[0]}")
            s.gecti = True; s.bulgular = []
        S.append(s)

    # ---------------------------------------------------------------- R17 kablo
    log("kablo...")
    ysrv = E6.E5.Servo((0, P5.LEVER_Y[1] + P5.HORN_T, P5.YAW_SERVO_Z), (0, -1, 0), (0, 0, -1))
    lsrv = E6.E5.Servo((P5.LS_SPLINE_X, P5.LS_S[0], P5.LS_S[1]), (-1, 0, 0), (0, 1, 0))
    psrv = E6.E5.Servo((P5.PITCH_SPLINE_X, 0, 0), (-1, 0, 0), (0, -1, 0))
    cik = lambda sv: tuple(sv.w((-P5.SG_SHAFT_OFF - 14.2, 2.15, 0)))
    kertik = (KAMA_ORTA_X, K6.BASE_ALT_Y + 1.0, P5.BASE_Z[0] - 1.0)
    uno = (KAMA_ORTA_X, K6.BASE_ALT_Y - 40.0, P5.BASE_Z[0] - UNO_UZAKLIK)
    kiris = (0.0, P5.RB_Y[0], P5.RB_Z[0])
    kablolar = [
        dict(ad="yaw servo (D3)", noktalar=[("frame", cik(ysrv)), ("frame", kiris), ("static", kertik), ("static", uno)], boy=SG90_KABLO, servis=KABLO_SERVIS),
        dict(ad="kapak servo (D6)", noktalar=[("frame", cik(lsrv)), ("frame", (P5.LS_WING_X[0], P5.RB_Y[0], -30.0)), ("frame", kiris),
                                              ("static", kertik), ("static", uno)], boy=SG90_KABLO, servis=KABLO_SERVIS),
        dict(ad="sag pitch MG90S (D5)", noktalar=[("static", cik(psrv)), ("static", (110.0, P5.BASE_Y[1], P5.BASE_Z[0])),
                                                  ("static", uno)], boy=SG90_KABLO, servis=KABLO_PAY_SABIT),
        dict(ad="sol pitch SG90 (D9)", noktalar=[("static", cik(E6.SOL_SRV)), ("static", (-101.0, P5.BASE_Y[1], P5.BASE_Z[0])),
                                                 ("static", uno)], boy=SG90_KABLO, servis=KABLO_PAY_SABIT)]
    kpz = {f"y{y:+.0f}_p{p:+.0f}": K6.pose(y, p, 0.5) for y in CHECK_YAWS for p in CHECK_PITCHES}
    s = kontrol_kablo(kablolar, kpz, ad="R17_kablo")
    s.olculmedi.append(f"Uno konumu [TAHMIN]: kertikten {UNO_UZAKLIK:.0f} mm geride masada; kablo boyu SG90 {SG90_KABLO:.0f} [KAYNAK, olc]; "
                       "yetmezse 15 cm servo uzatma (kullanici)")
    S.append(s)

    # ---------------------------------------------------------------- duzeltme turu 1 (denetim D2/D4/D6/D7/D8)
    log("duzeltme turu 1 kontrolleri...")
    v_kama = E6.make_kama().Volume()
    v_min = E6.make_kama(duvar=MIN_ET, deri=MIN_ET, nervur=MIN_ET).Volume()
    s = kontrol_hafiflik({"kama (yari)": dict(hacim=v_kama, hacim_min_et=v_min)}, oran_maks=KAMA_HAFIF_ORAN, rho_g_mm3=PLA_RHO,
                         ad="R9_kama_hafiflik")
    s.notlar.append(f"kama duvar/deri/nervur {KAMA_DUVAR}/{KAMA_DIS_DERI}/{KAMA_NERVUR}, nervur z {KAMA_NERVUR_Z}")
    S.append(s)
    v6d = os.path.normpath(os.path.join(E6.HERE, ".."))
    montaj = open(os.path.join(v6d, "MONTAJ.md"), encoding="utf-8").read()
    S.append(kontrol_adim_sirasi(montaj, KALIBRASYON_SIRASI, ad="MONTAJ_kalibrasyon_sirasi"))
    S.append(kontrol_sikma_talimati(montaj, ad="MONTAJ_sikma_talimati"))
    boy, enb, yer, npoz = matkap_olc(allp, PZ)
    hedef = None if ADP_PILOT_DERIN >= boy else ADP_PILOT_DERIN
    s = kontrol_kor_delik([dict(ad=f"frame_L serit O{M2_PILOT} x{len(ADP_VIDA_Y)}", malzeme_boyu=boy, hedef=hedef,
                                arka_kesisim=enb, el_hassasiyet=MATKAP_EL_HASSASIYET)], ad="R6_frame_L_matkap")
    s.olculdu.append(f"serit arkasi 0..10 mm O{M2_PILOT} silindir x {npoz} poz x tum parcalar (frame_L matkapsiz): "
                     f"en buyuk kesisim {enb:.4f} mm3" + (f" ({yer})" if yer else ""))
    S.append(s)
    bom = open(os.path.join(v6d, "BOM_FARK.md"), encoding="utf-8").read()
    env = open(ENVANTER_MD, encoding="utf-8").read() if os.path.exists(ENVANTER_MD) else ""
    s = kontrol_bom_belge_envanter(bom, env, BOM_ENVANTER_KALEM, ad="BOM_envanter_belge")
    s.olculmedi.append("envanter kopyasi dogrulama/envanter_plan-v2.md (2026-10-06 sayimi); cekmece sayimi kullanicida")
    S.append(s)

    # ---------------------------------------------------------------- plan sapmasi
    dec_yol = os.path.join(E6.HERE, "..", "DECISIONS.md")
    dec = open(dec_yol, encoding="utf-8").read() if os.path.exists(dec_yol) else ""
    kp30 = K6.kama_profil()
    kalem = [
        dict(anahtar="pitch ust siniri", plan="+20 (servo 110)", model=f"{PITCH_MAX:+.0f} (servo {90 + PITCH_MAX:.0f})"),
        dict(anahtar="asim pozu", plan="-13/+23", model=f"{PITCH_MIN - PITCH_ASIM:+.0f}/{PITCH_MAX + PITCH_ASIM:+.0f}"),
        dict(anahtar="M4x16 havsa", plan="envanter (bol)", model="1 satin + 2 sokulen"),
        dict(anahtar="adaptor vidasi", plan="2 sac vida", model="2x M2x8 (envanter) O1.6 pilota"),
        dict(anahtar="sol servo kulak vidasi", plan="M3x10 kulak", model="M2x8 + M2 somun"),
        dict(anahtar="sag ray", plan="x 95..116.5 z -62..-12", model="ayna: x 111.5..116.5 z -66..-24"),
        dict(anahtar="kama kutlesi", plan="~60 g toplam", model=f"{2 * v_kama * PLA_RHO:.0f} g (dolgu 1.0)"),
        dict(anahtar="kama kabugu", plan="2.4", model=f"{KAMA_DUVAR}"),
        dict(anahtar="pitch ekseni yuksekligi", plan="86 mm", model=f"{K6.pitch_ekseni_yukseklik():.1f} mm"),
        dict(anahtar="tork kapasitesi", plan="0.63 @4.8 V", model=f"{KAP_MG90S:.3f} @{RAY_MIN_V} V"),
        dict(anahtar="tutma testi", plan="315 g", model=f"{TEST_G:.0f} g"),
        dict(anahtar="maske takma yonu", plan="-Y", model="-Y (onde) + -Z 30 mm"),
    ]
    s = kontrol_plan_sapmasi(kalem, dec, ad="plan_sapmasi")
    S.append(s)

    # ---------------------------------------------------------------- KAPSAM: olculemeyenler (PLAN §7)
    s = Sonuc("kapsam_kullanici")
    for t in [
        "R11 MG90S ve sol SG90'in Uno 5 V'taki gercek torku -> tutma testi (MONTAJ 'Testler' T-R11)",
        "R12 MG90S olculeri (govde boy/en, kulak delik araligi, kulak alti->mil ucu, kol delik r, donus yonu) -> kumpas, T7 oncesi",
        "R13 krank ucu vidasi / A pimi gevsemesi -> boya kalemi cizgisi, 30 dk + stant sonrasi",
        "R14 Uno 5 V dusmesi (iki pitch servosu ayni anda) -> multimetre >= 4.6 V, reset yok",
        "R15 basili kamanin gercek acisi -> telefon egim olcer, 30 +-1 deg",
        "R18 yazilim (bakis eslemesi, hiz siniri, limitler 80..108 / sol 72..100, D9 kanali) -> AYRI IS (firmware/settings)",
        "R1 donen kutle ve AM -> tartı + kalem ustunde denge (PLAN R1); dolgu 0.6..1.0 taramasi raporda",
        "MG90S govdesi SG90 modeliyle temsil edildi (22.8x12.2 vs 22.5x12.5 [TAHMIN]); cep 22.9x12.7 -> R12",
        "Kaymaz ayak kalinligi 1.5 [TAHMIN]: masa boslugu (Z3b) ve pitch ekseni yuksekligi buna bagli",
        "SG90 kol vidasi boyu (kol 1.5 + pilot 3.9 = 5.4 mm'ye kadar) [OLCUM]",
        "frame_L'ye acilan 2x O1.6 pilot: konum adaptor deliklerinin kilavuzluguna bagli (MONTAJ A0); eksen kacikligi olculmedi",
        "D1 iki pitch servosunun yuk paylasimi (akim) -> MONTAJ 'Kullanici testleri' D1 (pay hesabi IKI_SERVO_VERIM 0.8 [TAHMIN])",
        "D3 Uno Vcc anlik cokmesi (bandgap + MCUSR) ve 1000 uF etkisi -> MONTAJ 'Kullanici testleri' D3",
        "D5 iki pitch milinin es eksenliligi (y/z) -> MONTAJ 'Kullanici testleri' D5 (kumpas, T9 oncesi)",
        "MG90S envanterde YOK (kullaniciya soruldu): yoksa sagda v5 SG90 kalir; tork hesabi zaten SG90 kapasitesiyle"]:
        s.olculmedi.append(t)
    S.append(s)

    baslik = (f"PROJECT EYE v6 (Agent Flow v2) | masa egimi {TILT_DEG} deg | pitch {PITCH_MIN:+.0f}..{PITCH_MAX:+.0f} | SECENEK B "
              f"(sol SG90 D9) | {len(allp)} parca | sure {time.time() - t0:.0f} s")
    metin = rapor_yaz(S, os.path.join(OUT, "check_report.txt"), baslik=baslik)
    with open(os.path.join(OUT, "check_ozet.json"), "w", encoding="utf-8") as f:
        json.dump(dict(z2=z2, z3=z3, sert_durma=q_dur, tork={str(k): v for k, v in tork_ozet.items()},
                       pitch_ekseni=K6.pitch_ekseni_yukseklik()), f, indent=1, default=float)
    print(metin[:6000])
    print(f"sure {time.time() - t0:.0f} s")


def BRK_SOMUN_Y_():
    return E6.BRK_SOMUN_Y


if __name__ == "__main__":
    main()
