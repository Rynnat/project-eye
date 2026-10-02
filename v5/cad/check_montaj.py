"""Montaj erisim taramasi - MONTAJ SIRASINA gore (PYTHONUTF8=1 python check_montaj.py [--detay] [--takili]).

Kullanici 2026-09-30: "vidanin takili hali modelde var ama oraya isinlanmayacak, o dogrultuda girecek".
check.py takili hali denetler; bu betik her vidanin GIRIS KORIDORUNU denetler:
  - kafa koridoru: kafanin ustunden geriye (vida ekseni boyunca, disari) VIDA BOYU kadar, kafa yaricapi + HEAD_PAY
    (vida takili konumuna kadar ekseni boyunca surulur),
  - uc koridoru: onun arkasinda BIT_L boyunca BIT_R yaricapli silindir (kisa uc tutucu / tornavida govdesi).
Koridor yalniz o ADIMA kadar ayni alt montajda takili parcalara karsi denetlenir (SIRA). Her vida:
  OK / ENGEL (vida yerlesir ama tornavida yolu kapali) / IMKANSIZ (vida bile yerlesemez).
--takili: sira yok say, her seyi takili kabul et (eski tarama, bilgi icin).
Montaj adimlari ../MONTAJ.md ile ayni tutulur.
"""
import fnmatch
import sys
import numpy as np
import eye_v5 as E
from params import *

HEAD_PAY = 0.2        # kafa koridoru yaricap payi (mm)
BIT_R = 3.3           # 1/4" altigen uc tutucu / tornavida govdesi yaricapi (M3/M4)
BIT_R_KUCUK = 2.0     # servo kolu sac vidalari (kucuk tornavida)
BIT_L = 30.0          # kafanin arkasinda gereken tornavida boyu (kisa uc tutucu)
ESIK = 1.0            # bu hacimden (mm3) kucuk kesisimler yok sayilir (ucgenleme temasi)
DILIM_ESIK = 0.5      # serbest mesafe taramasinda 1 mm dilim icin esik (yuva agzi siyirmasi sayilmaz)
# vida deseni -> gereken tornavida boyu (mm): kisa uc (ornek: 1/4" uc parmakla / kisa uc tutucu) gerektiren yerler
KISA_UC = {"scr_lpin_*": 20.0}
# bilinen ve kabul edilen temaslar: (vida deseni, parca deseni, en fazla mm3) -> UYARI
KABUL = [("scr_m3_*", "servo_*", 3.0)]    # M3 pan kafa SG90 govdesine ~0.15 mm biniyor (kulak deligi govdeye 2.65 mm)

KAYIT = {}            # id(shape) -> (kafa_ustu, eksen(govde yonu), kafa_r, boy)
_round, _csk = E.screw_round, E.screw_csk


def screw_round(head_face, axis, L, d_shank=E.M4_SHANK_MODEL, head_d=E.M4_HEAD_D, head_h=E.M4_HEAD_H):
    s = _round(head_face, axis, L, d_shank, head_d, head_h)
    a = E.unit(axis); p = np.asarray(head_face, float)
    KAYIT[id(s)] = (p - a * head_h, a, head_d / 2, L)
    return s


def screw_csk(face_pt, axis, L=16.0):
    s = _csk(face_pt, axis, L)
    a = E.unit(axis); p = np.asarray(face_pt, float)
    KAYIT[id(s)] = (p + a * (E.M4_CSK_D - E.M4_CSK_HEAD_D) / 2, a, E.M4_CSK_HEAD_D / 2, L)
    return s


E.screw_round, E.screw_csk = screw_round, screw_csk
E.screw_m3 = lambda head_face, axis, L=10.0: screw_round(head_face, axis, L, E.M3_SHANK_MODEL, E.M3_HEAD_D, E.M3_HEAD_H)

# ------------------------------------------------------------------ MONTAJ SIRASI
# (baslik, alt montaj ["a+b" = bu adimda a ile b birlesir, sonuc a], parca desenleri, kapak pozu)
SIRA = [
    ("Taban yarilarini yapistir", "taban", ["base_L", "base_R"], "open"),
    ("Pitch servosunu tabandaki tutucuya tak; M3 MILE UZAK kulaktan", "taban",
     ["servo_pitch", "cable_pitch", "nut_m3_pitch0", "scr_m3_pitch0"], "open"),
    ("Pitch braketini tabana vidala (alttan havsa)", "taban", ["pivot_bracket", "nut_brk_*", "scr_brk_*"], "open"),
    ("GOZ ALT MONTAJI: iki goz (iris+bebek), kollar goze kilitli (M3 alttan), kol ucu somunlari cepte, "
     "baglanti cubugu iki kol ucuna ALTTAN vidali", "gozler",
     ["eye_L", "eye_R", "iris_*", "pupil_*", "eye_lever_*", "nut_lock_*", "scr_lock_*", "nut_ltip_*", "coupler", "scr_ltip_*"], "open"),
    ("Kafes L: kapaklar + mentese vidalari (goz tarafindan, GOZ YOKKEN)", "kafes_L",
     ["frame_L", "lid_up_L", "lid_lo_L", "nut_lpin_L*", "scr_lpin_L*"], "open"),
    ("Kapak lamalari sol kafeste: ust lama U pimi (kapak kolundaki kanaldan), alt lama L pimi; krank uclari BOSTA", "kafes_L",
     ["link_up", "scr_U", "link_lo", "nut_L", "scr_L"], "open"),
    ("Kafes R: pitch servo kolunu dis plakaya vidala (servo YOKKEN)", "kafes_R", ["frame_R", "horn_pitch", "scr_horn_pitch*"], "open"),
    ("Kafes R: kapaklar + mentese vidalari (goz yokken)", "kafes_R", ["lid_up_R", "lid_lo_R", "nut_lpin_R*", "scr_lpin_R*"], "open"),
    ("Kafesleri yan yana koy; kapak bindirme vidalari (M3) KAPAKLAR KAPALIYKEN (GOZLERDEN ONCE)", "kafes_L+kafes_R",
     ["nut_lap_up*", "scr_lap_up*", "nut_lap_lo*", "scr_lap_lo*"], "closed"),
    ("Goz alt montajini IKI kafese birlikte arkadan kaydir; ust/alt goz pimleri", "kafes_L+gozler",
     ["nut_ptop_*", "scr_ptop_*", "nut_pbot_*", "scr_pbot_*"], "open"),
    ("Kapak servosunu kanada tak (lamalari yukari cevir, yol acilsin); M3 MILE UZAK kulaktan", "kafes_L",
     ["servo_lid", "cable_lid", "nut_m3_lid0", "scr_m3_lid0"], "open"),
    ("Kapak krank alt montaji: servo kolunu kranka vidala (servo YOKKEN); krank somunu cebe", "lidkrank",
     ["lid_crank", "horn_lid", "scr_horn_lid*", "nut_A"], "open"),
    ("Kapak krankini servoya gecir; A pimi (iki lama + krank)", "kafes_L+lidkrank", ["scr_A"], "open"),
    ("Yaw servosunu tutucuya tak; M3 MILE UZAK kulaktan", "yaw", ["yaw_mount", "servo_yaw", "cable_yaw", "nut_m3_yaw0", "scr_m3_yaw0"], "open"),
    ("Yaw krank alt montaji: servo kolunu kranka vidala (servo YOKKEN); krank ucu somunu cebe", "yawkrank",
     ["yaw_crank", "horn_yaw", "scr_horn_yaw*", "nut_ctip"], "open"),
    ("Yaw tutucuyu kafes altina (kirisin altindan havsa)", "kafes_L+yaw", ["nut_ym_*", "scr_ym_*"], "open"),
    ("Yaw krankini servoya gecir; krank ucunu baglanti cubuguna vidala", "kafes_L+yawkrank", ["scr_ctip"], "open"),
    ("Cerceveyi tabana: pitch kolu servoya, sol pim brakete", "taban+kafes_L", ["nut_fpivot", "scr_fpivot"], "open"),
    ("Maske (en son, alttan havsa)", "taban", ["mask", "nut_mask_*", "scr_mask_*"], "open"),
]

P = E.build(verbose=False)
_poz_cache = {}


def Ms_for(kapak):
    if kapak not in _poz_cache:
        _poz_cache[kapak] = E.pose_transforms(0.0, 0.0, kapak if kapak in ("open", "closed", "half") else float(kapak))
    return _poz_cache[kapak]


_sh_cache = {}


def at_pose(name, kapak):
    key = (name, kapak)
    if key not in _sh_cache:
        part = P[name]; Ms = Ms_for(kapak)
        _sh_cache[key] = E.xform(part.shape, np.asarray(Ms[part.group])) if part.group in Ms else part.shape
    return _sh_cache[key]


def bb_kesisir(a, b):
    return not (a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin)


def bit_boyu(n):
    for d, v in KISA_UC.items():
        if fnmatch.fnmatchcase(n, d):
            return v
    return BIT_L


def koridor(n, kapak, bas=0.0, son=None):
    ust, a, hr, L = KAYIT[id(P[n].shape)]
    part = P[n]; Ms = Ms_for(kapak)
    M = np.asarray(Ms[part.group]) if part.group in Ms else np.eye(4)
    u = (M @ np.append(ust, 1))[:3]; aw = M[:3, :3] @ a
    br = BIT_R_KUCUK if n.startswith("scr_horn_") else BIT_R
    son = L + bit_boyu(n) if son is None else son
    parcalar = []
    if bas < L:
        parcalar.append(E.cyl(u - aw * (0.05 + bas), u - aw * min(L, son), hr + HEAD_PAY))
    if son > L:
        parcalar.append(E.cyl(u - aw * max(L, bas), u - aw * son, min(br, hr + HEAD_PAY)))
    return E.U(*parcalar), L


def denetle(n, mevcut, kapak):
    kor, L = koridor(n, kapak)
    kb = kor.BoundingBox()
    engel = []
    for m in mevcut:
        if m == n or P[m].kind == "visual":
            continue
        sh = at_pose(m, kapak)
        if not bb_kesisir(kb, sh.BoundingBox()):
            continue
        try:
            v = E.I(kor, sh).Volume()
        except Exception:
            v = 0.0
        if v > ESIK:
            engel.append((m, round(v, 1)))
    uyari = [(m, v) for m, v in engel if any(fnmatch.fnmatchcase(n, a) and fnmatch.fnmatchcase(m, b) and v <= lim
                                             for a, b, lim in KABUL)]
    engel = [x for x in engel if x not in uyari]
    if not engel:
        return ("UYARI" if uyari else "OK"), uyari, None, L
    adaylar = [at_pose(m, kapak) for m, _ in engel]
    serbest = L + bit_boyu(n)
    for i in range(int(L + bit_boyu(n))):
        dil, _ = koridor(n, kapak, i, i + 1)
        if any(E.I(dil, sh).Volume() > DILIM_ESIK for sh in adaylar):
            serbest = float(i); break
    return ("IMKANSIZ" if serbest < L else "ENGEL"), sorted(engel, key=lambda x: -x[1]), serbest, L


def calistir(detay=False, takili=False):
    tum = set(P)
    satirlar = []
    if takili:
        for n in sorted(tum):
            if n.startswith("scr_"):
                satirlar.append(("(hepsi takili)", n, *denetle(n, tum, "open")))
    else:
        alt = {}; yerlesen = set()
        for i, (baslik, sa, desen, kapak) in enumerate(SIRA, 1):
            adlar = sa.split("+")
            hedef = adlar[0]
            alt[hedef] = set().union(*[alt.pop(x, set()) for x in adlar])
            yeni = [n for n in sorted(tum - yerlesen) if any(fnmatch.fnmatchcase(n, d) for d in desen)]
            yerlesen |= set(yeni)
            for n in [x for x in yeni if not x.startswith("scr_")]:
                alt[hedef].add(n)
            for n in [x for x in yeni if x.startswith("scr_")]:
                satirlar.append((f"{i:2d}. {baslik}", n, *denetle(n, alt[hedef], kapak)))
                alt[hedef].add(n)
        eksik = sorted(n for n in tum - yerlesen if P[n].kind != "visual")
        if eksik:
            print("SIRADA OLMAYAN PARCALAR:", eksik)
    adim = None
    for a, n, d, e, sb, L in satirlar:
        if detay and a != adim:
            print(a); adim = a
        if detay or d != "OK":
            ek = f"serbest {sb:4.0f} mm / vida {L:4.1f} mm   {e}" if d in ("ENGEL", "IMKANSIZ") else (f"bilinen temas {e}" if d == "UYARI" else "")
            print(f"   {d:8s} {n:16s} {ek}" if detay else f"  {d:8s} {n:16s} [{a}] {ek}")
    ims = sum(1 for r in satirlar if r[2] == "IMKANSIZ"); eng = sum(1 for r in satirlar if r[2] == "ENGEL")
    uy = sum(1 for r in satirlar if r[2] == "UYARI")
    print(f"SONUC ({'hepsi takili' if takili else 'sirali'}): vida {len(satirlar)}, imkansiz {ims}, engel (tornavida) {eng}, uyari {uy}; "
          f"koridor = kafa (r+{HEAD_PAY}) x vida boyu + uc (r {BIT_R}) x {BIT_L} mm")
    return ims, eng


if __name__ == "__main__":
    calistir("--detay" in sys.argv, "--takili" in sys.argv)
