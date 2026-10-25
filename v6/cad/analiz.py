"""Project Eye v6 - kutle / AM / tork (SECENEK B) / devrilme verisi (check.py bunu kullanir; tek basina: tablo yazar).

  PYTHONUTF8=1 python -B analiz.py      -> out/analiz.json + ekrana tablo
Kutle kurali (PLAN §5.0): basili = hacim x PLA 1.24 x dolgu; donanim = hacim x celik 7.85 (vidalar kok capta modelli, hafif eksik);
satin alinan: SG90 9 g, MG90S 13.4 g, kol 1 g (AM = govde modelinin hacim merkezi), kaymaz ayak 0.3 g. Kama dolgusu 1.0
(duvarlar 1.6 mm = 3 cevre + bosluk dolgusu: dilimleyicide dolu basilir; tur 1 oncesi 2-2.4).
"""
import json
import os
import sys

import numpy as np

import eye_v6 as E6
import kin_v6 as K6
from params_v6 import *
from checks.yercekimi_tork import kutle_tablosu, eylemsizlik, dinamik_tork, tork_egrisi

SATIN_G = {"SG90": SG90_G, "MG90S": MG90S_G, "SG90 kolu (cift)": KOL_G, "kaymaz ayak O10": AYAK_G}


def malzeme(p, dolgu):
    if p.kind == "printed":
        return "PLA_kama" if p.bom_key == "kama" else "PLA"
    if p.kind == "hardware":
        return "celik"
    if p.kind == "purchased":
        g = SATIN_G.get(p.bom_key)
        return None if g is None else ("kutle", g)          # kablo cikisi modelleri: servo kutlesine dahil
    return None


def kutleler(P, dolgu=DOLGU_ESAS, secim=None, M=None):
    """[(ad, g, am, I)] - secim(ad, part) True olanlar; M: {grup: 4x4} verilirse o pozda (dunya)."""
    yog = {"PLA": PLA_RHO * dolgu, "PLA_kama": PLA_RHO * 1.0, "celik": CELIK_RHO}
    parc = {}
    for n, p in P.items():
        if p.kind == "visual" or (secim and not secim(n, p)):
            continue
        mal = malzeme(p, dolgu)
        if mal is None:
            continue
        parc[n] = ((p.shape, M[p.group]) if M is not None else p.shape, mal)
    return kutle_tablosu(parc, yog)


def donen(n, p):
    return p.group != "static"


def tork_ozet(P, dolgu=DOLGU_ESAS, alfa=TILT_DEG):
    kl = kutleler(P, dolgu, donen)
    I = eylemsizlik(kl, (0, 0, 0), (1, 0, 0))
    dyn = dinamik_tork(I, HAREKET_DEG, HAREKET_S)
    acilar = list(np.arange(PITCH_MIN, PITCH_MAX + 0.01, 1.0))
    egri, M, am = tork_egrisi(kl, (0, 0, 0), (-1, 0, 0), K6.masa_yukari_mek(alfa), acilar)
    Tmax = max(abs(T) for _, T, _ in egri)
    gerek = Tmax + dyn + SURTUNME_KGCM
    capB = (KAP_MG90S + KAP_SG90) * IKI_SERVO_VERIM
    return dict(dolgu=dolgu, alfa=alfa, kutle_g=M, am=am.tolist(), I_kgm2=I, dinamik=dyn, Tmax=Tmax, gereken=gerek,
                pay_B=capB / gerek, pay_A=KAP_MG90S / gerek, kapB=capB, egri=[(q, T) for q, T, _ in egri])


def main():
    P = E6.build(verbose=False)
    out = {"tork": [tork_ozet(P, d) for d in DOLGU_TARAMA],
           "tork_alfa": [tork_ozet(P, DOLGU_ESAS, a) for a in (20.0, 30.0, 40.0)]}
    yeni = {n: p for n, p in P.items() if p.bom_key in E6.YENI and p.kind == "printed"}
    out["yeni_parca_g"] = {n: round(m, 1) for n, m, _, _ in kutleler(yeni, 0.75)}
    out["toplam_g"] = round(sum(m for _, m, _, _ in kutleler(P)), 1)
    os.makedirs(E6.OUT, exist_ok=True)
    with open(os.path.join(E6.OUT, "analiz.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1, default=float)
    for t in out["tork"] + out["tork_alfa"]:
        print(f"alfa {t['alfa']:.0f} dolgu {t['dolgu']}: donen {t['kutle_g']:.0f} g AM {np.round(t['am'], 1)} I {t['I_kgm2']:.2e} "
              f"Tmax {t['Tmax']:.3f} din {t['dinamik']:.3f} gereken {t['gereken']:.3f} -> pay B {t['pay_B']:.2f} (A tek MG90S {t['pay_A']:.2f})")
    print("yeni parcalar (dolgu 0.75):", out["yeni_parca_g"], "toplam", out["toplam_g"])


if __name__ == "__main__":
    main()
