"""D2: spline kademe hatasi vs mekanik durma payi; yercekimi yonu; MONTAJ'da kalibrasyon sirasi."""
import os, sys, json, re
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(V6, "cad"))
import numpy as np
import eye_v6 as E6, kin_v6 as K6, analiz as AN
from params_v6 import *

ozet = json.load(open(os.path.join(V6, "cad", "out", "check_ozet.json")))
durma = ozet["sert_durma"]; pay = durma - PITCH_MAX
print(f"mekanik durma {durma:.2f} deg, komut siniri +{PITCH_MAX} -> pay {pay:.2f} deg (asim izni {PITCH_ASIM})")
# spline dis sayilari: TowerPro datasheet [KAYNAK, modelde yok]: SG90 21T, MG90S 25T
for ad, N in (("SG90 (sol)", 21), ("MG90S (sag)", 25)):
    hs = 180.0 / N
    p_asan = max(0.0, (hs - pay) / (2 * hs))
    print(f"{ad}: N={N} yarim kademe {hs:.2f} deg; kalibrasyonsuz hata durma yonunde > {pay:.2f} deg olasiligi {p_asan*100:.0f}%")
print(f"iki servo arasi en kotu goreli kacikliklik: {180/21 + 180/25:.1f} deg")

# yercekimi yonu: donen gruplarin AM yuksekligi pitch ile azaliyor mu (azaliyorsa yercekimi + yone iter)
P = E6.build(verbose=False)
Mma = K6.masa_M(TILT_DEG)
secim = lambda n, p: p.group != "static"
def am_z(q):
    Ms = K6.pose(0, q, 0.0)
    kl = AN.kutleler(P, DOLGU_ESAS, secim, M={g: Mma @ M for g, M in Ms.items()})
    m = sum(k[1] for k in kl); return sum(k[1] * k[2][2] for k in kl) / m
zs = {q: am_z(q) for q in (-10, 0, 10, 18, 21)}
print("AM masa yuksekligi (mm):", {q: round(z, 2) for q, z in zs.items()})
dusen = all(zs[a] > zs[b] for a, b in ((-10, 0), (0, 10), (10, 18), (18, 21)))
print("pitch artarken AM alcaliyor (yercekimi durma yonune iter):", dusen)

# MONTAJ sirasi
t = open(os.path.join(V6, "MONTAJ.md"), encoding="utf-8").read().split("\n")
sat = lambda pat: [i + 1 for i, l in enumerate(t) if re.search(pat, l, re.I)]
print("MONTAJ 'kalibr' satirlari:", sat("kalibr"), "| R14 satiri:", sat(r"\| R14"), "| T9:", sat(r"\| T9"))
print("MONTAJ kalibrasyon ADIMI (Takma tablosunda):", [l for l in t if re.match(r"\|\s*\**\w+\**\s*\|", l) and re.search("kalibr", l, re.I)])
