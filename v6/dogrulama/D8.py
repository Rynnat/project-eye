"""D8: BOM_FARK envanter adetleri vs envanter.md@plan-v2 (2026-10-06)."""
import os, re
HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
env = open(os.path.join(HERE, "envanter_plan-v2.md"), encoding="utf-8").read().split("\n")
print("envanter SG90 satirlari:", [l.strip() for l in env if "SG90" in l])
print("envanter MG90S gecen satir sayisi:", sum("MG90S" in l for l in env))
bom = open(os.path.join(V6, "BOM_FARK.md"), encoding="utf-8").read().split("\n")
print("BOM_FARK:", [l.strip() for l in bom if re.search(r"\| (SG90|MG90S) \|", l)])
print("envanter M2 somun satiri:", [l.strip() for l in env if re.search(r"M2.*somun|somun.*M2", l, re.I)] or "yok (M2x8 'Somun: var' sutunu)")
