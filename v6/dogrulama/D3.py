"""D3: BOM'da servo hatti kondansatoru var mi (envanterde var mi); gerilim cokmesi modelle OLCULEMEZ."""
import os, re, glob
HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
env = open(os.path.join(HERE, "envanter_plan-v2.md"), encoding="utf-8").read().split("\n")
print("envanter:", [l.strip() for l in env if re.search("1000", l)])
for f in ("PLAN.md", "MONTAJ.md", "BOM_FARK.md", "DECISIONS.md", "STATE.md"):
    t = open(os.path.join(V6, f), encoding="utf-8").read()
    print(f, "kondansator/uF gecis sayisi:", len(re.findall(r"kondans|µF|uF|capacitor", t, re.I)))
r14 = [l for l in open(os.path.join(V6, "MONTAJ.md"), encoding="utf-8").read().split("\n") if l.startswith("| R14")][0]
print("R14 olcum araci multimetre mi:", "multimetre" in r14, "| bandgap/MCUSR var mi:", bool(re.search("MCUSR|bandgap|readVcc", r14)))
