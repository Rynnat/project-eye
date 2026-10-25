"""D1: tek-servo en kotu pay (model hesabi) + MONTAJ'da yuk paylasimi icin sayisal kriter var mi."""
import os, sys, json, re
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(V6, "cad"))
from params_v6 import *
ozet = json.load(open(os.path.join(V6, "cad", "out", "check_ozet.json")))
for d, (pay, gerek, *_ ) in ozet["tork"].items():
    tek = KAP_SG90 / gerek
    print(f"dolgu {d}: gereken {gerek:.3f} kg*cm | iki servo (verim {IKI_SERVO_VERIM}) pay {pay:.2f} | TEK servo pay {tek:.2f} (esik {TORK_HEDEF})")
t = open(os.path.join(V6, "MONTAJ.md"), encoding="utf-8").read()
r14 = [l for l in t.split("\n") if l.startswith("| R14")][0]
print("R14 satiri:", r14)
print("sayisal paylasim/akim kriteri var mi:", bool(re.search(r"mA|akim|akım|paylas|paylaş", r14)))
