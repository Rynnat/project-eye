"""v5 somun cebi pay testi: soldan saga +0.2 / +0.3 / +0.4 / +0.5 anahtar agzi payi.
Ust sira M4 (AF 7.0, t 3.2), alt sira M3 (AF 5.5, t 2.4). Cep derinligi t + 0.2 (v5 ile ayni).
Sol ust kose pahli = yon isareti (+0.2 tarafi). Tek govde, destek gerekmez (cepler ustten acik)."""
import math, sys, cadquery as cq
CLRS = (0.2, 0.3, 0.4, 0.5)
ROWS = (("M4", 7.0, 3.2, 4.3, 16.0), ("M3", 5.5, 2.4, 3.3, 5.5))   # ad, AF, t, delik, satir y
T, PITCH = 5.0, 13.0
W, H = PITCH * len(CLRS) + 4, 27.0
plate = cq.Workplane("XY").box(W, H, T, centered=(False, False, False))
plate = plate.cut(cq.Workplane("XY").polyline([(0, H - 4), (0, H + 1), (4 + 1, H + 1)]).close().extrude(T))   # yon pahi
for name, af, t, hole, y in ROWS:
    for i, c in enumerate(CLRS):
        x = 2 + PITCH / 2 + i * PITCH
        d = (af + c) / math.cos(math.radians(30))            # altigenin kose-kose capi
        plate = plate.cut(cq.Workplane("XY").workplane(offset=T - (t + 0.2)).center(x, y).polygon(6, d).extrude(t + 0.2 + 1))
        plate = plate.cut(cq.Workplane("XY").center(x, y).circle(hole / 2).extrude(T))
out = sys.argv[1]
cq.exporters.export(plate, out)
print("ok", out, round(W, 1), "x", H, "x", T)
