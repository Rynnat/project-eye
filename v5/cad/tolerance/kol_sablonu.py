"""SG90 cift kol delik yeri sablonu (olcmeden): ortadaki deligi kolun gobegine gecir, kolu cevir;
iki ucundaki deliklerin ikisi birden hangi siradaki deliklerle cakisiyor -> o siranin yaricapi.
Siralar 30 derece arayla, karsilikli cift delik: r = 5.5 / 6.0 / 6.5 / 7.0 / 7.5 / 8.0.
Sira numarasi = kenardaki oyuk nokta sayisi (1..6)."""
import math, sys, cadquery as cq
R_LIST = (5.5, 6.0, 6.5, 7.0, 7.5, 8.0)
DISC_R, T, HUB_D, HOLE_D = 22.0, 2.4, 7.6, 1.3
p = cq.Workplane("XY").circle(DISC_R).extrude(T)
p = p.cut(cq.Workplane("XY").circle(HUB_D / 2).extrude(T))
for i, r in enumerate(R_LIST):
    a = math.radians(i * 30)
    for s in (1, -1):
        p = p.cut(cq.Workplane("XY").center(s * r * math.cos(a), s * r * math.sin(a)).circle(HOLE_D / 2).extrude(T))
    for k in range(i + 1):                     # sira isareti: (i+1) oyuk nokta, yalniz + tarafta
        rr = 11.0 + k * 1.8
        p = p.cut(cq.Workplane("XY").workplane(offset=T - 0.6).center(rr * math.cos(a), rr * math.sin(a)).circle(0.55).extrude(1))
cq.exporters.export(p, sys.argv[1]); print("ok")
