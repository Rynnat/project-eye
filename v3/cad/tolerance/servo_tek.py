"""Tek SG90 cebi: kullanicinin olctugu servo (~22.5 x 12.5) + 0.4 toplam pay = 22.9 x 12.9, kablo yarikli.
v3'un servo tutucularindaki cep ile ayni olcu (params: SG_L 22.5, SG_W 12.5, SERVO_POCKET_CLR 0.4)."""
from pathlib import Path
import cadquery as cq

IW, IH = 22.9, 12.9          # cep ic olcusu
WALL, H = 2.0, 6.0
CABLE_SLOT_W = 6.0           # mile uzak kisa uc, alttan uste acik

ow, oh = IW + 2 * WALL, IH + 2 * WALL
p = (cq.Workplane("XY").box(ow, oh, H, centered=False)
     .cut(cq.Workplane("XY").center(WALL + IW / 2, WALL + IH / 2).rect(IW, IH).extrude(H))
     .cut(cq.Workplane("XY").center(WALL + IW + WALL / 2, WALL + IH / 2).rect(WALL + 0.2, CABLE_SLOT_W).extrude(H)))
v = p.val(); assert v.isValid()
out = Path(__file__).resolve().parent / "out" / "tolerans_servo_tek.stl"
cq.exporters.export(p, str(out), tolerance=0.01, angularTolerance=0.1)
bb = v.BoundingBox(); print(f"{out.name}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, hacim {v.Volume():.0f} mm3")
