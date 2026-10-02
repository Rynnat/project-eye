"""Havada vida/somun taramasi (PYTHONUTF8=1 python check_havada.py): her vida ve somun bir BASILAN parcaya ~temas etmeli.
Kullanici 2026-09-29 frame_R kapak servosu vidasini havada buldu; check.py bunu yakalamiyordu (carpisma degil)."""
import eye_v5 as E
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
P = E.build(verbose=False)
sh = lambda x: x.wrapped if hasattr(x, "wrapped") else x
def bb(s):
    b = Bnd_Box(); BRepBndLib.Add_s(sh(s), b); return b
pr = {n: p for n, p in P.items() if p.kind == "printed"}
pbb = {n: bb(p.shape) for n, p in pr.items()}
kotu = []
for n, p in P.items():
    if p.kind != "hardware": continue
    hb = bb(p.shape); hb.Enlarge(3.0)
    en = (99.0, None)
    for m, q in pr.items():
        if hb.IsOut(pbb[m]): continue
        d = BRepExtrema_DistShapeShape(sh(p.shape), sh(q.shape)); d.Perform()
        if d.Value() < en[0]: en = (d.Value(), m)
    if en[0] > 0.6: kotu.append((n, round(en[0], 2), en[1]))
    # vida: govdesi basili parca icinden ne kadar geciyor? (temas tek noktadan olabilir) -> ayrica 2 parcaya yakinlik
print("HAVADA (en yakin basili parcaya > 0.6 mm):", kotu or "yok")
