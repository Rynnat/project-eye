"""Project Eye v4 - carpisma / hareket taramasi + baski (STL) kontrolleri -> out/check_report.txt

  PYTHONUTF8=1 python check.py            # zorunlu 18 poz + ara tarama + STL kontrolleri
  PYTHONUTF8=1 python check.py --quick    # sadece zorunlu 18 poz

Yontem:
  * CadQuery katilari TOL_TESS sapmayla ucgenlenir; kesisim hacmi manifold3d boolean, mesafe python-fcl.
    ~TOL_TESS altindaki farklar 'temas' sayilmali.
  * Vidalar kok capa yakin (M4 O3.4, M3 O2.4) modellendi: dis plastige gomulur -> pilot delikle cakismaz.
  * Ayni hareket grubundaki parcalar (birbirine vidali) poz basina kontrol edilmez; notr pozda ayrica kontrol edilir.
  * Eklem ciftleri (eye_v5.JOINTS: pim-delik, AXIAL_GAP ile yan yana donen yuzler, kol-servo) tasarim geregi
    temas/0.4 bosluk; ASIL carpisma payi eklem DISI ciftlerin en kucuk boslugudur.
  * STL kontrolu (out/stl/*.stl, baski yonunde): (1) tek bagli govde, (2) en alt z = 0 ve 0.2 mm katmanlarla
    dilimlendiginde hicbir ada bir alt katmana degmeden havada baslamaz, (3) >45 deg sarkma alani (BOM'da isaretli).
"""
import glob
import itertools
import math
import os
import sys
import time

import numpy as np
import trimesh
import manifold3d as m3d
import fcl

import kin

from eye_v5 import build, pose_transforms, servo_angles, tess, OUT, JOINTS, print_mesh, FASTENERS
from params import *

TOL_TESS = 0.01
REPORT = os.path.join(OUT, "check_report.txt")
LAYER = 0.2


def is_joint(a, b):
    return tuple(sorted((a, b))) in JOINTS


def to_manifold(v, t):
    mesh = m3d.Mesh(vert_properties=np.asarray(v, np.float32), tri_verts=np.asarray(t, np.uint32))
    return m3d.Manifold(mesh)


class Body:
    def __init__(self, name, part):
        self.name = name; self.part = part; self.group = part.group
        v, t = tess(part.shape, TOL_TESS, 0.1)
        tm = trimesh.Trimesh(v, t, process=True, validate=True)
        tm.merge_vertices()
        self.watertight = tm.is_watertight
        self.v = np.asarray(tm.vertices); self.t = np.asarray(tm.faces)
        self.man = to_manifold(self.v, self.t)
        self.man_ok = self.man.status() == m3d.Error.NoError
        bvh = fcl.BVHModel(); bvh.beginModel(len(self.v), len(self.t)); bvh.addSubModel(self.v, self.t); bvh.endModel()
        self.bvh = bvh

    def aabb(self, M):
        c = np.array(list(itertools.product(*zip(self.v.min(0), self.v.max(0)))))
        w = c @ M[:3, :3].T + M[:3, 3]
        return w.min(0), w.max(0)


def m34(M):
    return [list(map(float, M[i])) for i in range(3)]


def pair_check(b1, M1, b2, M2, dist_margin=3.0):
    lo1, hi1 = b1.aabb(M1); lo2, hi2 = b2.aabb(M2)
    gap = np.maximum(0, np.maximum(lo1 - hi2, lo2 - hi1))
    if np.linalg.norm(gap) > dist_margin:
        return 0.0, None
    vol = 0.0
    if np.all(gap == 0):
        vol = (b1.man.transform(m34(M1)) ^ b2.man.transform(m34(M2))).volume()
    req = fcl.DistanceRequest(enable_nearest_points=False, enable_signed_distance=False)
    res = fcl.DistanceResult()
    d = fcl.distance(fcl.CollisionObject(b1.bvh, fcl.Transform(M1[:3, :3], M1[:3, 3])),
                     fcl.CollisionObject(b2.bvh, fcl.Transform(M2[:3, :3], M2[:3, 3])), req, res)
    return vol, max(d, 0.0)


def run_pose(bodies, yaw, pitch, lid):
    Ms = pose_transforms(yaw, pitch, lid)
    names = list(bodies); res = []
    for i in range(len(names)):
        bi = bodies[names[i]]
        for j in range(i + 1, len(names)):
            bj = bodies[names[j]]
            if bi.group == bj.group:
                continue
            vol, d = pair_check(bi, Ms[bi.group], bj, Ms[bj.group])
            if d is not None:
                res.append((names[i], names[j], vol, d))
    return res


def intra_group(bodies):
    Ms = pose_transforms(0, 0, "closed")
    out = []; names = list(bodies)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            bi, bj = bodies[names[i]], bodies[names[j]]
            if bi.group != bj.group:
                continue
            vol, d = pair_check(bi, Ms[bi.group], bj, Ms[bj.group], 0.5)
            if d is not None:
                out.append((names[i], names[j], vol, d))
    return out


def wall_thickness(part, n=4000, seed=1):
    v, t = tess(part.shape, 0.01, 0.1)
    m = trimesh.Trimesh(v, t, process=True)
    pts, fid = trimesh.sample.sample_surface(m, n, seed=seed)
    nrm = m.face_normals[fid]
    orig = pts - nrm * 1e-3
    loc, idx_ray, idx_tri = m.ray.intersects_location(orig, -nrm, multiple_hits=False)
    if len(idx_ray) == 0:
        return None
    th = np.linalg.norm(loc - orig[idx_ray], axis=1)
    keep = -(m.face_normals[idx_tri] * nrm[idx_ray]).sum(1) > 0.8
    th = th[keep]; where = loc[keep]
    if len(th) == 0:
        return None
    k = int(np.argmin(th))
    return float(th.min()), float(np.percentile(th, 1)), float(np.percentile(th, 5)), where[k]


# ------------------------------------------------------------------ STL (baski yonu) kontrolleri
def overhang_area(m, crit=45.0, bed_tol=0.3):
    nz = m.face_normals[:, 2]; zc = m.triangles_center[:, 2]
    down = nz < -math.cos(math.radians(crit))
    return float(m.area_faces[down & (zc > bed_tol)].sum()), float(m.area_faces[down & (zc <= bed_tol)].sum())


def floating_islands(m, layer=LAYER):
    """0.2 mm katmanlarla dilimle; her katmandaki her ada bir alt katmandaki malzemeye degmeli. Havada baslayan adalar:
    [(z, alan mm2)]. (Kopru yapan tavanlar ada olusturmaz; sadece tamamen yeni baslayan kesitler.)"""
    man = to_manifold(m.vertices, m.faces)
    zmax = m.bounds[1][2]
    prev = None; bad = []
    z = layer / 2
    while z < zmax:
        cs = man.slice(z)
        if prev is not None and not cs.is_empty():
            for comp in cs.decompose():
                a = comp.area()
                if a < 0.05:
                    continue
                ov = (comp ^ prev.offset(0.05, m3d.JoinType.Miter)).area() if not prev.is_empty() else 0.0
                if ov <= 1e-6:
                    bad.append((round(z, 2), round(a, 2)))
        prev = cs
        z += layer
    return bad


def stl_checks(L):
    L("7) STL / BASKI YONU KONTROLU (out/stl/*.stl oldugu gibi okunur)")
    L("   (1) tek bagli govde  (2) min z = 0, 0.2 mm katmanlarda havada baslayan ada yok  (3) >45 deg sarkma alani")
    fails = []
    for f in sorted(glob.glob(os.path.join(OUT, "stl", "*.stl"))):
        m = trimesh.load(f)
        nb = len(trimesh.graph.connected_components(m.face_adjacency, nodes=np.arange(len(m.faces))))
        if not m.is_watertight:
            L(f"   ! {os.path.basename(f)} su gecirmez degil (ucgenleme)")
        zmin = float(m.bounds[0][2])
        isl = floating_islands(m)
        ov, bed = overhang_area(m)
        ok = nb == 1 and abs(zmin) < 1e-3 and not isl
        if not ok:
            fails.append(os.path.basename(f))
        L(f"   {os.path.basename(f):22s} govde {nb}  zmin {zmin:6.3f}  havada ada {len(isl):2d} {isl[:4] if isl else ''}"
          f"  sarkma {ov:6.1f} mm2  tabla {bed:7.1f} mm2  boyut {np.round(m.extents, 1)}  {'OK' if ok else '<-- HATA'}")
    L(f"   STL SONUC: {'hepsi OK' if not fails else 'HATA: ' + str(fails)}")
    L("")
    return fails


# ------------------------------------------------------------------ kapali pozda iris gorunurlugu (id-tamponu)
def iris_visible(P, cache, yaw, pitch, lid, hide=(), view=(0.0, 0.0), px=0.1, rows=False):
    """Onden (dik izdusum, bakis yonu -Z; view=(yaw_deg, pitch_deg) ile egik) z-tamponu. Donus: {goz: gorunen iris+bebek
    alani mm2}. Iris/bebek ucgenleri id ile isaretlenir; en ondeki yuzey iris ise piksel sayilir."""
    from render import raster
    Ms = pose_transforms(yaw, pitch, lid)
    Rv = (kin.R(kin.EY, view[0]) @ kin.R(kin.EX, -view[1]))[:3, :3]
    tris = []; ids = []
    for n, p in P.items():
        if any(n.startswith(h) for h in hide):
            continue
        v, t = cache[n]
        M = Ms[p.group]
        vv = (v @ M[:3, :3].T + M[:3, 3]) @ Rv          # kameraya gore (kamera +Z'den bakar)
        tag = 1.0 if n.startswith("iris_L") or n.startswith("pupil_L") else 2.0 if (n.startswith("iris_R") or n.startswith("pupil_R")) else 0.0
        tris.append(vv[t]); ids.append(np.full(len(t), tag))
    T = np.concatenate(tris); idv = np.concatenate(ids)
    x0, x1, y0, y1 = -75.0, 75.0, -30.0, 30.0
    W = int((x1 - x0) / px); H = int((y1 - y0) / px)
    tri2 = np.stack([(T[..., 0] - x0) / px, (y1 - T[..., 1]) / px], -1).astype(np.float64)
    depth = (-T[..., 2]).astype(np.float64)
    col = np.zeros((len(T), 3)); col[:, 0] = idv
    zbuf = np.full((H, W), 1e18); img = np.zeros((H, W, 3))
    raster(tri2, depth, col, W, H, zbuf, img)
    if rows:
        ys = np.nonzero((img[..., 0] == 1).any(1))[0]
        return (y1 - ys.min() * px, y1 - ys.max() * px) if len(ys) else (None, None)
    return {"L": float((img[..., 0] == 1).sum()) * px * px, "R": float((img[..., 0] == 2).sum()) * px * px}


def iris_section(P, L):
    import kin as _k  # noqa
    cache = {n: tess(p.shape, 0.03, 0.2) for n, p in P.items()}
    L("10) KAPALI POZDA IRIS + BEBEK GORUNUR ALANI (onden z-tamponu, 0.1 mm piksel; hedef 0)")
    worst = 0.0
    for hide, lab in (((), "maske ile"), (("mask",), "maske YOK (yalniz kapaklar)")):
        for view in ((0, 0), (15, 0), (-15, 0), (0, 12), (0, -12)):
            rows = []
            for y in (-25, 0, 25):
                for p in (-20, 0, 20):
                    a = iris_visible(P, cache, y, p, "closed", hide, view)
                    rows.append(a["L"] + a["R"])
            worst = max(worst, max(rows))
            L(f"   {lab:28s} bakis (yaw {view[0]:+3d}, pitch {view[1]:+3d}): 9 goz pozunda en cok {max(rows):.2f} mm2")
    ref = iris_visible(P, cache, 0, 0, "open", ("mask",))
    L(f"   (karsilastirma: kapak ACIK, maskesiz, notr: {ref['L']:.0f} + {ref['R']:.0f} mm2 gorunur)")
    top, bot = iris_visible(P, cache, 0, 0, "open", ("mask",), rows=True)
    L(f"   kapak ACIK, notr, onden: gorunen iris ust siniri y = {top:.2f} (iris tepesi +{IRIS_R:.0f}) -> ust kapak irisin "
      f"{IRIS_R - top:.2f} mm'sini orter; alt sinir y = {bot:.2f} (iris dibi -{IRIS_R:.0f}; alt kapak {'irisin altinda' if bot <= -IRIS_R + 0.15 else 'irise biniyor'})")
    L(f"   IRIS SONUC: kapali pozda en cok {worst:.2f} mm2 gorunur")
    L("")
    return worst


def main():
    quick = "--quick" in sys.argv
    t0 = time.time()
    P = build(verbose=False)
    bodies = {n: Body(n, p) for n, p in P.items() if p.kind != "visual"}
    lines = []; L = lines.append
    L("PROJECT EYE v5 (v4 + somunlu baglantilar) - MEKANIK KONTROL RAPORU")
    L(f"uretim: check.py | ucgenleme {TOL_TESS} mm | kesisim esigi {INTERFERENCE_TOL_MM3} mm3 | eklem disi bosluk hedefi {TARGET_CLEARANCE} mm")
    L("")
    L("1) KATI GECERLILIGI")
    inval = [n for n, b in bodies.items() if not b.part.shape.isValid()]
    multi = [n for n, b in bodies.items() if b.part.kind == "printed" and len(b.part.shape.Solids()) != 1]
    nwt = [n for n, b in bodies.items() if not (b.watertight and b.man_ok)]
    keys = sorted({b.part.bom_key for b in bodies.values() if b.part.kind == "printed"})
    L(f"   parca sayisi {len(bodies)} (basilan {sum(b.part.kind == 'printed' for b in bodies.values())}, farkli basilan STL {len(keys)}: {keys})")
    L(f"   gecersiz kati: {inval or 'yok'} | tek-kati olmayan basilan: {multi or 'yok'} | su gecirmez olmayan ag: {nwt or 'yok'}")
    L("")
    L("2) GRUP ICI (birbirine sabit parcalar, notr poz) - kesisim >= esik:")
    bad_ig = [r for r in intra_group(bodies) if r[2] >= INTERFERENCE_TOL_MM3]
    for a, b, vol, d in bad_ig:
        L(f"   ! {a} x {b}: {vol:.3f} mm3")
    if not bad_ig:
        L("   yok")
    L("   not: ~0.01-0.02 mm3 = havsa kafa konisi ile yuvasi ayni yuzey (ucgenleme). scr_m3_* x servo_* GERCEK:")
    L("        SG90 kulak deligi govdeden ~2.5 mm; M3 kafa (O5.6) govde duvarina biniyor (DECISIONS.md, risk; v3'ten).")
    L("")
    poses = [(y, p, s) for s in ("open", "closed") for y in CHECK_YAWS for p in CHECK_PITCHES]
    extra = []
    if not quick:
        extra = [(y, p, s) for s in (0.0, 0.5, 1.0) for y in (-25, -12.5, 0, 12.5, 25) for p in (-20, -10, 0, 10, 20)
                 if (y, p, {0.0: "closed", 1.0: "open"}.get(s, s)) not in poses]
    L(f"3) HAREKET TARAMASI: zorunlu {len(poses)} poz (yaw {CHECK_YAWS} x pitch {CHECK_PITCHES} x kapak acik/kapali)"
      + (f" + ara tarama {len(extra)} poz (yaw 12.5, pitch 10 deg adim, kapak 0/0.5/1)" if extra else ""))
    L(f"   ust kapak kenari {UP_EDGE_CLOSED:+.1f} (dudak {UP_EDGE_CLOSED - LIP_DROP:+.1f}) -> {UP_EDGE_CLOSED + UP_OPEN:+.1f} deg; alt kapak {LO_EDGE_CLOSED:+.1f} -> {LO_EDGE_CLOSED - kin.LO_OPEN_ACT:+.1f} deg")
    total_bad = 0; per_pair = {}; rows = []
    for (y, p, s) in poses + extra:
        res = run_pose(bodies, y, p, s)
        bad = [r for r in res if r[2] >= INTERFERENCE_TOL_MM3]
        total_bad += len(bad)
        nj = [r for r in res if not is_joint(r[0], r[1])]
        mn = min(nj, key=lambda r: r[3]) if nj else None
        for a, b, vol, d in res:
            if (a, b) not in per_pair or d < per_pair[(a, b)][0]:
                per_pair[(a, b)] = (d, (y, p, s))
        rows.append((y, p, s, len(bad), mn, bad, (y, p, s) in poses))
        print(f"poz {y:6.1f} {p:6.1f} {str(s):6s}: cakisma {len(bad)} eklem-disi min {mn[3] if mn else float('nan'):.3f} "
              f"({mn[0] if mn else ''} x {mn[1] if mn else ''})", flush=True)
    L("   zorunlu pozlar:  yaw   pitch  kapak  | cakisma | eklem disi min bosluk | en yakin cift")
    for (y, p, s, nb, mn, bad, req) in rows:
        if req:
            L(f"                   {y:5.1f} {p:5.1f}  {str(s):6s} | {nb:7d} | {mn[3]:10.3f} | {mn[0]} x {mn[1]}")
            for a, b, vol, d in bad:
                L(f"        ! {a} x {b}: {vol:.3f} mm3")
    if extra:
        L(f"   ara tarama: cakisma {sum(r[3] for r in rows if not r[6])}; eklem disi en kucuk bosluk "
          f"{min(r[4][3] for r in rows if not r[6] and r[4]):.3f} mm")
        for (y, p, s, nb, mn, bad, req) in rows:
            if not req:
                for a, b, vol, d in bad:
                    L(f"        ! yaw {y} pitch {p} kapak {s}: {a} x {b}: {vol:.3f} mm3")
    L("")
    L(f"   TOPLAM CAKISMA (>= {INTERFERENCE_TOL_MM3} mm3): {total_bad}")
    items = sorted(per_pair.items(), key=lambda kv: kv[1][0])
    nj = [it for it in items if not is_joint(*it[0])]
    jj = [it for it in items if is_joint(*it[0])]
    L("4a) EKLEM DISI CIFTLERDE EN DAR 20 BOSLUK (asil carpisma payi):")
    for (a, b), (d, pose) in nj[:20]:
        L(f"   {d:7.3f}  {a} x {b}   @ yaw {pose[0]} pitch {pose[1]} kapak {pose[2]}")
    L("4b) EKLEM CIFTLERI (tasarim geregi temas / AXIAL_GAP; en dar 12):")
    for (a, b), (d, pose) in jj[:12]:
        L(f"   {d:7.3f}  {a} x {b}")
    L("")
    L("5) ET KALINLIGI (isin yontemi; min / %1 / %5, mm) - hedef >= 1.6:")
    seen = set(); thin = []
    for n, b in bodies.items():
        if b.part.kind != "printed" or b.part.bom_key in seen:
            continue
        seen.add(b.part.bom_key)
        r = wall_thickness(b.part)
        if r is None:
            continue
        mn_, p1, p5, where = r
        flag = "  <-- 1.6 alti" if p1 < MIN_WALL - 0.05 else ""
        if flag:
            thin.append(b.part.bom_key)
        L(f"   {b.part.bom_key:14s} min {mn_:5.2f}  %1 {p1:5.2f}  %5 {p5:5.2f}  (en ince ~ {np.round(where, 1)}){flag}")
    L("")
    L("6) BASKIDA YATAY DELIKLER (+%.1f mm pay verildi; DECISIONS.md):" % HORIZ_HOLE_EXTRA)
    seen = set()
    for n, p in P.items():
        if p.kind == "printed" and p.horizontal_holes and p.bom_key not in seen:
            seen.add(p.bom_key)
            L(f"   {p.bom_key:14s} {', '.join(p.horizontal_holes)}")
    L("")
    fails = stl_checks(L)
    L("8) VIDA SAYILARI (montaj kaydindan):")
    cnt = {}
    for n, p in P.items():
        if p.kind in ("hardware", "purchased"):
            cnt[p.bom_key] = cnt.get(p.bom_key, 0) + 1
    for k_, v_ in sorted(cnt.items()):
        L(f"   {v_:3d}  {k_}")
    m3 = cnt.get("M3x10 yuvarlak", 0)
    L(f"   M3 toplam {m3} (sinir 14) -> {'OK' if m3 <= 14 else 'ASIM!'}")
    L("8b) VIDA BOYU / SOMUN KONTROLU: vida ucu somunun dis yuzunden en az 1 dis (M3 0.5, M4 0.7) cikmali;")
    L("    vidalarin/somunlarin baska parcaya carpmamasi bolum 2-4'teki carpisma taramasinda (vida ve somun katilari dahil).")
    bad_len = []
    for scr, nut, typ, beyond in FASTENERS:
        ok = beyond >= THREAD_PITCH[typ] - 1e-6
        if not ok:
            bad_len.append(scr)
        L(f"   {scr:16s} + {nut:16s} {typ}: uc somundan {beyond:5.2f} mm tasar  {'OK' if ok else '<-- KISA'}")
    L(f"   somunlu baglanti {len(FASTENERS)}, kisa vida: {bad_len or 'yok'}")
    pil = [n for n in ("scr_U",) if n in P]
    L(f"   KILAVUZSUZ (plastige vidalanan) kalan: {pil} (M4, ust kapak tabi; bkz. DECISIONS) + SG90 kol vidalari (servo ile gelen)")
    L("")
    L("9) SERVO ACILARI (derece) + kapak acilari:")
    for l_ in (0, 0.25, 0.5, 0.75, 1.0):
        du, dl, b_ = kin.lid_deltas(l_)
        tr = kin.transmission(l_)
        L(f"   kapak {l_:4.2f}: ust +{du:5.2f} deg, alt -{dl:5.2f} deg, krank {b_:6.2f} deg, LIDS {kin.servo_angles(0,0,l_)['LIDS']:6.2f};"
          f" lama iletim sapmasi (0 ideal) ust {tr[0]:4.1f} alt {tr[1]:4.1f} krank-ust {tr[2]:4.1f} krank-alt {tr[3]:4.1f}")
    for (y, p, s) in ((0, 0, "closed"), (-25, 0, "closed"), (25, 0, "closed"), (0, -20, "closed"), (0, 20, "closed"),
                      (0, 0, "open"), (0, 0, 0.25), (0, 0, 0.5), (0, 0, 0.75)):
        a = servo_angles(y, p, s)
        L(f"   yaw {y:4} pitch {p:4} kapak {str(s):6s}: " + "  ".join(f"{k} {v:6.1f}" for k, v in a.items()))
    L("")
    lidgap = [(d, a_, b_, pose) for (a_, b_), (d, pose) in per_pair.items()
              if (a_ in ("eye_L", "eye_R") and b_.startswith("lid_") or b_ in ("eye_L", "eye_R") and a_.startswith("lid_"))
              and a_[-1] == b_[-1]]
    lg = min(lidgap) if lidgap else None
    L(f"9b) KAPAK - GOZ BOSLUGU (kapak ic yuzu - goz, butun pozlar): en kucuk {lg[0]:.3f} mm ({lg[1]} x {lg[2]}), "
      f"hedef <= {LID_EYE_GAP_MAX} ve >= {TARGET_CLEARANCE}  (tasarim: LID_R_IN - EYE_R = {LID_R_IN - EYE_R:.2f})")
    for d, a_, b_, pose in sorted(lidgap):
        L(f"   {d:6.3f}  {a_} x {b_}")
    L("")
    iris_worst = iris_section(P, L)
    njm = f"{nj[0][1][0]:.3f} mm ({nj[0][0][0]} x {nj[0][0][1]} @ {nj[0][1][1]})" if nj else "-"
    L(f"SONUC: cakisma {total_bad}, eklem disi en kucuk bosluk {njm}, gecersiz kati {len(inval)}, "
      f"1.6 alti et (%1) {thin or 'yok'}, STL hatasi {fails or 'yok'}, M3 {m3}/14, farkli STL {len(keys)}/19, "
      f"kapak-goz {lg[0]:.2f} mm, kapali iris {iris_worst:.2f} mm2, somunlu {len(FASTENERS)} (kisa vida {len(bad_len)})")
    L(f"sure {time.time() - t0:.0f} s")
    os.makedirs(OUT, exist_ok=True)
    with open(REPORT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
