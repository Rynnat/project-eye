"""Project Eye v2 - carpisma ve hareket taramasi.

  PYTHONUTF8=1 python check.py            # zorunlu 18 poz + yogun tarama, out/check_report.txt
  PYTHONUTF8=1 python check.py --quick    # sadece zorunlu 18 poz

Yontem (dogrulugu ve sinirlari):
  * Parcalar CadQuery'den TOL_TESS (mm) sapma ile ucgenlenir (egri yuzeylerde gercek geometriden en fazla bu kadar
    iceride/disarida). Kesisim hacmi manifold3d boolean (ucgen agi uzerinde, kesin); mesafe python-fcl (ucgen agi).
    Bu yuzden ~TOL_TESS altindaki bosluk/kesisim degerleri "temas" olarak yorumlanmali.
  * Vidalar kok capinda (O1.6) modellendi: dis plastige gomulur, kilavuzsuz delik 1.7 -> vida ile delik cakismaz.
  * Pitch cubugunun basili kuresel yuvalari gercek geometriyle (R4 kure, O4.9 kovuk, 61 deg agiz konisi) modellendi:
    rotil saplamasinin boynu yuvaya degerse eklem aci sinirini asmis demektir ve kesisim olarak yakalanir.
  * Poz kumeleri: zorunlu 18 (yaw -30/0/30 x pitch -25/0/25 x kapak acik/kapali) + firmware-sinir pozlari
    (pitch servosu guvenli sabit sinirda, yaw izgarasi) + yogun ara tarama (--quick ile atlanir).
  * Ayni hareket grubundaki parcalar (birbirine vidali) poz basina kontrol edilmez; notr pozda ayrica kontrol edilir.
"""
import sys
import time
import itertools
import math
import os

import numpy as np
import trimesh
import manifold3d as m3d
import fcl

import eye_v2
from eye_v2 import build, pose_transforms, servo_angles, tess, OUT
from params import *

TOL_TESS = 0.01
REPORT = os.path.join(OUT, "check_report.txt")

# Tasarim geregi birbirine dayanan / eklem olusturan ciftler (pim-yuva, vida-yatak, kol-mil, top-yuva).
# Bunlarda bosluk = eklem boslugu (0 olabilir); ASIL carpisma payi bu ciftler DISINDAKI en kucuk bosluktur.
import re
JOINT_RULES = [
    (r"lid_[UL][LR]", r"(scr_lid[AB]_|pin_)"), (r"link_[UL][LR]", r"scr_lid[AB]_"), (r"link_([UL][LR])", r"lid_\1"),
    (r"link_([UL][LR])", r"horn_lid_\1"), (r"servo_(lid_[UL][LR]|yaw|pitch)", r"horn_\1"),
    (r"stud_(pitch|rocker)_([LR])", r"rod_pitch_\2"),
    (r"scr_yawlink_[LRS]", r"yaw_coupler"), (r"yaw_lever_[LR]", r"yaw_coupler"),
    (r"horn_yaw", r"yaw_coupler"), (r"yaw_lever_([LR])", r"fixed_fork_\1"), (r"gobek_([LR])", r"(fixed_fork_|scr_fixfork_)\1"),
    (r"gobek_([LR])", r"(eye_fork_|scr_eyefork_)\1"), (r"scr_yawclamp_([LR])", r"fixed_fork_\1"),
    (r"(rocker|scr_rocker_bearing)", r"rocker_bearing"), (r"horn_pitch", r"servo_pitch"),
]


def is_joint(a, b):
    for x, y in ((a, b), (b, a)):
        for ra, rb in JOINT_RULES:
            m = re.fullmatch(ra, x)
            if m:
                rb2 = rb
                for i, gtxt in enumerate(m.groups() or ()):
                    rb2 = rb2.replace("\\%d" % (i + 1), gtxt)
                if re.match(rb2, y):
                    return True
    return False


def overhang(part, crit=45.0, bed_tol=0.3):
    """Baski yonunde: yatayla (90-crit)'ten dik olmayan asagi bakan yuzey alani (tablaya degen haric). mm2."""
    v, t = tess(part.shape, 0.02, 0.2)
    m = trimesh.Trimesh(v, t, process=True)
    Rm = np.eye(3) if part.print_R is None else np.asarray(part.print_R)
    m.apply_transform(np.block([[Rm, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
    m.apply_translation([0, 0, -m.bounds[0][2]])
    nz = m.face_normals[:, 2]
    zc = m.triangles_center[:, 2]
    down = nz < -math.cos(math.radians(crit))          # yatayla > crit egimli asagi yuzler
    ov = down & (zc > bed_tol)
    return float(m.area_faces[ov].sum()), float(m.area_faces[down & (zc <= bed_tol)].sum()), m.extents


def to_manifold(v, t):
    mesh = m3d.Mesh(vert_properties=np.asarray(v, np.float32), tri_verts=np.asarray(t, np.uint32))
    return m3d.Manifold(mesh)


class Body:
    def __init__(self, name, part):
        self.name = name; self.part = part; self.group = part.group
        v, t = tess(part.shape, TOL_TESS, 0.1)
        tm = trimesh.Trimesh(v, t, process=True)
        tm.merge_vertices()
        self.tm = tm
        self.watertight = tm.is_watertight
        self.v = np.asarray(tm.vertices); self.t = np.asarray(tm.faces)
        self.man = to_manifold(self.v, self.t)
        self.man_ok = self.man.status() == m3d.Error.NoError
        bvh = fcl.BVHModel(); bvh.beginModel(len(self.v), len(self.t)); bvh.addSubModel(self.v, self.t); bvh.endModel()
        self.bvh = bvh
        self.vol = part.shape.Volume()

    def aabb(self, M):
        c = np.array(list(itertools.product(*zip(self.v.min(0), self.v.max(0)))))
        w = c @ M[:3, :3].T + M[:3, 3]
        return w.min(0), w.max(0)


def fcl_obj(b, M):
    tf = fcl.Transform(M[:3, :3], M[:3, 3])
    return fcl.CollisionObject(b.bvh, tf)


def m34(M):
    return [list(map(float, M[i])) for i in range(3)]


def pair_check(b1, M1, b2, M2, dist_margin=3.0):
    """(kesisim hacmi, min mesafe veya None=margin disi, en yakin noktalar)"""
    lo1, hi1 = b1.aabb(M1); lo2, hi2 = b2.aabb(M2)
    gap = np.maximum(0, np.maximum(lo1 - hi2, lo2 - hi1))
    if np.linalg.norm(gap) > dist_margin:
        return 0.0, None, None
    vol = 0.0
    if np.all(gap == 0):
        inter = b1.man.transform(m34(M1)) ^ b2.man.transform(m34(M2))
        vol = inter.volume()
    req = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=False)
    res = fcl.DistanceResult()
    d = fcl.distance(fcl_obj(b1, M1), fcl_obj(b2, M2), req, res)
    return vol, max(d, 0.0), (res.nearest_points if d > 0 else None)


def run_pose(bodies, yaw, pitch, lids, dist_margin=3.0):
    Ms = pose_transforms(yaw, pitch, lids)
    names = list(bodies)
    res = []
    for i in range(len(names)):
        bi = bodies[names[i]]
        for j in range(i + 1, len(names)):
            bj = bodies[names[j]]
            if bi.group == bj.group:
                continue
            vol, d, _ = pair_check(bi, Ms[bi.group], bj, Ms[bj.group], dist_margin)
            if d is None:
                continue
            res.append((names[i], names[j], vol, d))
    return res


def intra_group(bodies):
    Ms = pose_transforms(0, 0, "closed")
    out = []
    names = list(bodies)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            bi, bj = bodies[names[i]], bodies[names[j]]
            if bi.group != bj.group:
                continue
            vol, d, _ = pair_check(bi, Ms[bi.group], bj, Ms[bj.group], 0.5)
            if d is None:
                continue
            out.append((names[i], names[j], vol, d))
    return out


def wall_thickness(part, n=4000, seed=1):
    """Isin yontemi: yuzeyden iceri normal boyunca karsi yuzeye mesafe. Keskin kenar/kose yakinindaki ornekler
    (kenardan <0.6 mm) atlanir. Donus: (min, %1, %5 yuzdelik, en ince noktanin konumu)."""
    v, t = tess(part.shape, 0.01, 0.1)
    m = trimesh.Trimesh(v, t, process=True)
    rng = np.random.default_rng(seed)
    pts, fid = trimesh.sample.sample_surface(m, n, seed=seed)
    nrm = m.face_normals[fid]
    orig = pts - nrm * 1e-3
    loc, idx_ray, idx_tri = m.ray.intersects_location(orig, -nrm, multiple_hits=False)
    if len(idx_ray) == 0:
        return None
    th = np.linalg.norm(loc - orig[idx_ray], axis=1)
    # kenar yakini ornekleri ayikla: isabet ucgeninin normali ile atis normali ~ ters degilse (egik/kose)
    cosang = -(m.face_normals[idx_tri] * nrm[idx_ray]).sum(1)
    keep = cosang > 0.8
    th = th[keep]; where = loc[keep]
    if len(th) == 0:
        return None
    k = int(np.argmin(th))
    return float(th.min()), float(np.percentile(th, 1)), float(np.percentile(th, 5)), where[k]


def main():
    quick = "--quick" in sys.argv
    t0 = time.time()
    P = build(verbose=False)
    bodies = {n: Body(n, p) for n, p in P.items() if p.kind != "visual"}
    lines = []
    L = lines.append
    L("PROJECT EYE v2 - MEKANIK KONTROL RAPORU")
    L(f"uretim: check.py  |  ucgenleme toleransi {TOL_TESS} mm  |  kesisim esigi {INTERFERENCE_TOL_MM3} mm3")
    L("")
    # 1) gecerlilik
    L("1) KATI GECERLILIGI (OCC BRepCheck) + ucgen agi su gecirmezligi")
    inval = [n for n, b in bodies.items() if not b.part.shape.isValid()]
    multi = [n for n, b in bodies.items() if b.part.kind == "printed" and len(b.part.shape.Solids()) != 1]
    nwt = [n for n, b in bodies.items() if not (b.watertight and b.man_ok)]
    L(f"   parca sayisi: {len(bodies)} (basilan {sum(b.part.kind == 'printed' for b in bodies.values())})")
    L(f"   gecersiz kati: {inval or 'yok'}")
    L(f"   basilan parcalarda tek-kati olmayan: {multi or 'yok'}")
    L(f"   su gecirmez olmayan ag: {nwt or 'yok'}")
    L("")
    # 2) grup ici
    L("2) GRUP ICI (birbirine sabit parcalar, notr poz) - kesisim >= esik:")
    ig = intra_group(bodies)
    bad_ig = [r for r in ig if r[2] >= INTERFERENCE_TOL_MM3]
    for a, b, vol, d in bad_ig:
        L(f"   ! {a} x {b}: {vol:.3f} mm3")
    if not bad_ig:
        L("   yok (temaslar: vida kafasi/yuzey, kulak/plaka vb. tasarim geregi 0 mesafe)")
    L("")
    # 3) poz taramasi
    poses = [(y, p, s) for s in ("open", "closed") for y in CHECK_YAWS for p in CHECK_PITCHES]
    # servo uzayi koseleri: firmware'in SABIT pitch servo sinirlari (kinematics.json 'firmware_safe') tum yaw'larda
    # goz pitch'ini +-25 icinde tutacak sekilde secildi (capraz etki). Burada bu sinirlar x yaw izgarasi taranir.
    import kin
    th_lo = max(kin.solve_pitch_servo(y, -PITCH_RANGE, "L")[0] for y in CHECK_YAWS)
    th_hi = min(kin.solve_pitch_servo(y, PITCH_RANGE, "L")[0] for y in CHECK_YAWS)
    servo_corners = [(y, round(kin.eye_pose_from_pitch_servo(th, y, "L"), 2), s) for s in ("open", "closed")
                     for y in (-30, -15, 0, 15, 30) for th in (th_lo, th_hi)]
    servo_corners = [c for c in servo_corners if (c[0], c[1], c[2]) not in poses]
    poses = poses + servo_corners
    extra = []
    if not quick:
        ys = (-30, -20, -10, 0, 10, 20, 30); ps = (-25, -12.5, 0, 12.5, 25)
        extra = [(y, p, s) for s in ("closed", "nominal", "open") for y in ys for p in ps
                 if (y, p, s) not in poses]
    L(f"3) HAREKET TARAMASI: zorunlu {len(poses) - len(servo_corners)} poz (yaw {CHECK_YAWS} x pitch {CHECK_PITCHES} x kapak acik/kapali)"
      + f" + firmware-sinir pozlari {len(servo_corners)} (pitch servosu guvenli sinirda {90 + th_lo:.1f}/{90 + th_hi:.1f}, yaw -30..30)"
      + (f" + yogun tarama {len(extra)} poz (yaw 10 deg, pitch 12.5 deg adim, kapak kapali/nominal/acik)" if extra else ""))
    L(f"   kapak 'acik' = mekanik tam acik (ust kenar {LID_UP_EDGE_OPEN_MAX:+.0f}, alt kenar {LID_LO_EDGE_OPEN_MAX:+.0f} deg), 'kapali' = kenarlar {LID_MEET:+.0f} deg'de bulusur (arada {LID_CLOSE_GAP} deg)")
    L("")
    total_bad = 0
    global_min = (1e9, None)
    per_pair_min = {}
    pose_rows = []
    for (y, p, s) in poses + extra:
        res = run_pose(bodies, y, p, s)
        bad = [r for r in res if r[2] >= INTERFERENCE_TOL_MM3]
        total_bad += len(bad)
        mn = min(res, key=lambda r: r[3]) if res else None
        for a, b, vol, d in res:
            key = (a, b)
            if key not in per_pair_min or d < per_pair_min[key][0]:
                per_pair_min[key] = (d, (y, p, s))
        if mn and mn[3] < global_min[0]:
            global_min = (mn[3], (mn[0], mn[1], y, p, s))
        pose_rows.append((y, p, s, len(bad), mn, bad, (y, p, s) in poses))
        print(f"poz yaw {y:6.1f} pitch {p:6.1f} kapak {s:8s}: cakisma {len(bad)}  min {mn[3] if mn else float('nan'):.3f} "
              f"({mn[0] if mn else ''} x {mn[1] if mn else ''})", flush=True)
    L("   zorunlu pozlar:")
    L("   yaw    pitch  kapak    | cakisma | min bosluk (mm) | en yakin cift")
    for (y, p, s, nb, mn, bad, req) in pose_rows:
        if not req:
            continue
        L(f"   {y:5.1f}  {p:5.1f}  {s:8s} | {nb:7d} | {mn[3]:15.3f} | {mn[0]} x {mn[1]}")
        for a, b, vol, d in bad:
            L(f"        ! {a} x {b}: {vol:.3f} mm3")
    if extra:
        nb_extra = sum(r[3] for r in pose_rows if not r[6])
        mn_extra = min((r[4] for r in pose_rows if not r[6] and r[4]), key=lambda m: m[3])
        L(f"   yogun tarama: toplam cakisma {nb_extra}; en kucuk bosluk {mn_extra[3]:.3f} mm ({mn_extra[0]} x {mn_extra[1]})")
        for (y, p, s, nb, mn, bad, req) in pose_rows:
            if not req and nb:
                for a, b, vol, d in bad:
                    L(f"        ! yaw {y} pitch {p} {s}: {a} x {b}: {vol:.3f} mm3")
    L("")
    L(f"   TOPLAM CAKISMA (>= {INTERFERENCE_TOL_MM3} mm3): {total_bad}")
    L(f"   EN KUCUK BOSLUK (tum pozlar, farkli gruplar): {global_min[0]:.3f} mm  {global_min[1]}")
    L("")
    items = sorted(per_pair_min.items(), key=lambda kv: kv[1][0])
    nj = [it for it in items if not is_joint(*it[0])]
    jj = [it for it in items if is_joint(*it[0])]
    L("4a) EKLEM DISI (birbirine degmemesi gereken) CIFTLERDE EN DAR 25 BOSLUK - asil carpisma payi:")
    for (a, b), (d, pose) in nj[:25]:
        L(f"   {d:7.3f}  {a} x {b}   @ yaw {pose[0]} pitch {pose[1]} {pose[2]}")
    L("4b) EKLEM CIFTLERI (tasarim geregi temas/yatak boslugu; bilgi icin, en dar 15):")
    for (a, b), (d, pose) in jj[:15]:
        L(f"   {d:7.3f}  {a} x {b}")
    L("")
    global NJ_MIN
    NJ_MIN = nj[0] if nj else None
    # 5) et kalinligi
    L("5) ET KALINLIGI (isin yontemi, basilan parcalar; min / %1 / %5 yuzdelik, mm):")
    L("   not: kenar ve kose yakinindaki egik isabetler ayiklanir; %1 degeri gercek ince bolgeyi daha iyi temsil eder.")
    seen = set()
    thin = []
    for n, b in bodies.items():
        if b.part.kind != "printed" or (b.part.bom_key in seen):
            continue
        seen.add(b.part.bom_key)
        r = wall_thickness(b.part)
        if r is None:
            continue
        mn_, p1, p5, where = r
        flag = "  <-- 1.2 alti" if p1 < MIN_WALL - 0.05 else ""
        if flag:
            thin.append(n)
        L(f"   {b.part.bom_key:24s} min {mn_:5.2f}  %1 {p1:5.2f}  %5 {p5:5.2f}   (en ince ~ {np.round(where, 1)}){flag}")
    L("")
    L("6) BASKI YONU ve DESTEK (eye_v2 print_R yonunde; tablaya degmeyen, yatayla 45 deg'den yatik asagi yuzey alani):")
    seen = set()
    for n, b in bodies.items():
        if b.part.kind != "printed" or b.part.bom_key in seen:
            continue
        seen.add(b.part.bom_key)
        ov, bed, ext = overhang(b.part)
        verdict = ("ihmal edilebilir (<30 mm2: delik tavani/kisa kopru)" if ov < 30 else
                   ("incele: kopru veya kucuk agac destek (BOM.md)" if ov < 150 else "buyuk sarkma (BOM.md aciklamasi)"))
        L(f"   {b.part.bom_key:24s} sarkma {ov:7.1f} mm2  tabla temas {bed:7.1f} mm2  boyut {np.round(ext, 1)}  -> {verdict}")
    L("")
    L("7) SERVO ACILARI (kontrol pozlarinda, derece):")
    for (y, p) in ((0, 0), (-30, 0), (30, 0), (0, -25), (0, 25), (-30, -25), (30, 25)):
        a = servo_angles(y, p, "closed")
        L(f"   yaw {y:4} pitch {p:4}: EYE_YAW {a['EYE_YAW']:6.1f}  EYE_PITCH {a['EYE_PITCH']:6.1f}")
    for s in ("closed", "nominal", "open"):
        a = servo_angles(0, 0, s)
        L(f"   kapak {s:8s}: " + "  ".join(f"{k} {a[k]:6.1f}" for k in ("LID_UL", "LID_LL", "LID_UR", "LID_LR")))
    L("")
    njm = f"{NJ_MIN[1][0]:.3f} mm ({NJ_MIN[0][0]} x {NJ_MIN[0][1]} @ {NJ_MIN[1][1]})" if NJ_MIN else "-"
    L(f"SONUC: cakisma {total_bad}, eklem disi en kucuk bosluk {njm}, gecersiz kati {len(inval)}, "
      f"1.2 alti et (yuzdelik %1) {len(thin)} parca {thin}")
    L(f"sure {time.time() - t0:.0f} s")
    os.makedirs(OUT, exist_ok=True)
    with open(REPORT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
