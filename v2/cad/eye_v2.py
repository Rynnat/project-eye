"""Project Eye v2 - parametrik parcalar + montaj (CadQuery).

Calistirma:  PYTHONUTF8=1 python eye_v2.py        -> out/step, out/stl, out/glb (+ rig.json)
Kullanim:    from eye_v2 import build, pose_transforms
             parts = build()                       # {isim: Part}
             Ms = pose_transforms(yaw, pitch, lids) # {grup: 4x4}  (rodlar dahil)

Tum parcalar dunya koordinatinda, NOTR pozda (yaw=pitch=0, kapaklar tam kapali) uretilir. Hareketli parcalar
`group` alanindaki gruba aittir; grubun 4x4 donusumu pose_transforms() ile hesaplanir (kin.py).
"""
import math
import os
import json
from dataclasses import dataclass, field

import numpy as np
import cadquery as cq

from params import *
import kin
from kin import EX, EY, EZ, E_L, E_R, EYES, LID

V = cq.Vector
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


# =============================================================================================== yardimcilar
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(p0, p1, r):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    d = p1 - p0; L = float(np.linalg.norm(d))
    return cq.Solid.makeCylinder(r, L, V(*p0), V(*(d / L)))


def sph(c, r):
    return cq.Solid.makeSphere(r, V(*c), angleDegrees1=-90, angleDegrees2=90)


def cone(p0, p1, r0, r1):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    d = p1 - p0; L = float(np.linalg.norm(d))
    return cq.Solid.makeCone(r0, r1, L, V(*p0), V(*(d / L)))


def U(*shapes):
    shapes = [s for s in shapes if s is not None]
    if len(shapes) == 1:
        return shapes[0]
    return shapes[0].fuse(*shapes[1:]).clean()


def D(a, *bs):
    bs = [b for b in bs if b is not None]
    return a.cut(*bs).clean() if bs else a


def I(a, b):
    return a.intersect(b).clean()


def wedge_x(phi1, phi2, x0, x1, R=60.0):
    """X ekseni etrafinda phi1..phi2 (deg, +Z'den +Y'ye) dilimi, x0..x1 arasi."""
    n = max(3, int(abs(phi2 - phi1) / 8) + 2)
    pts = [(0.0, 0.0)] + [(R * math.sin(math.radians(p)), R * math.cos(math.radians(p)))
                          for p in np.linspace(phi1, phi2, n)]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def slot_yz(A, B, w, x0, x1):
    """YZ duzleminde A(y,z)'den B'ye yuvarlak uclu cubuk, x0..x1 kalinlik."""
    A = np.asarray(A, float); B = np.asarray(B, float)
    L = float(np.linalg.norm(B - A)); m = (A + B) / 2
    ang = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
    return (cq.Workplane("YZ", origin=(x0, 0, 0)).center(m[0], m[1])
            .slot2D(L + w, w, ang).extrude(x1 - x0).val())


def slot_xz(A, B, w, y0, y1):
    """XZ duzleminde A(x,z)'den B'ye yuvarlak uclu cubuk, y0..y1."""
    A = np.asarray(A, float); B = np.asarray(B, float)
    L = float(np.linalg.norm(B - A)); m = (A + B) / 2
    # "XZ" duzlemi: xDir=+X, normal=-Y, yerel y = +Z
    ang = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
    s = (cq.Workplane("XZ", origin=(0, y1, 0)).center(m[0], m[1])
         .slot2D(L + w, w, ang).extrude(y1 - y0).val())
    return s


def xform(shape, M):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    M = np.asarray(M, float)
    t = gp_Trsf()
    t.SetValues(*[float(M[i, j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


def mirror_x(shape):
    return shape.mirror("YZ", (0, 0, 0))


def frame_M(origin, xdir, ydir):
    """Yerel x->xdir, y->ydir, z->x cross y; yerel orijin -> origin."""
    x = np.asarray(xdir, float); x /= np.linalg.norm(x)
    y = np.asarray(ydir, float); y /= np.linalg.norm(y)
    z = np.cross(x, y)
    M = np.eye(4); M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z; M[:3, 3] = origin
    return M


# =============================================================================================== donanim
def screw(head_pt, axis, L, head="socket"):
    """M2 vida. head_pt: kafanin oturma yuzeyi merkezi; axis: sapin gittigi yon. Sap kok caplidir (O1.6)."""
    a = np.asarray(axis, float); a /= np.linalg.norm(a); p = np.asarray(head_pt, float)
    shank = cyl(p, p + a * L, SCREW_MINOR_D / 2)
    if head == "socket":
        h = cyl(p - a * SCREW_M2_HEAD_H, p, SCREW_M2_HEAD_D / 2)
    else:  # havsa bas (DIN 965): koni O3.8 -> O2.0, 0.9 yukseklik; p = ust duz yuz merkezi
        h = cone(p, p + a * 0.9, SCREW_M2_HEAD_D / 2, 1.0)
        shank = cyl(p + a * 0.9, p + a * L, SCREW_MINOR_D / 2)
    return U(h, shank)


def ball_stud(face_pt, direction):
    """Rotil saplamasi: disli kisim tasiyiciya (O1.6, 4 mm), yaka, boyun, top. top merkezi = face + dir*BALL_H."""
    d = np.asarray(direction, float); d /= np.linalg.norm(d); p = np.asarray(face_pt, float)
    thread = cyl(p - d * 4.0, p, SCREW_MINOR_D / 2)
    collar = cyl(p, p + d * STUD_COLLAR_H, STUD_COLLAR_D / 2)
    neck = cyl(p + d * STUD_COLLAR_H, p + d * BALL_H, STUD_NECK_D / 2)
    ball = sph(p + d * BALL_H, BALL_D / 2)
    return U(thread, collar, neck, ball)


def socket_mouth(c, sd):
    """Yuva agzi: top merkezinden saplama tarafina (-sd) SOCKET_MOUTH_HALF yari acili koni."""
    hh = SOCKET_R + 2.0
    return cone(c, c - sd * hh, 0.0, hh * math.tan(math.radians(SOCKET_MOUTH_HALF)))


def pitch_link_local(L, stud_a_local, stud_b_local):
    """Basili pitch cubugu. Rod-yerel: A=(0,0,0) goz topu, B=(0,0,L) rocker topu. stud_*: saplama yonleri."""
    A = np.zeros(3); B = np.array([0, 0, L])
    sa = np.asarray(stud_a_local, float); sb = np.asarray(stud_b_local, float)
    t0, t1 = PITCH_LINK_T
    bar = box(-t0 / 2, t0 / 2, -t1 / 2, t1 / 2, 0, L)
    body = U(sph(A, SOCKET_R), sph(B, SOCKET_R), bar)
    cav = [sph(A, (BALL_D + SOCKET_CAV_CLR) / 2), sph(B, (BALL_D + SOCKET_CAV_CLR) / 2)]
    return D(body, *cav, socket_mouth(A, sa), socket_mouth(B, sb))


# =============================================================================================== SG90
def sg90_body():
    body = box(-SG_SHAFT_OFF, SG_L - SG_SHAFT_OFF, 0, SG_H, -SG_W / 2, SG_W / 2)
    ex = (SG_EAR_SPAN - SG_L) / 2
    ears = box(-SG_SHAFT_OFF - ex, SG_L - SG_SHAFT_OFF + ex, SG_EAR_Z0, SG_EAR_Z0 + SG_EAR_T, -SG_W / 2, SG_W / 2)
    hc = (SG_L - 2 * SG_SHAFT_OFF) / 2
    holes = [cyl((hc + s * SG_HOLE_SPACING / 2, SG_EAR_Z0 - 1, 0), (hc + s * SG_HOLE_SPACING / 2, SG_EAR_Z0 + 4, 0), 1.0)
             for s in (-1, 1)]
    boss = cyl((0, SG_H, 0), (0, SG_TOP, 0), SG_BOSS_R)
    spline = cyl((0, SG_TOP, 0), (0, SG_SPLINE_TOP, 0), 2.4)
    return D(U(body, ears, boss, spline), *holes)


def sg90_ear_holes_local():
    hc = (SG_L - 2 * SG_SHAFT_OFF) / 2
    return [np.array([hc + s * SG_HOLE_SPACING / 2, SG_EAR_Z0, 0.0]) for s in (-1, 1)]


def sg90_cutout_local(clr=SERVO_POCKET_CLR / 2):
    return box(-SG_SHAFT_OFF - clr, SG_L - SG_SHAFT_OFF + clr, -1, SG_EAR_Z0 + 0.01, -SG_W / 2 - clr, SG_W / 2 + clr)


def horn_single(r, hole=M2_PILOT):
    y0, y1 = SG_SPLINE_TOP, SG_SPLINE_TOP + HORN_T
    hub = cyl((0, y0, 0), (0, y1, 0), HORN_HUB_R)
    arm = box(0, r, y0, y1, -2.5, 2.5)
    tip = cyl((r, y0, 0), (r, y1, 0), 2.5)
    return D(U(hub, arm, tip), cyl((r, y0 - 1, 0), (r, y1 + 1, 0), hole / 2))


def horn_cross(r=8.0):
    y0, y1 = SG_SPLINE_TOP, SG_SPLINE_TOP + HORN_T
    hub = cyl((0, y0, 0), (0, y1, 0), HORN_HUB_R)
    a = box(-r, r, y0, y1, -2.5, 2.5)
    b = box(-2.5, 2.5, y0, y1, -r, r)
    return U(hub, a, b)


def servo_M(shaft_base_pt, shaft_dir, long_dir):
    """Servo yerel -> dunya. Yerel +Y = mil, yerel +X = govde boyu (milden uzaga), orijin = mil ekseni taban duzleminde."""
    return frame_M(shaft_base_pt, long_dir, shaft_dir)


def horn_angle_for(Mserv, world_dir):
    """Kolun yerel +X yonunu dunya world_dir'e ceviren yerel-Y donus acisi (sag-el)."""
    Rm = Mserv[:3, :3]
    loc = Rm.T @ np.asarray(world_dir, float)
    return math.degrees(math.atan2(-loc[2], loc[0]))


# =============================================================================================== Parca kaydi
@dataclass
class Part:
    name: str
    shape: object
    group: str
    color: str
    kind: str = "printed"          # printed | hardware | purchased | visual
    material: str = ""
    qty: int = 1                   # BOM icin (ayna/ikiz parcalar ayri kayitli olabilir)
    print_R: object = None         # 3x3 baski yonu donusu (None = oldugu gibi)
    note: str = ""
    bom_key: str = ""              # ayni STL'yi paylasan parcalar icin


C = dict(eye="#f2efe6", iris="#2f7f93", pupil="#0a0a0a", fork="#d8d2c2", hub="#2ed9e0", fix="#9aa5a8",
         lid="#d9a080", frame="#3b4449", servo="#3a64c8", horn="#f4f4f4", rod="#e3a455", metal="#6d747a",
         socket="#202020", mask="#55606a", link="#c9b28f")


# =============================================================================================== goz (goz-yerel)
def eye_shell_local():
    shell = I(D(sph((0, 0, 0), EYE_R), sph((0, 0, 0), EYE_R_IN)), box(-13, 13, -13, 13, EYE_RIM_Z, 13))
    # alt pad (iki serit) + pitch kancasi: kabukla tek parca, tablada pad'den dik yukselir
    opening = cone((0, 0, 0), (0, 0, -40), 0.0, 40 * math.tan(math.radians(EYE_OPEN_HALF)))
    pads = [box(*pp[0], *pp[1], EYE_RIM_Z, EYE_RIM_Z + PAD_T) for pp in (BOTTOM_PAD_A0, BOTTOM_PAD_C, BOTTOM_PAD_A1, BOTTOM_PAD_B)]
    pad = D(U(*pads), sph((0, 0, 0), EYE_R_IN), opening)
    hook = box(*PITCH_HOOK_X, *PITCH_HOOK_Y, *PITCH_HOOK_Z)
    yt = PITCH_HOOK_Y[1]; zc = HOOK_CHAMFER_Z; zr = EYE_RIM_Z - 1.0
    wedge = (cq.Workplane("YZ", origin=(PITCH_HOOK_X[0] - 1, 0, 0))
             .polyline([(yt + 1, zc), (yt, zc), (yt - (zc - zr), zr), (yt + 1, zr)]).close()
             .extrude(PITCH_HOOK_X[1] - PITCH_HOOK_X[0] + 2).val())
    hook = D(hook, wedge)
    s = U(shell, pad, hook)
    # yaw tupu icin ust yarik: tupun goz cercevesindeki supurmesi phi = 90 -+ 25
    rr = YAW_TUBE_D / 2 + YAW_SLOT_CLR
    cuts = []
    for phi in np.arange(90 - PITCH_RANGE, 90 + PITCH_RANGE + 0.01, 2.5):
        d = np.array([0, math.sin(math.radians(phi)), math.cos(math.radians(phi))])
        cuts.append(cyl(d * 7.0, d * 13.5, rr))
    slot = U(*cuts)
    # kardan vidalarina erisim: yanlar (yatay vidalar) ve alt (sabit catal alt vidasi)
    acc = [cyl((s_ * 9.0, 0, 0), (s_ * 13.0, 0, 0), ACCESS_HOLE_D / 2) for s_ in (-1, 1)]
    acc.append(cyl((0, -9.0, 0), (0, -13.0, 0), ACCESS_HOLE_D / 2))
    # bebek = siyah M2 havsa bas vida: havsa + delik
    pupil = U(cone((0, 0, EYE_R - 0.95), (0, 0, EYE_R + 0.05), 1.0 + CSK_CLR, 2.0 + CSK_CLR),
              cyl((0, 0, EYE_R_IN - 0.5), (0, 0, EYE_R), M2_CLEAR / 2))
    # kanca: rotil saplamasi pilot (+Y'ye bakar, blogun ust yuzunden 4 mm asagi)
    ytop = PITCH_HOOK_Y[1]
    stud = cyl((PITCH_BALL[0], ytop + 0.01, PITCH_BALL[2]), (PITCH_BALL[0], ytop - 4.0, PITCH_BALL[2]), M2_PILOT / 2)
    return D(s, slot, *acc, pupil, stud)


def eye_iris_visual_local():
    """Sadece gorsel: iris halkasi ve bebek (boya/cikartma rehberi)."""
    ang_i = math.degrees(math.asin(IRIS_D / 2 / EYE_R))
    cap = I(D(sph((0, 0, 0), EYE_R + 0.06), sph((0, 0, 0), EYE_R - 0.01)),
            cone((0, 0, 0), (0, 0, EYE_R + 1), 0.0, (EYE_R + 1) * math.tan(math.radians(ang_i))))
    return cap


def eye_fork_local():
    arms = []
    for sg in (-1, 1):
        xa, xb = sorted((sg * (FORK_C - FORK_T / 2), sg * (FORK_C + FORK_T / 2)))
        arms.append(box(xa, xb, -EYE_FORK_ARM_Y, EYE_FORK_ARM_Y, EYE_FORK_ARM_Z[0], 2.5))
        arms.append(box(xa, xb, -EYE_FORK_ARM_Y2, EYE_FORK_ARM_Y2, 2.49, EYE_FORK_BRIDGE_Z[0] + 0.01))
    zb0, zb1 = EYE_FORK_BRIDGE_Z; yb, yf = EYE_FORK_BRIDGE_Y, EYE_FORK_BRIDGE_Y_FRONT
    bridge = (cq.Workplane("YZ", origin=(-FORK_OUT, 0, 0))
              .polyline([(-yb, zb0), (yb, zb0), (yf, zb1), (-yf, zb1)]).close().extrude(2 * FORK_OUT).val())
    stem = cyl((0, 0, EYE_FORK_BRIDGE_Z[1] - 0.01), (0, 0, EYE_R), EYE_FORK_STEM_D / 2)
    f = I(U(*arms, bridge, stem), sph((0, 0, 0), EYE_R_IN - GLUE_GAP))
    holes = [cyl((-FORK_OUT - 1, 0, 0), (FORK_OUT + 1, 0, 0), M2_PILOT / 2),
             cyl((0, 0, EYE_FORK_BRIDGE_Z[1] - 0.05), (0, 0, EYE_R_IN + 0.1), M2_PILOT / 2)]
    return D(f, *holes)


def gobek_local():
    h = HUB / 2
    g = box(-h, h, -h, h, -h, h)
    holes = [cyl((s * h, 0, 0), (s * (h - HUB_HOLE_DEPTH), 0, 0), M2_CLEAR / 2) for s in (-1, 1)]
    holes.append(cyl((0, -h, 0), (0, -(h - HUB_HOLE_DEPTH), 0), M2_CLEAR / 2))        # alt: sabit vida ucu doner
    holes.append(cyl((0, h + 0.01, 0), (0, h - 2.9, 0), M2_PILOT / 2))                         # ust: yaw kolu vidasi
    return D(g, *holes)


def yaw_lever_local():
    y0, y1 = YAW_LEVER_Y
    tube = D(cyl((0, HUB / 2, 0), (0, y0 + 0.01, 0), YAW_TUBE_D / 2), cyl((0, 0, 0), (0, y0 + 1, 0), M2_CLEAR / 2))
    plate = slot_xz((0, 0), (0, -YAW_LEVER_R), YAW_LEVER_W, y0, y1)
    lev = U(tube, plate)
    holes = [cyl((0, y0 - 1, 0), (0, y1 + 1, 0), M2_CLEAR / 2),
             cyl((0, y1 - YAW_CLAMP_SEAT, 0), (0, y1 + 1, 0), CBORE_D / 2),
             cyl((0, y0 - 1, -YAW_LEVER_R), (0, y1 + 1, -YAW_LEVER_R), M2_PILOT / 2)]
    return D(lev, *holes)


def fixed_fork_local():
    ax = FIX_ARM_HALF_X
    arms = [box(-ax, ax, *sorted((s * (HUB / 2 + FORK_GAP), s * FORK_OUT)), FIX_FORK_BRIDGE_Z[1], 4.5)
            for s in (-1, 1)]
    bridge = box(-2.5, 2.5, -FORK_OUT, FORK_OUT, FIX_FORK_BRIDGE_Z[0], FIX_FORK_BRIDGE_Z[1])
    gus = (cq.Workplane("XY").workplane(offset=FIX_FORK_BRIDGE_Z[0] - 4.0).circle(POST_D / 2)
           .workplane(offset=4.0).rect(5.0, 2 * FORK_OUT).loft().val())
    cy, cz = FIX_ARM_CHAMFER
    steps = [box(-ax - 1, ax + 1, *sorted((s * (HUB / 2 + FORK_GAP - 0.1), s * (HUB / 2 + FORK_GAP + cy))),
                 4.5 - cz, 5.0) for s in (-1, 1)]
    head = D(I(U(*arms, bridge, gus), sph((0, 0, 0), FIX_FORK_TRIM_R)), *steps)
    fz = BEAM_Z[1] + POST_FLANGE[2]
    post = cyl((0, 0, fz - 0.01), (0, 0, FIX_FORK_BRIDGE_Z[0] + 0.5), POST_D / 2)
    fl = box(-POST_FLANGE[0] / 2, POST_FLANGE[0] / 2, -POST_FLANGE[1] / 2, POST_FLANGE[1] / 2, BEAM_Z[1], fz)
    f = U(head, post, fl)
    holes = [cyl((0, HUB / 2, 0), (0, FORK_OUT + 1, 0), YAW_TUBE_BEARING / 2),
             cyl((0, -HUB / 2, 0), (0, -FORK_OUT - 1, 0), M2_PILOT / 2)]
    holes += [cyl((s * 4.0, 0, BEAM_Z[1] - 0.1), (s * 4.0, 0, fz + 0.1), M2_PILOT / 2) for s in (-1, 1)]
    return D(f, *holes)


# =============================================================================================== kapaklar (sol goz, dunya)
def lid_local(which, side_out=-1):
    """which 'U' | 'Lo'. Goz-yerel, KAPALI poz. side_out: dis yan isareti (sol goz -1)."""
    if which == "U":
        p0 = kin.LID_UP_CLOSED_EDGE; p1 = p0 + LID_UP_W; hub_u = LID_UP_HUB_U; hole = M2_PILOT
    else:
        p1 = kin.LID_LO_CLOSED_EDGE; p0 = p1 - LID_LO_W; hub_u = LID_LO_HUB_U; hole = M2_CLEAR
    pc = (p0 + p1) / 2
    shell = D(sph((0, 0, 0), LID_R_OUT), sph((0, 0, 0), LID_R_IN))
    rmid = (LID_R_IN + LID_R_OUT) / 2
    half = math.degrees(math.acos(LID_TRUNC / rmid))
    polar = [cone((0, 0, 0), (sg * 20.0, 0, 0), 0.0, 20.0 * math.tan(math.radians(half))) for sg in (-1, 1)]
    lune = D(I(shell, wedge_x(p0, p1, -LID_R_OUT - 1, LID_R_OUT + 1)), *polar)
    parts = [lune]
    d = LID[which]
    arm_phi_closed = d["phi_mid"] - d["dmid"]
    for s in (-1, 1):
        u0 = 10.5
        xa, xb = sorted((s * u0, s * hub_u[0]))
        sec = I(wedge_x(pc - LID_EAR_HALF, pc + LID_EAR_HALF, xa, xb),
                cyl((xa, 0, 0), (xb, 0, 0), LID_EAR_RHO))
        ear = D(sec, cyl((xa - 1, 0, 0), (xb + 1, 0, 0), LID_EAR_RHO_IN), sph((0, 0, 0), LID_R_IN))
        ha, hb = sorted((s * hub_u[0], s * hub_u[1]))
        hubp = U(cyl((ha, 0, 0), (hb, 0, 0), LID_HUB_R),
                 I(wedge_x(pc - LID_EAR_HALF, pc + LID_EAR_HALF, ha, hb), cyl((ha, 0, 0), (hb, 0, 0), LID_EAR_RHO)))
        parts += [ear, hubp]
        if s == side_out:
            tip = LID_ARM_R * kin.dir_phi(arm_phi_closed)
            arm = slot_yz((0, 0), tip, LID_ARM_W, ha, hb)
            arm = D(arm, cyl((ha - 1, tip[0], tip[1]), (hb + 1, tip[0], tip[1]), hole / 2))
            parts.append(arm)
    lid = U(*parts)
    return D(lid, cyl((-30, 0, 0), (30, 0, 0), PIN_RUN / 2))


# =============================================================================================== montaj
def build(verbose=True):
    P = {}

    def add(name, shape, group, color, **kw):
        P[name] = Part(name, shape, group, color, **kw)

    def log(*a):
        if verbose:
            print(*a, flush=True)

    log("goz parcalari...")
    shell_l = eye_shell_local(); fork_l = eye_fork_local(); iris_l = eye_iris_visual_local()
    shell_r = mirror_x(shell_l)
    hub_l = gobek_local(); lever_l = yaw_lever_local(); fix_l = fixed_fork_local()
    for k, E in EYES.items():
        t = kin.T(E)
        g = f"eye_{k}"
        add(f"eye_shell_{k}", xform(shell_l if k == "L" else shell_r, t), g, C["eye"], material="PLA beyaz",
            qty=1, bom_key=f"eye_shell_{k}",
            note="kenar duzlemi tablada, on kutup yukarda; alt pad+kanca tek parca")
        add(f"eye_iris_{k}", xform(iris_l, t), g, C["iris"], kind="visual")
        add(f"eye_fork_{k}", xform(fork_l, t), g, C["fork"], material="PETG", bom_key="eye_fork",
            print_R=None, note="kollar tablada, sap yukari")
        for s in (-1, 1):
            add(f"scr_eyefork_{k}{'p' if s > 0 else 'm'}",
                xform(screw((s * FORK_OUT, 0, 0), (-s, 0, 0), 5.0), t), g, C["metal"], kind="hardware",
                bom_key="M2x5 ISO 4762")
        add(f"scr_pupil_{k}", xform(screw((0, 0, EYE_R - 0.05), (0, 0, -1), 4.0, head="csk"), t), g, C["pupil"],
            kind="hardware", bom_key="M2x4 DIN 965 havsa bas, siyah (bebek)")
        stud_face = np.array([PITCH_BALL[0], PITCH_HOOK_Y[1], PITCH_BALL[2]])
        add(f"stud_pitch_{k}", xform(ball_stud(stud_face, EY), t), g, C["metal"], kind="purchased",
            bom_key="rotil topu O4.8 M2 saplama")
        # gobek grubu (sadece yaw)
        gh = f"gobek_{k}"
        add(f"gobek_{k}", xform(hub_l, t), gh, C["hub"], material="PETG", bom_key="gobek")
        add(f"yaw_lever_{k}", xform(lever_l, t), gh, C["fork"], material="PETG", bom_key="yaw_lever",
            print_R=kin.R(EX, -90)[:3, :3], note="kol plakasi tablada, tup yukari")
        add(f"scr_yawclamp_{k}", xform(screw((0, YAW_LEVER_Y[1] - YAW_CLAMP_SEAT, 0), (0, -1, 0), 16.0), t), gh,
            C["metal"], kind="hardware", bom_key="M2x16 ISO 4762")
        add(f"scr_yawlink_{k}",
            xform(screw((0, YAW_COUPLER_Y[1] + 0.1, -YAW_LEVER_R), (0, -1, 0), 5.0), t), gh, C["metal"],
            kind="hardware", bom_key="M2x5 ISO 4762")
        # sabit catal + alt vida
        add(f"fixed_fork_{k}", xform(fix_l, t), "static", C["fix"], material="PETG", bom_key="fixed_fork",
            print_R=np.eye(3), note="flans tablada, catal yukari")
        add(f"scr_fixfork_{k}", xform(screw((0, -FORK_OUT, 0), (0, 1, 0), 5.0), t), "static", C["metal"],
            kind="hardware", bom_key="M2x5 ISO 4762")
        for s in (-1, 1):
            add(f"scr_flange_{k}{'p' if s > 0 else 'm'}",
                xform(screw((s * 4.0, 0, BEAM_Z[0] + BASE_CBORE_DEPTH), (0, 0, 1), 8.0), t), "static", C["metal"],
                kind="hardware", bom_key="M2x8 ISO 4762")

    # ---------------------------------------------------------------- kapaklar + pimler + kanatlar
    log("kapaklar...")
    lidU = lid_local("U", -1); lidL = lid_local("Lo", -1)
    for k, E in EYES.items():
        t = kin.T(E)
        mir = (k == "R")
        for nm, sh, code in (("U", lidU, "U"), ("Lo", lidL, "L")):
            s = xform(sh, t) if not mir else mirror_x(xform(sh, kin.T(E_L)))
            add(f"lid_{code}{k}", s, f"lid_{code}{k}", C["lid"], material="PLA ten rengi",
                bom_key=f"lid_{'upper' if nm == 'U' else 'lower'}_{k}",
                print_R=kin.R(EY, -90 if not mir else 90)[:3, :3], note="pim ekseni dik (X yukari), dis gobek tablada")
        for s in (-1, 1):
            u1 = LID_FIN_U[1]
            p0 = E + np.array([s * u1, 0, 0]); p1 = E + np.array([s * (u1 - LID_PIN_L), 0, 0])
            add(f"pin_{k}{'p' if s > 0 else 'm'}", cyl(p0, p1, PIN_PRESS / 2), "static", C["metal"],
                kind="purchased", bom_key="O2x12 celik pim DIN 7")

    # ---------------------------------------------------------------- kapak servolari, kollar, baglantilar
    log("kapak servolari...")
    for k, E in EYES.items():
        so = -1 if k == "L" else 1                       # dis yon
        for code, nm, link_u, horn_u0 in (("U", "U", LID_UP_LINK_U, LID_UP_LINK_U[1] + AXIAL_GAP),
                                           ("L", "Lo", LID_LO_LINK_U, LID_LO_LINK_U[1] + AXIAL_GAP)):
            d = LID[nm]
            # servo: mil disa (so*X), horn alt yuzu u=horn_u0
            shaft_dir = np.array([so, 0, 0.0])
            base_u = horn_u0 - SG_SPLINE_TOP
            base_pt = np.array([E[0] + so * base_u, d["S"][0], d["S"][1]])
            Ms = servo_M(base_pt, shaft_dir, -EZ)
            add(f"servo_lid_{code}{k}", xform(sg90_body(), Ms), "static", C["servo"], kind="purchased",
                bom_key="SG90 servo")
            n3 = np.array([0, d["n"][0], d["n"][1]])
            a0 = horn_angle_for(Ms, n3)
            hornM = Ms @ kin.R(EY, a0)
            hg = f"lidhorn_{code}{k}"
            add(f"horn_lid_{code}{k}", xform(horn_single(HORN_R_LID), hornM), hg, C["horn"], kind="purchased",
                bom_key="SG90 tek kol (servo ile gelir)")
            # link (orta pozda)
            A = LID_ARM_R * d["n"]; B = d["S"] + HORN_R_LID * d["n"]
            xa, xb = sorted((E[0] + so * link_u[0], E[0] + so * link_u[1]))
            lk = slot_yz(A, B, LINK_W, xa, xb)
            ha = M2_CLEAR if nm == "U" else M2_PILOT
            lk = D(lk, cyl((xa - 1, A[0], A[1]), (xb + 1, A[0], A[1]), ha / 2),
                   cyl((xa - 1, B[0], B[1]), (xb + 1, B[0], B[1]), M2_CLEAR / 2))
            lg = f"link_{code}{k}"
            add(f"link_{code}{k}", lk, lg, C["link"], material="PETG", bom_key=f"lid_link_{'upper' if nm == 'U' else 'lower'}",
                print_R=kin.R(EY, 90)[:3, :3], note="duz yatar")
            # vidalar (orta pozda uretilir, kendi gruplarinin mid->notr donusu ile yerlesir)
            if nm == "U":
                # kol ucu: link dis yuzunden kola (kolda kilavuzsuz) -> kapak grubu
                hp = np.array([E[0] + so * link_u[1], A[0], A[1]])
                scA = screw(hp, (-so, 0, 0), 4.0)
                # servo ucu: link ic yuzunden servo koluna -> horn grubu
                hpB = np.array([E[0] + so * link_u[0], B[0], B[1]])
                scB = screw(hpB, (so, 0, 0), 4.0)
                P[f"scr_lidA_{code}{k}"] = Part(f"scr_lidA_{code}{k}", scA, f"lid_{code}{k}@mid", C["metal"],
                                                 kind="hardware", bom_key="M2x4 ISO 4762")
            else:
                # kol ucu: kolun ic yuzunden, kolda gecis, linkte kilavuzsuz -> link grubu
                hp = np.array([E[0] + so * LID_LO_HUB_U[0], A[0], A[1]])
                scA = screw(hp, (so, 0, 0), 5.0)
                hpB = np.array([E[0] + so * link_u[0], B[0], B[1]])
                scB = screw(hpB, (so, 0, 0), 4.0)
                P[f"scr_lidA_{code}{k}"] = Part(f"scr_lidA_{code}{k}", scA, f"{lg}@mid", C["metal"],
                                                 kind="hardware", bom_key="M2x5 ISO 4762")
            P[f"scr_lidB_{code}{k}"] = Part(f"scr_lidB_{code}{k}", scB, f"{hg}@mid", C["metal"],
                                             kind="hardware", bom_key="M2x4 ISO 4762")
            P[f"link_{code}{k}"].group = f"{lg}@mid"
            P[f"horn_lid_{code}{k}"].group = f"{hg}@mid"

    # ---------------------------------------------------------------- yaw servo + paralelkenar cubugu
    log("yaw...")
    ybase = np.array([YAW_SERVO_X, kin.YAW_SERVO_BASE_Y, 0.0])
    Msy = servo_M(ybase, EY, -EZ)
    add("servo_yaw", xform(sg90_body(), Msy), "static", C["servo"], kind="purchased", bom_key="SG90 servo")
    a0 = horn_angle_for(Msy, -EZ)
    add("horn_yaw", xform(horn_single(HORN_R_LID), Msy @ kin.R(EY, a0)), "yawhorn", C["horn"], kind="purchased",
        bom_key="SG90 tek kol (servo ile gelir)")
    add("scr_yawlink_S", screw((YAW_SERVO_X, YAW_COUPLER_Y[1] + 0.1, -YAW_LEVER_R), (0, -1, 0), 5.0), "yawhorn",
        C["metal"], kind="hardware", bom_key="M2x5 ISO 4762")
    cz = -YAW_LEVER_R
    cp = slot_xz((E_L[0], cz), (E_R[0], cz), YAW_COUPLER_W, *YAW_COUPLER_Y)
    cp = D(cp, *[cyl((x, YAW_COUPLER_Y[0] - 1, cz), (x, YAW_COUPLER_Y[1] + 1, cz), YAW_COUPLER_HOLE / 2)
                 for x in (E_L[0], YAW_SERVO_X, E_R[0])])
    add("yaw_coupler", cp, "yawcoupler", C["link"], material="PETG", bom_key="yaw_coupler",
        print_R=kin.R(EX, 90)[:3, :3], note="duz yatar")

    # ---------------------------------------------------------------- pitch: rocker + servo + cubuklar
    log("pitch...")
    ay, az = kin.ROCKER_AXIS_YZ
    hw = 3.5
    bar = box(PITCH_ROCKER_FLANGE_X[1] - 0.01, ROCKER_END_X, ay - hw, ay + hw, az - hw, az + hw)
    flange = box(*PITCH_ROCKER_FLANGE_X, ay - 9.0, ay + 9.0, az - hw, az + hw)
    arms = []
    for k in EYES:
        sd = kin.ROCKER_STUD_DIR[k][0]
        x0, x1 = sorted((kin.ROCKER_ARM_FACE_X[k], kin.ROCKER_ARM_FACE_X[k] - 4.0 * sd))
        rb = kin.rocker_ball0(k)
        zt = rb[2]
        arms.append(cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(
            [(ay - hw, az - hw), (rb[1] + hw, az - hw), (rb[1] + hw, max(zt + hw, az + hw)), (ay - hw, az + hw)])
            .close().extrude(x1 - x0).val())
    rk = U(bar, flange, *arms)
    rholes = [cyl((kin.ROCKER_ARM_FACE_X[k] + 0.01 * kin.ROCKER_STUD_DIR[k][0], *kin.rocker_ball0(k)[1:]),
                  (kin.ROCKER_ARM_FACE_X[k] - 4.0 * kin.ROCKER_STUD_DIR[k][0], *kin.rocker_ball0(k)[1:]), M2_PILOT / 2)
              for k in EYES]
    rholes.append(cyl((ROCKER_END_X + 0.01, ay, az), (ROCKER_END_X - 4.6, ay, az), M2_PILOT / 2))
    rholes += [cyl((PITCH_ROCKER_FLANGE_X[0] - 0.1, ay + s * 6.0, az), (PITCH_ROCKER_FLANGE_X[1] + 0.1, ay + s * 6.0, az),
                   HORN_SCREW_PILOT / 2) for s in (-1, 1)]   # kol vidalari (servo ile gelen kucuk vidalar)
    rk = D(rk, *rholes)
    add("rocker", rk, "rocker", C["fork"], material="PETG", bom_key="rocker",
        print_R=np.eye(3), note="z=-23.5 yuzu tablada (duz tarak)")
    for k in EYES:
        add(f"stud_rocker_{k}", ball_stud(np.array([kin.ROCKER_ARM_FACE_X[k], *kin.rocker_ball0(k)[1:]]),
                                          kin.ROCKER_STUD_DIR[k]),
            "rocker", C["metal"], kind="purchased", bom_key="rotil topu O4.8 M2 saplama")
    add("scr_rocker_bearing", screw((ROCKER_END_X + AXIAL_GAP + MOUNT_T + SCREW_HEAD_GAP, ay, az), (-1, 0, 0), 8.0), "rocker", C["metal"],
        kind="hardware", bom_key="M2x8 ISO 4762")
    pbase = np.array([kin.PITCH_SERVO_BASE_X, ay, az])
    Msp = servo_M(pbase, EX, EZ)
    add("servo_pitch", xform(sg90_body(), Msp), "static", C["servo"], kind="purchased", bom_key="SG90 servo")
    add("horn_pitch", xform(horn_cross(8.0), Msp @ kin.R(EY, horn_angle_for(Msp, EY))), "rocker", C["horn"],
        kind="purchased", bom_key="SG90 hac kol (servo ile gelir)")
    # cubuklar (rod-yerel; pose_transforms yerlestirir)
    for k in EYES:
        M0, _ = rod_frame(EYES[k] + kin.B_PITCH, kin.rocker_ball0(k), EY)
        sb_local = M0[:3, :3].T @ kin.ROCKER_STUD_DIR[k]
        add(f"rod_pitch_{k}", pitch_link_local(PITCH_ROD_L, (1, 0, 0), sb_local), f"rod_pitch_{k}", C["rod"],
            material="PETG", bom_key=f"pitch_link_{k}", print_R=kin.R(EY, -90)[:3, :3], note="goz ucu yuva ekseni dik (agiz asagi)")

    # ---------------------------------------------------------------- cerceve
    log("cerceve...")
    beam = box(*BEAM_X, *BEAM_Y, *BEAM_Z)
    pillars = [box(xc - PILLAR_W / 2, xc + PILLAR_W / 2, BASE_Y[1], BEAM_Y[1], *BEAM_Z) for xc in PILLAR_XS]
    fins = []
    for k, E in EYES.items():
        for s in (-1, 1):
            xa, xb = sorted((E[0] + s * LID_FIN_U[0], E[0] + s * LID_FIN_U[1]))
            fins.append(U(box(xa, xb, -4.0, 4.0, BEAM_Z[1] - 0.01, 0.0), cyl((xa, 0, 0), (xb, 0, 0), 4.5)))
    # yaw servo rafi (kulaklarin altinda), ic kanatlarla birlesik. Govde icin |x|<=fin ic yuzu kadar bosaltilir.
    ear_y = kin.YAW_SERVO_BASE_Y + SG_EAR_Z0
    fin_in = EYE_X - LID_FIN_U[1]                       # ic kanat ic yuzu |x|
    shelf = box(-fin_in - 4.0, fin_in + 4.0, ear_y - MOUNT_T, ear_y, BEAM_Z[1] - 0.01, 12.5)
    body_z = sorted(kin.apply(Msy, p)[2] for p in ((-SG_SHAFT_OFF, 0, 0), (SG_L - SG_SHAFT_OFF, 0, 0)))
    clr = SERVO_POCKET_CLR / 2
    shelf = D(shelf, box(-fin_in, fin_in, ear_y - MOUNT_T - 1, ear_y + 1, body_z[0] - clr, body_z[1] + clr))
    bf = U(beam, *pillars, *fins, shelf)
    bh = []
    for k, E in EYES.items():
        for s in (-1, 1):
            bh.append(cyl((E[0] + s * 4.0, 0, BEAM_Z[0] - 0.1), (E[0] + s * 4.0, 0, BEAM_Z[1] + 0.1), M2_CLEAR / 2))
            bh.append(cyl((E[0] + s * 4.0, 0, BEAM_Z[0] - 0.1), (E[0] + s * 4.0, 0, BEAM_Z[0] + BASE_CBORE_DEPTH), CBORE_D / 2))
        for s in (-1, 1):
            xa, xb = sorted((E[0] + s * LID_FIN_U[0], E[0] + s * LID_FIN_U[1]))
            bh.append(cyl((xa - 0.1, 0, 0), (xb + 0.1, 0, 0), PIN_PRESS / 2))
    for hp in sg90_ear_holes_local():
        w = kin.apply(Msy, hp)
        bh.append(cyl((w[0], ear_y - MOUNT_T - 0.1, w[2]), (w[0], ear_y + 0.1, w[2]), M2_PILOT / 2))
    base_screws = []

    def foot_hole(x, z):
        base_screws.append((x, z))
        return cyl((x, BASE_Y[1] - 0.1, z), (x, BASE_Y[1] + FOOT_PILOT_DEPTH, z), M2_PILOT / 2)

    for xc in PILLAR_XS:
        bh.append(foot_hole(xc, (BEAM_Z[0] + BEAM_Z[1]) / 2))
    bf = D(bf, *bh)
    add("back_frame", bf, "static", C["frame"], material="PETG", bom_key="back_frame",
        print_R=np.eye(3), note="kiris arka yuzu (z=-36) tablada; kanatlar ve raf yukari")
    for i, hp in enumerate(sg90_ear_holes_local()):
        w = kin.apply(Msy, hp)
        add(f"scr_yawservo_{i}", screw((w[0], ear_y + SG_EAR_T, w[2]), (0, -1, 0), 5.0), "static", C["metal"],
            kind="hardware", bom_key="M2x5 ISO 4762")

    # pitch servo tutucu: kulak alti plaka + govdenin iki ucunda ayak
    ear_x = kin.PITCH_SERVO_BASE_X + SG_EAR_Z0
    ez = sorted(kin.apply(Msp, p)[2] for p in ((-SG_SHAFT_OFF, 0, 0), (SG_L - SG_SHAFT_OFF, 0, 0)))
    ear_ext = (SG_EAR_SPAN - SG_L) / 2
    pz0, pz1 = ez[0] - ear_ext - 7.5, ez[1] + ear_ext + 7.5
    pm = box(ear_x - MOUNT_T, ear_x, BASE_Y[1], ay + 12.0, pz0, pz1)
    feet = [box(ear_x - MOUNT_T, ear_x + 8.0, BASE_Y[1], BASE_Y[1] + FOOT_T, za, zb)
            for za, zb in ((pz0, ez[0] - ear_ext - 0.5), (ez[1] + ear_ext + 0.5, pz1))]
    pm = U(pm, *feet)
    pmh = [xform(sg90_cutout_local(), Msp)]
    for i, hp in enumerate(sg90_ear_holes_local()):
        w = kin.apply(Msp, hp)
        pmh.append(cyl((ear_x - MOUNT_T - 0.1, w[1], w[2]), (ear_x + 0.1, w[1], w[2]), M2_PILOT / 2))
        add(f"scr_pitchservo_{i}", screw((ear_x + SG_EAR_T, w[1], w[2]), (-1, 0, 0), 5.0), "static",
            C["metal"], kind="hardware", bom_key="M2x5 ISO 4762")
    for za, zb in ((pz0, ez[0] - ear_ext - 0.5), (ez[1] + ear_ext + 0.5, pz1)):
        pmh.append(foot_hole(ear_x + 3.5, (za + zb) / 2))
    add("mount_pitch_servo", D(pm, *pmh), "static", C["frame"], material="PETG", bom_key="mount_pitch_servo",
        print_R=kin.R(EY, -90)[:3, :3], note="plaka tablada, ayaklar yukari")

    # rocker yatak plakasi (sag)
    bx0 = ROCKER_END_X + AXIAL_GAP
    bp = U(box(bx0, bx0 + MOUNT_T, BASE_Y[1], ay + BEARING_TOP, az - 6.0, az + 6.0),
           box(bx0, bx0 + 10.0, BASE_Y[1], BASE_Y[1] + FOOT_T, az - 6.0, az + 6.0))
    bp = D(bp, cyl((bx0 - 0.1, ay, az), (bx0 + MOUNT_T + 0.1, ay, az), M2_CLEAR / 2), foot_hole(bx0 + 6.5, az))
    add("rocker_bearing", bp, "static", C["frame"], material="PETG", bom_key="rocker_bearing",
        print_R=kin.R(EY, -90)[:3, :3], note="plaka tablada")

    # kapak servo tutuculari (goz basina bir; iki servo; ust servonun kulaklari yukseltilmis pedlerde)
    for k, E in EYES.items():
        so = -1 if k == "L" else 1
        lo_base_u = LID_LO_LINK_U[1] + AXIAL_GAP - SG_SPLINE_TOP
        pl_u = (lo_base_u + SG_EAR_Z0 - MOUNT_T, lo_base_u + SG_EAR_Z0)
        xa, xb = sorted((E[0] + so * pl_u[0], E[0] + so * pl_u[1]))
        z0, z1 = LID_SERVO_UP_YZ[1] - 23.5, BEAM_Z[0] - 1.0
        plate = box(xa, xb, BASE_Y[1], LID_SERVO_UP_YZ[0] + 9.0, z0, z1)
        fa, fb = sorted((E[0] + so * pl_u[0], E[0] + so * 12.0))
        foot = box(fa, fb, BASE_Y[1], BASE_Y[1] + FOOT_T, z0, z1)
        cuts = []; extras = []
        for code in ("U", "L"):
            nm = "U" if code == "U" else "Lo"
            d = LID[nm]
            horn_u0 = (LID_UP_LINK_U[1] if nm == "U" else LID_LO_LINK_U[1]) + AXIAL_GAP
            base_u = horn_u0 - SG_SPLINE_TOP
            Ms = servo_M(np.array([E[0] + so * base_u, d["S"][0], d["S"][1]]), np.array([so, 0, 0.0]), -EZ)
            cuts.append(xform(sg90_cutout_local(), Ms))
            ear_u = base_u + SG_EAR_Z0
            for i, hp in enumerate(sg90_ear_holes_local()):
                w = kin.apply(Ms, hp)
                if ear_u > pl_u[1] + 0.01:
                    pa, pb = sorted((E[0] + so * pl_u[1] - so * 0.01, E[0] + so * ear_u))
                    extras.append(box(pa, pb, w[1] - 3.2, w[1] + 3.2, w[2] - 3.2, w[2] + 3.2))
                ha, hb = sorted((E[0] + so * (pl_u[0] - 0.1), E[0] + so * (ear_u + 0.1)))
                cuts.append(cyl((ha, w[1], w[2]), (hb, w[1], w[2]), M2_PILOT / 2))
                add(f"scr_lidservo_{code}{k}{i}",
                    screw((E[0] + so * (ear_u + SG_EAR_T), w[1], w[2]), (-so, 0, 0), 5.0), "static", C["metal"],
                    kind="hardware", bom_key="M2x5 ISO 4762")
        m = U(plate, foot, *extras)
        for zc in (z0 + 4.0, z1 - 4.0):
            cuts.append(foot_hole(E[0] + so * 8.0, zc))
        add(f"mount_lid_servos_{k}", D(m, *cuts), "static", C["frame"], material="PETG",
            bom_key=f"mount_lid_servos_{k}", print_R=kin.R(EY, -90 * so)[:3, :3], note="plaka tablada, ped/ayak yukari")

    # maske
    mask = box(*MASK_X, BASE_Y[1], MASK_Y[1], *MASK_Z)
    opens = [cq.Workplane("XY").workplane(offset=MASK_Z[0] - 1).center(E[0], 0)
             .ellipse(*MASK_OPEN_HALF).extrude(MASK_Z[1] - MASK_Z[0] + 2).val() for E in (E_L, E_R)]
    mfeet = [box(xc - 6, xc + 6, BASE_Y[1], BASE_Y[1] + FOOT_T, MASK_Z[0] - 8.0, MASK_Z[0] + 0.01) for xc in (-20.0, 20.0)]
    mh = [foot_hole(xc, MASK_Z[0] - 4.0) for xc in (-20.0, 20.0)]
    add("mask", D(U(mask, *mfeet), *opens, *mh), "static", C["mask"], material="PLA",
        bom_key="mask", print_R=kin.R(EX, 180)[:3, :3], note="on yuz tablada")

    # taban: alttan havsali M2x6 (4 mm taban: 2.2 havsa + 1.8 et; parcaya 4.2 mm girer)
    base = box(*BASE_X, *BASE_Y, *BASE_Z)
    bhs = []
    for (x, z) in base_screws:
        bhs.append(cyl((x, BASE_Y[0] - 0.1, z), (x, BASE_Y[1] + 0.1, z), M2_CLEAR / 2))
        bhs.append(cyl((x, BASE_Y[0] - 0.1, z), (x, BASE_Y[0] + BASE_CBORE_DEPTH, z), CBORE_D / 2))
    add("base", D(base, *bhs), "static", C["frame"], material="PETG", bom_key="base",
        print_R=kin.R(EX, 90)[:3, :3], note="alt yuz tablada")
    for i, (x, z) in enumerate(base_screws):
        add(f"scr_base_{i}", screw((x, BASE_Y[0] + BASE_CBORE_DEPTH, z), (0, 1, 0), 6.0), "static", C["metal"],
            kind="hardware", bom_key="M2x6 ISO 4762")

    # mid-poz gruplarini notr poza yerlestir (link ve vidalar orta pozda tasarlandi)
    for nm_, p in list(P.items()):
        if p.group.endswith("@mid"):
            g = p.group[:-4]
            Mmid = mid_to_neutral(g)
            p.shape = xform(p.shape, Mmid)
            p.group = g
    return P


# =============================================================================================== pozlar
def lid_deltas(state):
    """state: 'closed' | 'open' (mekanik tam acik) | 'nominal' | dict {UL: delta,...}"""
    if isinstance(state, dict):
        return state
    if state == "closed":
        return {f"{c}{k}": 0.0 for c in "UL" for k in "LR"}
    if state == "open":
        return {**{f"U{k}": kin.LID_UP_DMAX for k in "LR"}, **{f"L{k}": kin.LID_LO_DMIN for k in "LR"}}
    if state == "nominal":
        return {**{f"U{k}": kin.LID_UP_DNOM for k in "LR"}, **{f"L{k}": kin.LID_LO_DNOM for k in "LR"}}
    raise ValueError(state)


def _lid_link_pose(code, k, delta):
    nm = "U" if code == "U" else "Lo"
    d = LID[nm]
    A = kin.lid_arm_tip(nm, delta)
    phi, _ = kin.lid_horn_phi(nm, delta)
    B = d["S"] + HORN_R_LID * kin.dir_phi(phi)
    return A, B, phi


def _planar_M(A0, B0, A, B):
    """YZ duzleminde A0B0 -> AB rijit donusu (X ekseni etrafinda donus + oteleme)."""
    a0 = math.degrees(math.atan2(B0[0] - A0[0], B0[1] - A0[1]))   # phi (y,z)
    a1 = math.degrees(math.atan2(B[0] - A[0], B[1] - A[1]))
    dphi = a1 - a0
    return kin.T([0, A[0], A[1]]) @ kin.R(-EX, dphi) @ kin.T([0, -A0[0], -A0[1]])


def mid_to_neutral(g):
    """Orta pozda (d_mid) uretilen link/horn/kapak-vidasi gruplarini NOTR (kapali) poza tasiyan donusum."""
    code, k = g.split("_")[1][0], g.split("_")[1][1]
    nm = "U" if code == "U" else "Lo"
    d = LID[nm]
    if g.startswith("link_"):
        A0 = LID_ARM_R * d["n"]; B0 = d["S"] + HORN_R_LID * d["n"]
        A, B, _ = _lid_link_pose(code, k, 0.0)
        return _planar_M(A0, B0, A, B)
    if g.startswith("lidhorn_"):
        phi, _ = kin.lid_horn_phi(nm, 0.0)
        S3 = np.array([0, d["S"][0], d["S"][1]])
        return kin.about(S3, -EX, phi - d["phi_mid"])
    if g.startswith("lid_"):
        return kin.about(EYES[k], -EX, 0.0 - d["dmid"])
    raise ValueError(g)


def rod_frame(A, B, stud_a, stud_b_rodlocal=None):
    """Rod-yerel (A orijin, Z A->B, X = A saplama yonunun Z'ye dik bileseni) -> dunya."""
    z = B - A; L = np.linalg.norm(z); z = z / L
    x = stud_a - (stud_a @ z) * z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4); M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z; M[:3, 3] = A
    return M, L


def pose_transforms(yaw=0.0, pitch=0.0, lids="closed"):
    """Her grubun notr->poz 4x4 donusumu. Rod gruplari rod-yerel->dunya."""
    Ms = {"static": np.eye(4)}
    for k, E in EYES.items():
        Ms[f"eye_{k}"] = kin.eye_M(E, yaw, pitch)
        Ms[f"gobek_{k}"] = kin.hub_M(E, yaw)
    Ms["yawcoupler"] = kin.yaw_coupler_M(yaw)
    Ms["yawhorn"] = kin.yaw_servo_M(kin.solve_yaw_servo(yaw, pitch)[0])
    th, _ = kin.solve_pitch_servo(yaw, pitch, "L")
    Ms["rocker"] = kin.rocker_M(th)
    for k in EYES:
        A = kin.eye_ball_world(k, kin.B_PITCH, yaw, pitch)
        B = kin.rocker_ball(k, th)
        sa = kin.applyv(Ms[f"eye_{k}"], EY)
        M, L = rod_frame(A, B, sa)
        Ms[f"rod_pitch_{k}"] = M
    dl = lid_deltas(lids)
    for key, delta in dl.items():
        code, k = key[0], key[1]
        nm = "U" if code == "U" else "Lo"
        d = LID[nm]
        E = EYES[k]
        Ms[f"lid_{code}{k}"] = kin.lid_M(E, delta)
        # link: notr (kapali) -> poz
        A0, B0, phi0 = _lid_link_pose(code, k, 0.0)
        A, B, phi = _lid_link_pose(code, k, delta)
        Ms[f"link_{code}{k}"] = _planar_M(A0, B0, A, B)
        S3 = np.array([0, d["S"][0], d["S"][1]])
        Ms[f"lidhorn_{code}{k}"] = kin.about(S3, -EX, phi - phi0)
    return Ms


def servo_angles(yaw, pitch, lids="closed"):
    dl = lid_deltas(lids)
    out = {"EYE_YAW": 90 + kin.solve_yaw_servo(yaw, pitch)[0],
           "EYE_PITCH": 90 + kin.solve_pitch_servo(yaw, pitch, "L")[0]}
    for key, ch in (("UL", "LID_UL"), ("LL", "LID_LL"), ("UR", "LID_UR"), ("LR", "LID_LR")):
        nm = "U" if key[0] == "U" else "Lo"
        out[ch] = kin.lid_servo_deg(nm, key[1], dl[key])[0]
    return out


# =============================================================================================== disa aktarma
def tess(shape, tol=0.02, ang=0.2):
    v, t = shape.tessellate(tol, ang)
    return np.array([[p.x, p.y, p.z] for p in v]), np.array(t, dtype=np.int64)


def neutral_shapes(P):
    """Tum parcalar notr pozda dunya koordinatinda (rod-yerel uretilenler dahil)."""
    M0 = pose_transforms(0.0, 0.0, "closed")
    out = {}
    for n, p in P.items():
        M = M0[p.group]
        out[n] = p.shape if np.allclose(M, np.eye(4), atol=1e-9) else xform(p.shape, M)
    return out, M0


def export_all(P, tol=0.02):
    import trimesh
    for d_ in ("step", "stl", "glb"):
        os.makedirs(os.path.join(OUT, d_), exist_ok=True)
    W, M0 = neutral_shapes(P)
    done = set()
    asm = cq.Assembly(name="project_eye_v2")
    for name, p in P.items():
        if p.kind == "visual":
            continue
        asm.add(W[name], name=name, color=cq.Color(*[int(p.color[i:i + 2], 16) / 255 for i in (1, 3, 5)]))
        if p.kind == "printed":
            key = p.bom_key or name
            if key in done:
                continue
            done.add(key)
            W[name].exportStep(os.path.join(OUT, "step", f"{key}.step"))      # montaj konumunda (Fusion'a hizali)
            v, t = tess(p.shape, 0.01, 0.1)
            m = trimesh.Trimesh(v, t, process=True)
            Rm = np.eye(3) if p.print_R is None else np.asarray(p.print_R)
            m.apply_transform(np.block([[Rm, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
            bb = m.bounds
            m.apply_translation([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, -bb[0][2]])
            m.export(os.path.join(OUT, "stl", f"{key}.stl"))
    asm.save(os.path.join(OUT, "step", "assembly.step"))
    export_glb(P, W, M0, tol)


def export_glb(P, W, M0, tol=0.02):
    import trimesh
    scene = trimesh.Scene()
    groups = {}
    for name, p in P.items():
        groups.setdefault(p.group, []).append(name)
    for g, names in groups.items():
        scene.graph.update(frame_to="G_" + g, frame_from=scene.graph.base_frame, matrix=np.eye(4))
        for n in names:
            p = P[n]
            v, t = tess(W[n], tol, 0.2)
            m = trimesh.Trimesh(v, t, process=False)
            rgb = [int(p.color[i:i + 2], 16) for i in (1, 3, 5)]
            m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[*rgb, 255], metallicFactor=0.6 if p.kind == "hardware" else 0.05,
                roughnessFactor=0.5, name=n))
            scene.add_geometry(m, node_name=n, geom_name=n, parent_node_name="G_" + g)
    scene.export(os.path.join(OUT, "glb", "assembly.glb"))
    with open(os.path.join(OUT, "glb", "rig.json"), "w", encoding="utf-8") as f:
        json.dump(rig_json(P, M0), f, indent=1, ensure_ascii=False)


def _r(x, n=3):
    return [round(float(v), n) for v in x]


def rig_json(P, M0):
    """Goruntuleyici icin: grup dugumleri, pivot/eksenler ve bagli uzuvlar icin ornek tablolar (SPEC §1, mm)."""
    groups = {}
    for name, p in P.items():
        groups.setdefault(p.group, []).append(name)
    r = {"_units": "mm, derece", "_file": "assembly.glb: her anahtar (grup) 'G_<grup>' adli bir dugum, parcalar onun cocuklari (dugum adi = parca adi); mesh'ler dunya koordinatinda, NOTR pozda (yaw=pitch=0, kapaklar kapali)",
         "_coords": "SPEC §1: +X robotun sagi, +Y yukari, +Z ileri. Orijin iki goz merkezinin ortasi.",
         "_rotation": "Donen gruplar: M = T(pivot) * R(eksen, aci) * T(-pivot), sag-el kurali. three.js: grup dugumunun pivot etrafinda donmesi icin once -pivot oteleyin.",
         "_convention_eye": "Goz: M = T(E) * Ry(yaw) * Rx(-pitch) * T(-E)  (three.js: rotation.order='YXZ', rotation.y=yaw, rotation.x=-pitch)"}
    for k, E in EYES.items():
        r[f"eye_{k}"] = {"nodes": groups[f"eye_{k}"], "pivot": _r(E), "axes": {"yaw": [0, 1, 0], "pitch": [-1, 0, 0]},
                         "range": {"yaw": [-YAW_RANGE, YAW_RANGE], "pitch": [-PITCH_RANGE, PITCH_RANGE]},
                         "order": "once pitch (goz yerel X), sonra yaw (dunya Y)"}
        r[f"gobek_{k}"] = {"nodes": groups[f"gobek_{k}"], "pivot": _r(E), "axes": {"yaw": [0, 1, 0]},
                           "note": "gobek + yaw tupu/kolu: sadece yaw"}
        for code in "UL":
            nm = "U" if code == "U" else "Lo"
            d = LID[nm]
            tab = []
            for dl in np.linspace(d["dmin"], d["dmax"], 13):
                A, B, phi = _lid_link_pose(code, k, dl)
                _, _, phi0 = _lid_link_pose(code, k, 0.0)
                tab.append({"delta": round(float(dl), 3), "horn_deg_about_-X": round(phi - phi0, 3),
                            "arm_tip_yz": _r(A), "horn_tip_yz": _r(B)})
            r[f"lid_{code}{k}"] = {"nodes": groups[f"lid_{code}{k}"], "pivot": _r(E), "axes": {"delta": [-1, 0, 0]},
                                   "range_delta": [round(d["dmin"], 3), round(d["dmax"], 3)],
                                   "closed_edge_deg": round(kin.LID_UP_CLOSED_EDGE if nm == "U" else kin.LID_LO_CLOSED_EDGE, 3),
                                   "note": "delta: kapali=0; ust kapak + ile, alt kapak - ile acilir. Kenar acisi = closed_edge + delta"}
            S3 = [float(E[0]), float(d["S"][0]), float(d["S"][1])]
            r[f"lidhorn_{code}{k}"] = {"nodes": groups[f"lidhorn_{code}{k}"], "pivot": _r(S3), "axes": {"psi": [-1, 0, 0]},
                                       "table_from_lid": f"lid_{code}{k}.table -> horn_deg_about_-X"}
            r[f"link_{code}{k}"] = {"nodes": groups[f"link_{code}{k}"], "type": "planar_link_yz",
                                    "neutral_ends_yz": {"arm_tip": tab[0]["arm_tip_yz"] if d["dmin"] == 0 else None},
                                    "note": ("YZ duzleminde rijit: notrdeki (kapali) arm_tip->horn_tip dogru parcasini tablodaki "
                                             "arm_tip->horn_tip'e tasiyan donus(-X etrafinda)+oteleme")}
            r[f"lid_{code}{k}"]["table"] = tab
        r[f"rod_pitch_{k}"] = {"nodes": groups[f"rod_pitch_{k}"], "type": "rod",
                               "ends": {"a": {"group": f"eye_{k}", "point": _r(E + kin.B_PITCH), "stud_dir": [0, 1, 0]},
                                        "b": {"group": "rocker", "point": _r(kin.rocker_ball0(k)), "stud_dir": _r(kin.ROCKER_STUD_DIR[k])}},
                               "neutral_frame": [_r(row, 6) for row in M0[f"rod_pitch_{k}"]],
                               "note": ("mesh notr pozda. Yeni poz: a,b noktalarini gruplariyla tasiyin; F = [x z*x y... ] cerceve: "
                                        "z=(b-a)/|b-a|, x=stud_a'nin z'ye dik birimi, y=z cross x, orijin a. M = F * inverse(neutral_frame)")}
    ay, az = kin.ROCKER_AXIS_YZ
    tabr = {"yaw": [-30, -15, 0, 15, 30], "pitch": [-25, -12.5, 0, 12.5, 25]}
    tabr["theta"] = [[round(kin.solve_pitch_servo(y, p, "L")[0], 3) for y in tabr["yaw"]] for p in tabr["pitch"]]
    r["rocker"] = {"nodes": groups["rocker"], "pivot": [0.0, round(float(ay), 3), round(float(az), 3)], "axes": {"theta": [1, 0, 0]},
                   "table": tabr, "note": "theta = EYE_PITCH servo_deg - 90; tablo satir=pitch, sutun=yaw"}
    r["yawhorn"] = {"nodes": groups["yawhorn"], "pivot": [YAW_SERVO_X, 0.0, 0.0], "axes": {"yaw": [0, 1, 0]},
                    "note": "paralelkenar: aci = goz yaw"}
    r["yawcoupler"] = {"nodes": groups["yawcoupler"], "type": "translate",
                       "note": f"oteleme = (-{YAW_LEVER_R}*sin(yaw), 0, {YAW_LEVER_R}*(1-cos(yaw)))"}
    r["static"] = {"nodes": groups["static"]}
    for g in list(r):
        if not g.startswith("_"):
            r[g]["glb_node"] = "G_" + g
    return r


if __name__ == "__main__":
    import time
    t0 = time.time()
    P = build()
    print(f"{len(P)} parca, {time.time() - t0:.1f}s")
    bad = [n for n, p in P.items() if not p.shape.isValid()]
    print("gecersiz:", bad)
    export_all(P)
    import kinematics_export
    kinematics_export.write()
    print(f"bitti {time.time() - t0:.1f}s")
