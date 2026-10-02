"""Project Eye v3 (POC) - parametrik parcalar + montaj (CadQuery).

Calistirma:  PYTHONUTF8=1 python eye_v3.py        -> out/step, out/stl, out/glb (+ rig.json), out/kinematics.json
Kullanim:    from eye_v3 import build, pose_transforms, servo_angles, tess
             parts = build()                          # {isim: Part}
             Ms = pose_transforms(yaw, pitch, lid)    # {grup: 4x4}; lid 0=kapali..1=acik ya da 'open'/'closed'

Tum parcalar dunya koordinatinda NOTR pozda (yaw=pitch=0, kapaklar kapali) uretilir. Hareketli parcalar `group`
alanindaki gruba aittir; grubun 4x4 donusumu pose_transforms() ile (kin.py). GLB'de grup dugumu 'G_<grup>'.
Gruplar: static, frame, eye_L, eye_R, coupler, yawcrank, lids.
"""
import math
import os
import json
from dataclasses import dataclass, field

import numpy as np
import cadquery as cq

from params import *
import kin
from kin import EX, EY, EZ, EYES, SIDE_SIGN, YAW_SERVO_P

V = cq.Vector
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


# =============================================================================================== yardimcilar
def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1)
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


def wedge_x(phi1, phi2, x0, x1, R=80.0):
    """X ekseni etrafinda phi1..phi2 (deg, phi = atan2(y, z)) dilimi, x0..x1 arasi."""
    x0, x1 = min(x0, x1), max(x0, x1)
    n = max(3, int(abs(phi2 - phi1) / 8) + 2)
    pts = [(0.0, 0.0)] + [(R * math.sin(math.radians(p)), R * math.cos(math.radians(p)))
                          for p in np.linspace(phi1, phi2, n)]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def poly_yz(pts, x0, x1):
    x0, x1 = min(x0, x1), max(x0, x1)
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline([tuple(map(float, p)) for p in pts]).close().extrude(x1 - x0).val()


def disk_x(yz, r, x0, x1):
    x0, x1 = min(x0, x1), max(x0, x1)
    return cyl((x0, yz[0], yz[1]), (x1, yz[0], yz[1]), r)


def slot_xz(A, B, w, y0, y1):
    """XZ duzleminde A(x,z)'den B'ye yuvarlak uclu lama, y0..y1."""
    A = np.asarray(A, float); B = np.asarray(B, float)
    L = float(np.linalg.norm(B - A)); m = (A + B) / 2
    ang = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
    return (cq.Workplane("XZ", origin=(0, y1, 0)).center(m[0], m[1])
            .slot2D(L + w, w, ang).extrude(y1 - y0).val())


def disk_y(xz, r, y0, y1):
    return cyl((xz[0], y0, xz[1]), (xz[0], y1, xz[1]), r)


def xform(shape, M):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    M = np.asarray(M, float)
    t = gp_Trsf()
    t.SetValues(*[float(M[i, j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


def frame_M(origin, xdir, ydir):
    x = np.asarray(xdir, float); x /= np.linalg.norm(x)
    y = np.asarray(ydir, float); y /= np.linalg.norm(y)
    z = np.cross(x, y)
    M = np.eye(4); M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z; M[:3, 3] = origin
    return M


def unit(v):
    v = np.asarray(v, float); return v / np.linalg.norm(v)


# ------------------------------------------------------------------ delikler (baski yonu kurallari)
def hole(face_pt, direction, depth, d, chamfer=False, horizontal=False):
    """face_pt: delik agzi (yuzey uzerinde), direction: iceri. horizontal=True -> baskida yatay delik, +HORIZ_HOLE_EXTRA.
    chamfer=True -> agiz tablada: BOTTOM_CHAMFER x 45 pah."""
    a = unit(direction); p = np.asarray(face_pt, float)
    dd = d + (HORIZ_HOLE_EXTRA if horizontal else 0.0)
    h = cyl(p - a * 0.2, p + a * depth, dd / 2)
    if chamfer:
        c = BOTTOM_CHAMFER
        h = U(h, cone(p - a * 0.2, p + a * c, dd / 2 + c + 0.2, dd / 2))
    return h


def csk_seat(face_pt, direction, depth, d_pilot, horizontal=False):
    """Havsa yuvasi (90 deg, agiz M4_CSK_D) + devaminda d_pilot delik. direction: iceri."""
    a = unit(direction); p = np.asarray(face_pt, float)
    r = M4_CSK_D / 2
    seat = U(cyl(p - a * 3.0, p, r + 0.01), cone(p, p + a * r, r, 0.0))
    return U(seat, hole(p, a, depth, d_pilot, horizontal=horizontal))


# ------------------------------------------------------------------ vidalar (govde kok capta modellenir)
def screw_round(head_face, axis, L, d_shank=M4_SHANK_MODEL, head_d=M4_HEAD_D, head_h=M4_HEAD_H):
    """head_face: kafanin oturdugu yuzey merkezi; axis: govdenin gittigi yon; L: govde boyu."""
    a = unit(axis); p = np.asarray(head_face, float)
    return U(cyl(p - a * head_h, p, head_d / 2), cyl(p, p + a * L, d_shank / 2))


def screw_csk(face_pt, axis, L=16.0):
    """M4 havsa bas: face_pt = parca yuzeyi (havsa agzi) merkezi. Kafa konisi yuvaya oturur ->
    kafa ustu yuzeyden (M4_CSK_D - kafa)/2 iceride. L = toplam boy (kafa dahil)."""
    a = unit(axis); p = np.asarray(face_pt, float)
    t0 = (M4_CSK_D - M4_CSK_HEAD_D) / 2
    top = p + a * t0
    hr = M4_CSK_HEAD_D / 2
    head = cone(top, top + a * (hr - 2.0), hr, 2.0)
    return U(head, cyl(top + a * (hr - 2.0 - 0.01), top + a * L, M4_SHANK_MODEL / 2))


def screw_m3(head_face, axis, L=10.0):
    return screw_round(head_face, axis, L, M3_SHANK_MODEL, M3_HEAD_D, M3_HEAD_H)


# =============================================================================================== SG90
def sg90_body():
    """Servo-yerel: +Y mil, +X govde boyu, orijin mil ekseni taban duzleminde (v2 ile ayni)."""
    body = box(-SG_SHAFT_OFF, SG_L - SG_SHAFT_OFF, 0, SG_H, -SG_W / 2, SG_W / 2)
    ex = (SG_EAR_SPAN - SG_L) / 2
    ears = box(-SG_SHAFT_OFF - ex, SG_L - SG_SHAFT_OFF + ex, SG_EAR_Z0, SG_EAR_Z0 + SG_EAR_T, -SG_W / 2, SG_W / 2)
    holes = [cyl((hx, SG_EAR_Z0 - 1, 0), (hx, SG_EAR_Z0 + 4, 0), SG_EAR_HOLE_DRILL / 2) for hx in sg90_ear_hole_x()]
    boss = cyl((0, SG_H, 0), (0, SG_TOP, 0), SG_BOSS_R)
    spline = cyl((0, SG_TOP, 0), (0, SG_SPLINE_TOP, 0), 2.4)
    return D(U(body, ears, boss, spline), *holes)


def sg90_ear_hole_x():
    hc = (SG_L - 2 * SG_SHAFT_OFF) / 2
    return [hc - SG_HOLE_SPACING / 2, hc + SG_HOLE_SPACING / 2]


class Servo:
    """Yerlestirilmis SG90. spline_top: mil ucu (dunya), shaft_dir: mil yonu (disari), long_dir: govde boyu yonu."""

    def __init__(self, spline_top, shaft_dir, long_dir):
        self.sd = unit(shaft_dir); self.ld = unit(long_dir)
        self.tip = np.asarray(spline_top, float)
        self.base = self.tip - self.sd * SG_SPLINE_TOP
        self.M = frame_M(self.base, self.ld, self.sd)

    def body(self):
        return xform(sg90_body(), self.M)

    def w(self, local):
        return kin.apply(self.M, local)

    def ear_holes(self):
        """(kulak alt yuzu (tabana bakan) noktasi, kulak ust yuzu noktasi) her delik icin."""
        return [(self.w((hx, SG_EAR_Z0, 0)), self.w((hx, SG_EAR_Z0 + SG_EAR_T, 0))) for hx in sg90_ear_hole_x()]

    def pocket(self, clr=SERVO_POCKET_CLR / 2):
        loc = box(-SG_SHAFT_OFF - clr, SG_L - SG_SHAFT_OFF + clr, -1, SG_H + 0.01, -SG_W / 2 - clr, SG_W / 2 + clr)
        # kablo yarigi: mile uzak kisa uc (+x), govde boyunca alttan uste, duvari delip gecer
        x1 = SG_L - SG_SHAFT_OFF + clr
        slot = box(x1 - 0.5, x1 + CABLE_SLOT_OUT, -1, SG_H + 0.01, -CABLE_SLOT_W / 2, CABLE_SLOT_W / 2)
        return xform(U(loc, slot), self.M)

    def horn(self, arm_dir):
        """Cift kollu kol, mil ucunun ustunde (HORN_T). arm_dir: kollarin dunya yonu."""
        a = unit(arm_dir); t0 = self.tip; t1 = self.tip + self.sd * HORN_T
        hub = cyl(t0, t1, HORN_HUB_R)
        arm = U(cyl(t0 - a * HORN_R, t1 - a * HORN_R, 2.5), cyl(t0 + a * HORN_R, t1 + a * HORN_R, 2.5))
        b = np.cross(self.sd, a)
        pts = []
        # kol govdesi (ince dikdortgen): 4 kose, sd boyunca ekstrude
        rect = U(*[cyl(t0 + a * s * r, t1 + a * s * r, 2.5) for s in (-1, 1) for r in np.linspace(0, HORN_R, 7)])
        body = U(hub, arm, rect)
        holes = [cyl(t0 + a * s * HORN_SCREW_R - self.sd * 0.1, t1 + a * s * HORN_SCREW_R + self.sd * 0.1, 0.6) for s in (-1, 1)]
        return D(body, *holes)

    def horn_face(self):
        return self.tip + self.sd * HORN_T


# =============================================================================================== Parca kaydi
@dataclass
class Part:
    name: str
    shape: object
    group: str
    color: str
    kind: str = "printed"          # printed | hardware | purchased | visual
    material: str = ""
    print_R: object = None         # 3x3 baski yonu donusu (dunya -> baski; +Z yukari, tabla z=0)
    note: str = ""
    bom_key: str = ""              # ayni STL'yi paylasan parcalar icin
    horizontal_holes: list = field(default_factory=list)


C = dict(eye="#f2efe6", iris="#2f7f93", pupil="#0a0a0a", lever="#d8d2c2", frame="#9aa5a8", lid="#d9a080",
         carrier="#c9b28f", base="#3b4449", servo="#3a64c8", horn="#f4f4f4", metal="#6d747a", coupler="#e3a455",
         crank="#e3a455", mount="#7f8c8d")

JOINTS = set()        # tasarim geregi temas/eklem ciftleri (check.py eklem disi boslugu ayri raporlar)


def joint(a, b):
    JOINTS.add(tuple(sorted((a, b))))


def Rx(d):
    return kin.R(EX, d)[:3, :3]


def Ry(d):
    return kin.R(EY, d)[:3, :3]


PR_YUP = Rx(90)       # dunya +Y -> baski +Z (alt yuz tablada)
PR_YDOWN = Rx(-90)    # dunya -Y -> baski +Z (ust yuz tablada)


# =============================================================================================== build
def build(verbose=True):
    P = {}
    JOINTS.clear()

    def add(name, shape, group, color, **kw):
        P[name] = Part(name, shape, group, color, **kw)

    def log(*a):
        if verbose:
            print(*a, flush=True)

    ytop, ybot = LEVER_Y[1], LEVER_Y[0]          # -13.4, -16.9
    cy0, cy1 = COUPLER_Y                          # -20.8, -17.3

    # ------------------------------------------------------------------ gozler + yaw kollari
    log("gozler...")
    for k, E in EYES.items():
        s = SIDE_SIGN[k]; ex = E[0]; g = f"eye_{k}"
        eye = I(sph(E, EYE_R), box(ex - EYE_R - 1, ex + EYE_R + 1, -EYE_FLAT_Y, EYE_FLAT_Y, EYE_BACK_Z, EYE_R + 1))
        eye = D(eye,
                hole((ex, EYE_FLAT_Y, 0), (0, -1, 0), PIN_TOP_RUN_DEPTH, M4_RUN),
                hole((ex, -EYE_FLAT_Y, 0), (0, 1, 0), PIN_BOT_RUN_DEPTH, M4_RUN, chamfer=True),
                hole((ex, -EYE_FLAT_Y, LEVER_LOCK_Z), (0, 1, 0), LEVER_LOCK_PILOT_DEPTH, M4_PILOT, chamfer=True))
        add(f"eye_{k}", eye, g, C["eye"], material="PLA beyaz", bom_key="eye", print_R=PR_YUP,
            note="alt duz yuz (y=-13) tablada; delikler dikey")
        add(f"iris_{k}", I(sph(E, EYE_R + 0.05), box(ex - 10, ex + 10, -10, 10, math.sqrt(EYE_R ** 2 - IRIS_R ** 2), 30)),
            g, C["iris"], kind="visual")
        add(f"pupil_{k}", I(sph(E, EYE_R + 0.1), box(ex - 5, ex + 5, -5, 5, math.sqrt(EYE_R ** 2 - PUPIL_R ** 2), 30)),
            g, C["pupil"], kind="visual")
        # yaw kolu (goz alt yuzune havsa vidayla kilitli, pim govdesi icinden gecer)
        tip = (ex, -LEVER_R)
        lev = U(slot_xz((ex, LEVER_LOCK_Z), tip, LEVER_W, ybot, ytop), disk_y((ex, 0.0), 6.5, ybot, ytop),
                disk_y((ex, LEVER_LOCK_Z), 6.2, ybot, ytop))
        lev = D(lev,
                hole((ex, ytop, 0), (0, -1, 0), 4.0, M4_RUN, chamfer=True),
                csk_seat((ex, ybot, LEVER_LOCK_Z), (0, 1, 0), 4.0, M4_RUN),
                hole((ex, ytop, LEVER_LOCK_Z), (0, -1, 0), 1.0, M4_RUN, chamfer=True),
                hole((ex, ytop, -LEVER_R), (0, -1, 0), 4.0, M4_PILOT, chamfer=True))
        add(f"eye_lever_{k}", lev, g, C["lever"], material="PLA", bom_key="eye_lever", print_R=PR_YDOWN,
            note="ust yuz tablada (havsa yukarida)")
        add(f"scr_lock_{k}", screw_csk((ex, ybot, LEVER_LOCK_Z), (0, 1, 0)), g, C["metal"], kind="hardware",
            bom_key="M4x16 havsa")
        add(f"scr_ltip_{k}", screw_round((ex, cy0, -LEVER_R), (0, 1, 0), 10.0), g, C["metal"], kind="hardware",
            bom_key="M4x10 yuvarlak")
        joint(f"scr_ltip_{k}", "coupler"); joint(f"eye_lever_{k}", "coupler"); joint(f"eye_lever_{k}", "scr_ctip")

    # ------------------------------------------------------------------ cerceve yarilari
    log("cerceve...")
    pitch_srv = Servo((PITCH_SPLINE_X, 0, 0), (-1, 0, 0), (0, -1, 0))
    for k, E in EYES.items():
        s = SIDE_SIGN[k]; ex = E[0]; g = "frame"
        xr = lambda xrel: ex + s * xrel
        side = wedge_x(SIDE_PHI[0], SIDE_PHI[1], xr(SIDE_X[0]), xr(SIDE_X[1]), R=SIDE_R)
        # ust kol: kure kabuk (18.4..20.8) + pim altinda duz yuz (y>=13.4), ayak izi |z|<=7
        shell = U(D(sph(E, FT_OUT), sph(E, FT_IN)), I(sph(E, FT_OUT), box(ex - 25, ex + 25, FT_FLAT_Y, 30, -30, 30)))
        top = I(shell, box(xr(FT_XREL[0]), xr(FT_XREL[1]), 0, 30, -FT_HALF_Z, FT_HALF_Z))
        blk = D(box(xr(FT_XREL[1]), xr(SIDE_X[0] + 0.01), 7.5, FT_BLOCK_R, -FT_HALF_Z, FT_HALF_Z), sph(E, FT_IN))
        bot = box(xr(FB_XREL[0]), xr(FB_XREL[1] + 0.01), FB_Y[0], FB_Y[1], -FB_HALF_Z, FB_HALF_Z)
        xm = U(box(s * 0.1, xr(SIDE_X[1]), XM_Y[0], XM_Y[1], XM_Z[0], XM_Z[1]),
               box(s * XM_PAD_X[0], xr(SIDE_X[1]), XM_Y[0], XM_Y[1], XM_PAD_Z[0], XM_PAD_Z[1]))
        fr = U(side, top, blk, bot, xm)
        cuts = []
        seat_top = TOP_CSK_TOP_Y + (M4_CSK_D - M4_CSK_HEAD_D) / 2
        cuts.append(csk_seat((ex, seat_top, 0), (0, -1, 0), seat_top - FT_FLAT_Y + 0.5, M4_PILOT, horizontal=True))
        cuts.append(cyl((ex, seat_top, 0), (ex, 30, 0), M4_CSK_D / 2 + 0.3))
        cuts.append(csk_seat((ex, FB_Y[0], 0), (0, 1, 0), FB_Y[1] - FB_Y[0] + 0.5, M4_PILOT, horizontal=True))
        for zc in YM_SCREW_Z:
            cuts.append(hole((s * YM_SCREW_X, XM_Y[1], zc), (0, -1, 0), XM_Y[1] - XM_Y[0] + 0.2, M4_PILOT, horizontal=True))
        outer = xr(SIDE_X[1])
        if k == "L":
            cuts.append(hole((outer, 0, 0), (s * -1, 0, 0), SIDE_X[1] - SIDE_X[0], M4_PILOT, chamfer=True))
        else:
            cuts.append(hole((outer, 0, 0), (-1, 0, 0), 6.0, 7.0, chamfer=True))
            for yy in (-HORN_SCREW_R, HORN_SCREW_R):
                cuts.append(hole((outer, yy, 0), (-1, 0, 0), 5.2, HORN_SCREW_PILOT, chamfer=True))
        fr = D(fr, *cuts)
        add(f"frame_{k}", fr, g, C["frame"], material="PLA/PETG", bom_key=f"frame_{k}",
            print_R=Ry(90 * s), note="yan plakanin dis yuzu tablada; kollar ve kiris yukari",
            horizontal_holes=["ust pim pilotu (Y)", "alt pim pilotu (Y)", "2x yaw_mount pilotu (Y)"])
        # pim vidalari (cercevede sabit; goz RUN deliginde doner)
        add(f"scr_ptop_{k}", screw_csk((ex, seat_top, 0), (0, -1, 0)), g, C["metal"], kind="hardware", bom_key="M4x16 havsa")
        add(f"scr_pbot_{k}", screw_csk((ex, FB_Y[0], 0), (0, 1, 0)), g, C["metal"], kind="hardware", bom_key="M4x16 havsa")
        for nm in (f"scr_ptop_{k}", f"scr_pbot_{k}", f"frame_{k}"):
            joint(nm, f"eye_{k}")
        joint(f"frame_{k}", f"eye_lever_{k}"); joint(f"scr_pbot_{k}", f"eye_lever_{k}")

    # sol pim (cercevede sabit, sabit yatakta doner) ve sag pitch servosu (direkt)
    xo = -(EYE_X + SIDE_X[1] + AXIAL_GAP + BRACKET_T)
    add("scr_fpivot", screw_round((xo, 0, 0), (1, 0, 0), 10.0), "frame", C["metal"], kind="hardware", bom_key="M4x10 yuvarlak")
    joint("scr_fpivot", "base"); joint("frame_L", "base")
    add("servo_pitch", pitch_srv.body(), "static", C["servo"], kind="purchased", bom_key="SG90")
    add("horn_pitch", pitch_srv.horn((0, 1, 0)), "frame", C["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    joint("horn_pitch", "servo_pitch"); joint("frame_R", "servo_pitch")

    # ------------------------------------------------------------------ yaw servosu + yaw_mount (cercevede)
    log("yaw servo + mount + lama...")
    ysrv = Servo((0, ytop + HORN_T, YAW_SERVO_Z), (0, -1, 0), (0, 0, -1))
    add("servo_yaw", ysrv.body(), "frame", C["servo"], kind="purchased", bom_key="SG90")
    y_ear_top = ysrv.base[1] - SG_EAR_Z0            # kulagin tabana bakan yuzu (y=2.0)
    py0, py1 = y_ear_top, y_ear_top + YM_PLATE_T
    plate = U(box(-YM_LEG_X[1], YM_LEG_X[1], py0, py1, *YM_PLATE_Z),
              *[box(sg * YM_LEG_X[0], sg * YM_LEG_X[1], py0, py1, YM_PLATE_Z[1] - 1, XM_PAD_Z[1]) for sg in (-1, 1)])
    legs = [box(sg * YM_LEG_X[0], sg * YM_LEG_X[1], XM_Y[1], py0 + 0.01, *XM_PAD_Z) for sg in (-1, 1)]
    ym = U(plate, *legs)
    ymc = [ysrv.pocket()]
    for (e_lo, e_hi) in ysrv.ear_holes():
        ymc.append(hole((e_lo[0], py1, e_lo[2]), (0, -1, 0), YM_PLATE_T + 0.2, M3_PILOT, chamfer=True))
    for sg in (-1, 1):
        for zc in YM_SCREW_Z:
            ymc.append(csk_seat((sg * YM_SCREW_X, py1, zc), (0, -1, 0), py1 - XM_Y[1] + 0.2, M4_RUN))
            add(f"scr_ym_{'L' if sg < 0 else 'R'}{int(-zc)}", screw_csk((sg * YM_SCREW_X, py1, zc), (0, -1, 0)), "frame",
                C["metal"], kind="hardware", bom_key="M4x16 havsa")
    add("yaw_mount", D(ym, *ymc), "frame", C["mount"], material="PLA", bom_key="yaw_mount", print_R=PR_YDOWN,
        note="plaka ust yuzu tablada, bacaklar yukari")
    for i, (e_lo, e_hi) in enumerate(ysrv.ear_holes()):
        add(f"scr_m3_yaw{i}", screw_m3(e_hi, (0, 1, 0)), "frame", C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")

    # krank (servo kolu + basili krank)
    P0 = np.array([0.0, 0.0, YAW_SERVO_Z])
    add("horn_yaw", ysrv.horn((1, 0, 0)), "yawcrank", C["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    joint("horn_yaw", "servo_yaw")
    ctip = (0.0, YAW_SERVO_Z - LEVER_R)
    crank = U(disk_y((0, YAW_SERVO_Z), 8.0, ybot, ytop), slot_xz((0, YAW_SERVO_Z), ctip, LEVER_W, ybot, ytop),
              slot_xz((-13, YAW_SERVO_Z), (13, YAW_SERVO_Z), 7.0, ybot, ytop))
    crank = D(crank, hole((0, ybot, YAW_SERVO_Z), (0, 1, 0), 4.0, 7.0, chamfer=True),
              *[hole((sx, ybot, YAW_SERVO_Z), (0, 1, 0), 4.0, HORN_SCREW_PILOT, chamfer=True) for sx in (-HORN_SCREW_R, HORN_SCREW_R)],
              hole((0, ybot, ctip[1]), (0, 1, 0), 4.0, M4_PILOT, chamfer=True))
    add("yaw_crank", crank, "yawcrank", C["crank"], material="PLA", bom_key="yaw_crank", print_R=PR_YUP,
        note="alt yuz tablada")
    add("scr_ctip", screw_round((0, cy0, ctip[1]), (0, 1, 0), 10.0), "yawcrank", C["metal"], kind="hardware",
        bom_key="M4x10 yuvarlak")
    joint("scr_ctip", "coupler"); joint("yaw_crank", "coupler")

    # paralelkenar lamasi
    cp = U(slot_xz((-EYE_X, -LEVER_R), (EYE_X, -LEVER_R), COUPLER_W, cy0, cy1),
           slot_xz((0, -LEVER_R), ctip, COUPLER_W, cy0, cy1))
    cp = D(cp, *[hole((x, cy0, z), (0, 1, 0), 4.0, M4_RUN, chamfer=True)
                 for (x, z) in ((-EYE_X, -LEVER_R), (EYE_X, -LEVER_R), ctip)])
    add("coupler", cp, "coupler", C["coupler"], material="PLA", bom_key="coupler", print_R=PR_YUP,
        note="duz lama, alt yuz tablada")

    # ------------------------------------------------------------------ kapaklar + tasiyici (lids grubu)
    log("kapaklar...")
    lsrv = Servo((LID_SPLINE_X, 0, 0), (1, 0, 0), (0, -math.sin(math.radians(60)), math.cos(math.radians(60))))
    add("servo_lid", lsrv.body(), "static", C["servo"], kind="purchased", bom_key="SG90")
    add("horn_lid", lsrv.horn((0, 1, 0)), "lids", C["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    joint("horn_lid", "servo_lid")
    hx0, hx1 = CAR_HUB_X
    hub = U(poly_yz([(CAR_HUB_R, 0), (0, CAR_HUB_R), (-CAR_HUB_R, 0), (0, -CAR_HUB_R)], hx0, hx1),   # 45 deg kenarli (desteksiz)
            poly_yz([(-13, -3), (-13, 5), (CAR_BAR_Y[1], 5), (CAR_BAR_Y[1], -3)], hx0, hx1))
    bar = box(-(EYE_X - LID_HALF_X), EYE_X - LID_HALF_X, CAR_BAR_Y[0], CAR_BAR_Y[1], -CAR_BAR_HALF_Z, CAR_BAR_HALF_Z)
    tabs = []
    car_cuts = [hole((hx0, 0, 0), (1, 0, 0), 5.0, 7.0, horizontal=True)]
    car_cuts += [hole((hx0, yy, 0), (1, 0, 0), 5.0, HORN_SCREW_PILOT, horizontal=True) for yy in (-HORN_SCREW_R, HORN_SCREW_R)]
    for k, E in EYES.items():
        s = SIDE_SIGN[k]; ex = E[0]
        xr = lambda xrel: ex + s * xrel
        tabs.append(box(xr(-LID_HALF_X - CAR_TAB_T), xr(-LID_HALF_X), CAR_TAB_Y[0], CAR_TAB_Y[1], -CAR_TAB_HALF_Z, CAR_TAB_HALF_Z))
        for (yy, zz) in LID_SCREW_YZ:
            car_cuts.append(hole((xr(-LID_HALF_X - CAR_TAB_T), yy, zz), (s, 0, 0), CAR_TAB_T + 0.2, M4_RUN, horizontal=True))
    add("lid_carrier", D(U(hub, bar, *tabs), *car_cuts), "lids", C["carrier"], material="PLA", bom_key="lid_carrier",
        print_R=PR_YDOWN, note="ust kopru ust yuzu tablada; gobek plakasi ve kulaklar yukari",
        horizontal_holes=["4x kapak vidasi RUN (X)", "2x kol vidasi + orta delik (X)"])
    joint("lid_carrier", "horn_lid")
    for k, E in EYES.items():
        s = SIDE_SIGN[k]; ex = E[0]
        xr = lambda xrel: ex + s * xrel
        band = I(D(sph(E, LID_R_OUT), sph(E, LID_R_IN)),
                 box(xr(-LID_HALF_X), xr(LID_HALF_X), -40, 40, -40, 40))
        band = I(band, wedge_x(LID_EDGE_CLOSED, LID_BACK, ex - 40, ex + 40))
        boss = I(disk_x((0, 0), LID_BOSS_R, xr(LID_BOSS_XREL[0]), xr(LID_BOSS_XREL[1])),
                 wedge_x(LID_BOSS_PHI[0], LID_BOSS_PHI[1], ex - 40, ex + 40))
        boss = D(boss, sph(E, LID_R_IN))
        lid = U(band, boss)
        lid = D(lid, *[hole((xr(-LID_HALF_X), yy, zz), (s, 0, 0), 6.5, M4_PILOT, chamfer=True) for (yy, zz) in LID_SCREW_YZ])
        add(f"lid_{k}", lid, "lids", C["lid"], material="PLA ten rengi", bom_key=f"lid_{k}", print_R=Ry(-90 * s),
            note="ic uc yuzu (takviye) tablada; kabuk yukari")
        for i, (yy, zz) in enumerate(LID_SCREW_YZ):
            nm = f"scr_lid_{k}{i}"
            add(nm, screw_round((xr(-LID_HALF_X - CAR_TAB_T), yy, zz), (s, 0, 0), 10.0), "lids", C["metal"],
                kind="hardware", bom_key="M4x10 yuvarlak")

    # ------------------------------------------------------------------ sabit govde (base)
    log("base...")
    base = box(*BASE_X, *BASE_Y, *BASE_Z)
    by = BASE_Y[1] - 0.01
    # sol yatak
    bx0 = xo; bx1 = xo + BRACKET_T
    brk = U(disk_x((0, 0), 8.0, bx0, bx1), poly_yz([(BASE_Y[1], -8), (BASE_Y[1], 8), (0, 8), (0, -8)], bx0, bx1))
    bcuts = [hole((bx0, 0, 0), (1, 0, 0), BRACKET_T, M4_RUN, horizontal=True)]
    # orta direk (kapak servosu kulaklari)
    u = lsrv.ld; v = np.cross(lsrv.sd, lsrv.ld)       # YZ duzleminde
    ear_x = [e[0][0] for e in lsrv.ear_holes()]       # kulagin tabana bakan yuzu x
    px1 = ear_x[0]; px0 = px1 - MOUNT_T
    corners = [(a * u + b * v) for a, b in ((-13, -9.5), (24.5, -9.5), (24.5, 9.5), (-13, 9.5))]
    post = U(poly_yz([(c[1], c[2]) for c in corners], px0, px1),
             poly_yz([(by, 3), (by, 19), (-14, 19), (-14, 3)], px0, px1))
    pcuts = [lsrv.pocket()]
    for (e_lo, e_hi) in lsrv.ear_holes():
        pcuts.append(hole((px1, e_lo[1], e_lo[2]), (-1, 0, 0), MOUNT_T + 0.2, M3_PILOT, horizontal=True))
    # sag pitch servo tutucu
    qx0 = pitch_srv.ear_holes()[0][0][0]; qx1 = qx0 + MOUNT_T
    pm = poly_yz([(by, -9.5), (by, 9.5), (13.5, 9.5), (13.5, -9.5)], qx0, qx1)
    qcuts = [pitch_srv.pocket()]
    for (e_lo, e_hi) in pitch_srv.ear_holes():
        qcuts.append(hole((qx0, e_lo[1], e_lo[2]), (1, 0, 0), MOUNT_T + 0.2, M3_PILOT, horizontal=True))
    fy0 = BASE_Y[1]; fy1 = fy0 + BRK_FOOT_T
    brk = U(brk, box(*BRK_FOOT_X, fy0, fy1, -10.0, 10.0))
    base_cuts = []
    for sx in BRK_SCREW_X:
        bcuts.append(hole((sx, fy0, 0), (0, 1, 0), BRK_FOOT_T + 0.2, M4_PILOT, chamfer=True))
        base_cuts.append(csk_seat((sx, BASE_Y[0], 0), (0, 1, 0), BASE_Y[1] - BASE_Y[0] + 0.2, M4_RUN))
        add(f"scr_brk_{int(-sx)}", screw_csk((sx, BASE_Y[0], 0), (0, 1, 0)), "static", C["metal"], kind="hardware",
            bom_key="M4x16 havsa")
    add("pivot_bracket", D(brk, *bcuts), "static", C["base"], material="PLA/PETG", bom_key="pivot_bracket",
        print_R=PR_YUP, note="ayak tablada; M4_RUN yatagi yatay (+0.2)",
        horizontal_holes=["sol pitch yatagi M4_RUN (X)"])
    joint("pivot_bracket", "scr_fpivot"); joint("pivot_bracket", "frame_L")
    b = D(U(base, D(post, *pcuts), D(pm, *qcuts)), *base_cuts)
    add("base", b, "static", C["base"], material="PLA/PETG", bom_key="base", print_R=PR_YUP,
        note="taban plakasi tablada; direk/tutucu dik duvarlar; alttan 2 havsa (sol yatak)",
        horizontal_holes=["4x servo kulagi M3 pilot (X)"])
    for i, (e_lo, e_hi) in enumerate(lsrv.ear_holes()):
        add(f"scr_m3_lid{i}", screw_m3(e_hi, (-1, 0, 0)), "static", C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")
    for i, (e_lo, e_hi) in enumerate(pitch_srv.ear_holes()):
        add(f"scr_m3_pitch{i}", screw_m3(e_hi, (1, 0, 0)), "static", C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")
    # servo kol vidalari (servo ile gelen kucuk vidalar) - gorsel
    for tag, srv, arm, grp in (("pitch", pitch_srv, (0, 1, 0), "frame"), ("yaw", ysrv, (1, 0, 0), "yawcrank"),
                               ("lid", lsrv, (0, 1, 0), "lids")):
        f = srv.horn_face()
        for i, sg in enumerate((-1, 1)):
            p = f + unit(arm) * sg * HORN_SCREW_R
            add(f"scr_horn_{tag}{i}", screw_round(p - srv.sd * HORN_T, srv.sd, 5.0, 1.2, 3.0, 1.2), grp, C["metal"],
                kind="hardware", bom_key="SG90 kol vidasi (servo ile gelir)")
            joint(f"scr_horn_{tag}{i}", f"servo_{tag}")
    return P


# =============================================================================================== pozlar
def pose_transforms(yaw=0.0, pitch=0.0, lid="closed"):
    """{grup: 4x4} notr -> poz. lid: 0 kapali .. 1 acik, ya da 'closed'/'open'/'half'."""
    return kin.pose(yaw, pitch, lid)


def servo_angles(yaw=0.0, pitch=0.0, lid="closed"):
    return kin.servo_angles(yaw, pitch, lid)


# =============================================================================================== disa aktarma
def tess(shape, tol=0.02, ang=0.2):
    v, t = shape.tessellate(tol, ang)
    return np.array([[p.x, p.y, p.z] for p in v]), np.array(t, dtype=np.int64)


def print_mesh(part, tol=0.01, ang=0.1):
    """Parca baski yonunde, tabla z=0, XY ortali trimesh."""
    import trimesh
    v, t = tess(part.shape, tol, ang)
    m = trimesh.Trimesh(v, t, process=True, validate=True)   # validate: dejenere ucgenleri ayikla
    m.merge_vertices()
    Rm = np.eye(3) if part.print_R is None else np.asarray(part.print_R)
    m.apply_transform(np.block([[Rm, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
    bb = m.bounds
    m.apply_translation([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, -bb[0][2]])
    return m


def export_all(P, tol=0.02):
    for d_ in ("step", "stl", "glb"):
        os.makedirs(os.path.join(OUT, d_), exist_ok=True)
    for d_ in ("stl", "step"):
        for f in os.listdir(os.path.join(OUT, d_)):
            os.remove(os.path.join(OUT, d_, f))
    done = set()
    asm = cq.Assembly(name="project_eye_v3")
    for name, p in P.items():
        if p.kind == "visual":
            continue
        asm.add(p.shape, name=name, color=cq.Color(*[int(p.color[i:i + 2], 16) / 255 for i in (1, 3, 5)]))
        if p.kind == "printed":
            key = p.bom_key or name
            if key in done:
                continue
            done.add(key)
            p.shape.exportStep(os.path.join(OUT, "step", f"{key}.step"))
            print_mesh(p).export(os.path.join(OUT, "stl", f"{key}.stl"))
    asm.save(os.path.join(OUT, "step", "assembly.step"))
    export_glb(P, tol)


def export_glb(P, tol=0.02):
    import trimesh
    scene = trimesh.Scene()
    groups = {}
    for name, p in P.items():
        groups.setdefault(p.group, []).append(name)
    for g, names in groups.items():
        scene.graph.update(frame_to="G_" + g, frame_from=scene.graph.base_frame, matrix=np.eye(4))
        for n in names:
            p = P[n]
            v, t = tess(p.shape, tol, 0.2)
            m = trimesh.Trimesh(v, t, process=False)
            rgb = [int(p.color[i:i + 2], 16) for i in (1, 3, 5)]
            m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[*rgb, 255], metallicFactor=0.6 if p.kind == "hardware" else 0.05,
                roughnessFactor=0.5, name=n))
            scene.add_geometry(m, node_name=n, geom_name=n, parent_node_name="G_" + g)
    scene.export(os.path.join(OUT, "glb", "assembly.glb"))
    with open(os.path.join(OUT, "glb", "rig.json"), "w", encoding="utf-8") as f:
        json.dump(rig_json(P), f, indent=1, ensure_ascii=False)


def _r(x, n=3):
    return [round(float(v), n) for v in x]


def rig_json(P):
    groups = {}
    for name, p in P.items():
        groups.setdefault(p.group, []).append(name)
    r = {"_units": "mm, derece",
         "_file": ("assembly.glb: her anahtar (grup) 'G_<grup>' adli dugum, parcalar onun cocuklari (dugum adi = parca adi); "
                   "mesh'ler dunya koordinatinda, NOTR pozda (yaw=pitch=0, kapaklar kapali)."),
         "_coords": "+X robotun sagi, +Y yukari, +Z ileri. Orijin iki goz merkezinin ortasi (goz merkezleri X ekseninde).",
         "_rotation": "Donen grup: M = T(pivot) * R(eksen, aci) * T(-pivot), sag-el. Zincirli gruplarda 'parent' donusumu ONCE uygulanir: M = M_parent * M_yerel.",
         "_api": "eye_v3.pose_transforms(yaw, pitch, lid) ayni matrisleri verir (lid 0..1 ya da 'open'/'closed').",
         "static": {"nodes": groups["static"]},
         "frame": {"nodes": groups["frame"], "pivot": [0, 0, 0], "axis": [-1, 0, 0], "angle": "pitch",
                   "range": [-PITCH_RANGE, PITCH_RANGE], "note": "pitch cercevesi: M_frame = Rx(-pitch) (pitch + = yukari)"},
         "coupler": {"nodes": groups["coupler"], "parent": "frame", "type": "translate",
                     "note": f"M = M_frame * T(-{LEVER_R}*sin(yaw), 0, {LEVER_R}*(1-cos(yaw)))"},
         "yawcrank": {"nodes": groups["yawcrank"], "parent": "frame", "pivot": _r(YAW_SERVO_P), "axis": [0, 1, 0],
                      "angle": "yaw"},
         "lids": {"nodes": groups["lids"], "pivot": [0, 0, 0], "axis": [-1, 0, 0], "angle": "delta",
                  "range": [0, LID_OPEN], "note": f"delta = lid * {LID_OPEN} (0 kapali, 1 tam acik); M = Rx(-delta)"}}
    for k, E in EYES.items():
        r[f"eye_{k}"] = {"nodes": groups[f"eye_{k}"], "parent": "frame", "pivot": _r(E), "axis": [0, 1, 0],
                         "angle": "yaw", "range": [-YAW_RANGE, YAW_RANGE],
                         "note": "M = M_frame * T(E) Ry(yaw) T(-E). three.js: rotation.order='XYZ', rotation.x=-pitch, rotation.y=yaw (pivot E)"}
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
