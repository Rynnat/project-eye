"""Project Eye v4 (v3 POC'nin mekanik revizyonu) - parametrik parcalar + montaj (CadQuery).

Calistirma:  PYTHONUTF8=1 python eye_v4.py        -> out/step, out/stl, out/glb (+ rig.json), out/kinematics.json
Kullanim:    from eye_v4 import build, pose_transforms, servo_angles, tess
             parts = build()                          # {isim: Part}
             Ms = pose_transforms(yaw, pitch, lid)    # {grup: 4x4}; lid 0=kapali..1=acik ya da 'open'/'closed'

v3 ile ayni API. Tum parcalar dunya koordinatinda NOTR pozda (yaw=pitch=0, kapaklar kapali) uretilir. Hareketli parcalar
`group` alanindaki gruba aittir; grubun 4x4 donusumu pose_transforms() ile (kin.py). GLB'de grup dugumu 'G_<grup>'.
Gruplar: static, frame, eye_L, eye_R, coupler, yawcrank, lids (ust kapaklar), lids_lo (alt kapaklar), lidcrank,
         link_up, link_lo.
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
    y0, y1 = min(y0, y1), max(y0, y1)
    z0, z1 = min(z0, z1), max(z0, z1)
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def rbox(x0, x1, y0, y1, z0, z1, r=EDGE_R):
    """Kenarlari r yuvarlatilmis kutu (gorunen kenarlar >= 1 mm)."""
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    z0, z1 = min(z0, z1), max(z0, z1)
    r = min(r, 0.45 * min(x1 - x0, y1 - y0, z1 - z0))
    w = (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
         .translate((x0, y0, z0)))
    try:
        sh = w.edges().fillet(r).val()
    except Exception:
        sh = w.val()
    sols = sh.Solids()
    return sols[0] if len(sols) == 1 else sh


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
    r = shapes[0].fuse(*shapes[1:]).clean()
    if len(r.Solids()) > 1:                      # cok-parcali birlesimde OCC bazen ayri birakir: sirali birlestir
        r = shapes[0]
        for sh in shapes[1:]:
            r = r.fuse(sh).clean()
    sols = r.Solids()
    return sols[0] if len(sols) == 1 else r


def D(a, *bs):
    bs = [b for b in bs if b is not None]
    return a.cut(*bs).clean() if bs else a


def I(a, b):
    return a.intersect(b).clean()


def wedge_x(phi1, phi2, x0, x1, R=80.0):
    """X ekseni etrafinda phi1..phi2 (deg, phi = atan2(y, z)) dilimi, x0..x1 arasi. Aci farki < 180."""
    x0, x1 = min(x0, x1), max(x0, x1)
    n = max(3, int(abs(phi2 - phi1) / 6) + 2)
    pts = [(0.0, 0.0)] + [(R * math.sin(math.radians(p)), R * math.cos(math.radians(p)))
                          for p in np.linspace(phi1, phi2, n)]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def ann(x0, x1, r0, r1, phi1, phi2):
    """X ekseni etrafinda halka dilimi: r0..r1, phi1..phi2, x0..x1."""
    x0, x1 = min(x0, x1), max(x0, x1)
    ring = cyl((x0, 0, 0), (x1, 0, 0), r1)
    if r0 > 0:
        ring = D(ring, cyl((x0 - 1, 0, 0), (x1 + 1, 0, 0), r0))
    return I(ring, wedge_x(phi1, phi2, x0 - 1, x1 + 1, R=(r1 + 2) / math.cos(math.radians(min(60, abs(phi2 - phi1) / 2 + 1)))))


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


def slot_yz(A, B, w, x0, x1):
    """YZ duzleminde A(y,z)'den B(y,z)'ye yuvarlak uclu lama, x0..x1."""
    x0, x1 = min(x0, x1), max(x0, x1)
    A = np.asarray(A, float); B = np.asarray(B, float)
    return U(disk_x(A, w / 2, x0, x1), disk_x(B, w / 2, x0, x1), _bar_yz(A, B, w, x0, x1))


def _bar_yz(A, B, w, x0, x1):
    d = (B - A) / np.linalg.norm(B - A); n = np.array([-d[1], d[0]]) * w / 2
    return poly_yz([A + n, B + n, B - n, A - n], x0, x1)


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


def P3(x, rho, phi):
    """(x, rho, phi) -> dunya (x, y, z); phi = atan2(y, z)."""
    return np.array([x, rho * math.sin(math.radians(phi)), rho * math.cos(math.radians(phi))])


def torus_edge(E, R, r, phi, xhalf):
    """Kapak kenari yuvarlatmasi: E merkezli, phi duzleminde R yaricapli yarim cember boyunca r yaricapli tup.
    |x_rel| <= xhalf ile kirpilir, yalniz on (phi yonundeki) yari."""
    E = np.asarray(E, float)
    n = np.array([0.0, math.cos(math.radians(phi)), -math.sin(math.radians(phi))])     # duzlem normali
    tor = cq.Solid.makeTorus(R, r, V(*E), V(*n))
    u = np.array([0.0, math.sin(math.radians(phi)), math.cos(math.radians(phi))])
    half = xform(box(-xhalf, xhalf, -0.001, 60, -60, 60), frame_M(E, (1, 0, 0), u))
    return I(tor, half)


def polar_keep(E, alpha_deg, L=40.0):
    """E merkezli, X ekseninden alpha'dan UZAK (|x_rel| <= rho*cot(alpha)) bolge: kapak uclarini RADYAL keser (keskin kama yok)."""
    E = np.asarray(E, float)
    r = L * math.tan(math.radians(alpha_deg))
    big = box(E[0] - L, E[0] + L, -L, L, -L, L)
    c1 = cq.Solid.makeCone(0.0, r, L, V(*E), V(1, 0, 0))
    c2 = cq.Solid.makeCone(0.0, r, L, V(*E), V(-1, 0, 0))
    return D(big, c1, c2)


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
    """M4 havsa bas: face_pt = parca yuzeyi (havsa agzi) merkezi. L = toplam boy (kafa dahil)."""
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

    def pocket(self, clr=SERVO_POCKET_CLR / 2, clr_w=SERVO_POCKET_CLR_W / 2):
        """Cep: BOY yonunde SERVO_POCKET_CLR, EN yonunde SERVO_POCKET_CLR_W (toplam) + kablo yarigi (mile uzak uc)."""
        loc = box(-SG_SHAFT_OFF - clr, SG_L - SG_SHAFT_OFF + clr, -1, SG_H + 0.01, -SG_W / 2 - clr_w, SG_W / 2 + clr_w)
        x1 = SG_L - SG_SHAFT_OFF + clr
        slot = box(x1 - 0.5, x1 + CABLE_SLOT_OUT, -1, SG_H + 0.01, -CABLE_SLOT_W / 2, CABLE_SLOT_W / 2)
        return xform(U(loc, slot), self.M)

    def horn(self, arm_dir):
        """Cift kollu kol, mil ucunun ustunde (HORN_T). arm_dir: kollarin dunya yonu."""
        a = unit(arm_dir); t0 = self.tip; t1 = self.tip + self.sd * HORN_T
        hub = cyl(t0, t1, HORN_HUB_R)
        rect = U(*[cyl(t0 + a * s * r, t1 + a * s * r, 2.5) for s in (-1, 1) for r in np.linspace(0, HORN_R, 7)])
        body = U(hub, rect)
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


C = dict(eye="#f4f1ea", iris="#2f7f93", pupil="#0a0a0a", lever="#d8d2c2", frame="#5b646b", lid="#d9a282",
         lid_lo="#cf9676", base="#2e3438", servo="#3a64c8", horn="#f4f4f4", metal="#8a9096", coupler="#e3a455",
         crank="#e3a455", mount="#6f7a80", link="#c9b28f", mask="#3c4650")

JOINTS = set()        # tasarim geregi temas/eklem ciftleri (check.py eklem disi boslugu ayri raporlar)


def joint(a, b):
    JOINTS.add(tuple(sorted((a, b))))


def Rx(d):
    return kin.R(EX, d)[:3, :3]


def Ry(d):
    return kin.R(EY, d)[:3, :3]


PR_YUP = Rx(90)       # dunya +Y -> baski +Z (alt yuz tablada)
PR_YDOWN = Rx(-90)    # dunya -Y -> baski +Z (ust yuz tablada)
PR_FRONT_DOWN = Ry(180)   # dunya +Z (on) yuz tablada: dunya -Z -> baski +Z
PR_XUP = Ry(-90)      # dunya +X -> baski +Z (min-x yuzu tablada)
PR_XDOWN = Ry(90)     # dunya -X -> baski +Z (max-x yuzu tablada)


def crank_local():
    """yaw_crank STL'i (ayni parca LIDS krank olarak da kullanilir). Yerel: servo yuzu y=0 (+y servoya bakar),
    govde y -3.5..0, ucu (0, *, -LEVER_R), kol vidalari x = +-HORN_SCREW_R."""
    y0, y1 = -3.5, 0.0
    tip = (0.0, -LEVER_R)
    crank = U(disk_y((0, 0), 8.0, y0, y1), slot_xz((0, 0), tip, LEVER_W, y0, y1),
              slot_xz((-9.5, 0), (9.5, 0), 7.0, y0, y1))
    return D(crank, hole((0, y0, 0), (0, 1, 0), 4.0, 7.0, chamfer=True),
             *[hole((sx, y0, 0), (0, 1, 0), 4.0, HORN_SCREW_PILOT, chamfer=True) for sx in (-HORN_SCREW_R, HORN_SCREW_R)],
             hole((0, y0, tip[1]), (0, 1, 0), 4.0, M4_PILOT, chamfer=True))


def fillet_edge_phi(sh, phi, r):
    """phi duzlemindeki (kapak kenari) kenarlari r ile yuvarlat (ic ve dis yay). Basarisizsa oldugu gibi."""
    try:
        es = []
        for e in sh.Edges():
            c = e.Center()
            if abs(c.y) + abs(c.z) < 1e-6:
                continue
            p = math.degrees(math.atan2(c.y, c.z))
            if abs(p - phi) < 0.05 and e.Length() > 5:
                es.append(e)
        if not es:
            return sh
        out = sh.fillet(r, es)
        return out if out.isValid() else sh
    except Exception:
        return sh


def lap_parts(k, LP, x_hub, rho_full, phi_full):
    """Ortadaki bindirme: sol parca (L) ic kat x[a, b], sag parca (R) dis kat x[a+0.4, b+0.4] + hub'a kadar tam kesit."""
    a, b = LP["x"]; r0, rm, r1 = LP["rho"]; ph = LP["phi"]
    if k == "L":
        return [ann(a - 0.5, b, r0, rm, *ph)]
    return [ann(x_hub, b + AXIAL_GAP, *rho_full, *phi_full),
            ann(b + AXIAL_GAP, b + AXIAL_GAP + 0.6, r0, r1, *ph),
            ann(a + AXIAL_GAP, b + AXIAL_GAP, rm, r1, *ph)]


def lap_cuts(k, LP):
    r0, rm, r1 = LP["rho"]; phc = (LP["phi"][0] + LP["phi"][1]) / 2
    ax = -unit(P3(0, 1, phc)); out = []
    for bx, typ in LP["bolts"]:
        if k == "L":
            out.append(hole(P3(bx, rm, phc), ax, rm - r0 + 0.3, M4_PILOT if typ == "M4" else M3_PILOT))
        else:
            out.append(hole(P3(bx, r1, phc), ax, r1 - rm + 0.3, M4_RUN if typ == "M4" else M3_CLEAR))
    return out


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
        eye = I(sph(E, EYE_R), box(ex - EYE_R - 1, ex + EYE_R + 1, -EYE_FLAT_Y, EYE_TOP_Y, EYE_BACK_Z, EYE_R + 1))
        eye = D(eye,
                hole((ex, EYE_TOP_Y, 0), (0, -1, 0), PIN_TOP_RUN_DEPTH, M4_RUN),
                hole((ex, -EYE_FLAT_Y, 0), (0, 1, 0), PIN_BOT_RUN_DEPTH, M4_RUN, chamfer=True),
                hole((ex, -EYE_FLAT_Y, LEVER_LOCK_Z), (0, 1, 0), LEVER_LOCK_PILOT_DEPTH, M3_PILOT, chamfer=True))
        add(f"eye_{k}", eye, g, C["eye"], material="PLA beyaz", bom_key="eye", print_R=PR_YUP,
            note="alt duz yuz (y=-13) tablada; delikler dikey")
        add(f"iris_{k}", I(sph(E, EYE_R + 0.05), box(ex - 10, ex + 10, -10, 10, math.sqrt(EYE_R ** 2 - IRIS_R ** 2), 30)),
            g, C["iris"], kind="visual")
        add(f"pupil_{k}", I(sph(E, EYE_R + 0.1), box(ex - 5, ex + 5, -5, 5, math.sqrt(EYE_R ** 2 - PUPIL_R ** 2), 30)),
            g, C["pupil"], kind="visual")
        tip = (ex, -LEVER_R)
        lev = U(slot_xz((ex, 0.0), tip, LEVER_W, ybot, ytop), disk_y((ex, 0.0), 6.5, ybot, ytop))
        lev = D(lev,
                hole((ex, ytop, 0), (0, -1, 0), 4.0, M4_RUN, chamfer=True),
                hole((ex, ytop, LEVER_LOCK_Z), (0, -1, 0), 4.0, M3_CLEAR, chamfer=True),
                hole((ex, ytop, -LEVER_R), (0, -1, 0), 4.0, M4_PILOT, chamfer=True))
        add(f"eye_lever_{k}", lev, g, C["lever"], material="PLA", bom_key="eye_lever", print_R=PR_YDOWN,
            note="ust yuz tablada")
        add(f"scr_lock_{k}", screw_m3((ex, ybot, LEVER_LOCK_Z), (0, 1, 0)), g, C["metal"], kind="hardware",
            bom_key="M3x10 yuvarlak")
        add(f"scr_ltip_{k}", screw_round((ex, cy0, -LEVER_R), (0, 1, 0), 10.0), g, C["metal"], kind="hardware",
            bom_key="M4x10 yuvarlak")
        joint(f"scr_ltip_{k}", "coupler"); joint(f"eye_lever_{k}", "coupler"); joint(f"eye_lever_{k}", "scr_ctip")

    # ------------------------------------------------------------------ kafesler (frame_L / frame_R)
    log("kafesler...")
    pitch_srv = Servo((PITCH_SPLINE_X, 0, 0), (-1, 0, 0), (0, -1, 0))
    lsrv = Servo((LS_SPLINE_X, LS_S[0], LS_S[1]), (-1, 0, 0), (0, 1, 0))
    fz0, fz1 = STRIP_Z
    for k, E in EYES.items():
        s = SIDE_SIGN[k]; ex = E[0]; g = "frame"
        xr = lambda xrel: ex + s * xrel
        parts = []
        for side in (+1, -1):             # +1 dis plaka, -1 ic plaka
            a0, a1 = xr(side * PLATE_X[0]), xr(side * PLATE_X[1])
            plate = U(rbox(a0, a1, RB_Y[0], TB_Y[1], fz0, fz1),
                      I(disk_x((0, 0), PLATE_DISK_R, a0, a1), box(a0, a1, -10, 10, -10, FRONT_Z)))
            parts.append(plate)
        # sol dis plaka: pitch pimi gobegi
        if k == "L":
            parts.append(I(disk_x((0, 0), PLATE_DISK_R, xr(PITCH_PIN_BOSS_X[0] - 0.5), xr(PITCH_PIN_BOSS_X[1])),
                           box(xr(PITCH_PIN_BOSS_X[0] - 1), xr(PITCH_PIN_BOSS_X[1]), -10, 10, -10, FRONT_Z)))
        # ust kopru + ust pim gobegi, alt kopru
        parts.append(rbox(xr(-PLATE_X[1]), xr(PLATE_X[1]), TB_Y[0], TB_Y[1], *TB_Z))
        parts.append(I(disk_y((ex, 0), TOP_BOSS_R, EYE_TOP_Y + AXIAL_GAP, TB_Y[0] + 0.5), box(ex - 5, ex + 5, 0, 30, -10, FRONT_Z)))
        parts.append(rbox(xr(-PLATE_X[1]), xr(PLATE_X[1]), BB_Y[0], BB_Y[1], *BB_Z))
        # arka kiris yarisi (x=0'dan dis plakaya)
        parts.append(rbox(s * 0.1, xr(PLATE_X[1]), RB_Y[0], RB_Y[1], *RB_Z))
        # sag ic plaka: LIDS servo kanadi
        if k == "R":
            wing = U(rbox(*LS_WING_X, *LS_WING_Y, *LS_WING_Z),
                     rbox(*LS_WING_X, 15.2, LS_WING_Y[1], LS_WING_Z[1] - 1.0, fz0 + 1.0),
                     rbox(*LS_WING_X, LS_WING_Y[0], LS_WING_Y[0] + 2.0, LS_WING_Z[1] - 1.0, fz0 + 1.0))
            parts.append(wing)
        fr = U(*parts)
        cuts = []
        # goz pimleri (Y) - baskida yatay
        seat_top = TB_Y[1]
        cuts.append(csk_seat((ex, seat_top, 0), (0, -1, 0), seat_top - EYE_TOP_Y, M4_PILOT, horizontal=True))
        cuts.append(csk_seat((ex, BB_Y[0], 0), (0, 1, 0), BB_Y[1] - BB_Y[0] + 0.5, M4_PILOT, horizontal=True))
        # kapak mentese pilotlari (X, kor), iki kutup
        for side in (+1, -1):
            f = xr(side * PLATE_X[0])
            dep = PITCH_PIN_BOSS_X[1] - PLATE_X[0] + 0.3 if (k == "L" and side == 1) else PLATE_X[1] - PLATE_X[0] + 0.3
            cuts.append(hole((f, 0, 0), (s * side, 0, 0), dep, M4_PILOT, horizontal=True))
        # yaw_mount vidalari (kirisin altindan havsa)
        for zc in YM_SCREW_Z:
            cuts.append(csk_seat((s * YM_SCREW_X, RB_Y[0], zc), (0, 1, 0), RB_Y[1] - RB_Y[0] + 0.2, M4_RUN))
        if k == "R":
            outer = xr(PLATE_X[1])
            for yy in (-HORN_SCREW_R, HORN_SCREW_R):
                cuts.append(hole((outer, yy, 0), (-1, 0, 0), 4.0, HORN_SCREW_PILOT, horizontal=True))
            # LIDS servo cebi + kulak pilotlari
            cuts.append(lsrv.pocket())
            for (e_lo, e_hi) in lsrv.ear_holes():
                cuts.append(hole((LS_WING_X[0], e_lo[1], e_lo[2]), (1, 0, 0), LS_WING_X[1] - LS_WING_X[0] + 0.2, M3_PILOT,
                                 horizontal=True))
        fr = D(fr, *cuts)
        hh = ["ust/alt goz pimi pilotu (Y)", "4x kapak mentese pilotu (X)"]
        hh += ["pitch pimi pilotu (X)"] if k == "L" else ["2x pitch kol vidasi (X)", "2x LIDS servo kulagi M3 (X)"]
        add(f"frame_{k}", fr, g, C["frame"], material="PLA/PETG", bom_key=f"frame_{k}", print_R=PR_FRONT_DOWN,
            note="ON yuzu (z=+3.5) tablada; plakalar/kopruler/kiris z yonunde prizmatik", horizontal_holes=hh)
        add(f"scr_ptop_{k}", screw_csk((ex, seat_top, 0), (0, -1, 0)), g, C["metal"], kind="hardware", bom_key="M4x16 havsa")
        add(f"scr_pbot_{k}", screw_csk((ex, BB_Y[0], 0), (0, 1, 0)), g, C["metal"], kind="hardware", bom_key="M4x16 havsa")
        for nm in (f"scr_ptop_{k}", f"scr_pbot_{k}", f"frame_{k}"):
            joint(nm, f"eye_{k}")
        joint(f"frame_{k}", f"eye_lever_{k}"); joint(f"scr_pbot_{k}", f"eye_lever_{k}")
        # kapak mentese vidalari (M4x10, kafa ust kapak gobeginin cebinde, plakada pilot)
        for side, tag in ((+1, "o"), (-1, "i")):
            hf = np.array([xr(side * UP_CBORE_FLOOR), 0, 0])
            nm = f"scr_lpin_{k}{tag}"
            add(nm, screw_round(hf, (s * side, 0, 0), 10.0), g, C["metal"], kind="hardware", bom_key="M4x10 yuvarlak")
            joint(nm, f"lid_up_{k}"); joint(nm, f"lid_lo_{k}")
        joint(f"frame_{k}", f"lid_lo_{k}"); joint(f"lid_up_{k}", f"lid_lo_{k}")
        for zc in YM_SCREW_Z:
            add(f"scr_ym_{k}{int(-zc)}", screw_csk((s * YM_SCREW_X, RB_Y[0], zc), (0, 1, 0)), g, C["metal"],
                kind="hardware", bom_key="M4x16 havsa")

    # sol pitch pimi (cercevede sabit, pivot_bracket'te doner) ve sag pitch servosu (direkt)
    brk_x0 = -(EYE_X + PITCH_PIN_BOSS_X[1] + AXIAL_GAP)          # yatak ic yuzu
    brk_x1 = brk_x0 - BRACKET_T
    add("scr_fpivot", screw_round((brk_x1, 0, 0), (1, 0, 0), 10.0), "frame", C["metal"], kind="hardware",
        bom_key="M4x10 yuvarlak")
    joint("scr_fpivot", "pivot_bracket"); joint("frame_L", "pivot_bracket")
    add("servo_pitch", pitch_srv.body(), "static", C["servo"], kind="purchased", bom_key="SG90")
    add("horn_pitch", pitch_srv.horn((0, 1, 0)), "frame", C["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    joint("horn_pitch", "servo_pitch"); joint("frame_R", "servo_pitch")

    # ------------------------------------------------------------------ yaw servosu + yaw_mount (cercevede)
    log("yaw servo + mount + lama...")
    ysrv = Servo((0, ytop + HORN_T, YAW_SERVO_Z), (0, -1, 0), (0, 0, -1))
    add("servo_yaw", ysrv.body(), "frame", C["servo"], kind="purchased", bom_key="SG90")
    y_ear_top = ysrv.base[1] - SG_EAR_Z0
    py0, py1 = y_ear_top, y_ear_top + YM_PLATE_T
    plate = rbox(-YM_LEG_X[1], YM_LEG_X[1], py0, py1, *YM_PLATE_Z)
    legs = [rbox(sg * YM_LEG_X[0], sg * YM_LEG_X[1], RB_Y[1], py0 + 1.0, *YM_LEG_Z) for sg in (-1, 1)]
    ym = U(plate, *legs)
    ymc = [ysrv.pocket()]
    for (e_lo, e_hi) in ysrv.ear_holes():
        ymc.append(hole((e_lo[0], py1, e_lo[2]), (0, -1, 0), YM_PLATE_T + 0.2, M3_PILOT, chamfer=True))
    for sg in (-1, 1):
        for zc in YM_SCREW_Z:
            ymc.append(hole((sg * YM_SCREW_X, RB_Y[1], zc), (0, 1, 0), 12.0, M4_PILOT))
    add("yaw_mount", D(ym, *ymc), "frame", C["mount"], material="PLA", bom_key="yaw_mount", print_R=PR_YDOWN,
        note="plaka ust yuzu tablada, bacaklar yukari")
    for i, (e_lo, e_hi) in enumerate(ysrv.ear_holes()[:1]):
        add(f"scr_m3_yaw{i}", screw_m3(e_hi, (0, 1, 0)), "frame", C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")
    for k in ("L", "R"):
        for zc in YM_SCREW_Z:
            joint(f"scr_ym_{k}{int(-zc)}", "yaw_mount")

    # krank (yaw) = crank_local yerlestirilmis
    add("horn_yaw", ysrv.horn((1, 0, 0)), "yawcrank", C["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    joint("horn_yaw", "servo_yaw")
    CL = crank_local()
    add("yaw_crank", xform(CL, kin.T((0, ytop, YAW_SERVO_Z))), "yawcrank", C["crank"], material="PLA",
        bom_key="yaw_crank", print_R=PR_YUP, note="alt yuz tablada; ayni STL LIDS krankinda da kullanilir")
    ctip = (0.0, YAW_SERVO_Z - LEVER_R)
    add("scr_ctip", screw_round((0, cy0, ctip[1]), (0, 1, 0), 10.0), "yawcrank", C["metal"], kind="hardware",
        bom_key="M4x10 yuvarlak")
    joint("scr_ctip", "coupler"); joint("yaw_crank", "coupler")
    segs = [slot_xz(COUPLER_PATH[i], COUPLER_PATH[i + 1], COUPLER_W, cy0, cy1) for i in range(len(COUPLER_PATH) - 1)]
    cp = U(*segs, slot_xz((0, COUPLER_PATH[2][1]), ctip, COUPLER_W, cy0, cy1))
    cp = D(cp, *[hole((x, cy0, z), (0, 1, 0), 4.0, M4_RUN, chamfer=True)
                 for (x, z) in ((-EYE_X, -LEVER_R), (EYE_X, -LEVER_R), ctip)])
    add("coupler", cp, "coupler", C["coupler"], material="PLA", bom_key="coupler", print_R=PR_YUP,
        note="duz lama, alt yuz tablada")

    # ------------------------------------------------------------------ LIDS: servo (cercevede), krank, iki lama
    log("LIDS surucusu...")
    add("servo_lid", lsrv.body(), "frame", C["servo"], kind="purchased", bom_key="SG90")
    for i, (e_lo, e_hi) in enumerate(lsrv.ear_holes()[:1]):
        add(f"scr_m3_lid{i}", screw_m3(e_hi, (1, 0, 0)), "frame", C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")
    t_ = np.array([0.0, math.sin(math.radians(LS_A_PHI0)), math.cos(math.radians(LS_A_PHI0))])
    n_ = np.array([1.0, 0.0, 0.0])
    b_ = np.cross(n_, t_)
    face_x = LS_SPLINE_X - HORN_T                                   # krankin servo yuzu
    Mc = np.eye(4); Mc[:3, 0] = -b_; Mc[:3, 1] = n_; Mc[:3, 2] = -t_; Mc[:3, 3] = (face_x, LS_S[0], LS_S[1])
    add("lid_crank", xform(CL, Mc), "lidcrank", C["crank"], material="PLA", bom_key="yaw_crank", print_R=PR_YUP,
        note="yaw_crank ile ayni STL")
    add("horn_lid", lsrv.horn(b_), "lidcrank", C["horn"], kind="purchased", bom_key="SG90 kolu (cift)")
    joint("horn_lid", "servo_lid")
    cx0, cx1 = face_x - 3.5, face_x                                 # krank x
    lu0, lu1 = cx0 - AXIAL_GAP - LINK_T, cx0 - AXIAL_GAP            # ust lama
    ll0, ll1 = lu0 - AXIAL_GAP - LINK_LO_T, lu0 - AXIAL_GAP        # alt lama
    A0, U0, L0 = kin.A0, kin.U0, kin.L0
    lk_u = D(slot_yz(A0, U0, LINK_W, lu0, lu1),
             hole((lu0, *A0), (1, 0, 0), LINK_T + 0.4, M4_RUN, chamfer=True),
             hole((lu0, *U0), (1, 0, 0), LINK_T + 0.4, M4_RUN, chamfer=True))
    add("link_up", lk_u, "link_up", C["link"], material="PLA", bom_key="link_up", print_R=PR_XUP, note="duz lama")
    lk_l = D(slot_yz(A0, L0, LINK_W + 1.0, ll0, ll1),
             csk_seat((ll0, *A0), (1, 0, 0), LINK_LO_T + 0.4, M4_RUN),
             hole((ll0, *L0), (1, 0, 0), LINK_LO_T + 0.4, M4_RUN, chamfer=True))
    add("link_lo", lk_l, "link_lo", C["link"], material="PLA", bom_key="link_lo", print_R=PR_XUP,
        note="duz lama; A ucunda havsa yuvasi")
    add("scr_A", screw_csk((ll0 - 0.06, *A0), (1, 0, 0)), "lidcrank", C["metal"], kind="hardware", bom_key="M4x16 havsa")
    for a, b in (("scr_A", "link_up"), ("scr_A", "link_lo"), ("lid_crank", "link_up"), ("link_up", "link_lo"),
                 ("horn_lid", "scr_A")):
        joint(a, b)
    # ust pim (ust kapak tabinda sabit; kafa alt lamanin duzleminde) / alt pim (alt kapak tabinda sabit)
    add("scr_U", screw_round((lu0, *U0), (1, 0, 0), 10.0), "lids", C["metal"], kind="hardware", bom_key="M4x10 yuvarlak")
    tabL0, tabL1 = lu0, lu1                                          # alt kapak tabi ust lamanin duzleminde
    add("scr_L", screw_round((tabL1, *L0), (-1, 0, 0), 10.0), "lids_lo", C["metal"], kind="hardware",
        bom_key="M4x10 yuvarlak")
    for a, b in (("scr_U", "link_up"), ("link_up", "lid_up_L"), ("scr_L", "link_lo"), ("link_lo", "lid_lo_L")):
        joint(a, b)
    for i, sg in enumerate((-1, 1)):
        p = lsrv.horn_face() + b_ * sg * HORN_SCREW_R
        add(f"scr_horn_lid{i}", screw_round(p - lsrv.sd * HORN_T, lsrv.sd, 5.0, 1.2, 3.0, 1.2), "lidcrank", C["metal"],
            kind="hardware", bom_key="SG90 kol vidasi (servo ile gelir)")
        joint(f"scr_horn_lid{i}", "servo_lid")

    # ------------------------------------------------------------------ kapaklar
    log("kapaklar...")
    for k, E in EYES.items():
        s = SIDE_SIGN[k]; ex = E[0]
        xr = lambda xrel: ex + s * xrel
        shell = D(sph(E, LID_R_OUT), sph(E, LID_R_IN))
        # --- ust kapak
        lune = I(shell, wedge_x(UP_EDGE_CLOSED, UP_BACK_CLOSED, ex - 30, ex + 30))
        lip_edge = UP_EDGE_CLOSED - LIP_DROP
        lip_mid = (LIP_R_IN + LIP_R_OUT) / 2; lip_r = (LIP_R_OUT - LIP_R_IN) / 2
        nose = math.degrees(lip_r / lip_mid)
        # dudak kutuptan kutba surekli bant (uclarda kama/ada yok); kutup cevresinde rho >= 5.5 (gobege kaynar)
        keep = box(ex - LIP_HALF_X, ex + LIP_HALF_X, -40, 40, -40, 40)
        lip = I(I(D(sph(E, LIP_R_OUT), sph(E, LIP_R_IN)), wedge_x(lip_edge + nose, UP_EDGE_CLOSED + 12, ex - 30, ex + 30)), keep)
        conn = I(I(D(sph(E, LIP_R_IN + 0.1), sph(E, LID_R_OUT - 0.1)), wedge_x(UP_EDGE_CLOSED, UP_EDGE_CLOSED + 12, ex - 30, ex + 30)), keep)
        lip_nose = torus_edge(E, lip_mid, lip_r, lip_edge + nose, LIP_HALF_X)
        hubs = [disk_x((0, 0), HUB_R, xr(sd * UP_HUB_X[0]), xr(sd * UP_HUB_X[1])) for sd in (1, -1)]
        up = [lune, lip, conn, lip_nose, *hubs]
        # ic kutup: kol + cubuk (ortaya) + bindirme
        xi0 = xr(-UP_HUB_X[0]); xi1 = xr(-UP_HUB_X[1])
        up.append(ann(xi0, xi1, 3.0, UP_BAR_RHO_FULL[1], *UP_BAR_PHI))
        if k == "L":
            up.append(ann(xi1, -12.0, *UP_BAR_RHO_FULL, *UP_BAR_PHI))
            up.append(ann(-12.5, UP_LAP["x"][0] + 0.5, *UP_BAR_RHO, *UP_BAR_PHI))
            tx0, tx1 = cx0, cx1 - AXIAL_GAP
            up.append(U(ann(tx0, tx1, UP_BAR_RHO[0], LS_U[0] + 2.0, LS_U[1] - 8, UP_BAR_PHI[1]), disk_x(U0, 4.0, tx0, tx1)))
        up += lap_parts(k, UP_LAP, xi1, (UP_LAP["rho"][0], UP_LAP["rho"][2]), UP_LAP["phi"])
        lid = U(*up)
        cuts = []
        for sd in (1, -1):
            f = xr(sd * 17.0)
            cuts.append(cyl((f, 0, 0), (xr(sd * UP_CBORE_FLOOR), 0, 0), M4_CBORE_D / 2))
            cuts.append(hole((xr(sd * UP_CBORE_FLOOR), 0, 0), (s * sd, 0, 0), UP_HUB_X[1] - UP_CBORE_FLOOR + 0.3, M4_RUN))
        cuts += lap_cuts(k, UP_LAP)
        if k == "L":
            cuts.append(hole((cx0, *U0), (1, 0, 0), 3.8, M4_PILOT))
        lid = D(lid, *cuts)
        add(f"lid_up_{k}", lid, "lids", C["lid"], material="PLA ten rengi", bom_key=f"lid_up_{k}",
            print_R=PR_XUP if k == "L" else PR_XDOWN,
            note="dis gobek yuzu tablada (X dik); kutup kubbesi altinda sarkma", horizontal_holes=["2x bindirme civatasi (radyal)"])
        # --- alt kapak
        lune = I(shell, wedge_x(LO_BACK_CLOSED, LO_EDGE_CLOSED, ex - 30, ex + 30))
        lune = fillet_edge_phi(lune, LO_EDGE_CLOSED, 0.9)
        # kutup cevresinde ust gobegi bosalt
        lune = D(lune, *[disk_x((0, 0), HUB_R + AXIAL_GAP, xr(sd * 17.0), xr(sd * LO_HUB_X[0]))
                         for sd in (1, -1)])
        lo = [lune]
        for sd in (1, -1):
            lo.append(disk_x((0, 0), HUB_R, xr(sd * LO_HUB_X[0]), xr(sd * LO_HUB_X[1])))
            ear = ann(xr(sd * 16.0), xr(sd * LO_HUB_X[1]), LO_EAR_RHO[0], LO_EAR_RHO[1], *LO_EAR_PHI)
            lo.append(D(ear, sph(E, LID_R_IN)))
            lo.append(ann(xr(sd * LO_HUB_X[0]), xr(sd * LO_HUB_X[1]), 3.0, LO_EAR_RHO[1], *LO_EAR_PHI))
        xi0 = xr(-LO_HUB_X[0]); xi1 = xr(-LO_HUB_X[1])
        lo.append(ann(xi0, xi1, 3.0, LO_BAR_RHO[1], *LO_BAR_PHI))
        if k == "L":
            lo.append(ann(xi1, LO_LAP["x"][0] + 0.5, *LO_BAR_RHO, *LO_BAR_PHI))
            lo.append(U(ann(tabL0, tabL1, LO_BAR_RHO[0], LS_L[0] + 2.0, LS_L[1] - 4, LO_BAR_PHI[1]),
                        disk_x(L0, 4.2, tabL0, tabL1)))
        lo += lap_parts(k, LO_LAP, xi1, LO_BAR_RHO, LO_BAR_PHI)
        lid = U(*lo)
        cuts = [hole((xr(sd * LO_HUB_X[0]) - s * sd * 0.2, 0, 0), (s * sd, 0, 0), LO_HUB_X[1] - LO_HUB_X[0] + 0.4, M4_RUN)
                for sd in (1, -1)]
        cuts += lap_cuts(k, LO_LAP)
        if k == "L":
            cuts.append(hole((tabL1, *L0), (-1, 0, 0), 3.8, M4_PILOT))
        lid = D(lid, *cuts)
        add(f"lid_lo_{k}", lid, "lids_lo", C["lid_lo"], material="PLA ten rengi", bom_key=f"lid_lo_{k}",
            print_R=PR_XUP if k == "L" else PR_XDOWN, note="dis gobek yuzu tablada (X dik)",
            horizontal_holes=["2x bindirme civatasi (radyal)"])
    joint("lid_up_L", "lid_up_R"); joint("lid_lo_L", "lid_lo_R")
    # bindirme civatalari (radyal, disaridan; sag parcada gecis, sol parcada pilot)
    for grp, LP, tag in (("lids", UP_LAP, "up"), ("lids_lo", LO_LAP, "lo")):
        phc = (LP["phi"][0] + LP["phi"][1]) / 2
        for i, (bx, typ) in enumerate(LP["bolts"]):
            po = P3(bx, LP["rho"][2], phc); ax = -unit(P3(0, 1, phc))
            if typ == "M4":
                add(f"scr_lap_{tag}{i}", screw_round(po, ax, 10.0), grp, C["metal"], kind="hardware", bom_key="M4x10 yuvarlak")
            else:
                add(f"scr_lap_{tag}{i}", screw_m3(po, ax), grp, C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")

    # ------------------------------------------------------------------ sabit govde (base), sol yatak, maske
    log("base + maske...")
    by = BASE_Y[1]
    bx0, bx1 = brk_x1, brk_x0
    brk = U(disk_x((0, 0), 8.0, bx0, bx1), poly_yz([(by, -8), (by, 8), (0, 8), (0, -8)], bx0, bx1),
            rbox(bx0 - 3.0, bx1 + 3.0, by, by + BRK_FOOT_T, -20.0, 20.0))
    bcuts = [hole((bx1 + 0.1, 0, 0), (-1, 0, 0), BRACKET_T + 0.3, M4_RUN, horizontal=True)]
    base_cuts = []
    for sz in (-14.0, 14.0):
        sx = (bx0 + bx1) / 2
        bcuts.append(hole((sx, by, sz), (0, 1, 0), BRK_FOOT_T + 0.2, M4_PILOT, chamfer=True))
        base_cuts.append(csk_seat((sx, BASE_Y[0], sz), (0, 1, 0), BASE_Y[1] - BASE_Y[0] + 0.2, M4_RUN))
        add(f"scr_brk_{int(sz) + 10}", screw_csk((sx, BASE_Y[0], sz), (0, 1, 0)), "static", C["metal"], kind="hardware",
            bom_key="M4x16 havsa")
    add("pivot_bracket", D(brk, *bcuts), "static", C["base"], material="PLA/PETG", bom_key="pivot_bracket",
        print_R=PR_YUP, note="ayak tablada; M4_RUN yatagi yatay (+0.2)", horizontal_holes=["sol pitch yatagi M4_RUN (X)"])
    joint("pivot_bracket", "scr_fpivot"); joint("pivot_bracket", "frame_L")
    base = rbox(*BASE_X, *BASE_Y, *BASE_Z, r=3.0)
    # sag pitch servo tutucu
    qx0 = pitch_srv.ear_holes()[0][0][0]; qx1 = qx0 + MOUNT_T
    pm = rbox(qx0, qx1, by - 1.0, 13.5, -9.5, 9.5)
    qcuts = [pitch_srv.pocket()]
    for (e_lo, e_hi) in pitch_srv.ear_holes():
        qcuts.append(hole((qx0, e_lo[1], e_lo[2]), (1, 0, 0), MOUNT_T + 0.2, M3_PILOT, horizontal=True))
    # maske vidalari
    for mx in MASK_SCREW_X:
        base_cuts.append(csk_seat((mx, BASE_Y[0], 10.0), (0, 1, 0), BASE_Y[1] - BASE_Y[0] + 0.2, M4_RUN))
        add(f"scr_mask_{int(mx) + 100}", screw_csk((mx, BASE_Y[0], 10.0), (0, 1, 0)), "static", C["metal"],
            kind="hardware", bom_key="M4x16 havsa")
    b = D(U(base, D(pm, *qcuts)), *base_cuts)
    add("base", b, "static", C["base"], material="PLA/PETG", bom_key="base", print_R=PR_YUP,
        note="taban plakasi tablada; pitch servo tutucusu dik duvar; alttan 4 havsa", horizontal_holes=["2x pitch servo kulagi M3 (X)"])
    for i, (e_lo, e_hi) in enumerate(pitch_srv.ear_holes()[:1]):
        add(f"scr_m3_pitch{i}", screw_m3(e_hi, (1, 0, 0)), "static", C["metal"], kind="hardware", bom_key="M3x10 yuvarlak")
    for tag, srv, arm, grp in (("pitch", pitch_srv, (0, 1, 0), "frame"), ("yaw", ysrv, (1, 0, 0), "yawcrank")):
        f = srv.horn_face()
        for i, sg in enumerate((-1, 1)):
            p = f + unit(arm) * sg * HORN_SCREW_R
            add(f"scr_horn_{tag}{i}", screw_round(p - srv.sd * HORN_T, srv.sd, 5.0, 1.2, 3.0, 1.2), grp, C["metal"],
                kind="hardware", bom_key="SG90 kol vidasi (servo ile gelir)")
            joint(f"scr_horn_{tag}{i}", f"servo_{tag}")

    # maske: duz plaka (on yuz tablada), badem acikliklar (arkadan one genisleyen pah), alt ayak (tabana 2x M4)
    mz0, mz1 = MASK_Z
    mask = rbox(*MASK_X, MASK_Y[0], MASK_Y[1], mz0, mz1, r=1.2)
    mask = I(mask, _round_rect_prism(MASK_X, (MASK_Y[0], MASK_Y[1]), mz0 - 1, mz1 + 1, MASK_R))
    opens = [_almond(ex, mz0 - 0.5, mz1 + 0.5) for ex in (-EYE_X, EYE_X)]
    foot = rbox(-80.0, 94.0, by, by + MASK_FOOT_T, MASK_FOOT_Z[0], MASK_FOOT_Z[1] + 1.0)
    pads = [rbox(mx - 7, mx + 7, by, by + 12.0, 4.0, 16.0) for mx in MASK_SCREW_X]
    mcuts = [hole((mx, by, 10.0), (0, 1, 0), 10.3, M4_PILOT, chamfer=False) for mx in MASK_SCREW_X]
    add("mask", D(U(mask, foot, *pads), *opens, *mcuts), "static", C["mask"], material="PLA (koyu / ten)", bom_key="mask",
        print_R=PR_FRONT_DOWN, note="on yuz tablada; ayak ve civata pedleri dik duvar")
    return P


def _round_rect_prism(X, Y, z0, z1, r):
    w = X[1] - X[0]; h = Y[1] - Y[0]
    return (cq.Workplane("XY", origin=((X[0] + X[1]) / 2, (Y[0] + Y[1]) / 2, z0)).rect(w, h).extrude(z1 - z0)
            .edges("|Z").fillet(r).val())


def _almond_circles(a, b):
    c = (a * a - b * b) / (2 * b)
    return c, b + c


def _almond(ex, z0, z1):
    """Badem aciklik: iki cember kesisimi; arka yuzde MASK_OPEN, on yuzde MASK_OPEN_FRONT (egimli pah)."""
    cb, Rb = _almond_circles(*MASK_OPEN)
    cf, Rf = _almond_circles(*MASK_OPEN_FRONT)
    zb, zf = MASK_Z
    k0 = (z0 - zb) / (zf - zb); k1 = (z1 - zb) / (zf - zb)
    lerp = lambda a, b, t: a + (b - a) * t

    def fr(sign):
        c0 = lerp(cb, cf, k0); c1 = lerp(cb, cf, k1); r0 = lerp(Rb, Rf, k0); r1 = lerp(Rb, Rf, k1)
        w0 = cq.Wire.makeCircle(r0, V(ex, -sign * c0, z0), V(0, 0, 1))
        w1 = cq.Wire.makeCircle(r1, V(ex, -sign * c1, z1), V(0, 0, 1))
        return cq.Solid.makeLoft([w0, w1])
    return I(fr(+1), fr(-1))


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
    m = trimesh.Trimesh(v, t, process=True, validate=True)
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
    asm = cq.Assembly(name="project_eye_v4")
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
    S = [0.0, LS_S[0], LS_S[1]]
    r = {"_units": "mm, derece",
         "_file": ("assembly.glb: her anahtar (grup) 'G_<grup>' adli dugum, parcalar onun cocuklari (dugum adi = parca adi); "
                   "mesh'ler dunya koordinatinda, NOTR pozda (yaw=pitch=0, kapaklar kapali)."),
         "_coords": "+X robotun sagi, +Y yukari, +Z ileri. Orijin iki goz merkezinin ortasi (goz merkezleri X ekseninde).",
         "_rotation": "Donen grup: M = T(pivot) * R(eksen, aci) * T(-pivot), sag-el. Zincirli gruplarda 'parent' donusumu ONCE uygulanir: M = M_parent * M_yerel.",
         "_api": "eye_v4.pose_transforms(yaw, pitch, lid) butun gruplarin matrislerini verir (lid 0..1 ya da 'open'/'closed'). "
                 "Kapak gruplari dort-cubuk cozumu gerektirir: acilar kinematics.json -> channels.LIDS.table.",
         "static": {"nodes": groups["static"]},
         "frame": {"nodes": groups["frame"], "pivot": [0, 0, 0], "axis": [-1, 0, 0], "angle": "pitch",
                   "range": [-PITCH_RANGE, PITCH_RANGE], "note": "pitch cercevesi: M_frame = Rx(-pitch) (pitch + = yukari)"},
         "coupler": {"nodes": groups["coupler"], "parent": "frame", "type": "translate",
                     "note": f"M = M_frame * T(-{LEVER_R}*sin(yaw), 0, {LEVER_R}*(1-cos(yaw)))"},
         "yawcrank": {"nodes": groups["yawcrank"], "parent": "frame", "pivot": _r(YAW_SERVO_P), "axis": [0, 1, 0],
                      "angle": "yaw"},
         "lids": {"nodes": groups["lids"], "parent": "frame", "pivot": [0, 0, 0], "axis": [-1, 0, 0], "angle": "du",
                  "range": [0, UP_OPEN], "note": f"ust kapaklar: du = lid * {UP_OPEN}; M = M_frame * Rx(-du)"},
         "lids_lo": {"nodes": groups["lids_lo"], "parent": "frame", "pivot": [0, 0, 0], "axis": [1, 0, 0], "angle": "dl",
                     "range": [0, round(kin.LO_OPEN_ACT, 2)], "note": "alt kapaklar: dl dort-cubuktan (kinematics.json); M = M_frame * Rx(+dl)"},
         "lidcrank": {"nodes": groups["lidcrank"], "parent": "frame", "pivot": _r(S), "axis": [-1, 0, 0], "angle": "b",
                      "note": "LIDS krank: b (<0 acarken); M = M_frame * about(S, X, -b)"},
         "link_up": {"nodes": groups["link_up"], "parent": "frame", "type": "planar",
                     "note": "iki pim (krank ucu A, ust kapak pimi U) ile belirlenen duzlemsel hareket; pose_transforms kullanin"},
         "link_lo": {"nodes": groups["link_lo"], "parent": "frame", "type": "planar",
                     "note": "iki pim (A, alt kapak pimi L); pose_transforms kullanin"}}
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
    multi = [n for n, p in P.items() if p.kind == "printed" and len(p.shape.Solids()) != 1]
    print("tek kati olmayan:", multi)
    export_all(P)
    import kinematics_export
    kinematics_export.write()
    print(f"bitti {time.time() - t0:.1f}s")
