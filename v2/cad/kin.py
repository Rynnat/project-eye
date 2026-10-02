"""Project Eye v2 - kinematik (saf numpy, CadQuery'siz).

Tum bagli uzuvlarin (servo kolu, cubuk, goz/kapak) notr geometrisi ve poz -> servo acisi cozumleri.
eye_v2.py (geometri), check.py (tarama) ve kinematics.json ayni fonksiyonlari kullanir.

Isaret kurallari:
  * Goz pozu:   R_eye = Ry(yaw) * Rx(-pitch)  (SPEC §1: yaw+ robotun sagi (+X), pitch+ yukari)
  * Kapak:      delta = kapagin phi acisindaki artis (phi = atan2(y, z), +Z'den +Y'ye).  R_lid = Rx(-delta)
  * Servo:      servo_deg = 90 + (kolun, servo milinin DISA bakan ekseni etrafinda sag-el donusu).
                Mil ucundan (kol tarafindan) bakinca saat yonunun tersi = pozitif.
                SG90'da darbe genisligi artinca hangi yone dondugu uretime gore degisir -> settings.json 'invert'.
"""
import math
import numpy as np
from params import *

D2R = math.pi / 180


# ---------------------------------------------------------------- 4x4 yardimcilar
def T(v):
    M = np.eye(4); M[:3, 3] = v; return M


def R(axis, deg):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    t = deg * D2R; c, s = math.cos(t), math.sin(t); x, y, z = a
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                 [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                 [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]]
    return M


def about(point, axis, deg):
    p = np.asarray(point, float)
    return T(p) @ R(axis, deg) @ T(-p)


def apply(M, p):
    p = np.asarray(p, float)
    return (M[:3, :3] @ p) + M[:3, 3]


def applyv(M, v):
    return M[:3, :3] @ np.asarray(v, float)


EX, EY, EZ = np.eye(3)
E_L = np.array([-EYE_X, 0.0, 0.0])
E_R = np.array([+EYE_X, 0.0, 0.0])
EYES = {"L": E_L, "R": E_R}


def eye_M(E, yaw, pitch):
    return T(E) @ R(EY, yaw) @ R(EX, -pitch) @ T(-E)


def hub_M(E, yaw):
    return T(E) @ R(EY, yaw) @ T(-E)


def lid_M(E, delta):
    return T(E) @ R(EX, -delta) @ T(-E)


def phi_of(y, z):
    return math.degrees(math.atan2(y, z))


def dir_phi(phi):
    """(y, z) birim vektor, phi derece."""
    return np.array([math.sin(phi * D2R), math.cos(phi * D2R)])


# ---------------------------------------------------------------- goz baglanti noktalari (goz-yerel)
B_PITCH = np.array(PITCH_BALL, float)
STUD_PITCH_DIR = +EY        # pitch kancasindaki saplama +Y (yaw ekseni)

# YAW paralelkenari: iki goz gobek kolu + servo kolu, hepsi r=YAW_LEVER_R, notrde -Z'ye bakar, eksenler z=0'da.
YAW_SERVO_SHAFT = np.array([YAW_SERVO_X, 0.0, 0.0])
YAW_HORN_TOP_Y = YAW_LEVER_Y[1]                                  # servo kolu ust yuzu = goz kollari ust yuzu
YAW_SERVO_BASE_Y = YAW_HORN_TOP_Y - HORN_T - SG_SPLINE_TOP       # servo taban duzlemi (mil +Y)


def yaw_coupler_M(yaw):
    """Paralelkenar cubugu saf oteleme yapar: kol ucu yer degistirmesi."""
    tip0 = np.array([0, 0, -YAW_LEVER_R])
    tip = applyv(R(EY, yaw), tip0)
    return T(tip - tip0)


def yaw_servo_M(yaw):
    return about(YAW_SERVO_SHAFT, EY, yaw)


# Pitch rocker (sol dis servo, mil +X). Cubuk notrde -Z boyunca (alt kutuptaki topun yay tegeti).
_a = math.radians(PITCH_ROD_INCL)
PITCH_ROD_DIR_YZ = np.array([-math.sin(_a), -math.cos(_a)])   # (y,z) goz topundan rocker topuna
PITCH_ARM_DIR_YZ = np.array([math.cos(_a), -math.sin(_a)])    # kol notrde cubuga dik, yukari bakar
_rb = B_PITCH[1:] + PITCH_ROD_L * PITCH_ROD_DIR_YZ
ROCKER_AXIS_YZ = _rb - PITCH_ARM_R * PITCH_ARM_DIR_YZ
ROCKER_BALL_X = {k: EYES[k][0] + B_PITCH[0] for k in EYES}
ROCKER_STUD_DIR = {"L": +EX, "R": -EX}                              # rocker kollari gozlerin DIS tarafinda
ROCKER_ARM_FACE_X = {k: ROCKER_BALL_X[k] - BALL_H * ROCKER_STUD_DIR[k][0] for k in EYES}
PITCH_HORN_TOP_X = PITCH_ROCKER_FLANGE_X[0]
PITCH_SERVO_BASE_X = PITCH_HORN_TOP_X - HORN_T - SG_SPLINE_TOP


def rocker_ball0(k):
    return np.array([ROCKER_BALL_X[k], _rb[0], _rb[1]])


def rocker_M(theta):
    p = np.array([0.0, ROCKER_AXIS_YZ[0], ROCKER_AXIS_YZ[1]])
    return about(p, EX, theta)


# ---------------------------------------------------------------- kapak 4-cubuk tasarimi
LID_UP_CLOSED_EDGE = LID_MEET + LID_CLOSE_GAP / 2       # -4.25
LID_LO_CLOSED_EDGE = LID_MEET - LID_CLOSE_GAP / 2       # -5.75
LID_UP_DMAX = LID_UP_EDGE_OPEN_MAX - LID_UP_CLOSED_EDGE  # 44.25
LID_LO_DMIN = LID_LO_EDGE_OPEN_MAX - LID_LO_CLOSED_EDGE  # -39.25
LID_UP_DNOM = LID_UP_EDGE_NOMINAL - LID_UP_CLOSED_EDGE
LID_LO_DNOM = LID_LO_EDGE_NOMINAL - LID_LO_CLOSED_EDGE
LID_UP_DMID = LID_UP_DMAX / 2
LID_LO_DMID = LID_LO_DMIN / 2


def design_4bar(S_yz, a, h, prefer):
    """O=(0,0) kapak pimi, S servo mili (y,z). Ortada kol ve servo kolu cubuga dik, paralel (n).
    prefer: (y,z) kabaca istenen kol yonu. Donus: n, link_len."""
    g = np.asarray(S_yz, float); G = np.linalg.norm(g); gh = g / G
    gp = np.array([-gh[1], gh[0]])       # +90
    beta = math.acos((a - h) / G)
    best = None
    for sgn in (+1, -1):
        n = math.cos(beta) * gh + sgn * math.sin(beta) * gp
        if best is None or n @ prefer > best @ prefer:
            best = n
    L = math.sqrt(G * G - (a - h) ** 2)
    return best, L


LID = {}
for name, S, prefer, dmid, dmin, dmax in (
        ("U", LID_SERVO_UP_YZ, (-1.0, 0.0), LID_UP_DMID, 0.0, LID_UP_DMAX),
        ("Lo", LID_SERVO_LO_YZ, (-0.8, 0.6), LID_LO_DMID, LID_LO_DMIN, 0.0)):
    n, L = design_4bar(S, LID_ARM_R, HORN_R_LID, np.array(prefer))
    LID[name] = dict(S=np.array(S, float), n=n, L=L, dmid=dmid, dmin=dmin, dmax=dmax,
                     phi_mid=phi_of(*n))


def lid_arm_tip(name, delta):
    d = LID[name]
    return LID_ARM_R * dir_phi(d["phi_mid"] + delta - d["dmid"])


def _solve_circle(center, r, target, L, guess_phi):
    """center etrafinda r yaricapli nokta, target'a L uzakta; guess'e en yakin phi."""
    best, bphi = 1e9, guess_phi
    for dphi in np.arange(-90, 90.01, 0.5):
        phi = guess_phi + dphi
        e = abs(np.linalg.norm(center + r * dir_phi(phi) - target) - L)
        if e < best:
            best, bphi = e, phi
    lo, hi = bphi - 0.6, bphi + 0.6
    f = lambda ph: np.linalg.norm(center + r * dir_phi(ph) - target) - L
    # yerel minimum etrafinda: bisection eger isaret degisirse, yoksa altin oran
    for _ in range(60):
        m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        if abs(f(m1)) < abs(f(m2)):
            hi = m2
        else:
            lo = m1
    return (lo + hi) / 2, abs(f((lo + hi) / 2))


def lid_horn_phi(name, delta):
    d = LID[name]
    A = lid_arm_tip(name, delta)
    phi, err = _solve_circle(d["S"], HORN_R_LID, A, d["L"], d["phi_mid"])
    return phi, err


def lid_servo_deg(name, side, delta):
    """side 'L' (mil -X) veya 'R' (mil +X). phi artisi = -X etrafinda sag-el donus."""
    phi, err = lid_horn_phi(name, delta)
    rel = phi - LID[name]["phi_mid"]
    return 90.0 + (rel if side == "L" else -rel), err


def lid_delta_from_edge(name, edge):
    return edge - (LID_UP_CLOSED_EDGE if name == "U" else LID_LO_CLOSED_EDGE)


# ---------------------------------------------------------------- goz yaw / pitch servo cozumleri
def eye_ball_world(k, local, yaw, pitch):
    return apply(eye_M(EYES[k], yaw, pitch), EYES[k] + local)


def solve_yaw_servo(yaw, pitch):
    """Paralelkenar: servo acisi = goz yaw'i (sag-el +Y). pitch'ten bagimsiz."""
    return float(yaw), 0.0


def rocker_ball(k, theta):
    return apply(rocker_M(theta), rocker_ball0(k))


def solve_pitch_servo(yaw, pitch, k="L"):
    P = eye_ball_world(k, B_PITCH, yaw, pitch)
    f = lambda th: np.linalg.norm(rocker_ball(k, th) - P) - PITCH_ROD_L
    ths = np.arange(-89, 89.01, 0.25)
    vals = np.array([f(t) for t in ths])
    i = int(np.argmin(np.abs(vals) + 0.002 * np.abs(ths)))
    lo, hi = ths[max(i - 1, 0)], ths[min(i + 1, len(ths) - 1)]
    th = _refine(f, lo, hi)
    return th, abs(f(th))


def _refine(f, lo, hi):
    if f(lo) * f(hi) < 0:
        for _ in range(80):
            m = (lo + hi) / 2
            if f(lo) * f(m) <= 0:
                hi = m
            else:
                lo = m
        return (lo + hi) / 2
    for _ in range(80):
        m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        if abs(f(m1)) < abs(f(m2)):
            hi = m2
        else:
            lo = m1
    return (lo + hi) / 2


def eye_pose_from_pitch_servo(theta, yaw, k):
    """Verilen rocker acisinda k gozunun pitch'i (capraz etki hesabi icin)."""
    f = lambda p: np.linalg.norm(rocker_ball(k, theta) - eye_ball_world(k, B_PITCH, yaw, p)) - PITCH_ROD_L
    ps = np.arange(-40, 40.01, 0.25)
    vals = np.array([f(p) for p in ps])
    i = int(np.argmin(np.abs(vals)))
    return _refine(f, ps[max(i - 1, 0)], ps[min(i + 1, len(ps) - 1)])


def transmission_angle(rod_a, rod_b, arm_center, arm_axis):
    """Servo kolu ile cubuk arasindaki aci (90 = ideal, olu nokta = 0/180)."""
    arm = rod_b - arm_center
    arm = arm - (arm @ arm_axis) * arm_axis
    rod = rod_a - rod_b
    c = abs(arm @ rod) / (np.linalg.norm(arm) * np.linalg.norm(rod))
    return math.degrees(math.acos(min(1.0, c)))


if __name__ == "__main__":
    print("yaw servo base y", YAW_SERVO_BASE_Y, "rocker axis yz", ROCKER_AXIS_YZ)
    for k, d in LID.items():
        print("lid", k, {kk: (np.round(v, 3) if isinstance(v, np.ndarray) else round(v, 3)) for kk, v in d.items()})
    for y in (-30, 0, 30):
        for p in (-25, 0, 25):
            tp, ep = solve_pitch_servo(y, p, "L")
            tpr, _ = solve_pitch_servo(y, p, "R")
            print(f"yaw {y:4} pitch {p:4}: rocker L {tp:7.2f} R {tpr:7.2f} err {ep:.1e}")
    for nm, side in (("U", "L"), ("Lo", "L")):
        d = LID[nm]
        for dl in np.linspace(d["dmin"], d["dmax"], 5):
            print(nm, round(dl, 2), lid_servo_deg(nm, side, dl))
