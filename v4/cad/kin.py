"""Project Eye v3 - kinematik (saf numpy).  Revizyon 2: kapaklar cercevede, ust + alt, tek LIDS servosu + iki lama.

Grup donusumleri (notr = yaw 0, pitch 0, kapak kapali):
  frame     F = Rx(-pitch)                       (X ekseni goz merkezlerinden gecer)
  eye_k     F * T(E) * Ry(yaw) * T(-E)
  coupler   F * T(d(yaw)),  d = Ry(yaw)*(0,0,-R) - (0,0,-R)
  yawcrank  F * T(P) * Ry(yaw) * T(-P)           P = yaw servo mili
  lids      F * Rx(-du)          ust kapaklar (phi artar = acilir), du = lid * UP_OPEN
  lids_lo   F * Rx(+dl)          alt kapaklar (phi azalir = acilir), dl = dort-cubuk cozumu
  lidcrank  F * about(S, X, -b)  LIDS krank (phi b kadar artar; b < 0)
  link_up / link_lo  F * (duzlemsel rijit hareket: iki pim noktasi)

Dort-cubuk (YZ duzleminde, cerceve koordinati; (y, z); phi = atan2(y, z)):
  krank ucu A = S + h*(sin, cos)(phiA0 + b);  ust pim U = rU*(sin, cos)(phiU0 + du);  alt pim L = rL*(sin, cos)(phiL0 - dl)
  |A - U| = lU,  |A - L| = lL  (kapali pozdan sabit).  du verilir -> b, sonra dl cozulur.

Servo aci kurali (v2 ile ayni): servo_deg = 90 + (kolun, mil ucundan DISARI bakan eksen etrafinda sag-el donusu).
  EYE_YAW  : mil -Y  -> 90 - yaw
  EYE_PITCH: mil -X (sag) -> 90 + pitch
  LIDS     : mil -X (sag gozun arkasi); -X etrafinda sag-el donus = phi artisi = b -> 90 + (b - b_orta)
"""
import math
from functools import lru_cache

import numpy as np
from params import *

D2R = math.pi / 180
EX, EY, EZ = np.eye(3)
E_L = np.array([-EYE_X, 0.0, 0.0])
E_R = np.array([+EYE_X, 0.0, 0.0])
EYES = {"L": E_L, "R": E_R}
SIDE_SIGN = {"L": -1.0, "R": +1.0}          # x_rel (disari) -> dunya x isareti
YAW_SERVO_P = np.array([0.0, LEVER_Y[1], YAW_SERVO_Z])
LS_H = LEVER_R                              # LIDS krank = yaw_crank ile ayni STL -> r = LEVER_R


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
    return M[:3, :3] @ np.asarray(p, float) + M[:3, 3]


# ------------------------------------------------------------------ LIDS dort-cubuk (2B, (y, z))
def P2(r, phi):
    return np.array([r * math.sin(phi * D2R), r * math.cos(phi * D2R)])


def ang2(v):
    return math.degrees(math.atan2(v[0], v[1]))


def _circ(c, r, p, L, prev):
    d = float(np.linalg.norm(p - c))
    if d > r + L or d < abs(r - L) or d == 0:
        raise ValueError("dort-cubuk kilitlendi")
    a = (r * r - L * L + d * d) / (2 * d); h = math.sqrt(max(r * r - a * a, 0.0))
    m = c + a * (p - c) / d; perp = np.array([-(p - c)[1], (p - c)[0]]) / d
    return min((m + h * perp, m - h * perp), key=lambda X: np.linalg.norm(X - prev))


S2 = np.array(LS_S, float)
A0 = S2 + P2(LS_H, LS_A_PHI0)
U0 = P2(*LS_U)
L0 = P2(*LS_L)
LEN_U = float(np.linalg.norm(A0 - U0))
LEN_L = float(np.linalg.norm(A0 - L0))


@lru_cache(maxsize=4096)
def _lid_solve_cached(du):
    """du (ust kapak acilma) -> (b krank acisi, dl alt kapak acilma, A, U, L). Kapali pozdan adim adim."""
    A = A0.copy(); Lp = L0.copy()
    n = max(2, int(abs(du) / 0.5) + 2)
    for t in np.linspace(0.0, du, n):
        U = P2(LS_U[0], LS_U[1] + t)
        A = _circ(S2, LS_H, U, LEN_U, A)
        Lp = _circ(np.zeros(2), LS_L[0], A, LEN_L, Lp)
    b = (ang2(A - S2) - LS_A_PHI0 + 540) % 360 - 180
    dl = (LS_L[1] - ang2(Lp) + 540) % 360 - 180
    return b, dl, tuple(A), tuple(U), tuple(Lp)


def lid_solve(du):
    return _lid_solve_cached(round(float(du), 6))


def lid_u(lid):
    """lid: 0 = kapali .. 1 = tam acik, ya da 'closed' / 'open' / 'half'."""
    if isinstance(lid, str):
        lid = {"closed": 0.0, "open": 1.0, "half": 0.5}[lid]
    return float(np.clip(lid, 0.0, 1.0))


def lid_deltas(lid):
    du = lid_u(lid) * UP_OPEN
    b, dl, *_ = lid_solve(du)
    return du, dl, b


B_OPEN = lid_solve(UP_OPEN)[0]
B_MID = B_OPEN / 2.0
LO_OPEN_ACT = lid_solve(UP_OPEN)[1]        # alt kapagin gercek acilma acisi (dort-cubuktan)


def frame_M(pitch):
    return R(EX, -pitch)


def _planar(Pa0, Pb0, Pa, Pb, x=0.0):
    """YZ duzleminde iki noktayi (Pa0, Pb0) -> (Pa, Pb) goturen rijit hareket (X ekseni etrafinda donme + oteleme)."""
    d = ang2(np.asarray(Pb) - np.asarray(Pa)) - ang2(np.asarray(Pb0) - np.asarray(Pa0))
    M = R(EX, -d)                                   # phi d kadar artar
    p0 = np.array([x, Pa0[0], Pa0[1]]); p1 = np.array([x, Pa[0], Pa[1]])
    M[:3, 3] = p1 - M[:3, :3] @ p0
    return M


def pose(yaw=0.0, pitch=0.0, lid="closed"):
    F = frame_M(pitch)
    Ms = {"static": np.eye(4), "frame": F}
    for k, E in EYES.items():
        Ms[f"eye_{k}"] = F @ about(E, EY, yaw)
    tip0 = np.array([0.0, 0.0, -LEVER_R])
    d = (R(EY, yaw)[:3, :3] @ tip0) - tip0
    Ms["coupler"] = F @ T(d)
    Ms["yawcrank"] = F @ about(YAW_SERVO_P, EY, yaw)
    du = lid_u(lid) * UP_OPEN
    b, dl, A, U, L = lid_solve(du)
    Ms["lids"] = F @ R(EX, -du)
    Ms["lids_lo"] = F @ R(EX, +dl)
    Ms["lidcrank"] = F @ about((0.0, S2[0], S2[1]), EX, -b)
    Ms["link_up"] = F @ _planar(A0, U0, A, U)
    Ms["link_lo"] = F @ _planar(A0, L0, A, L)
    return Ms


def servo_angles(yaw=0.0, pitch=0.0, lid="closed"):
    b = lid_deltas(lid)[2]
    return {"EYE_YAW": 90.0 - yaw, "EYE_PITCH": 90.0 + pitch, "LIDS": 90.0 + (b - B_MID)}


def transmission(lid):
    """(lama-ust pim teget sapmasi, lama-alt pim, lama-krank) derece; 0 ideal (90 deg iletim)."""
    du = lid_u(lid) * UP_OPEN
    b, dl, A, U, L = lid_solve(du)
    A, U, L = map(np.asarray, (A, U, L))

    def dev(link, rad):
        t = np.array([rad[1], -rad[0]]); t /= np.linalg.norm(t)
        return math.degrees(math.acos(min(1.0, abs(float(np.dot(link / np.linalg.norm(link), t))))))
    return dev(A - U, U), dev(A - L, L), dev(A - U, A - S2), dev(A - L, A - S2)


if __name__ == "__main__":
    print("lU %.2f lL %.2f  b_open %.2f  lo_open %.2f" % (LEN_U, LEN_L, B_OPEN, LO_OPEN_ACT))
    for l in (0, 0.25, 0.5, 0.75, 1.0):
        du, dl, b = lid_deltas(l)
        print(l, round(du, 2), round(dl, 2), round(b, 2), servo_angles(0, 0, l)["LIDS"], [round(x, 1) for x in transmission(l)])
