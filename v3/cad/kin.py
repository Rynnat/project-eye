"""Project Eye v3 - kinematik (saf numpy). Tum baglantilar paralelkenar ya da direkt -> kapali formul.

Grup donusumleri (notr = yaw 0, pitch 0, kapak kapali):
  frame     F = Rx(-pitch)                       (X ekseni goz merkezlerinden gecer)
  eye_k     F * T(E) * Ry(yaw) * T(-E)           (yaw cerceve-yerel dikey eksen etrafinda)
  coupler   F * T(d(yaw)),  d = Ry(yaw)*(0,0,-R) - (0,0,-R)   (paralelkenar lamasi oteleme yapar)
  yawcrank  F * T(P) * Ry(yaw) * T(-P)           P = yaw servo mili
  lids      Rx(-delta)                           delta = kapak acilma acisi (0 kapali .. LID_OPEN)

Servo aci kurali (v2 ile ayni): servo_deg = 90 + (kolun, mil ucundan DISARI bakan eksen etrafinda sag-el donusu).
  EYE_YAW  : mil -Y'ye bakar (servo cercevede ters asili)  -> 90 - yaw
  EYE_PITCH: mil -X'e bakar (sag tarafta)                  -> 90 + pitch
  LIDS     : mil +X'e bakar (ortada)                       -> 90 - (delta - LID_SERVO_MID)
"""
import math
import numpy as np
from params import *

D2R = math.pi / 180
EX, EY, EZ = np.eye(3)
E_L = np.array([-EYE_X, 0.0, 0.0])
E_R = np.array([+EYE_X, 0.0, 0.0])
EYES = {"L": E_L, "R": E_R}
SIDE_SIGN = {"L": -1.0, "R": +1.0}          # x_rel (disari) -> dunya x isareti
YAW_SERVO_P = np.array([0.0, LEVER_Y[1], YAW_SERVO_Z])


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


def lid_delta(lid):
    """lid: 0 = kapali .. 1 = tam acik, ya da 'closed' / 'open' / 'half'."""
    if isinstance(lid, str):
        lid = {"closed": 0.0, "open": 1.0, "half": 0.5}[lid]
    return float(np.clip(lid, 0.0, 1.0)) * LID_OPEN


def frame_M(pitch):
    return R(EX, -pitch)


def pose(yaw=0.0, pitch=0.0, lid="closed"):
    F = frame_M(pitch)
    Ms = {"static": np.eye(4), "frame": F}
    for k, E in EYES.items():
        Ms[f"eye_{k}"] = F @ about(E, EY, yaw)
    tip0 = np.array([0.0, 0.0, -LEVER_R])
    d = (R(EY, yaw)[:3, :3] @ tip0) - tip0
    Ms["coupler"] = F @ T(d)
    Ms["yawcrank"] = F @ about(YAW_SERVO_P, EY, yaw)
    Ms["lids"] = R(EX, -lid_delta(lid))
    return Ms


def servo_angles(yaw=0.0, pitch=0.0, lid="closed"):
    dl = lid_delta(lid)
    return {"EYE_YAW": 90.0 - yaw, "EYE_PITCH": 90.0 + pitch, "LIDS": 90.0 - (dl - LID_SERVO_MID)}


if __name__ == "__main__":
    for s in ((0, 0, "closed"), (25, 20, "open"), (-25, -20, 0.5)):
        print(s, servo_angles(*s))
