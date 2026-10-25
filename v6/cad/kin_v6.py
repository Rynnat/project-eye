"""Project Eye v6 - kinematik: v5 kin.pose AYNEN + masa (egim) donusumu + kama profili (saf numpy).

Gruplar (v5 ile ayni adlar): static, frame, eye_L/R, coupler, yawcrank, lids, lids_lo, lidcrank, link_up, link_lo.
v6 yeni parcalari mevcut gruplara girer: sol_kol_adaptoru + sol kol = 'frame'; kama, sol braket, sol servo = 'static'.

Masa cercevesi ("masa", z YUKARI, masa yuzu z = 0):  M_masa = masa_M(alfa) ; dunya = M_masa @ M_grup @ p
  1) R(X, -alfa): mekanizma burnu yukari (+Z ileri -> (0, sin a, cos a))  [v5 pitch isaretiyle ayni]
  2) Y-yukari -> Z-yukari: (x, y, z) -> (x, -z, y)   (masa Y'si = mekanizmanin GERISI)
  3) masa yuzu (kaymaz ayaklarin alti) z = 0
Kama profili (mekanizma YZ duzleminde, (y, z)): bkz. kama_profil().
"""
import math

import numpy as np

import kin as K5                     # v5/cad/kin.py (sys.path'te v5/cad olmali)
from params_v6 import *

EX = np.array([1.0, 0.0, 0.0])
BASE_ALT_Y = -60.0                   # [v5] BASE_Y[0]
BASE_ARKA_Z = -62.0                  # [v5] BASE_Z[0]


def pose(yaw=0.0, pitch=0.0, lid=0.0):
    return K5.pose(yaw, pitch, lid)


def servo_angles(yaw=0.0, pitch=0.0, lid=0.0):
    a = K5.servo_angles(yaw, pitch, lid)
    a["EYE_PITCH_L"] = 180.0 - a["EYE_PITCH"]          # SECENEK B: sol servo ayna -> ters aci (D9)
    return a


def kama_profil(alfa=TILT_DEG):
    """Kama yan profili (mekanizma (y, z)) ve masa duzlemi. Doner dict:
    profil  : dis cevre [A on-ust, B on-alt, R0 arka-alt, dudak...]  (taban alti y = -60 kamanin ust yuzu)
    govde   : cep icin ana bolge (dudaksiz)
    n       : masa 'yukari' yonu (y, z)   (mekanizmada)
    d_alt   : kama alt yuzu n.p = d_alt ; d_masa: kaymaz ayak alti (masa) n.p = d_masa
    ayak    : [(y, z)] ayak merkezleri (alt yuzde; arka, on)"""
    a = math.radians(alfa)
    n = np.array([math.cos(a), math.sin(a)])
    z_arka = BASE_ARKA_Z - KAMA_DUDAK_PAY - KAMA_DUDAK_T
    R0 = np.array([BASE_ALT_Y - KAMA_T0, z_arka])
    d_alt = float(n @ R0)
    A = np.array([BASE_ALT_Y, KAMA_ON_Z])
    t = float(n @ A) - d_alt
    B = A - t * n                                        # on yuz dunyada DIKEY (masa normali boyunca)
    dud_ic = BASE_ARKA_Z - KAMA_DUDAK_PAY
    profil = [A, B, R0, np.array([KAMA_DUDAK_UST, z_arka]), np.array([KAMA_DUDAK_UST, dud_ic]),
              np.array([BASE_ALT_Y, dud_ic])]
    govde = [A, B, R0, np.array([BASE_ALT_Y, z_arka])]
    u = (B - R0) / np.linalg.norm(B - R0)
    ayak = [R0 + u * AYAK_KENAR, B - u * AYAK_KENAR]
    return dict(profil=[tuple(map(float, p)) for p in profil], govde=[tuple(map(float, p)) for p in govde],
                n=n, d_alt=d_alt, d_masa=d_alt - (AYAK_T - AYAK_IZ), ayak=[tuple(map(float, p)) for p in ayak],
                A=A, B=B, R0=R0, on_yukseklik=float(t), z_arka=z_arka)


def masa_M(alfa=TILT_DEG):
    """Mekanizma -> masa (z yukari, masa yuzu z = 0) 4x4."""
    kp = kama_profil(alfa)
    Rt = K5.R(EX, -alfa)                                 # burun yukari
    Zup = np.eye(4); Zup[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    M = Zup @ Rt
    M[2, 3] -= kp["d_masa"]                              # dunya y = n.p ; masa z = n.p - d_masa
    return M


def masa_yukari_mek(alfa=TILT_DEG):
    """Masa 'yukari' yonu mekanizma koordinatinda (3B). Yercekimi = -bu."""
    a = math.radians(alfa)
    return np.array([0.0, math.cos(a), math.sin(a)])


def pitch_ekseni_yukseklik(alfa=TILT_DEG):
    return float(masa_M(alfa)[2, 3])                     # orijin (pitch ekseni ortasi) masa ustunde


if __name__ == "__main__":
    for a in (20, 30, 40):
        kp = kama_profil(a)
        print(f"alfa {a}: on yukseklik (kama, mek. y) {kp['on_yukseklik']:.1f}, B {np.round(kp['B'], 2)}, "
              f"pitch ekseni masadan {pitch_ekseni_yukseklik(a):.1f} mm")
    print(servo_angles(0, 20, 0))
