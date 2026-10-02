"""out/kinematics.json: servo acisi <-> goz/kapak acisi tablolari (kalibrasyon baslangic degerleri).

  PYTHONUTF8=1 python kinematics_export.py

Tum degerler kin.py'deki geometrik model ile hesaplanir (baski/montaj toleransi, servo kol splinesinin dis atlamasi ve
servo dogrusalsizligi DAHIL DEGIL). Gercek degerler montajdan sonra calibrate.py ile bulunur (SPEC §6).
"""
import json
import math
import os

import numpy as np

import kin
from kin import EX, EY, LID
from params import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def r1(x):
    return float(round(x, 2))


def pitch_transmission(yaw, pitch):
    th, _ = kin.solve_pitch_servo(yaw, pitch, "L")
    P = kin.eye_ball_world("L", kin.B_PITCH, yaw, pitch)
    Q = kin.rocker_ball("L", th)
    ax = np.array([Q[0], kin.ROCKER_AXIS_YZ[0], kin.ROCKER_AXIS_YZ[1]])
    return kin.transmission_angle(P, Q, ax, EX)


def eye_stud_tilt(yaw, pitch):
    """Goz ucu rotilinde saplama ekseni ile cubuga dik duzlem arasi aci (yuva agzi izni ~31 deg)."""
    th, _ = kin.solve_pitch_servo(yaw, pitch, "L")
    M = kin.eye_M(kin.E_L, yaw, pitch)
    P = kin.apply(M, kin.E_L + kin.B_PITCH)
    Q = kin.rocker_ball("L", th)
    rod = (Q - P) / np.linalg.norm(Q - P)
    return math.degrees(math.asin(abs(kin.applyv(M, EY) @ rod)))


def lid_transmission(nm, delta):
    d = LID[nm]
    A = kin.lid_arm_tip(nm, delta)
    phi, _ = kin.lid_horn_phi(nm, delta)
    B = d["S"] + HORN_R_LID * kin.dir_phi(phi)
    link = B - A
    def ang(u, v):
        return math.degrees(math.acos(min(1, abs(u @ v) / (np.linalg.norm(u) * np.linalg.norm(v)))))
    return ang(A, link), ang(B - d["S"], link)     # kol-link, servo kolu-link (90 ideal)


def build_json():
    J = {
        "_about": "Project Eye v2 mekanik kinematik tablolari (cad/kinematics_export.py). Model degerleri; montaj sonrasi kalibre edin.",
        "_servo_convention": ("servo_deg = 90 + (servo kolunun, mil ucundan DISARI bakan eksen etrafinda sag-el donusu). "
                              "Yani kol tarafindan bakinca saat yonunun tersi = aci artar. SG90'da darbe genisligi artinca "
                              "hangi yone dondugu ureticiye gore degisebilir: ters ise settings.json 'invert' kullanin. "
                              "90 = notr (kol cubuga dik; SPEC §4 olu nokta kurali)."),
        "_eye_convention": "SPEC §1: yaw + = robot sagina (+X), pitch + = yukari; R_goz = Ry(yaw) * Rx(-pitch).",
        "_lid_convention": ("kenar acisi phi = atan2(y, z) goz merkezinden (+Z'den +Y'ye, derece). Ust kapak alt kenari; "
                            "alt kapak ust kenari. openness: 0 = tam kapali (kenarlar LID_MEET=-5'te bulusur), "
                            "1 = SPEC §3 'acik' (ust kenar +22: irisi ~1.5 mm orter; alt kenar -28). >1 = bakis takibi payi."),
        "channels": {}, "limits_checked": {}, "cross_coupling": {}, "transmission": {}, "settings_suggestion": {},
    }
    ch = J["channels"]
    # EYE_YAW
    ch["EYE_YAW"] = {"channel": 0, "mechanism": "paralelkenar: iki goz gobek kolu + servo kolu (r=10), tek cubuk",
                     "input": "eye_yaw_deg", "table": [{"eye_yaw_deg": y, "servo_deg": r1(90 + kin.solve_yaw_servo(y, 0)[0])}
                                                       for y in (-30, -20, -10, 0, 10, 20, 30)],
                     "note": "birebir: servo_deg = 90 + yaw; pitch'ten bagimsiz (gobek sadece yaw yapar)"}
    # EYE_PITCH
    ps = (-25, -18.75, -12.5, -6.25, 0, 6.25, 12.5, 18.75, 25)
    ch["EYE_PITCH"] = {"channel": 1, "mechanism": "sallanan mil (rocker, sol servo eksenel) + iki basili cubuk -> alt kutup toplari",
                       "input": "eye_pitch_deg (yaw=0)",
                       "table": [{"eye_pitch_deg": p, "servo_deg": r1(90 + kin.solve_pitch_servo(0, p, "L")[0])} for p in ps],
                       "note": "dogrusal degil: servo acisi pitch'e gore egri; iki goz ayni geometri -> sag/sol pitch esit"}
    grid_y = (-30, -15, 0, 15, 30)
    J["cross_coupling"]["EYE_PITCH_servo_deg_grid"] = {
        "yaw_deg": list(grid_y), "pitch_deg": list(ps),
        "servo_deg": [[r1(90 + kin.solve_pitch_servo(y, p, "L")[0]) for y in grid_y] for p in ps],
        "note": ("pitch servosu icin gereken aci yaw'a da biraz bagli (top alt kutupta; pitch=0'da yaw etkisi SIFIR, "
                 "koselerde birkac derece). Telafi icin bu 2B tablo kullanilabilir.")}
    unc = []
    for y in (-30, 30):
        for p in (-25, 25):
            th = kin.solve_pitch_servo(0, p, "L")[0]            # yaw=0 kalibrasyonu ile verilen servo acisi
            p_real = kin.eye_pose_from_pitch_servo(th, y, "L")
            unc.append({"yaw": y, "pitch_cmd": p, "pitch_real": r1(p_real), "error_deg": r1(p_real - p)})
    J["cross_coupling"]["EYE_PITCH_uncompensated_error"] = unc
    J["cross_coupling"]["EYE_YAW"] = "yok (paralelkenar gobege bagli; pitch yaw'i etkilemez)"
    # kapaklar
    for chname, code, side, chno in (("LID_UL", "U", "L", 2), ("LID_LL", "Lo", "L", 3), ("LID_UR", "U", "R", 4), ("LID_LR", "Lo", "R", 5)):
        d = LID[code]
        closed = kin.LID_UP_CLOSED_EDGE if code == "U" else kin.LID_LO_CLOSED_EDGE
        nomedge = LID_UP_EDGE_NOMINAL if code == "U" else LID_LO_EDGE_NOMINAL
        maxedge = LID_UP_EDGE_OPEN_MAX if code == "U" else LID_LO_EDGE_OPEN_MAX
        edges = np.linspace(closed, maxedge, 7)
        edges = sorted(set([float(e) for e in edges] + [nomedge]), key=lambda e: abs(e - closed))
        rows = []
        for e in edges:
            delta = e - closed
            sd, err = kin.lid_servo_deg(code, side, delta)
            rows.append({"edge_deg": r1(e), "openness": r1((e - closed) / (nomedge - closed)), "servo_deg": r1(sd)})
        ch[chname] = {"channel": chno, "mechanism": f"4-cubuk: kapak kolu r={LID_ARM_R}, servo kolu r={HORN_R_LID}, link {r1(d['L'])} mm",
                      "input": "lid edge angle", "table": rows}
    # sinirlar
    J["limits_checked"] = {
        "EYE_YAW": [r1(90 - YAW_RANGE), r1(90 + YAW_RANGE)],
        "EYE_PITCH": [r1(90 + min(kin.solve_pitch_servo(y, -PITCH_RANGE, "L")[0] for y in grid_y)),
                      r1(90 + max(kin.solve_pitch_servo(y, PITCH_RANGE, "L")[0] for y in grid_y))],
        **{c: sorted([r1(ch[c]["table"][0]["servo_deg"]), r1(ch[c]["table"][-1]["servo_deg"])])
           for c in ("LID_UL", "LID_LL", "LID_UR", "LID_LR")},
        "_note": ("check.py bu araliklarin koselerinde carpisma taradi. Firmware sabit limitleri (SPEC §5) bu araliklarin "
                  "disina cikmamali; araligin disi DOGRULANMADI."),
    }
    lo_safe = max(kin.solve_pitch_servo(y, -PITCH_RANGE, "L")[0] for y in grid_y)
    hi_safe = min(kin.solve_pitch_servo(y, PITCH_RANGE, "L")[0] for y in grid_y)
    J["limits_checked"]["EYE_PITCH_firmware_safe"] = [r1(90 + lo_safe), r1(90 + hi_safe)]
    J["limits_checked"]["EYE_PITCH_firmware_safe_note"] = (
        "Capraz etki yuzunden SABIT servo siniri yaw'dan bagimsiz guvenli olmali: bu aralikta her yaw icin goz pitch'i "
        f"+-25 icinde kalir. yaw=0'da ulasilan goz pitch'i {r1(kin.eye_pose_from_pitch_servo(lo_safe, 0, 'L'))} .. "
        f"{r1(kin.eye_pose_from_pitch_servo(hi_safe, 0, 'L'))}. Tam +-25 icin yazilim cross_coupling tablosuyla yaw'a bagli "
        "sinir uygulamali. Yaw=0 sinirlari (EYE_PITCH) yaw +-30'da goz pitch'ini -31.8/+27'ye goturur: o bolge DOGRULANMADI "
        "ve yarik/direk carpismasi var.")
    # iletim acilari
    tr = {"_note": "kol ile cubuk arasi aci; 90 ideal, 0/180 olu nokta. >= ~40 deg rahat kabul edilir."}
    tr["EYE_YAW"] = {"neutral": 90.0, "at_+-30": 60.0}
    pt = [pitch_transmission(y, p) for y in grid_y for p in (-25, 0, 25)]
    tr["EYE_PITCH"] = {"neutral": r1(pitch_transmission(0, 0)), "min_over_range": r1(min(pt))}
    tr["EYE_PITCH_ball_joint_tilt_deg"] = {"max_eye_end": r1(max(eye_stud_tilt(y, p) for y in grid_y for p in ps)),
                                           "allowed_by_socket_mouth": r1(SOCKET_MOUTH_HALF - math.degrees(math.asin(STUD_NECK_D / BALL_D)))}
    for code in ("U", "Lo"):
        d = LID[code]
        vals = [lid_transmission(code, dl) for dl in np.linspace(d["dmin"], d["dmax"], 9)]
        tr["LID_" + ("UPPER" if code == "U" else "LOWER")] = {
            "neutral(mid)": [r1(x) for x in lid_transmission(code, d["dmid"])],
            "min_arm_link": r1(min(v[0] for v in vals)), "min_horn_link": r1(min(v[1] for v in vals))}
    J["transmission"] = tr
    # settings.json onerisi (SPEC §6 semasi)
    pmin, pmax = J["limits_checked"]["EYE_PITCH"]
    up = 90 + kin.solve_pitch_servo(0, 25, "L")[0]; dn = 90 + kin.solve_pitch_servo(0, -25, "L")[0]
    fs = J["limits_checked"]["EYE_PITCH_firmware_safe"]
    S = {"EYE_YAW": {"center": 90, "min": 60, "max": 120, "invert": False, "deg_per_unit": 30},
         "EYE_PITCH": {"center": 90, "min": math.ceil(fs[0]), "max": math.floor(fs[1]), "invert": False,
                       "deg_per_unit": r1((up - dn) / 2),
                       "_note": (f"SPEC'in dogrusal modeli yaklasik: +25 -> {r1(up)}, -25 -> {r1(dn)} (asimetrik; tablo daha dogru). "
                                 "min/max = firmware_safe (yaw'dan bagimsiz guvenli).")}}
    for c in ("LID_UL", "LID_LL", "LID_UR", "LID_LR"):
        rows = ch[c]["table"]
        clo = rows[0]["servo_deg"]
        opn = [r for r in rows if abs(r["openness"] - 1.0) < 1e-6][0]["servo_deg"]
        S[c] = {"closed": clo, "open": opn, "invert": False}
    J["settings_suggestion"] = {"channels": S, "_note": "Model tabanli baslangic. invert yonu donanimda dogrulanmali."}
    return J


def write():
    J = build_json()
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "kinematics.json"), "w", encoding="utf-8") as f:
        json.dump(J, f, indent=1, ensure_ascii=False)
    return J


if __name__ == "__main__":
    J = write()
    print(json.dumps({k: J[k] for k in ("limits_checked", "transmission", "settings_suggestion")}, indent=1, ensure_ascii=False))
    print(json.dumps(J["cross_coupling"]["EYE_PITCH_uncompensated_error"]))
