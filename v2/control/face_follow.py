"""
Project Eye v2 - webcam yuz takibi -> gozler.

Webcam -> MediaPipe BlazeFace (Lunar'in modeli) -> en buyuk yuz (PRIMARY)
-> ``EyeController.look(u, v)``. Davranis Lunar ``ultimatum.py`` EyeRig'inden:

* yuz kilitlenince irkilme (kapak 1.0, 0.35 sn)
* takipte kapak 0.85; yuz hizli hareket ederse "nisan alir" gibi 0.58'e kisilir
* yuz kaybolunca bir kirpma; ``idle_after_s`` sonra uykulu kapak (0.38) ve
  yavas etrafi kolacan etme (EyeController.idle_wander)
* rastgele kirpma ve kapaklarin pitch takibi EyeController'da

Donanim yoksa EyeController sim'e duser: pencerede kamera onizlemesi,
iki gozun sematik hali ve telemetri gorunur.

Calistirma (mediapipe + opencv Lunar venv'inde var):
    C:\\Users\\LENOVO\\lunar-tracker\\.venv\\Scripts\\python.exe face_follow.py
    ... face_follow.py --sim          # donanimi arama
    ... face_follow.py --port COM5 --camera 1

Tuslar: q / ESC cikis, b kirp, l / r sol / sag wink, m ayna (u yonunu cevir),
        h kontrolu firmware idle'ina birak / geri al, d servolari birak (detach).

Kamera koordinati: kamera robotun uzerinde/yaninda ILERI (izleyiciye) bakiyor
varsayilir. O zaman ham karede sagdaki yuz robotun SAGINDADIR (+u). Kamera
farkli takiliysa ``m`` ya da ``--mirror``.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from eye_control import EyeController, clamp, lid_follow  # noqa: E402

WINDOW_NAME = "Project Eye v2 - Face Follow"
DEFAULT_MODEL = r"C:\Users\LENOVO\lunar-tracker\assets\blaze_face_short_range.tflite"
LOCK_PATH = os.path.join(HERE, ".face_follow.lock")

# --- davranis (ultimatum.py EyeRig ile ayni aile) ---------------------------
AIM_TAU_S = 0.08            # tespit gurultusunu yumusat (servo hissi)
LID_TAU_S = 0.07
FOCUS_TAU_S = 0.20
LID_OPEN_IDLE = 0.38
LID_OPEN_TRACK = 0.85
LID_OPEN_FOCUS = 0.58
LID_OPEN_STARTLE = 1.0
STARTLE_S = 0.35
FOCUS_SPEED_MIN = 0.4       # birim/sn: bundan yavas hareket "odak" saymaz
FOCUS_SPEED_RAMP = 1.2
TARGET_DEBOUNCE_FRAMES = 5
GAIN = 1.0                  # kare kenari -> u = +-GAIN


def _ema(current: float, target: float, dt: float, tau: float) -> float:
    """Kare hizindan bagimsiz ustel yumusatma."""
    if tau <= 0:
        return target
    return current + (target - current) * (1.0 - math.exp(-dt / tau))


class FollowBrain:
    """Tespit -> goz davranisi. cv2/mediapipe gerektirmez (test edilebilir).

    ``step(now, dt, target)``: target = (u, v) (robot koordinatinda, -1..1)
    ya da None (yuz yok).
    """

    def __init__(self, eye: EyeController):
        self.eye = eye
        self.aim = None             # yumusatilmis (u, v)
        self.prev_aim = None
        self.focus = 0.0
        self.lid = LID_OPEN_TRACK
        self.startle_until = 0.0
        self.had_target = False
        self.streak = 0
        self.status = "SEARCHING"

    def step(self, now: float, dt: float, target) -> None:
        # --- kilit / kayip (debounce'lu) ----------------------------------
        has = target is not None
        if has == self.had_target:
            self.streak = 0
        else:
            self.streak += 1
            if self.streak >= TARGET_DEBOUNCE_FRAMES:
                if has:
                    self.startle_until = now + STARTLE_S
                else:
                    self.eye.blink()
                self.had_target = has
                self.streak = 0

        # --- nisan + odak --------------------------------------------------
        if has:
            u, v = target
            if self.aim is None:
                self.aim = (u, v)
            else:
                self.aim = (_ema(self.aim[0], u, dt, AIM_TAU_S), _ema(self.aim[1], v, dt, AIM_TAU_S))
            speed = 0.0
            if self.prev_aim is not None and dt > 0:
                speed = math.hypot(self.aim[0] - self.prev_aim[0], self.aim[1] - self.prev_aim[1]) / dt
            self.prev_aim = self.aim
            raw_focus = clamp((speed - FOCUS_SPEED_MIN) / FOCUS_SPEED_RAMP, 0.0, 1.0)
            self.focus = _ema(self.focus, raw_focus, dt, FOCUS_TAU_S)
            self.eye.look(*self.aim)
        else:
            self.prev_aim = None
            self.focus = _ema(self.focus, 0.0, dt, FOCUS_TAU_S)

        # --- kapak ---------------------------------------------------------
        idle = (not has) and self.eye.idle
        if now < self.startle_until:
            lid_target, self.status = LID_OPEN_STARTLE, "LOCK!"
        elif has:
            lid_target = LID_OPEN_TRACK + (LID_OPEN_FOCUS - LID_OPEN_TRACK) * self.focus
            self.status = "TRACKING" if self.focus < 0.3 else "FOCUS"
        elif idle:
            lid_target, self.status = LID_OPEN_IDLE, "DORMANT"
        else:
            lid_target, self.status = LID_OPEN_TRACK, "SEARCHING"
        if not has:
            self.aim = None if idle else self.aim
        self.lid = _ema(self.lid, lid_target, dt, LID_TAU_S)
        self.eye.lids(self.lid)


# ---------------------------------------------------------------------------
# Tekil ornek kilidi (ayni kamera + ayni COM portunu iki kopya istemesin)
# ---------------------------------------------------------------------------
def acquire_single_instance_lock(path: str = LOCK_PATH):
    f = open(path, "a+")
    try:
        if os.name == "nt":
            import msvcrt
            f.seek(0)
            msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return None
    return f


# ---------------------------------------------------------------------------
# Cizim
# ---------------------------------------------------------------------------
def draw_eye_schematic(cv2, img, x, y, w, h, eye: EyeController) -> None:
    """Izleyicinin gordugu gibi iki goz: robotun SAG gozu solda, bebek -u yonunde."""
    cv2.rectangle(img, (x, y), (x + w, y + h), (20, 18, 16), -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 140, 255), 1)
    r = int(min(w * 0.2, h * 0.38))
    cy = y + h // 2
    k = eye.settings["behaviour"]["lid_follow_pitch"] if eye.follow_pitch else 0.0
    # (ekran x merkezi, taraf) - izleyiciye gore sol = robotun sagi
    for cx, side in ((x + int(w * 0.28), "right"), (x + int(w * 0.72), "left")):
        o = eye.effective_open[1] if side == "right" else eye.effective_open[0]
        upper, lower = lid_follow(o, eye.gaze_v, k)  # sema icin 0..1 yeterli
        cv2.circle(img, (cx, cy), r, (235, 235, 235), -1, cv2.LINE_AA)
        px = int(cx - eye.gaze_u * r * 0.55)
        py = int(cy - eye.gaze_v * r * 0.5)
        cv2.circle(img, (px, py), int(r * 0.45), (160, 110, 20), -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), int(r * 0.18), (10, 10, 10), -1, cv2.LINE_AA)
        # kapaklar: ust kapak kenari yukaridan, alt kapak asagidan; kapaliyken
        # ortanin biraz altinda bulusurlar (SPEC §3)
        meet = cy + int(r * 0.1)
        top_edge = int(meet - upper * (meet - (cy - r)))
        bot_edge = int(meet + lower * ((cy + r) - meet))
        # kapaklari yalnizca goz dairesinin icine boya
        x0, y0 = cx - r, cy - r
        roi = img[y0:y0 + 2 * r + 1, x0:x0 + 2 * r + 1]
        mask = roi.copy()
        mask[:] = 0
        cv2.circle(mask, (r, r), r, (255, 255, 255), -1)
        lids = roi.copy()
        cv2.rectangle(lids, (0, 0), (2 * r, max(0, top_edge - y0)), (70, 66, 62), -1)
        cv2.rectangle(lids, (0, min(2 * r, bot_edge - y0)), (2 * r, 2 * r), (70, 66, 62), -1)
        inside = mask[:, :, 0] > 0
        roi[inside] = lids[inside]
        cv2.circle(img, (cx, cy), r, (0, 140, 255), 2, cv2.LINE_AA)


def put(cv2, img, text, org, scale=0.5, color=(230, 230, 230), thick=1):
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x, y = org
    cv2.rectangle(img, (x - 4, y - th - 4), (x + tw + 4, y + base + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


# ---------------------------------------------------------------------------
def open_camera(cv2, index: int):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(index)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Webcam {index} acilamadi. Baska bir uygulama (Lunar?) kamerayi kullaniyor "
            "olabilir ya da --camera yanlis.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    return cap


def create_detector(model_path: str, min_conf: float):
    from mediapipe.tasks.python import vision
    from mediapipe.tasks.python.core.base_options import BaseOptions
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"BlazeFace modeli yok: {model_path} (--model ile ver)")
    opts = vision.FaceDetectorOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=vision.RunningMode.VIDEO,
        min_detection_confidence=min_conf,
    )
    return vision.FaceDetector.create_from_options(opts)


def run(args) -> int:
    import cv2
    import mediapipe as mp

    eye = EyeController(port="sim" if args.sim else args.port)
    brain = FollowBrain(eye)
    mirror = args.mirror
    detector = create_detector(args.model, args.min_conf)
    cap = open_camera(cv2, args.camera)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    start = time.monotonic()
    prev = start
    last_ts_ms = -1
    fps = 0.0
    released = False
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Kare okunamadi - cikiliyor.")
                break
            now = time.monotonic()
            dt = max(1e-3, now - prev)
            prev = now
            h, w = frame.shape[:2]

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            # VIDEO modu KESIN artan zaman damgasi ister (ayni ms'e dusen iki kare cokertir)
            ts_ms = max(last_ts_ms + 1, int((now - start) * 1000))
            last_ts_ms = ts_ms
            result = detector.detect_for_video(image, ts_ms)

            boxes = []
            for det in result.detections or []:
                bb = det.bounding_box
                x1, y1 = max(0, bb.origin_x), max(0, bb.origin_y)
                x2, y2 = min(w, bb.origin_x + bb.width), min(h, bb.origin_y + bb.height)
                if x2 > x1 and y2 > y1:
                    boxes.append((x1, y1, x2, y2))
            primary = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1])) if boxes else None

            target = None
            if primary is not None:
                cx = (primary[0] + primary[2]) / 2
                cy = (primary[1] + primary[3]) / 2
                u = clamp((cx - w / 2) / (w / 2) * GAIN, -1, 1)
                v = clamp(-(cy - h / 2) / (h / 2) * GAIN, -1, 1)
                target = (-u if mirror else u, v)

            brain.step(now, dt, target)
            angles = eye.update(dt)

            # --- onizleme (izleyici icin aynali) ----------------------------
            view = cv2.flip(frame, 1)
            for b in boxes:
                x1, x2 = w - b[2], w - b[0]
                col = (0, 0, 255) if b is primary else (255, 220, 0)
                cv2.rectangle(view, (x1, b[1]), (x2, b[3]), col, 2 if b is primary else 1)
            # bakisin karedeki karsiligi (aynali karede robotun sagi = sol)
            gu = -eye.gaze_u if not mirror else eye.gaze_u
            gx = int(w / 2 * (1 + gu / GAIN))
            gy = int(h / 2 * (1 - eye.gaze_v / GAIN))
            cv2.circle(view, (gx, gy), 18, (0, 140, 255), 2, cv2.LINE_AA)

            fps = 0.9 * fps + 0.1 / dt
            mode = f"{eye.mode} {eye.mapping}" + (" (firmware idle)" if released else "")
            put(cv2, view, f"{brain.status}   {mode}", (12, 28), 0.7,
                (0, 0, 255) if brain.status in ("LOCK!", "FOCUS") else (0, 220, 255), 2)
            put(cv2, view, "YAW {:5.1f} PIT {:5.1f} | UL {:5.1f} LL {:5.1f} UR {:5.1f} LR {:5.1f}".format(*angles),
                (12, h - 40), 0.5)
            put(cv2, view, f"lid {eye.effective_open[0]:.2f}/{eye.effective_open[1]:.2f}  "
                           f"S {eye.sent_count}  ERR {eye.link.err_count}  FPS {fps:4.1f}"
                           f"{'  MIRROR' if mirror else ''}   q:cik b:kirp l/r:wink m:ayna h:idle d:detach",
                (12, h - 14), 0.45, (180, 180, 180))
            sw, sh = int(w * 0.34), int(h * 0.24)
            draw_eye_schematic(cv2, view, w - sw - 12, 12, sw, sh, eye)

            cv2.imshow(WINDOW_NAME, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if args.seconds and now - start >= args.seconds:
                break
            # X ile kapatildiysa: waitKey pencereyi yeniden yaratmadan once cik
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("b"):
                eye.blink()
            elif key == ord("l"):
                eye.wink("left")
            elif key == ord("r"):
                eye.wink("right")
            elif key == ord("m"):
                mirror = not mirror
            elif key == ord("d"):
                eye.detach()
            elif key == ord("h"):
                released = not released
                if released:
                    eye.release()
                else:
                    eye.resume()
    finally:
        print(f"cikis: {eye.mode}, gonderilen S={eye.sent_count}, ERR={eye.link.err_count}, "
              f"son durum={brain.status}")
        detector.close()
        cap.release()
        eye.close()
        cv2.destroyAllWindows()
    return 0


def main(argv=None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Project Eye v2 yuz takibi")
    ap.add_argument("--port", default=None, help="'auto' (varsayilan, settings.json), 'sim' ya da COMx")
    ap.add_argument("--sim", action="store_true", help="donanimi arama")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--min-conf", type=float, default=0.5)
    ap.add_argument("--mirror", action="store_true", help="u yonunu cevir")
    ap.add_argument("--seconds", type=float, default=0.0, help="N sn sonra kendiliginden cik (0 = sinirsiz)")
    args = ap.parse_args(argv)

    lock = acquire_single_instance_lock()
    if lock is None:
        print("face_follow zaten calisiyor (ayni kamera + seri port). Once onu kapat.")
        return 1
    try:
        return run(args)
    finally:
        lock.close()


if __name__ == "__main__":
    sys.exit(main())
