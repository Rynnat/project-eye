"""
Project Eye v5 - webcam yuz takibi -> gozler (3 servo).

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

Modlar (penceredeki butonlar ya da 1-4 tuslari; kamera her modda acik):
    IDLE      yuzu yok sayar, uykulu kapakla yavasca etrafi kolacan eder
    CANLI     yukaridaki tam davranis (takip + irkilme + odak + kirpma + idle)
    NOTR      gozler ortada, kapak acik, kirpma yok
    TRACKING  yalniz takip: yuze bakar, kapak sabit acik, irkilme/kirpma yok;
              yuz kaybolunca son noktada bekler

Tuslar: q / ESC cikis, 1-4 mod, b kirp, m ayna (u yonunu cevir),
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

from eye_control import EyeController, clamp  # noqa: E402

WINDOW_NAME = "Project Eye v5 - Face Follow"
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

MODES = ("IDLE", "CANLI", "NOTR", "TRACKING")
MODE_LABELS = {"IDLE": "IDLE", "CANLI": "CANLI", "NOTR": "NÖTR", "TRACKING": "TRACKING"}
DEFAULT_MODE = "CANLI"


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


class ModeBrain:
    """Mod secici: CANLI = FollowBrain, digerleri sade davranislar. cv2 gerektirmez."""

    def __init__(self, eye: EyeController, mode: str = DEFAULT_MODE):
        self.eye = eye
        self.live = FollowBrain(eye)
        self.mode = None
        self.aim = None
        self.lid = eye.openness
        self.status = ""
        self.set_mode(mode)

    def set_mode(self, mode: str) -> None:
        if mode not in MODES or mode == self.mode:
            return
        self.mode = mode
        eye = self.eye
        eye.auto_blink = mode in ("IDLE", "CANLI")
        eye.idle_wander = mode in ("IDLE", "CANLI")
        if mode == "IDLE":
            eye._last_look = -1e9       # look() gelmiyor sayilsin: hemen kolacan etmeye basla
        elif mode == "CANLI":
            self.live = FollowBrain(eye)
            self.live.lid = self.lid
        self.aim = None

    def step(self, now: float, dt: float, target) -> None:
        if self.mode == "CANLI":
            self.live.step(now, dt, target)
            self.lid, self.status = self.live.lid, self.live.status
            return
        if self.mode == "IDLE":
            lid_target, self.status = LID_OPEN_IDLE, "IDLE"
        elif self.mode == "NOTR":
            self.eye.look(0.0, 0.0)
            lid_target, self.status = LID_OPEN_TRACK, "NOTR"
        else:  # TRACKING
            if target is not None:
                self.aim = target if self.aim is None else (
                    _ema(self.aim[0], target[0], dt, AIM_TAU_S), _ema(self.aim[1], target[1], dt, AIM_TAU_S))
                self.status = "TRACKING"
            else:
                self.status = "TRACKING - yuz yok"
            self.eye.look(*(self.aim or (0.0, 0.0)))
            lid_target = LID_OPEN_TRACK
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
BTN_W, BTN_H, BTN_GAP, BTN_X0, BTN_Y0 = 120, 34, 8, 12, 12


def mode_button_rects():
    return [(m, BTN_X0 + i * (BTN_W + BTN_GAP), BTN_Y0) for i, m in enumerate(MODES)]


def make_button_images():
    """Mod butonlari (pasif/aktif) bir kez cizilir; Turkce harf icin PIL (cv2 yazisi 'O' basamaz)."""
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    font = None
    for name in ("seguisb.ttf", "segoeuib.ttf", "arialbd.ttf"):
        try:
            font = ImageFont.truetype(name, 17)
            break
        except OSError:
            pass
    font = font or ImageFont.load_default()
    imgs = {}
    for m in MODES:
        for active in (False, True):
            # Edgerunners paleti (panel ile ayni): koyu mor zemin, cyan vurgu
            bg, fg, edge = ((0, 240, 255), (26, 10, 46), (0, 240, 255)) if active else ((36, 18, 61), (244, 213, 253), (110, 70, 150))
            im = Image.new("RGB", (BTN_W, BTN_H), bg)
            d = ImageDraw.Draw(im)
            d.rectangle([0, 0, BTN_W - 1, BTN_H - 1], outline=edge, width=2)
            text = MODE_LABELS[m]
            x0, y0, x1, y1 = d.textbbox((0, 0), text, font=font)
            d.text(((BTN_W - (x1 - x0)) / 2 - x0, (BTN_H - (y1 - y0)) / 2 - y0), text, fill=fg, font=font)
            imgs[m, active] = np.ascontiguousarray(np.asarray(im)[:, :, ::-1])
    return imgs


def draw_mode_buttons(img, imgs, current: str) -> None:
    h, w = img.shape[:2]
    for m, x, y in mode_button_rects():
        if x + BTN_W <= w and y + BTN_H <= h:
            img[y:y + BTN_H, x:x + BTN_W] = imgs[m, m == current]


def hit_mode_button(px: int, py: int):
    for m, x, y in mode_button_rects():
        if x <= px < x + BTN_W and y <= py < y + BTN_H:
            return m
    return None


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
    brain = ModeBrain(eye, args.mode)
    mirror = args.mirror
    detector = create_detector(args.model, args.min_conf)
    cap = open_camera(cv2, args.camera)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    btn_imgs = make_button_images()
    clicked = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            m = hit_mode_button(x, y)
            if m:
                clicked.append(m)

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

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

            while clicked:
                brain.set_mode(clicked.pop(0))
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
            draw_mode_buttons(view, btn_imgs, brain.mode)
            put(cv2, view, f"{brain.status}   {mode}", (12, BTN_Y0 + BTN_H + 30), 0.7,
                (0, 0, 255) if brain.status in ("LOCK!", "FOCUS") else (0, 220, 255), 2)
            # pitch_dead iken servo merkezde; ekranda hesaplanan pitch gosterilir
            shown = (angles[0], eye.pitch_wanted, angles[2])
            put(cv2, view, "YAW {:6.2f}  PITCH {:6.2f}  LIDS {:6.2f}".format(*shown),
                (12, h - 40), 0.5)
            put(cv2, view, f"lid {eye.effective_open:.2f}  "
                           f"S {eye.sent_count}  ERR {eye.link.err_count}  FPS {fps:4.1f}"
                           f"{'  MIRROR' if mirror else ''}   q:cik 1-4:mod b:kirp m:ayna h:fw-idle d:detach",
                (12, h - 14), 0.45, (180, 180, 180))

            cv2.imshow(WINDOW_NAME, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if args.seconds and now - start >= args.seconds:
                break
            # X ile kapatildiysa: waitKey pencereyi yeniden yaratmadan once cik
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if ord("1") <= key <= ord("4"):
                brain.set_mode(MODES[key - ord("1")])
            elif key == ord("b"):
                eye.blink()
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
    ap = argparse.ArgumentParser(description="Project Eye v5 yuz takibi")
    ap.add_argument("--port", default=None, help="'auto' (varsayilan, settings.json), 'sim' ya da COMx")
    ap.add_argument("--sim", action="store_true", help="donanimi arama")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--min-conf", type=float, default=0.5)
    ap.add_argument("--mode", choices=MODES, default=DEFAULT_MODE, help="baslangic modu")
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
