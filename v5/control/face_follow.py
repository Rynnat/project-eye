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

Modlar (penceredeki butonlar ya da 1-5 tuslari; kamera her modda acik):
    LIVE     yuzu yok sayar; insan istatistikli bakinma + kirpma, kapak acik
    T LIVE   yuz varken tam takip (irkilme + odak + kirpma); yuz yokken LIVE gibi
             bakinir, yuz gorunce kilitlenir
    T        yalniz takip: kirpma/irkilme/bakinma yok, kapak sabit acik;
             yuz gidince son noktada bekler
    NEUTRAL  gozler ortada, kapak acik, kirpma yok
    MANUAL   motorlar elle: goruntude surukle = bakis, tekerlek = kapak,
             ok tuslari = bakis, [ ] = kapak
D2 butonu (breadboard) otomatik modlar arasinda doner (MANUAL haric: fare ister).

Tuslar: q / ESC cikis, 1-5 mod, b kirp, i teknik bilgi, m ayna (u yonunu cevir),
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

from eye_control import EyeController, EyeLink, candidate_ports, clamp, find_eye_port, lid_follow  # noqa: E402

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

MODES = ("SERBEST", "TAKIP", "CANSIZ", "NOTR", "MANUEL")
MODE_LABELS = {"SERBEST": "LIVE", "TAKIP": "T LIVE", "CANSIZ": "T", "NOTR": "NEUTRAL", "MANUEL": "MANUAL"}
BUTTON_CYCLE = ("SERBEST", "TAKIP", "CANSIZ", "NOTR")   # D2 butonu; MANUEL fare ister
MANUAL_KEY_STEP = 0.06       # ok tusu basina bakis adimi (-1..1)
MANUAL_LID_STEP = 0.08
DEFAULT_MODE = "TAKIP"


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
    """Mod secici: TAKIP = FollowBrain, SERBEST = takipsiz canli, CANSIZ = sade takip,
    NOTR = sabit, MANUEL = elle (manual_set). cv2 gerektirmez."""

    def __init__(self, eye: EyeController, mode: str = DEFAULT_MODE):
        self.eye = eye
        self.live = FollowBrain(eye)
        self.mode = None
        self.aim = None
        self.lid = eye.openness
        self.status = ""
        self.manual = [0.0, 0.0, LID_OPEN_TRACK]      # u, v, kapak
        self.set_mode(mode)

    def set_mode(self, mode: str) -> None:
        if mode not in MODES or mode == self.mode:
            return
        self.mode = mode
        eye = self.eye
        eye.auto_blink = eye.idle_wander = mode in ("SERBEST", "TAKIP")
        self.aim = None
        if mode == "SERBEST":
            eye._last_look = -1e9       # look() gelmiyor sayilsin: hemen bakinmaya basla
        elif mode == "TAKIP":
            self.live = FollowBrain(eye)
            self.live.lid = self.lid
        elif mode == "MANUEL":
            self.manual = [eye.gaze_u, eye.gaze_v, self.lid]   # bulundugu yerden devral, sicrama yok

    def next_mode(self) -> None:
        """Breadboard'daki D2 butonu: otomatik modlar arasinda sirayla dolas."""
        i = BUTTON_CYCLE.index(self.mode) if self.mode in BUTTON_CYCLE else -1
        self.set_mode(BUTTON_CYCLE[(i + 1) % len(BUTTON_CYCLE)])

    def manual_set(self, u=None, v=None, lid=None, du=0.0, dv=0.0, dlid=0.0) -> None:
        """MANUEL modda hedef: mutlak (u/v/lid) ya da goreli (du/dv/dlid)."""
        m = self.manual
        m[0] = clamp((m[0] if u is None else u) + du, -1.0, 1.0)
        m[1] = clamp((m[1] if v is None else v) + dv, -1.0, 1.0)
        m[2] = clamp((m[2] if lid is None else lid) + dlid, 0.0, 1.0)

    def step(self, now: float, dt: float, target) -> None:
        # karsisinda biri varken insanlar daha sik kirpar (Bentivoglio 1997: ~26/dk vs ~17/dk)
        self.eye.blink_context = "conversation" if target is not None and self.mode == "TAKIP" else "rest"
        if self.mode == "TAKIP":
            self.live.step(now, dt, target)
            if target is not None or self.live.had_target:
                self.lid, self.status = self.live.lid, self.live.status
                return
            # yuz yok (kayip kesinlesti ya da hic gorulmedi): yuz gorene kadar takipsiz canli hareket
            self.eye._last_look = -1e9
            self.lid = self.live.lid = _ema(self.lid, LID_OPEN_TRACK, dt, LID_TAU_S)
            self.eye.lids(self.lid)
            self.status = "CANLI - yuz araniyor"
            return
        if self.mode == "SERBEST":
            self.status = "CANLI"
        elif self.mode == "CANSIZ":
            if target is not None:
                self.aim = target if self.aim is None else (
                    _ema(self.aim[0], target[0], dt, AIM_TAU_S), _ema(self.aim[1], target[1], dt, AIM_TAU_S))
                self.status = "TRACKING"
            else:
                self.status = "TRACKING - yuz yok"
            self.eye.look(*(self.aim or (0.0, 0.0)))
        elif self.mode == "MANUEL":
            self.eye.look(self.manual[0], self.manual[1])
            self.lid = self.manual[2]
            self.eye.lids(self.lid)
            self.status = "MANUEL"
            return
        else:
            self.eye.look(0.0, 0.0)
            self.status = "NOTR"
        self.lid = _ema(self.lid, LID_OPEN_TRACK, dt, LID_TAU_S)
        self.eye.lids(self.lid)


class Reconnector:
    """Arka planda Project Eye kartini arar (el sikisma ~2-3 sn surer; ekran donmasin).

    Port listesi degisince (kablo takildi) hemen, yoksa RETRY_S'de bir dener - surekli
    taramak ayni USB'deki baska kartlari (Lunar gimbal) her seferinde resetlerdi.
    """

    RETRY_S = 10.0

    def __init__(self, port: str, finder=None, lister=None):
        import threading
        self._threading = threading
        self.port = port or "auto"
        self.finder = finder or (lambda prt: find_eye_port(prt, log=lambda *_: None))
        self.lister = lister or (lambda: candidate_ports(os.environ.get("EYE_EXCLUDE_PORTS", "").split(",")))
        self._thread = None
        self._result = None
        self._last_ports = None
        self._next_try = 0.0

    def poll(self, now: float, force: bool = False):
        """-> (ser, dev) bulunduysa, yoksa None. Bloklamaz."""
        if self._thread is not None and self._thread.is_alive():
            return None
        if self._result is not None:
            found, self._result = self._result, None
            return found
        try:
            ports = tuple(sorted(self.lister()))
        except Exception:
            ports = ()
        if force or ports != self._last_ports or now >= self._next_try:
            self._last_ports = ports
            self._next_try = now + self.RETRY_S

            def work():
                try:
                    ser, dev = self.finder(self.port)
                except Exception:
                    ser, dev = None, None
                if ser is not None:
                    self._result = (ser, dev)

            self._thread = self._threading.Thread(target=work, daemon=True)
            self._thread.start()
        return None


class LinkMonitor:
    """Sag ustteki gozler gercegi gostersin: kart cevap veriyor mu? Hata gecince kendiliginden duzelir.

    Her HEARTBEAT_S'de '?' gonderir; karttan SILENT_S boyunca satir gelmezse "silent",
    port koptuysa "lost", acilis banner'i yeniden geldiyse (kart resetlendi - cogu zaman
    servo akimiyla brown-out) RESET_SHOW_S boyunca "reset", donanim yoksa "sim".
    Duzelme: "silent" kendi gecer (kart yeniden cevap verirse); RECONNECT_SILENT_S'den uzun
    surerse port kapatilip yeniden baglanilir. "lost" ve "sim"de Reconnector karti arar,
    bulununca baglanti degistirilir.
    NOT: servo besleme hatti tek basina kesilirse kart USB'den calismaya devam eder ve
    bunu seri hattan ANLAYAMAYIZ; onun icin besleme olcumu gerekir.
    """

    HEARTBEAT_S = 0.5
    SILENT_S = 1.5
    RECONNECT_SILENT_S = 5.0
    RESET_SHOW_S = 5.0

    def __init__(self, eye: EyeController, reconnector: Reconnector | None = None):
        self.eye = eye
        self.reconnector = reconnector or Reconnector(eye.settings["serial_port"])
        self._next_ping = 0.0
        self._resets = eye.link.resets
        self._reset_until = -1.0
        self._force_scan = False

    def _swap(self, ser, dev) -> None:
        old = self.eye.link
        old.close()
        new = EyeLink(ser, dev, simulated=False, clock=old.clock)
        self.eye.link = new
        self.eye._last_sent = None          # yeni karta hemen tam poz gitsin
        self.eye._lost_reported = False
        self._resets = new.resets
        self._reset_until = -1.0
        self._next_ping = 0.0

    def update(self, now: float) -> str:
        link = self.eye.link
        if self.eye.simulated or not link.alive:
            if not link.alive and not link.simulated:
                link.close()                # portu birak ki yeniden acilabilsin
            found = self.reconnector.poll(now, force=self._force_scan)
            self._force_scan = False
            if found is None:
                return "sim" if self.eye.simulated else "lost"
            self._swap(*found)
            link = self.eye.link
        if now >= self._next_ping:
            link.send("?")
            self._next_ping = now + self.HEARTBEAT_S
        if link.resets != self._resets:
            self._resets = link.resets
            self._reset_until = now + self.RESET_SHOW_S
        silent_for = now - link.last_rx_t
        if silent_for > self.RECONNECT_SILENT_S:
            link.close()                    # takildi: kapat, yeniden ara
            self._force_scan = True
            return "lost"
        if silent_for > self.SILENT_S:
            return "silent"
        if now < self._reset_until:
            return "reset"
        return "ok"


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
    import hud
    hud.panel(img, x, y, w, h, edge=hud.PANEL_EDGE)
    r = int(min(w * 0.2, h * 0.34))
    cy = y + h // 2
    k = eye.settings["behaviour"]["lid_follow_pitch"] if eye.follow_pitch else 0.0
    # (ekran x merkezi, taraf) - izleyiciye gore sol = robotun sagi
    for cx, side in ((x + int(w * 0.28), "right"), (x + int(w * 0.72), "left")):
        upper = lid_follow(eye.effective_open, eye.gaze_v, k)
        lower = upper  # v4: alt kapaklar ayni servoda, ayni aciklik
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
        cv2.circle(img, (cx, cy), r, hud.CYAN, 2, cv2.LINE_AA)


# cv2.waitKeyEx ok tuslari (Windows) -> ekranda (sag +, yukari +)
ARROWS = {2424832: (-1, 0), 2555904: (1, 0), 2490368: (0, 1), 2621440: (0, -1)}


def schematic_rect(w: int, h: int):
    """Goz semasi sag ustte."""
    sw, sh = int(w * 0.30), int(h * 0.22)
    return w - sw - 12, 12, sw, sh




def wheel_delta(flags: int) -> int:
    """Fare tekerlegi: flags'in ust 16 biti isaretli delta (+120 yukari). cv2.getMouseWheelDelta
    OpenCV 5'te (Lunar venv) YOK - cagri hata verip olayi sessizce yutuyordu."""
    d = (flags >> 16) & 0xFFFF
    return d - 0x10000 if d >= 0x8000 else d


def screen_to_gaze(x, y, w, h, mirror):
    """Ekrandaki nokta -> bakis (u, v). Nisangahin cizildigi formulun tersi."""
    gu = clamp(x / (w / 2) - 1.0, -1.0, 1.0) * GAIN
    v = clamp(1.0 - y / (h / 2), -1.0, 1.0) * GAIN
    return (gu if mirror else -gu), v


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
    import hud
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    labels = [MODE_LABELS[m] for m in MODES]
    clicked = []
    frame_size = [960, 540]
    show_debug = False

    dragging = [False]

    def on_mouse(event, x, y, flags, param):
        w_, h_ = frame_size
        if event == cv2.EVENT_LBUTTONDOWN:
            i = hud.hit_pill(labels, w_, h_, x, y)
            if i is not None:
                clicked.append(MODES[i])
                return
            dragging[0] = brain.mode == "MANUEL"
        if event == cv2.EVENT_LBUTTONUP:
            dragging[0] = False
        if brain.mode != "MANUEL":
            return
        if dragging[0] and event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_MOUSEMOVE):
            u, v = screen_to_gaze(x, y, w_, h_, mirror)
            brain.manual_set(u=u, v=v)
        elif event == cv2.EVENT_MOUSEWHEEL:
            d = wheel_delta(flags)
            if d:
                brain.manual_set(dlid=MANUAL_LID_STEP if d > 0 else -MANUAL_LID_STEP)

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    seen_presses = eye.link.button_presses
    counted_link = eye.link
    monitor = LinkMonitor(eye)
    frozen = None

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
            frame_size[:] = [w, h]

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
                    score = det.categories[0].score if det.categories else None
                    boxes.append((x1, y1, x2, y2, score))
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
            if eye.link is not counted_link:                  # yeniden baglanildi: sayac sifirdan
                counted_link, seen_presses = eye.link, eye.link.button_presses
            while seen_presses < eye.link.button_presses:      # D2 butonu (firmware "BTN")
                seen_presses += 1
                brain.next_mode()
            brain.step(now, dt, target)
            angles = eye.update(dt)

            # --- stant ekrani (izleyici icin aynali) ------------------------
            view = cv2.flip(frame, 1)
            for bx in boxes:
                hud.draw_face(view, (w - bx[2], bx[1], w - bx[0], bx[3]), bx is primary, bx[4], brain.status)
            # gozun baktigi nokta (aynali karede robotun sagi = sol); pitch_dead iken hesaplanan bakis
            gu = -eye.gaze_u if not mirror else eye.gaze_u
            gx = int(w / 2 * (1 + gu / GAIN))
            gy = int(h / 2 * (1 - eye.gaze_v / GAIN))
            hud.reticle(view, gx, gy, hud.MAGENTA)

            fps = 0.9 * fps + 0.1 / dt
            # pitch_dead iken servo merkezde; ekranda hesaplanan pitch gosterilir
            sub = "YAW {:5.1f}°  PITCH {:5.1f}°  LID {:d}%".format(
                angles[0], eye.pitch_wanted, int(round(eye.effective_open * 100)))
            hud.draw_header(view, brain.status, sub)
            srect = schematic_rect(w, h)
            health = monitor.update(now)
            sx, sy, sw, sh = srect
            if health == "ok" or frozen is None or frozen.shape[:2] != (sh, sw):
                draw_eye_schematic(cv2, view, *srect, eye)
                frozen = view[sy:sy + sh, sx:sx + sw].copy()   # son saglikli kare
            else:
                # donanim yok / cevap yok: gozler son bilinen halde DONAR (gercek motorlar da hareket etmiyor)
                view[sy:sy + sh, sx:sx + sw] = frozen
            hud.draw_health(view, *srect, health)
            hud.draw_pills(view, labels, MODES.index(brain.mode))
            if show_debug:
                dbg = (f"{eye.mode} {eye.mapping}{' fw-idle' if released else ''}  S {eye.sent_count}  "
                       f"ERR {eye.link.err_count}  BTN {eye.link.button_presses}  FPS {fps:4.1f}"
                       f"{'  MIRROR' if mirror else ''}  |  q quit  1-5 mode  b blink  m mirror  h fw-idle  d detach  i hide")
                hud.draw_text(view, dbg, 14, h - 70, 12, hud.INK_DIM, kind="mono")

            cv2.imshow(WINDOW_NAME, view)
            full = cv2.waitKeyEx(1)
            key = full & 0xFF if 0 <= full < 0x10000 else -1
            if key in (ord("q"), 27):
                break
            if args.seconds and now - start >= args.seconds:
                break
            # X ile kapatildiysa: waitKey pencereyi yeniden yaratmadan once cik
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if ord("1") <= key < ord("1") + len(MODES):
                brain.set_mode(MODES[key - ord("1")])
            elif full in ARROWS and brain.mode == "MANUEL":
                du, dv = ARROWS[full]
                # ekranda sol/sag: aynali goruntude robotun sagi solda
                brain.manual_set(du=(du if mirror else -du) * MANUAL_KEY_STEP, dv=dv * MANUAL_KEY_STEP)
            elif key in (ord("["), ord("]")) and brain.mode == "MANUEL":
                brain.manual_set(dlid=MANUAL_LID_STEP if key == ord("]") else -MANUAL_LID_STEP)
            elif key == ord("b"):
                eye.blink()
            elif key == ord("i"):
                show_debug = not show_debug
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
