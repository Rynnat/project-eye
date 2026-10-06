"""
Project Eye v5 - stant ekrani (HUD). "Bakin bilgisayar ne goruyor da bizi takip ediyor."

Ekrandaki tum yazilar INGILIZCE (kullanici 2026-10-06). Kamera goruntusunun ustune: baslik + durum, yuz kutulari (HUD koseleri + guven yuzdesi),
gozun baktigi nokta, sag ustte iki goz semasi, altta mod butonlari, kucuk telemetri.
Yazilar PIL ile (Turkce harf + Bahnschrift); sabit yazilar onbellekte.
Panel paleti Edgerunners (Epic paneli ile ayni).
"""

from __future__ import annotations

from functools import lru_cache

import cv2
import numpy as np

# BGR
CYAN = (255, 240, 0)
MAGENTA = (255, 0, 255)
INK = (253, 213, 244)
INK_DIM = (190, 160, 185)
PANEL = (46, 18, 26)
PANEL_EDGE = (150, 70, 110)
RED = (70, 60, 255)

FONT_FILES = {
    "disp": ("bahnschrift.ttf", "seguisb.ttf", "arialbd.ttf"),
    "mono": ("consola.ttf", "cour.ttf"),
}

# ---------------------------------------------------------------- durum metni
STATUS_TEXT = {
    "LOCK!": ("LOCKED ON", MAGENTA),
    "FOCUS": ("FOCUSED", MAGENTA),
    "TRACKING": ("TRACKING", CYAN),
    "SEARCHING": ("SEARCHING", INK),
    "DORMANT": ("RESTING", INK_DIM),
    "CANLI": ("LOOKING AROUND", INK),
    "CANLI - yuz araniyor": ("SEARCHING", INK),
    "TRACKING - yuz yok": ("NO FACE · HOLDING", INK_DIM),
    "NOTR": ("NEUTRAL", INK_DIM),
    "MANUEL": ("MANUAL CONTROL", MAGENTA),
}


def status_text(status: str):
    return STATUS_TEXT.get(status, (status, INK))


# ---------------------------------------------------------------- yazi
@lru_cache(maxsize=8)
def _font(kind: str, size: int):
    from PIL import ImageFont
    for name in FONT_FILES[kind]:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


@lru_cache(maxsize=512)
def text_sprite(text: str, size: int, color: tuple, kind: str = "disp", spacing: float = 0.0):
    """-> (bgr, alpha[0..1]) dizileri. spacing: harf arasi ek bosluk (px)."""
    from PIL import Image, ImageDraw
    font = _font(kind, size)
    if spacing:
        widths = [font.getlength(ch) + spacing for ch in text]
        tw = int(sum(widths) - spacing) + 2
    else:
        tw = int(font.getlength(text)) + 2
    asc, desc = font.getmetrics()
    pad = max(2, size // 4)          # Turkce buyuk harf noktasi (I/Ğ/Ş) ascender'in ustune tasar
    im = Image.new("L", (max(1, tw), asc + desc + pad), 0)
    d = ImageDraw.Draw(im)
    if spacing:
        x = 0.0
        for ch, cw in zip(text, widths):
            d.text((x, pad), ch, fill=255, font=font)
            x += cw
    else:
        d.text((0, pad), text, fill=255, font=font)
    a = np.asarray(im, dtype=np.float32) / 255.0
    bgr = np.empty(a.shape + (3,), np.uint8)
    bgr[:] = color
    return bgr, a


def text_size(text, size, kind="disp", spacing=0.0):
    bgr, _ = text_sprite(text, size, (255, 255, 255), kind, spacing)
    return bgr.shape[1], bgr.shape[0]


def draw_text(img, text, x, y, size, color, kind="disp", spacing=0.0, alpha=1.0):
    """(x, y) = sol ust. Kare disina tasan kisim kirpilir."""
    bgr, a = text_sprite(text, size, tuple(color), kind, spacing)
    _blit(img, bgr, a * alpha, int(x), int(y))
    return bgr.shape[1], bgr.shape[0]


def _blit(img, bgr, a, x, y):
    h, w = img.shape[:2]
    sh, sw = a.shape
    x0, y0, x1, y1 = max(0, x), max(0, y), min(w, x + sw), min(h, y + sh)
    if x1 <= x0 or y1 <= y0:
        return
    roi = img[y0:y1, x0:x1].astype(np.float32)
    aa = a[y0 - y:y1 - y, x0 - x:x1 - x, None]
    roi = roi * (1 - aa) + bgr[y0 - y:y1 - y, x0 - x:x1 - x].astype(np.float32) * aa
    img[y0:y1, x0:x1] = roi.astype(np.uint8)


# ---------------------------------------------------------------- sekiller
def panel(img, x, y, w, h, color=PANEL, alpha=0.62, radius=8, edge=None):
    """Yari saydam yuvarlak koseli panel."""
    H, W = img.shape[:2]
    x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + w), min(H, y + h)
    if x1 <= x0 or y1 <= y0:
        return
    mask = np.zeros((h, w), np.uint8)
    r = min(radius, h // 2, w // 2)
    cv2.rectangle(mask, (r, 0), (w - 1 - r, h - 1), 255, -1)
    cv2.rectangle(mask, (0, r), (w - 1, h - 1 - r), 255, -1)
    for cx, cy in ((r, r), (w - 1 - r, r), (r, h - 1 - r), (w - 1 - r, h - 1 - r)):
        cv2.circle(mask, (cx, cy), r, 255, -1, cv2.LINE_AA)
    a = mask[y0 - y:y1 - y, x0 - x:x1 - x].astype(np.float32)[..., None] / 255.0 * alpha
    roi = img[y0:y1, x0:x1].astype(np.float32)
    img[y0:y1, x0:x1] = (roi * (1 - a) + np.array(color, np.float32) * a).astype(np.uint8)
    if edge is not None:
        _round_outline(img, x, y, w, h, r, edge)


def _round_outline(img, x, y, w, h, r, color, t=1):
    cv2.line(img, (x + r, y), (x + w - 1 - r, y), color, t, cv2.LINE_AA)
    cv2.line(img, (x + r, y + h - 1), (x + w - 1 - r, y + h - 1), color, t, cv2.LINE_AA)
    cv2.line(img, (x, y + r), (x, y + h - 1 - r), color, t, cv2.LINE_AA)
    cv2.line(img, (x + w - 1, y + r), (x + w - 1, y + h - 1 - r), color, t, cv2.LINE_AA)
    for (cx, cy), a0 in (((x + r, y + r), 180), ((x + w - 1 - r, y + r), 270),
                         ((x + w - 1 - r, y + h - 1 - r), 0), ((x + r, y + h - 1 - r), 90)):
        cv2.ellipse(img, (cx, cy), (r, r), 0, a0, a0 + 90, color, t, cv2.LINE_AA)


def brackets(img, x1, y1, x2, y2, color, t=2, frac=0.22):
    """HUD kilit koseleri."""
    L = int(max(10, min(x2 - x1, y2 - y1) * frac))
    for (cx, cy, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
        cv2.line(img, (cx, cy), (cx + dx * L, cy), color, t, cv2.LINE_AA)
        cv2.line(img, (cx, cy), (cx, cy + dy * L), color, t, cv2.LINE_AA)


def reticle(img, x, y, color, r=16, t=2):
    cv2.circle(img, (x, y), r, color, t, cv2.LINE_AA)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cv2.line(img, (x + dx * (r - 5), y + dy * (r - 5)), (x + dx * (r + 7), y + dy * (r + 7)), color, t, cv2.LINE_AA)
    cv2.circle(img, (x, y), 2, color, -1, cv2.LINE_AA)


# ---------------------------------------------------------------- mod butonlari
PILL_H, PILL_GAP, PILL_PAD, PILL_FONT, PILL_BOTTOM = 26, 6, 14, 13, 16


def pill_rects(labels, w, h):
    """Alt ortada mod butonlari -> [(i, x, y, bw, bh)]."""
    widths = [text_size(lb, PILL_FONT, spacing=0.6)[0] + 2 * PILL_PAD for lb in labels]
    total = sum(widths) + PILL_GAP * (len(widths) - 1)
    x = (w - total) // 2
    y = h - PILL_BOTTOM - PILL_H
    out = []
    for i, bw in enumerate(widths):
        out.append((i, x, y, bw, PILL_H))
        x += bw + PILL_GAP
    return out


def draw_pills(img, labels, active: int, hint: str | None = None):
    h, w = img.shape[:2]
    rects = pill_rects(labels, w, h)
    for i, x, y, bw, bh in rects:
        on = i == active
        panel(img, x, y, bw, bh, CYAN if on else PANEL, 0.95 if on else 0.62, radius=bh // 2,
              edge=None if on else PANEL_EDGE)
        tw, th = text_size(labels[i], PILL_FONT, spacing=0.6)
        draw_text(img, labels[i], x + (bw - tw) // 2, y + (bh - th) // 2, PILL_FONT,
                  (46, 10, 26) if on else INK, spacing=0.6)
    if hint:
        tw, th = text_size(hint, 11, spacing=0.8)
        draw_text(img, hint, (w - tw) // 2, rects[0][2] - th - 4, 11, INK_DIM, spacing=0.8, alpha=0.8)
    return rects


def hit_pill(labels, w, h, px, py):
    for i, x, y, bw, bh in pill_rects(labels, w, h):
        if x <= px < x + bw and y <= py < y + bh:
            return i
    return None


# ---------------------------------------------------------------- ust sol
def draw_header(img, status: str, sub: str):
    text, color = status_text(status)
    tw1, _ = text_size("PROJECT EYE", 20, spacing=2.0)
    tw2, _ = text_size(text, 15, spacing=1.0)
    tw3, _ = text_size(sub, 12, kind="mono")
    pw = max(tw1, tw2 + 22, tw3) + 28
    panel(img, 12, 12, pw, 86, edge=PANEL_EDGE)
    draw_text(img, "PROJECT EYE", 26, 20, 20, CYAN, spacing=2.0)
    cv2.circle(img, (31, 58), 5, color, -1, cv2.LINE_AA)
    draw_text(img, text, 44, 48, 15, color, spacing=1.0)
    draw_text(img, sub, 26, 74, 12, INK_DIM, kind="mono")


def draw_face(img, box, primary: bool, score: float | None, status: str):
    x1, y1, x2, y2 = box
    if not primary:
        brackets(img, x1, y1, x2, y2, INK_DIM, 1, 0.15)
        return
    color = MAGENTA if status in ("LOCK!", "FOCUS") else CYAN
    brackets(img, x1, y1, x2, y2, color, 2)
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.line(img, (cx - 6, cy), (cx + 6, cy), color, 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy - 6), (cx, cy + 6), color, 1, cv2.LINE_AA)
    label = "HUMAN" + (f" · {int(round(score * 100))}%" if score is not None else "")
    tw, th = text_size(label, 13, spacing=1.0)
    ly = y1 - th - 10 if y1 - th - 10 > 4 else y2 + 6
    panel(img, x1, ly - 3, tw + 14, th + 6, color, 0.9, radius=4)
    draw_text(img, label, x1 + 7, ly, 13, (46, 10, 26), spacing=1.0)


# ---------------------------------------------------------------- donanim sagligi
HEALTH_TEXT = {
    "ok": None,
    "sim": ("NO HARDWARE", RED),
    "lost": ("CONNECTION LOST", RED),
    "silent": ("NO RESPONSE FROM BOARD", RED),
    "reset": ("BOARD RESET · CHECK POWER", RED),
}


def draw_health(img, x, y, w, h, health: str):
    """Goz semasinin ustune: donanim calismiyorsa semayi soldur + uyari."""
    info = HEALTH_TEXT.get(health)
    if info is None:
        return
    text, color = info
    # tum hatalar ayni: soluk + kirmizi cerceve (gozler face_follow'da donuk)
    roi = img[y:y + h, x:x + w]
    gray = cv2.cvtColor(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
    img[y:y + h, x:x + w] = (gray * 0.45).astype(np.uint8)
    _round_outline(img, x, y, w, h, 8, color, 2)
    tw, th = text_size(text, 14, spacing=1.0)
    bx, by = x + (w - tw) // 2 - 10, y + h - th - 14
    panel(img, bx, by, tw + 20, th + 8, color, 0.92, radius=4)
    draw_text(img, text, bx + 10, by + 2, 14, (20, 10, 20), spacing=1.0)
