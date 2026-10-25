"""Project Eye v6 kontrol goruntuleri -> out/renders/*.png (masa egik, z-buffer rasterizer; v5 render.py'den uyarlandi,
v5 modulu import EDILMEZ: numba onbellegi v5/cad/__pycache__'e yazardi).

  PYTHONUTF8=1 python -B render.py
"""
import math
import os
import sys

import numpy as np
from numba import njit
from PIL import Image, ImageDraw

import eye_v6 as E6
import kin_v6 as K6
from params_v6 import TILT_DEG

OUT = os.path.join(E6.OUT, "renders")
BG = np.array([235, 238, 240], np.float64)


@njit(cache=False)
def raster(tri2, depth, col, W, H, zbuf, img):
    for i in range(tri2.shape[0]):
        x0, y0 = tri2[i, 0, 0], tri2[i, 0, 1]
        x1, y1 = tri2[i, 1, 0], tri2[i, 1, 1]
        x2, y2 = tri2[i, 2, 0], tri2[i, 2, 1]
        minx = max(int(math.floor(min(x0, min(x1, x2)))), 0)
        maxx = min(int(math.ceil(max(x0, max(x1, x2)))), W - 1)
        miny = max(int(math.floor(min(y0, min(y1, y2)))), 0)
        maxy = min(int(math.ceil(max(y0, max(y1, y2)))), H - 1)
        den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(den) < 1e-12:
            continue
        for py in range(miny, maxy + 1):
            for px in range(minx, maxx + 1):
                fx = px + 0.5; fy = py + 0.5
                a = ((y1 - y2) * (fx - x2) + (x2 - x1) * (fy - y2)) / den
                b = ((y2 - y0) * (fx - x2) + (x0 - x2) * (fy - y2)) / den
                c = 1.0 - a - b
                if a < 0 or b < 0 or c < 0:
                    continue
                z = a * depth[i, 0] + b * depth[i, 1] + c * depth[i, 2]
                if z < zbuf[py, px]:
                    zbuf[py, px] = z
                    img[py, px, 0] = col[i, 0]; img[py, px, 1] = col[i, 1]; img[py, px, 2] = col[i, 2]


def look_at(eye, target, up=(0, 0, 1)):
    f = np.asarray(target, float) - np.asarray(eye, float); f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.array([r, u, -f]), np.asarray(eye, float)


def render(meshes, cam_pos, target, fname, W=1400, H=1000, fov=30, title=None):
    Rc, e = look_at(cam_pos, target)
    L1 = np.array([0.3, -0.5, 0.8]); L1 /= np.linalg.norm(L1)
    L2 = np.array([-0.6, 0.4, 0.2]); L2 /= np.linalg.norm(L2)
    tris, cols = [], []
    for v, t, rgb in meshes:
        T = v[t]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        nn = np.linalg.norm(n, axis=1); ok = nn > 1e-12
        T = T[ok]; n = n[ok] / nn[ok, None]
        vd = T.mean(1) - e
        n[(n * vd).sum(1) > 0] *= -1
        sh = 0.35 + 0.55 * np.clip(n @ L1, 0, 1) + 0.2 * np.clip(n @ L2, 0, 1)
        tris.append(T); cols.append(np.clip(np.asarray(rgb, float)[None, :] * sh[:, None], 0, 255))
    T = np.concatenate(tris); Cc = np.concatenate(cols)
    P = (T - e) @ Rc.T
    z = -P[..., 2]
    f = (H / 2) / math.tan(math.radians(fov) / 2)
    zz = np.maximum(z, 1e-3)
    tri2 = np.stack([P[..., 0] / zz * f + W / 2, -P[..., 1] / zz * f + H / 2], -1).astype(np.float64)
    zbuf = np.full((H, W), 1e18); img = np.empty((H, W, 3), np.float64); img[:] = BG
    raster(tri2, z.astype(np.float64), Cc.astype(np.float64), W, H, zbuf, img)
    zb = np.nan_to_num(np.where(zbuf > 1e17, np.nan, zbuf), nan=1e5)
    dz = np.zeros_like(zbuf)
    dz[1:, :] = np.maximum(dz[1:, :], np.abs(np.diff(zb, axis=0)))
    dz[:, 1:] = np.maximum(dz[:, 1:], np.abs(np.diff(zb, axis=1)))
    img[dz > 1.5] *= 0.35
    im = Image.fromarray(img.astype(np.uint8))
    if title:
        ImageDraw.Draw(im).text((16, 12), title, fill=(20, 20, 20))
    im.save(fname)


def scene(P, cache, yaw, pitch, lid, hide=(), alfa=TILT_DEG):
    Ms = K6.pose(yaw, pitch, lid); Mm = K6.masa_M(alfa)
    out = []
    for n, p in P.items():
        if any(n == h or (h.endswith("*") and n.startswith(h[:-1])) for h in hide):
            continue
        v, t = cache[n]
        M = Mm @ Ms[p.group]
        out.append((v @ M[:3, :3].T + M[:3, 3], t, [int(p.color[i:i + 2], 16) for i in (1, 3, 5)]))
    return out


GORUNUM = [
    ("01_yan_sag_notr.png", (420, 40, 90), (0, 30, 70), 0, 0, 0.8, (), "sagdan: egik masa (30 deg), notr, kapak 0.8"),
    ("02_on_34.png", (260, -330, 230), (0, 30, 70), 0, 0, 0.8, (), "on 3/4: tum montaj, masa egik"),
    ("03_arka_34_p18.png", (-300, 260, 220), (0, 20, 60), 0, 18, 0.8, ("mask",), "arka 3/4: pitch +18 (ust sinir, arka en alcak), maske gizli"),
    ("04_sol_servo.png", (-330, -140, 150), (-100, 0, 80), 0, 0, 0.8, ("mask",), "soldan: sol servo + braket + adaptor (SECENEK B)"),
    ("05_krank_alt.png", (60, 240, -60), (0, 30, 60), 25, 18, 0.8, ("masa", "kama_*", "base_*", "mask"), "alttan-arkadan: krank ucu M4 havsa + alt emniyet somunu, yaw +25 pitch +18"),
    ("06_kama_bos.png", (260, -260, 200), (0, 20, 30), 0, 0, 0.8, ("masa",), "kama + taban (masa gizli)"),
    ("07_yan_sag_p-13.png", (420, 40, 90), (0, 30, 70), 0, -13, 1.0, (), "sagdan: pitch -13 (asim), kapak acik"),
]


def main(secim=None):
    P = E6.build(verbose=False)
    cache = {n: E6.E5.tess(p.shape, 0.05, 0.3) for n, p in P.items()}
    os.makedirs(OUT, exist_ok=True)
    for fn, cam, tgt, y, p, lid, hide, title in GORUNUM:
        if secim and fn[:2] not in secim:
            continue
        render(scene(P, cache, y, p, lid, hide), cam, tgt, os.path.join(OUT, fn), title=title)
        print("yazildi", fn, flush=True)


if __name__ == "__main__":
    main(sys.argv[1].split(",") if len(sys.argv) > 1 else None)
