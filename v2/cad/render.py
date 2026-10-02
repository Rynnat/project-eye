"""Kontrol goruntuleri: out/renders/*.png (numba z-buffer rasterizer; OpenGL gerektirmez).

  PYTHONUTF8=1 python render.py
"""
import math
import os

import numpy as np
from numba import njit
from PIL import Image, ImageDraw

from eye_v2 import build, pose_transforms, tess, OUT

BG = np.array([235, 238, 240], np.float64)


@njit(cache=True)
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


def look_at(eye, target, up=(0, 1, 0)):
    f = np.asarray(target, float) - np.asarray(eye, float); f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.array([r, u, -f]), np.asarray(eye, float)


def render(meshes, cam_pos, target, fname, W=1400, H=1000, fov=30, title=None, ortho=None):
    Rc, e = look_at(cam_pos, target)
    L1 = np.array([0.4, 0.8, 0.6]); L1 /= np.linalg.norm(L1)
    L2 = np.array([-0.6, 0.2, -0.3]); L2 /= np.linalg.norm(L2)
    tris = []; cols = []
    for v, t, rgb in meshes:
        T = v[t]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        nn = np.linalg.norm(n, axis=1); ok = nn > 1e-12
        T = T[ok]; n = n[ok] / nn[ok, None]
        # iki yuzlu isik (ince kabuklar icin)
        vd = T.mean(1) - e
        flip = (n * vd).sum(1) > 0
        n[flip] *= -1
        sh = 0.35 + 0.55 * np.clip(n @ L1, 0, 1) + 0.2 * np.clip(n @ L2, 0, 1)
        c = np.clip(np.asarray(rgb, float)[None, :] * sh[:, None], 0, 255)
        tris.append(T); cols.append(c)
    T = np.concatenate(tris); Cc = np.concatenate(cols)
    P = (T - e) @ Rc.T                                  # kamera: -z ileri
    z = -P[..., 2]
    if ortho:
        s = min(W, H) / ortho
        sx = P[..., 0] * s + W / 2; sy = -P[..., 1] * s + H / 2
    else:
        f = (H / 2) / math.tan(math.radians(fov) / 2)
        zz = np.maximum(z, 1e-3)
        sx = P[..., 0] / zz * f + W / 2; sy = -P[..., 1] / zz * f + H / 2
    tri2 = np.stack([sx, sy], -1).astype(np.float64)
    zbuf = np.full((H, W), 1e18); img = np.empty((H, W, 3), np.float64); img[:] = BG
    raster(tri2, z.astype(np.float64), Cc.astype(np.float64), W, H, zbuf, img)
    # kenar belirginlestirme: derinlik sureksizligi
    zb = np.where(zbuf > 1e17, np.nan, zbuf)
    dz = np.zeros_like(zbuf)
    for ax in (0, 1):
        d = np.abs(np.diff(np.nan_to_num(zb, nan=1e5), axis=ax))
        if ax == 0:
            dz[1:, :] = np.maximum(dz[1:, :], d)
        else:
            dz[:, 1:] = np.maximum(dz[:, 1:], d)
    edge = dz > 1.5
    img[edge] *= 0.35
    im = Image.fromarray(img.astype(np.uint8))
    if title:
        dr = ImageDraw.Draw(im)
        dr.text((16, 12), title, fill=(20, 20, 20))
    im.save(fname)


def scene_meshes(P, cache, yaw, pitch, lids, hide=()):
    Ms = pose_transforms(yaw, pitch, lids)
    out = []
    for n, p in P.items():
        if any(n.startswith(h) for h in hide):
            continue
        v, t = cache[n]
        M = Ms[p.group]
        vv = v @ M[:3, :3].T + M[:3, 3]
        rgb = [int(p.color[i:i + 2], 16) for i in (1, 3, 5)]
        out.append((vv, t, rgb))
    return out


def main():
    P = build(verbose=False)
    cache = {n: tess(p.shape, 0.05, 0.3) for n, p in P.items()}
    os.makedirs(os.path.join(OUT, "renders"), exist_ok=True)
    R = lambda f: os.path.join(OUT, "renders", f)
    tgt = np.array([0, -8, -20])
    views = [
        ("01_front_open.png", (0, -5, 260), (0, 0, 0), 0, 0, "nominal", ("mask",), "onden (maske gizli), kapaklar nominal acik"),
        ("02_front_mask.png", (0, 0, 260), (0, 0, 0), 0, 0, "nominal", (), "onden, maske ile, nominal"),
        ("03_three_quarter.png", (170, 120, 190), tgt, 0, 0, "nominal", ("mask",), "3/4 on-ust, maske gizli"),
        ("04_lids_closed.png", (60, 40, 230), (0, 0, 0), 0, 0, "closed", ("mask",), "kapaklar kapali"),
        ("05_look_right_up.png", (80, 60, 220), (0, 0, 0), 30, 25, "open", ("mask",), "yaw +30 (robot sagi) pitch +25, kapaklar tam acik"),
        ("06_rear_three_quarter.png", (-170, 110, -210), tgt, -30, -25, "open", ("mask", "base"), "arka 3/4: yaw -30 pitch -25, kapak tam acik"),
        ("07_side_left.png", (-260, 0, -15), (0, -8, -20), 0, -25, "closed", ("mask",), "soldan: pitch -25 (asagi), kapak kapali"),
        ("08_bottom_rear.png", (60, -200, -160), tgt, 30, -25, "open", ("mask", "base"), "alttan-arkadan: pitch mekanizmasi, yaw +30 pitch -25"),
    ]
    for fn, cam, target, y, p, lids, hide, title in views:
        render(scene_meshes(P, cache, y, p, lids, hide), cam, target, R(fn), title=title)
        print("yazildi", fn, flush=True)
    # tek goz yakin (kardan + kanca)
    render(scene_meshes(P, cache, 20, -20, "open", ("mask", "base", "back_frame", "lid_", "servo_lid", "mount_", "link_",
                                                    "horn_lid", "scr_lid", "yaw_coupler", "servo_yaw", "horn_yaw")),
           (-120, -40, -90), (-31.5, -8, -8), R("09_eye_gimbal_closeup.png"), fov=22,
           title="sol goz yakin: kardan, yaw tupu+kolu, pitch kancasi, rotil cubugu (yaw 20, pitch -20)")
    print("yazildi 09", flush=True)


if __name__ == "__main__":
    main()
