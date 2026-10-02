"""Project Eye v2 - tolerans test parcalari (Creality Ender 3 Neo, 0.4 nozul icin).

Asil parcalari basmadan once yazicinin gercek gecmelerini olcmek icin. Her satir
params.py'deki bir [TOL] degerini, o degerin etrafinda 5 varyantla dener.
Kullanici hangi varyantin dogru oturdugunu raporlar; o deger params.py'ye yazilir
ve tum parcalar yeniden uretilir.

Cikti: out/*.stl (baski yonunde, duz yuz tablada) + out/onizleme.png
Calistirma:
  C:\\Users\\LENOVO\\epic-project\\.venv-cad\\Scripts\\python.exe tolerance_test.py
"""
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"
OUT.mkdir(exist_ok=True)

# ---- denenecek varyantlar (ortadaki = params.py'deki mevcut deger) ----
ROWS = [
    # harf, params adi, varyantlar (mm), sekil
    ("A", "M2_PILOT",   [1.5, 1.6, 1.7, 1.8, 1.9],     "delik"),   # M2 vida plastige kendi yolunu acarak tutunmali
    ("B", "M2_CLEAR",   [2.0, 2.1, 2.2, 2.3, 2.4],     "delik"),   # M2 vida serbest gecmeli, sallanmamali
    ("C", "PIN_PRESS",  [1.85, 1.90, 1.95, 2.00, 2.05], "delik"),  # O2 celik pim sikica (cekicle hafif) girmeli
    ("D", "PIN_RUN",    [2.15, 2.20, 2.25, 2.30, 2.35], "delik"),  # O2 pim serbest donmeli, boslugu hissedilmemeli
    ("E", "CBORE_D",    [3.9, 4.0, 4.2, 4.4, 4.6],     "havsa"),   # M2 silindir bas vida kafasi havsaya oturmali (kafa O3.8)
]
PITCH_X, PITCH_Y = 11.0, 8.0          # varyantlar arasi / satirlar arasi
X0, Y0 = 14.0, 7.0                    # ilk delik merkezi (sol alt koseden)
PLATE_T = 5.0                         # delik derinligi = plaka kalinligi (M2x5 vidayla ayni)
CBORE_DEPTH = 2.2                     # ISO 4762 M2 kafa yuksekligi 2.0 + 0.2
TEXT_H, TEXT_D = 4.0, 0.6             # kabartma yazi (0.4 nozulda okunur)


def plaka():
    w = X0 + PITCH_X * 4 + 9
    h = Y0 + PITCH_Y * (len(ROWS) - 1) + 12
    p = cq.Workplane("XY").box(w, h, PLATE_T, centered=False)
    for r, (harf, _ad, varyant, sekil) in enumerate(ROWS):
        y = Y0 + PITCH_Y * r
        for c, d in enumerate(varyant):
            x = X0 + PITCH_X * c
            if sekil == "delik":
                p = p.cut(cq.Workplane("XY").center(x, y).circle(d / 2).extrude(PLATE_T))
            else:   # havsa: ustten silindirik cep (vida kafasi) + alttan M2 gecis deligi (2.2)
                cep = (cq.Workplane("XY").workplane(offset=PLATE_T - CBORE_DEPTH).center(x, y)
                       .circle(d / 2).extrude(CBORE_DEPTH))
                p = p.cut(cep).cut(cq.Workplane("XY").center(x, y).circle(1.1).extrude(PLATE_T))
        # satir harfi (sol kenar)
        p = p.union(cq.Workplane("XY").workplane(offset=PLATE_T).center(4.5, y)
                    .text(harf, TEXT_H, TEXT_D, kind="bold", halign="center", valign="center"))
    # sutun numaralari (ust kenar)
    ust = Y0 + PITCH_Y * (len(ROWS) - 1) + 7
    for c in range(5):
        p = p.union(cq.Workplane("XY").workplane(offset=PLATE_T).center(X0 + PITCH_X * c, ust)
                    .text(str(c + 1), TEXT_H, TEXT_D, kind="bold", halign="center", valign="center"))
    return p


# ---- kardan gobegi: 8 mm kup + catal araliklari ----
HUB = 8.0
SLOTS = [8.6, 8.8, 9.0, 9.2]          # her yana bosluk 0.3 / 0.4 / 0.5 / 0.6 (params AXIAL_GAP=0.4 -> 8.8)
ARM_T = 2.0                           # catal kolu kalinligi (params ile ayni)


def gobek():
    k = cq.Workplane("XY").box(HUB, HUB, HUB)
    for yon in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)):   # 4 yanda M2 pilot, 3 derin
        k = k.cut(cq.Workplane(cq.Plane(origin=tuple(4.0 * v for v in yon), xDir=(0, 0, 1),
                                        normal=tuple(-v for v in yon))).circle(1.7 / 2).extrude(3.0))
    return k.translate((0, 0, HUB / 2))


def catal_tarak():
    """Tabanda 4 U-yuva; kup her yuvaya ayri ayri sokulup cevrilerek denenir."""
    taban_t, yuk = 2.0, 10.0
    x = 0.0
    parcalar = []
    for i, s in enumerate(SLOTS):
        g = s + 2 * ARM_T
        u = (cq.Workplane("XY").box(g, 12.0, taban_t + yuk, centered=(False, True, False))
             .cut(cq.Workplane("XY").workplane(offset=taban_t).center(ARM_T + s / 2, 0).rect(s, 12.0).extrude(yuk)))
        parcalar.append(u.translate((x, 0, 0)))
        x += g + 3.0
    t = parcalar[0]
    for q in parcalar[1:]:
        t = t.union(q)
    # yuvalari bir kopru ile birlestir ve numarala (kopru uzerine kabartma)
    kopru = cq.Workplane("XY").box(x - 3.0, 6.0, taban_t, centered=False).translate((0, 6.0, 0))
    t = t.union(kopru)
    xx = 0.0
    for i, s in enumerate(SLOTS):
        g = s + 2 * ARM_T
        t = t.union(cq.Workplane("XY").workplane(offset=taban_t).center(xx + g / 2, 9.0)
                    .text(str(i + 1), 3.5, TEXT_D, kind="bold", halign="center", valign="center"))
        xx += g + 3.0
    return t


# ---- SG90 cepleri ----
SG_BODY = (22.8, 12.4)                # [KAYNAK] Luxorparts datasheet
SERVO_CLR = [0.2, 0.3, 0.4]           # toplam bosluk (params SERVO_POCKET_CLR=0.3)
WALL, FRAME_H = 2.0, 6.0


def servo_cepleri():
    x = 0.0
    t = None
    for i, c in enumerate(SERVO_CLR):
        iw, ih = SG_BODY[0] + c, SG_BODY[1] + c
        ow, oh = iw + 2 * WALL, ih + 2 * WALL
        f = (cq.Workplane("XY").box(ow, oh, FRAME_H, centered=False)
             .cut(cq.Workplane("XY").center(WALL + iw / 2, WALL + ih / 2).rect(iw, ih).extrude(FRAME_H)))
        f = f.union(cq.Workplane("XY").workplane(offset=FRAME_H).center(ow / 2, oh + 3.0)
                    .text(str(i + 1), 3.5, TEXT_D, kind="bold", halign="center", valign="center"))
        f = f.union(cq.Workplane("XY").box(ow, 6.0, 1.2, centered=False).translate((0, oh, 0)))   # yazi dili
        f = f.translate((x, 0, 0))
        t = f if t is None else t.union(f)
        x += ow + 2.0
    return t


PARCALAR = {
    "tolerans_plaka": plaka,
    "tolerans_gobek": gobek,
    "tolerans_catal": catal_tarak,
    "tolerans_servo": servo_cepleri,
}

if __name__ == "__main__":
    for ad, fn in PARCALAR.items():
        sekil = fn()
        v = sekil.val()
        assert v.isValid(), f"{ad}: gecersiz kati"
        bb = v.BoundingBox()
        cq.exporters.export(sekil, str(OUT / f"{ad}.stl"), tolerance=0.01, angularTolerance=0.1)
        print(f"{ad:16s} {bb.xlen:6.1f} x {bb.ylen:5.1f} x {bb.zlen:4.1f} mm  hacim {v.Volume():7.0f} mm3")
