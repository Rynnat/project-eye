"""Project Eye v3 (POC, M3/M4) - tolerans test parcalari (Creality Ender 3 Neo, 0.4 nozul icin).

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
    ("A", "M3_PILOT",  [2.5, 2.6, 2.7, 2.8, 2.9], "delik"),   # M3 vida plastige kendi yolunu acarak tutunmali
    ("B", "M3_CLEAR",  [3.2, 3.3, 3.4, 3.5, 3.6], "delik"),   # M3 vida serbest gecmeli (kontrol: ust uca pay birakildi)
    ("C", "M4_PILOT",  [3.3, 3.4, 3.5, 3.6, 3.7], "delik"),   # M4 vida plastige tutunmali
    ("D", "M4_RUN",    [4.2, 4.3, 4.4, 4.5, 4.6], "delik"),   # M4 vida PIM olarak: parca uzerinde serbest donmeli (eklemler)
    ("E", "M4_CSK_D",  [8.0, 8.3, 8.6, 8.9, 9.2], "havsa"),   # M4x16 koni kafa: DIN 7991 ~8.0, ISO 10642 ~8.5-9.0 -> genis aralik
]
PITCH_X, PITCH_Y = 13.0, 11.0          # varyantlar arasi / satirlar arasi
X0, Y0 = 14.0, 7.0                    # ilk delik merkezi (sol alt koseden)
PLATE_T = 6.0                         # M3x10 / M4x10 icin yeterli tutunma boyu
CSK_ANGLE = 90.0                      # metrik havsa bas vida (ISO 10642 / DIN 7991) konisi
TEXT_H, TEXT_D = 4.0, 0.6             # kabartma yazi (0.4 nozulda okunur)


BOTTOM_CHAMFER = 0.4   # ilk katman ezilmesi (elephant foot comp. 0) delik agzini daraltir -> tabanda 0.4x45 pah.
                       # Asil parcalarda da AYNI pah kullanilacak ki test gercegi olcsun.


def pah(x, y, d):
    c = BOTTOM_CHAMFER
    return (cq.Workplane("XY").center(x, y).circle(d / 2 + c).workplane(offset=c).circle(d / 2).loft())


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
                p = p.cut(pah(x, y, d))
            else:   # 90 derece koni havsa (ustten) + M4 gecis deligi
                h = d / 2                              # 90 derece: derinlik = yaricap
                koni = (cq.Workplane("XY").workplane(offset=PLATE_T - h).center(x, y)
                        .circle(0.01).workplane(offset=h).circle(d / 2).loft())
                p = p.cut(koni).cut(cq.Workplane("XY").center(x, y).circle(4.4 / 2).extrude(PLATE_T)).cut(pah(x, y, 4.4))
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
# Cep ICOLCULERI dogrudan (boy x en). SG90 klonlari 22.2x11.8 ile 22.8x12.4 arasinda degisiyor:
# eski test hepsini 22.8x12.4'un USTUNE koydugu icin kucuk klonda dort cep de bol cikti (kullanici yakaladi).
SERVO_CEPLER = [(22.2, 11.8), (22.5, 12.1), (22.8, 12.4), (23.1, 12.7)]
WALL, FRAME_H = 2.0, 6.0
CABLE_SLOT_W = 6.0     # SG90 kablosu govdenin mile uzak ucundan, alta yakin cikar -> o uc duvarinda
                       # alttan uste acik yarik; servo kablosuyla birlikte yukaridan kayarak girer


def servo_cepleri():
    x = 0.0
    t = None
    for i, (iw, ih) in enumerate(SERVO_CEPLER):
        ow, oh = iw + 2 * WALL, ih + 2 * WALL
        f = (cq.Workplane("XY").box(ow, oh, FRAME_H, centered=False)
             .cut(cq.Workplane("XY").center(WALL + iw / 2, WALL + ih / 2).rect(iw, ih).extrude(FRAME_H))
             # kablo yarigi: sag kisa duvar boyunca, tam yukseklik
             .cut(cq.Workplane("XY").center(WALL + iw + WALL / 2, WALL + ih / 2).rect(WALL + 0.2, CABLE_SLOT_W).extrude(FRAME_H)))
        TAB_H = 1.2
        f = f.union(cq.Workplane("XY").box(ow, 6.0, TAB_H, centered=False).translate((0, oh, 0)))   # yazi dili
        f = f.union(cq.Workplane("XY").workplane(offset=TAB_H).center(ow / 2, oh + 3.0)
                    .text(str(i + 1), 3.5, TEXT_D, kind="bold", halign="center", valign="center"))
        f = f.translate((x, 0, 0))
        t = f if t is None else t.union(f)
        x += ow + 2.0
    # tek govde: alt duvar hizasinda, aradaki bosluklari kapatan ince serit
    t = t.union(cq.Workplane("XY").box(x - 2.0, WALL, 1.2, centered=False))
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
