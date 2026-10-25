"""D5: Z8 zinciri - rapor sayilarini yeniden uret; denetcinin 3 ek halkasi icin duyarlilik (halka degerleri olculmedi)."""
import math
mevcut = [("braket ayak z", .15), ("sol servo cep", .10), ("MG90S cep", .10), ("adaptor merkez", .25), ("frame_L/R es eksen", .10), ("kol-mil", .07)]
izin = 0.5
def hesap(h):
    return izin - sum(t for _, t in h), izin - math.sqrt(sum(t * t for _, t in h))
print("rapordaki zincir: en kotu %.3f, RSS %.3f (rapor: -0.270 / 0.154)" % hesap(mevcut))
# (iii) MG90S cep: TowerPro 22.8x12.2 -> cep 12.7: yarim bosluk 0.25 (modeldeki 0.10 yerine). SG90 sol: kullanici olcumu 12.5 -> 0.10 kalir
g = [x for x in mevcut if x[0] != "MG90S cep"] + [("MG90S cep (12.2 datasheet)", .25)]
print("sadece (iii) MG90S cep duzeltmesi: en kotu %.3f, RSS %.3f" % hesap(g))
for ti, tii in ((0.15, 0.15), (0.2, 0.2), (0.3, 0.3)):
    print("(i) MG90S mil/govde +-%.2f, (ii) yapistirma +-%.2f ekle: en kotu %.3f, RSS %.3f (negatif = 0.5 izni asilir)" % ((ti, tii) + hesap(g + [("mil", ti), ("yapistirma", tii)])))
print("NOT: (i) ve (ii) halkalari olculmedi; sonuc yalniz kumpas olcumuyle (MONTAJ T9 oncesi) kesinlesir.")
