# Tolerans testi — Project Eye v3 (POC, M3/M4) · Ender 3 Neo · OrcaSlicer

POC eldeki vidalarla yapılıyor: **M4×10 yuvarlak kafa, M4×16 havşa (koni) kafa, M3×10 yuvarlak kafa.**
Bu test, bu vidaların senin yazıcında hangi delik ölçülerine oturduğunu bulur. Test bağımsız bir kontrolden geçti ve düzeltildi.

## Basılacaklar
| Dosya | Ne test eder | Öncelik |
|---|---|---|
| `tolerans_plaka.stl` | A–E: M3/M4 delikleri, M4 vida-pim eklemi, M4 havşa (75 × 63 × 6.6 mm) | **Asıl test** |
| `tolerans_servo_tek.stl` | S: tek SG90 cebi, 22.9 × 12.9 (ölçtüğün servo 22.5 × 12.5 + 0.4 pay), kablo yarıklı | Önemli |
| `tolerans_gobek.stl` + `tolerans_catal.stl` | F: iki düz yüz arasındaki kayma boşluğu | İsteğe bağlı (aşağıya bak) |

## Dilimleyici (OrcaSlicer, "Epic POC" profili)
- Profil zaten doğru: 0.2 mm, 3 duvar, %20, destek yok, **X-Y hole compensation 0, X-Y contour compensation 0, Elephant foot compensation 0, polyholes kapalı.**
- Asıl parçalar da **aynı profil ve aynı filamentle** basılacak.

## Denemeden önce
1. **Deliklerin alt ağzı:** Plakadaki deliklerin altında küçük bir pah var. Yine de alt ağızda çapak varsa bir çakıyla temizle.
2. **Vidalar hep yazılı (üst) yüzden girsin.**
3. **Kumpasın varsa ölç:** M4 koni kafanın **kafa çapı** ve SG90 gövdesinin **en ve boyu** (datasheet: 22.8 × 12.4).

## Nasıl karar verilir
Plakada satırlar harfle (A–E, sol kenar), varyantlar numarayla (1–5, üst kenar) işaretli.

| Test | 1 → 5 (mm) | Doğru olan |
|---|---|---|
| **A** M3 vidalanan delik | 2.5 2.6 2.7 2.8 2.9 | Vidayı sonuna kadar sık. Sıkınca **boşa dönmeyen EN BÜYÜK** delik. Söküp tekrar tak, ikinci takışta da tutmalı. |
| **B** M3 geçiş deliği | 3.2 3.3 3.4 3.5 3.6 | Vida elle, **dişi takılmadan geçen EN KÜÇÜK** delik |
| **C** M4 vidalanan delik | 3.3 3.4 3.5 3.6 3.7 | A ile aynı kural, M4 vidayla: **boşa dönmeyen EN BÜYÜK**, ikinci takışta da tutan |
| **D** M4 vida-pim eklemi ⭐ | 4.2 4.3 4.4 4.5 4.6 | M4 vidayı geçir. **Vida parmakla sürtünmeden dönen EN KÜÇÜK** delik. |
| **E** M4 havşa | 8.0 8.3 8.6 8.9 9.2 | M4×16 koni kafa **yüzeyle aynı hizaya ya da biraz altına inen EN KÜÇÜK** havşa |
| **S** SG90 cebi (tek) | cep 22.9 × 12.9 | Rapor: **"S sıkı"** (girmiyor/zorla giriyor), **"S tam"** (elle itilince giriyor, sallanmıyor) ya da **"S bol"** (sallanıyor) |
| **F** (isteğe bağlı, tarak 1→4) | yuva 8.6 8.8 9.0 9.2 | Küp yuvaya **sürtünmeden kayan EN DAR** yuva |

**F hakkında:** Bu test sadece iki düz yüz arasındaki kayma boşluğunu ölçüyor. POC'deki vida-pim eklemlerinde bu boşluğu daha çok vidanın ne kadar sıkıldığı belirler. Zamanın darsa F'yi atlayabilirsin, varsayılan 0.4 mm ile devam ederiz.

## Rapor
Tek satır, her test için doğru olan numara:
```
A3 B2 C3 D4 E3 S2 F2
```
- Arada kaldıysan: `A3-4`
- Hiçbiri olmadıysa yönünü yaz: `D: 5 bile sıkı`, `E: 1 bile gömülüyor`
- Ölçtüysen ekle: `M4 kafa 8.4`, `SG90 22.9 × 12.5`
