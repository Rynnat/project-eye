# Tolerans testi — Ender 3 Neo

Asıl parçaları basmadan önce yazıcının gerçek geçmelerini ölçmek için.
Her test, `params.py`'deki bir tolerans değerini onun etrafında birkaç varyantla dener.
Sen hangisinin oturduğunu söylersin, o değer `params.py`'ye yazılır ve bütün parçalar yeniden üretilir.

## Basılacaklar (`out/`)
| Dosya | Ne test eder | Yaklaşık boyut |
|---|---|---|
| `tolerans_plaka.stl` | M2 vida delikleri, Ø2 pim delikleri, M2 vida başı havşası (5 satır × 5 varyant) | 67 × 51 × 5.6 |
| `tolerans_gobek.stl` + `tolerans_catal.stl` | Kardan göbeği (8 mm küp) ile çatal arasındaki boşluk (4 varyant) | küp 8 mm, tarak 61 × 18 × 12 |
| `tolerans_servo.stl` | SG90 gövde cebi (3 varyant) | 85 × 23 × 6.6 |

Hepsi tek tablaya sığar. Düz yüzleri tablaya bakacak şekilde yönlendirilmiş, destek gerekmez.

## Dilimleyici ayarları (PLA)
- Katman 0.2 mm, duvar (wall) 3, üst/alt 4 katman, doluluk %20, destek yok.
- **Önemli:** "Horizontal Expansion" ve "Hole Horizontal Expansion" (Cura) ya da "XY size compensation" (PrusaSlicer) **0** olsun. Test, yazıcının ham davranışını ölçmeli.
- Asıl parçalarda da **aynı ayarları ve aynı filamenti** kullan; aksi halde test geçersiz olur.

## Gerekenler
- 1–2 adet **M2 silindir başlı vida** (ISO 4762, 6–10 mm). Somun gerekmiyor: tasarımda somun yok.
- **Ø2 çelik pim** (DIN 7 / ISO 2338; yoksa Ø2 matkap ucunun sapı da iş görür)
- 1 adet **SG90**

## Nasıl denenir
Plakada satırlar harfle (A–E, sol kenar), varyantlar numarayla (1–5, üst kenar) işaretli.
Ortadaki varyant (3), `params.py`'deki şu anki değerdir.

| Test | Değerler 1→5 (mm) | Doğru olan |
|---|---|---|
| **A** M2 kılavuzsuz vida deliği | 1.5 1.6 1.7 1.8 1.9 | Vida elle, zorlanarak kendi yolunu açar ve **sıkıca tutar**; plastik çatlamaz |
| **B** M2 geçiş deliği | 2.0 2.1 2.2 2.3 2.4 | Vida **serbestçe geçer**, yanlara neredeyse hiç sallanmaz |
| **C** Ø2 pim, sıkı geçme | 1.85 1.90 1.95 2.00 2.05 | Pim hafif bastırarak ya da hafif çekiçle girer ve **elle çıkmaz** |
| **D** Ø2 pim, dönen geçme | 2.15 2.20 2.25 2.30 2.35 | Plaka pim üzerinde **serbest döner**, boşluk hissedilmez |
| **E** M2 vida başı havşası | 3.9 4.0 4.2 4.4 4.6 | Vida kafası (Ø3.8) **sürtmeden** havşaya iner ve yüzeyle aynı hizada kalır |
| **F** Göbek–çatal (tarak, 1→4) | yuva 8.6 8.8 9.0 9.2 | Küp yuvada **rahat döner**, yana oynaması az |
| **S** SG90 cebi (1→3) | boşluk 0.2 0.3 0.4 | Servo elle **itilince girer**, sallanmaz |

## Raporlama
Her test için doğru oturan varyantı yaz, örneğin:

```
A3 B4 C2 D3 E3 F2 S2
```

İki varyant arasında kaldıysa ikisini de yaz (`A3-4`). Hiçbiri olmadıysa yönünü söyle (`C: 1 bile gevşek`).

## Bilinen sınırlama
Bu testteki delikler **dikey** basılıyor (delik ekseni Z). Yatay basılan deliklerde (ekseni tablaya paralel)
FDM delikleri genelde biraz daha küçük ve oval çıkar. Asıl parçalarda yatay basılması gereken delikler varsa,
onlar için ayrıca küçük bir test gerekebilir. Parçalar netleşince buna bakacağız.
