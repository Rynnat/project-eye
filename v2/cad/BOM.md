# Project Eye v2 — Mekanik malzeme listesi (BOM)

Kaynak etiketleri: **[KAYNAK]** datasheet/standart, **[SEÇİM]** bu tasarımın kararı, **[VARSAYIM]** tedarik edilen
parçada ölçülmeli. Gerekçeler `DECISIONS.md`'de. Sayılar `eye_v2.py` montaj kaydından sayıldı (elle değil).

## 1. Basılacak parçalar (`out/stl/`, baskı yönüne döndürülmüş)

Yazıcı: Ender 3 Neo, 0.4 nozul. Toleranslar henüz **test baskısıyla doğrulanmadı**; `cad/tolerance/` sonucu gelince
`params.py [TOL]` değerleri güncellenip her şey yeniden üretilecek.

| STL | Adet | Malzeme [SEÇİM] | Hacim | Baskı yönü | Destek |
|---|---|---|---|---|---|
| `eye_shell_L`, `eye_shell_R` (aynalı) | 1+1 | PLA beyaz | 2.4 cm³ | kenar düzlemi tablada, ön kutup yukarıda | Hayır. İç kubbe tavanı köprülenir (görünmez yüzey, çatal sapı buraya yapışır) |
| `eye_fork` | 2 | PETG | 0.3 | kollar tablada, sap yukarı | Hayır (9 mm köprü) |
| `gobek` | 2 | PETG | 0.5 | herhangi yüz | Hayır |
| `fixed_fork` | 2 | PETG | 1.3 | flanş tablada | Küçük ağaç destek önerilir (kolların köprüden 1.7 mm taşan altları) |
| `yaw_lever` | 2 | PETG | 0.4 | kol plakası tablada, tüp yukarı | Hayır |
| `lid_upper_L/R`, `lid_lower_L/R` (aynalı) | 1+1+1+1 | PLA ten rengi | 0.5–0.7 | pim ekseni dik, dış göbek tablada | Evet, az (ağaç). ~60–70 mm² sarkma (kubbe dilimi uçları, göbek diski) |
| `lid_link_upper`, `lid_link_lower` | 2+2 | PETG | 0.5 | düz | Hayır |
| `yaw_coupler` | 1 | PETG | 0.8 | düz | Hayır |
| `rocker` | 1 | PETG | 5.5 | düz tarak (kol yüzü tablada) | Hayır |
| `pitch_link_L`, `pitch_link_R` (aynalı) | 1+1 | PETG | 0.5 | göz ucu yuva ağzı aşağı | Evet (ağaç, yuvaların altı). Geçmeli yuva: **deneme gerekir** [DOĞRULANMADI] |
| `back_frame` | 1 | PETG | 16.5 | kiriş arka yüzü tablada | Hayır (raf penceresi ve havşa tavanları köprü) |
| `mount_lid_servos_L/R` | 1+1 | PETG | 5.6 | plaka tablada | Hayır |
| `mount_pitch_servo` | 1 | PETG | 3.0 | plaka tablada | Hayır |
| `rocker_bearing` | 1 | PETG | 1.0 | plaka tablada | Hayır |
| `mask` | 1 | PLA | 25.1 | ön yüz tablada | Hayır |
| `base` | 1 | PETG | 58.3 | alt yüz tablada (158×93) | Hayır (havşa tavanları köprü) |

Toplam ≈ 137 cm³ katı hacim (dolgu hariç). Sarkma alanları `check_report.txt §6`'dan.

**Baskı ayarı önerisi [SEÇİM, doğrulanmadı]:** hareketli ve vidalı parçalar PETG, 0.16–0.2 mm katman, 4 çevre,
%40 gyroid. Göz ve kapaklar 0.12 mm katman (küre yüzeyi için). Küçük delikleri (Ø1.7/Ø2.2) matkap ucuyla
(1.6/2.1) temizleyin.

## 2. Satın alınan parçalar

| Parça | Adet | Nerede | Etiket |
|---|---|---|---|
| SG90 mikro servo | 6 | kanal 0–5 | [KAYNAK] SPEC §4 |
| SG90 tek kollu servo kolu (servoyla gelir) | 5 | 4 kapak + yaw | r=10 deliği **Ø1.6'ya büyütülür** (M2 vida) [VARSAYIM: r=10 deliği var] |
| SG90 çapraz kol (servoyla gelir) + 2 küçük kol vidası | 1 | pitch servosu → rocker flanşı | [VARSAYIM] |
| RC rotil topu, Ø4.8 top, M2 saplama (4 mm diş) | 4 | 2 göz kancası + 2 rocker kolu | [VARSAYIM] top merkezi yakadan 4.9 mm; farklıysa `BALL_H` güncellenir |
| Ø2×12 çelik pim (DIN 7 / ISO 2338 m6) | 4 | kapak pimleri (kanatlara sıkı geçme) | [KAYNAK] standart |

## 3. Vidalar (hepsi M2, ISO 4762 silindir başlı; tek istisna bebek vidası)

| Ölçü | Adet | Kullanım |
|---|---|---|
| M2×4 DIN 965 **siyah havşa başlı** | 2 | göz çatalı sapı → kabuk (vida başı = bebek) |
| M2×4 | 6 | kapak bağlantı kolu eklemleri (4 servo ucu + 2 üst kapak kol ucu) |
| M2×5 | 23 | kardan yatay (4) + sabit çatal alt (2); paralelkenar eklemleri (3); servo kulakları (12); alt kapak kol ucu (2) |
| M2×6 | 12 | tabandan tutuculara (alttan havşalı) |
| M2×8 | 5 | direk flanşları → kiriş (4), rocker yatağı (1) |
| M2×16 | 2 | yaw kolu → göbek (tüp içinden) |
| **Toplam** | **50** | somun yok: tüm vidalar plastiğe kılavuzsuz (Ø1.7) veya geçiş (Ø2.2) |

## 4. Sarf

- Siyanoakrilat yapıştırıcı: göz çatalı sapı ↔ kabuk iç kutbu (vidaya ek, dönmeyi önler)
- Orta güç diş sabitleyici (plastik uyumlu) veya bir damla CA: eklem vidaları (M2×4/M2×5) gevşemesin
- İris boyası/çıkartması (Ø12), isteğe bağlı erişim deliği tapaları/boya
- Kablo, 5 V ≥ 3 A güç kaynağı: elektronik iş kolu (SPEC §4)

## 5. Montaj sırası (kısa)

1. Göz: çatalı kabuğa sok, bebek vidası + CA. Göbeği içeri koy, yan erişim deliklerinden 2×M2×5 (çatal kollarına vidalanır, göbek serbest döner).
2. Sabit çatalı arkadan eğik sok (kafa 8.4×13, açıklık Ø16.4), alt erişim deliğinden M2×5. Yaw kolu tüpünü üst yarıktan göbeğe oturt, M2×16 ile sık.
3. Direk flanşlarını kirişe 2×M2×8 ile (arkadan) bağla.
4. Kapakları önden göze geçir: **önce alt kapak** (iç göbek), sonra üst kapak (kulağı alt göbeğin üstünden geçer). Ø2 pimleri kanatlardan çak. **İç pimler yaw servosundan önce** takılmalı (servo iki iç kanat arasına 0.5 mm boşlukla girer).
5. Servoları orta konuma (90°) al, **sonra** kolları takılacak yönde tak: yaw kolu −Z'ye, kapak kolları kinematics.json nötr açısına (kol bağlantıya dik), rocker kolu yukarı.
6. Paralelkenar çubuğunu 3×M2×5 ile, pitch çubuklarını rotil toplarına bastırarak, kapak bağlantı kollarını M2×4/5 ile bağla.
7. `calibrate.py` ile kanal kanal sınırları bul; başlangıç için `out/kinematics.json → settings_suggestion`.
