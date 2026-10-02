# Project Eye v4 (v3 revizyonu) — malzeme listesi

Sayılar `eye_v4.py` montaj kaydından (`check.py §8`). Ender 3 Neo, 0.4 nozül, PLA, 0.2 mm.
Toleranslar tolerans testiyle **henüz tam doğrulanmadı** (servo cebi ölçüsü kullanıcı testinden: boy +0.4, en +0.2).

## 1. Basılacak parçalar (`out/stl/`, baskı yönüne çevrilmiş, tabla z=0) — 16 farklı STL, 19 baskı

| STL | Adet | Boyut (mm) | Baskı yönü | Destek (check §7) |
|---|---|---|---|---|
| `eye` | 2 | 36×28×29 | alt düz yüz (y −13) tablada | Yok |
| `eye_lever` | 2 | 13×31×3.5 | düz | Yok |
| `coupler` (yaw laması, ortada geriye kıvrık) | 1 | 104×69×3.5 | düz | Yok |
| `yaw_crank` | **2** | 26×33×3.5 | düz | Yok. **Biri yaw, biri LIDS krankı** |
| `link_up` / `link_lo` (kapak lamaları) | 1+1 | 19×26×3.5 / 37×16×4.5 | düz | Yok |
| `frame_L`, `frame_R` (göz kafesleri) | 1+1 | 86×55×78 / 80×55×78 | **ön yüz tablada** | Yok (sarkma 120 / 320 mm²: frame_R'de LIDS servo kanadının alt kenarı ~28 mm köprü) |
| `yaw_mount` | 1 | 63×37×30 | plaka üst yüzü tablada, bacaklar yukarı | Yok |
| `lid_up_L`, `lid_up_R` (üst kapaklar + orta çubuk) | 1+1 | 40×21×90 / 41×21×68 | dış menteşe göbeği tablada (X dik) | **Destek önerilir** (kutup kubbesi + dudak altı, ~450 mm²). Havada ada yok. Brim önerilir |
| `lid_lo_L`, `lid_lo_R` (alt kapaklar + orta çubuk) | 1+1 | 26×26×88 / 26×20×76 | dış menteşe göbeği tablada | **Destek önerilir** (~300 mm²). Brim önerilir |
| `pivot_bracket` (sol pitch yatağı) | 1 | 11×40×62 | ayak tablada | Yok. Yatak deliği yatay (+0.2) |
| `base` | 1 | 212×89×74 | taban tablada | Yok |
| `mask` | 1 | 212×92×30 | **ön yüz tablada** | Yok |

Hepsi tek gövde, havada başlayan ada yok (`check.py §7`). En büyük parçalar (base, mask) 212 mm: 220 tablaya sığar.

## 2. Vidalar (eldeki malzeme)

| Vida | Adet | Nerede |
|---|---|---|
| **M4×10 yuvarlak** | **13** | göz kolu uçları 2 + yaw krank ucu 1 (lama pimleri, kafa lamanın altında); **kapak menteşeleri 4** (kafa üst kapak göbeğinin cebinde, alt kapaktan geçip kafes plakasına vidalı); sol pitch pimi 1; kapak lama pimleri 2 (U: üst kapakta sabit, L: alt kapakta sabit); orta bindirme 3 (üst 1, alt 2; radyal) |
| **M4×16 havşa** | **13** | göz üst pimi 2, alt pimi 2 (kafeste sabit, göz döner); yaw_mount ↔ kiriş 4 (kirişin altından); pivot_bracket ↔ base 2, maske ↔ base 2 (base'in altından); LIDS krank ucu pimi 1 (alt lamanın havşasında, krankta sabit) |
| **M3×10 yuvarlak** | **6 / 14** ✅ | göz kolu kilidi 2 (alttan, göze); SG90 kulağı 3 (her servoda **mil tarafı** kulak; kablo tarafı kulak kablo yarığına denk geliyor, vida yok — DECISIONS §2.3); orta üst bindirme 1 |
| SG90'ın küçük vidaları | 6 | servo kolu ↔ basılı parça (pitch: frame_R dış plakası; yaw ve LIDS: yaw_crank), göbeğe en yakın delikler (r≈7), ≤ 5 mm |

M2 / somun / pim / rotil / tel **kullanılmıyor**. Kulak delikleri 3.2 mm matkapla büyütülür.

## 3. Satın alınan / eldeki
- SG90 × 3 (kanal 0 EYE_YAW, 1 EYE_PITCH, 2 LIDS) + çift kollu kolları. Arduino Uno, 5 V ≥ 2 A.

## 4. Montaj sırası (önemli: kapaklar gözden ÖNCE, göz ARKADAN)
1. Kulak deliklerini 3.2 mm aç. Servoları 90°'ye getir (kol takmadan).
2. **Kollar:** pitch servosunun kolunu frame_R dış plakasına, yaw ve LIDS kollarını birer yaw_crank'e küçük vidalarla bağla (vida kafaları servo tarafında).
3. **Kapaklar (her göz için, kafes boşken):** alt kapak göbeğini kafes plakasının iç yüzüne, üst kapak göbeğini onun iç yüzüne hizala; M4×10'u **göz tarafından** üst göbeğin cebine koyup plakaya vidala (iki kutup). Kapaklar elle serbest dönene kadar geri al. Sol ve sağ kapak parçalarını ortada bindir: üst bindirmeye M4×10 + M3×10, alta 2× M4×10 (dışarıdan, radyal).
4. **Gözler:** kapaklar açıkken gözü **arkadan** kafese kaydır (üst düz yüz üst köprü göbeğinin altından geçer). Üst ve alt M4×16 havşa pimleri tak (göz serbest dönmeli; sıkıysa gözdeki RUN deliği 4.5). Göz kolunu alttan M3×10 ile göze kilitle (pimin arkasında).
5. **Kafesleri birleştir:** yaw_mount'u iki kafesin kirişine alttan 4× M4×16 ile bağla; yaw servosunu yaw_mount'a M3 ile as, yaw krankını (kollu) mile tak.
6. Yaw lamasını göz kollarına ve kranka M4×10 ile bağla (kafa lamanın altında).
7. **LIDS:** servoyu sağ kafesin kanadına cebinden geçir, mil tarafı kulağı M3 ile vidala; krankı (kollu) mile tak (kapaklar kapalıyken servo 102.3° — kinematics.json). Üst lamayı üst kapak pimine (M4×10), alt lamayı alt kapak pimine (M4×10) tak; iki lamayı krank ucuna M4×16 havşa ile (alt lamanın havşasından) bağla.
8. **Çerçeveyi base'e indir:** frame_R dış plakasındaki kolu pitch servosunun miline geçir (merkez vidası yok — DECISIONS §6.4). pivot_bracket'ı sol dış plakanın göbeğine oturt, alttan 2× M4×16 ile base'e, M4×10'u dıştan yataktan geçirip göbeğe vidala.
9. Maskeyi alttan 2× M4×16 ile base'e vidala.
10. `out/kinematics.json → settings_suggestion` ile başla. **LIDS aralığı v3'ten farklı: kapalı 102.3°, açık 77.7°.** `lid_follow_pitch` ayarını 0'a yakın başlat (kapaklar pitch'i mekanik izliyor).
