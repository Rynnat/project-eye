# Project Eye v3 (POC) — malzeme listesi

Sayılar `eye_v3.py` montaj kaydından (`check.py §8`). Ender 3 Neo, 0.4 nozül, "Epic POC" profili (0.2 mm, destek yok).
Toleranslar tolerans testiyle **henüz doğrulanmadı** → test sonucu `params.py §1`'e girilip her şey yeniden üretilecek.

## 1. Basılacak parçalar (`out/stl/`, baskı yönüne çevrilmiş, tabla z=0)

| STL | Adet | Boyut (mm) | Baskı yönü | Destek / sarkma (check §7) |
|---|---|---|---|---|
| `eye` | 2 | 36×28×26 | alt düz yüz tablada | Yok. 24 mm² = delik tavanları |
| `eye_lever` | 2 | 13×39×3.5 | üst yüz tablada (havşa yukarı) | Yok |
| `coupler` (yaw laması) | 1 | 104×51×3.5 | düz | Yok |
| `yaw_crank` | 1 | 33×33×3.5 | düz | Yok |
| `frame_L`, `frame_R` (aynalı) | 1+1 | 62×48×71 | yan plakanın dış yüzü tablada | **Yok**, ama ~87 mm² sarkma: üst kolun küre kabuğu dış yüzü ~47° (sınırda) + delik tavanları. Üst kolun alt kenarında hafif pürüz olabilir. |
| `yaw_mount` | 1 | 39×44×9 | plaka üst yüzü tablada | Yok (35 mm²: havşa konileri 45°) |
| `lid_L`, `lid_R` (aynalı) | 1+1 | 34×29×30 | kapağın iç uç yüzü (takviye) tablada | Yok (19 mm² = pilot tavanları). Tabla teması küçük (238 mm²): **brim önerilir** |
| `lid_carrier` | 1 | 65×18×44 | üst köprünün üst yüzü tablada | Yok (82 mm²: 45° kenarlı göbek + delik tavanları) |
| `pivot_bracket` (sol pitch yatağı) | 1 | 22×20×58 | ayak tablada | Yok. Yatak deliği yatay (+0.2) |
| `base` | 1 | 181×52×70 | taban tablada | Yok. 160 mm² = servo cebi tavanları (12.7 mm köprü) + kapak servosu penceresinin **30° eğik üst kenarı** (hafif sarkabilir, temizleyin) |

12 farklı STL, 14 baskı. Hepsi tek gövde, havada başlayan ada yok (`check.py §7`).

## 2. Vidalar (eldeki malzeme)

| Vida | Adet | Nerede |
|---|---|---|
| **M4×16 havşa** | **12** | göz üst pimi 2, göz alt pimi 2 (çerçevede sabit, göz döner); göz kolu kilidi 2 (alttan gözün önüne); yaw_mount → arka kiriş 4; base → pivot_bracket 2 (alttan) |
| **M4×10 yuvarlak** | **8** | sol pitch pimi 1 (çerçevede sabit, yatakta döner); lama pimleri 3 (2 göz kolu + krank; kafa lamanın altında); kapak → taşıyıcı 4 |
| **M3×10 yuvarlak** | **6 / 14** ✅ | SG90 kulakları, 3 servo × 2 (kulak delikleri **3.2 mm matkapla** büyütülür) |
| SG90'ın kendi küçük vidaları | 6 | servo kolu ↔ basılı parça (pitch: frame_R, yaw: yaw_crank, kapak: lid_carrier), kolun göbeğe en yakın delikleri. **≤ 5 mm** olmalı (yaw krankında uzun vida lamaya değer) |
| SG90'ın kol merkez vidası | 3 | kolu mile; basılı parçadaki Ø7 delikten sıkılır |

M2 / somun / pim / rotil / tel **kullanılmıyor**.

## 3. Satın alınan / eldeki
- SG90 × 3 (kanal 0 EYE_YAW, 1 EYE_PITCH, 2 LIDS) + her birinin çift kollu kolu.
- Arduino Uno, 5 V ≥ 2 A güç (elektronik kolu).

## 4. Montaj sırası (kısa)
1. SG90 kulak deliklerini 3.2 mm aç. Servoları 90°'ye getir (kol takmadan).
2. **Kollar:** her servo kolunu basılı parçasına (frame_R dış yüzü, yaw_crank, lid_carrier göbeği) servonun küçük vidalarıyla, vida kafaları kolun SERVO tarafında kalacak şekilde vidala.
3. **Base:** kapak servosunu orta direğe, pitch servosunu sağ tutucuya M3 ile vidala. Kapak taşıyıcısını (kolu takılı) kapak servosunun miline tak, merkez vidayı taşıyıcıdaki Ø7 delikten sağ taraftan sık (bu adımda göz/çerçeve yok, erişim serbest). Kapak kapalıyken servo 112.5° olmalı (kinematics.json).
4. **Çerçeve:** iki yarıyı yaw_mount ile birleştir (4× M4×16), yaw servosunu yaw_mount'a M3 ile as, yaw krankını (kolu takılı) mile tak ve sık (krank −Z'ye, arkaya bakmalı).
5. Çerçeveyi base'e indir: sağ yan plakadaki kolu pitch servosunun miline geçir; merkez vidayı yan plakadaki Ø7 delikten **içeriden kısa (stubby) tornavidayla** sık (gözler henüz yok). Sonra **pivot_bracket**'ı sol yan plakanın dışına oturt, alttan 2× M4×16 havşa ile base'e vidala, M4×10'u dıştan yataktan geçirip yan plakaya vidala.
6. **Gözler:** yaw kolunu gözün altına M4×16 havşa ile (kolun ön ucundan) kilitle. Gözü çerçeve yarısının üst/alt kolları arasına koy, üst ve alt M4×16 havşa pimleri çerçeveye vidala (göz serbest dönmeli; sıkıysa gözdeki RUN deliğini 4.5 matkapla aç).
7. Lamayı göz kollarına ve kranka M4×10 ile bağla (kafa lamanın altında, vida kol/krankta sabit).
8. Kapakları taşıyıcının kulaklarına 2'şer M4×10 ile vidala (en son).
9. `out/kinematics.json → settings_suggestion` ile başla; yönleri donanımda doğrula (EYE_YAW modelde ters: `invert: true`).
