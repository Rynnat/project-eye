# Project Eye v5 (v4 + somunlu bağlantılar) — malzeme listesi

Sayılar `eye_v5.py` montaj kaydından (`check.py §8`). Ender 3 Neo, 0.4 nozül, PLA, 0.2 mm.
Servo cebi 22.9 × 12.7 (SG_L 22.5 + 0.4, SG_W 12.5 + 0.2). Somun cebi AF + 0.3, derinlik + 0.2 ([TOL], test edilmedi).

## 1. Basılacak parçalar (`out/stl/`) — 16 farklı STL, 19 baskı

| STL | Adet | Boyut (mm) | Baskı yönü | Not |
|---|---|---|---|---|
| `eye` | 2 | 36×32×29 | alt düz yüz tablada | M3 somun cebi (arka düzden kaydırmalı) |
| `eye_lever` | 2 | 13×32×5 | **alt yüz tablada** (v4: üst) | uç göbeğinde üstten açık M4 somun cebi |
| `coupler` | 1 | 104×69×3.5 | düz | |
| `yaw_crank` | **2** | 26×34×5 | alt yüz tablada | biri yaw, biri LIDS krankı; uçta M4 somun cebi |
| `link_up` / `link_lo` | 1+1 | 19×26×3.5 / 37×16×4.5 | düz | |
| `frame_L`, `frame_R` | 1+1 | 88×57×78 / 83×57×78 | ön yüz tablada | somun cepleri ön yüze (tablaya) açık kanal; sarkma 230 / 440 mm² |
| `yaw_mount` | 1 | 69×35×30 | plaka üstü tablada | bacaklarda dışa açık M4 somun kanalı |
| `lid_up_L/R`, `lid_lo_L/R` | 1+1+1+1 | ~40×21×90 … | dış menteşe göbeği tablada | **destek önerilir** (kubbe altı 280–460 mm²), brim |
| `pivot_bracket` | 1 | 11×40×62 | ayak tablada | pitch pimi somunu (üstten kanal), ayakta 2 somun |
| `base` | 1 | **217.5**×89×74 | taban tablada | tablaya sığar (220), kenarlara dikkat |
| `mask` | 1 | **217.5**×92×30 | ön yüz tablada | pedlerde arkadan kanallı M4 somun |

Hepsi tek gövde, havada başlayan ada yok, et %1 ≥ 1.6 (`check.py §5, §7`).

## 2. Vidalar ve somunlar

| Parça | Adet | Nerede |
|---|---|---|
| **M4×16 havşa** | **15** | göz üst pimi 2, alt pimi 2; kapak menteşesi 4 (kafa üst kapak göbeğinin havşasında); LIDS krank ucu 1; yaw_mount ↔ kiriş 2; pivot_bracket ↔ base 2; maske ↔ base 2 |
| **M4×10 yuvarlak** | **6** | göz kolu uçları 2 + yaw krank ucu 1 (lama pimleri); sol pitch pimi 1; kapak lama pimleri 2 (U: plastiğe vidalı, L: somunlu) |
| **M3×10 yuvarlak** | **8 / 14** ✅ | göz kolu kilidi 2; SG90 kulakları 3 (mile UZAK kulak); orta bindirmeler 3 (üst 1, alt 2) |
| **M4 somun** | **20** | yukarıdaki M4'lerin U pimi hariç hepsi |
| **M3 somun** | **8** | bütün M3'ler |
| SG90 küçük vidaları | 6 | servo kolu ↔ basılı parça (servoyla gelir) |

## 3. Montaj sırası
**`../MONTAJ.md`'ye taşındı** (2026-09-30). Eski sıra fiziksel olarak uygulanamıyordu (üst lama pimi, göz kolu ucu vidaları,
bindirme vidası). Yeni sıra `check_montaj.py` ile vida vida denetlenir (her vidanın giriş yolu + tornavida).
