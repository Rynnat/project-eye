# Project Eye v6 — BOM farkı (v5'e göre)

v5 BOM: `../v5/cad/BOM.md` (19 STL, hepsi basılı, **yeniden basılan 0**). Bu belge yalnız değişeni listeler.

## 1. YENİ basılacak parçalar (`cad/out/stl/`) — 3 STL, 4 baskı
Süre/gram: OrcaSlicer CLI, Ender 3 Neo + "Epic POC" (0.2 mm), `qr-stand/araclar/dilimle.py` ile ölçüldü.

| STL | Adet | Baskı boyutu (mm) | Baskı yönü | Epic POC süre / gram | Not |
|---|---|---|---|---|---|
| `kama` | **2** (2. adet dilimleyicide **X aynala** = sağ yarı) | 93.0 × 51.5 × 109.6 | dış yüz tablada, desteksiz | **7 sa 42 dk / 60.5 g** (yarı başına; ilk tur 10 sa 15 dk / 89 g) | duvar/deri/nervür 1.6 (MIN_ET), 2 nervür (denetim D4). Hassas geçme yok → **öneri 0.28 katman, dış 50 / iç 70 mm/s: 4 sa 09 dk / 59.6 g**. Altta 4 Ø10 ayak izi |
| `sol_servo_braketi` | 1 | 31.9 × 40.0 × 67.5 | ayak altı tablada | 1 sa 29 dk / 9.8 g | servo cebi 22.9 × 12.7 (v5 doğrulanmış); ayakta ±3 mm yarık + üstten açık M4 somun cepleri |
| `sol_kol_adaptoru` | 1 | 20.0 × 40.0 × 14.0 | flanş temas yüzü tablada | 28 dk / 3.0 g | D-delik (göbek + 0.25), kol pilotları Ø1.6 × 3.9 |
| **Toplam** | 4 baskı | | | **~17.4 sa / 134 g** (önerilen kama ayarıyla **~10.3 sa / 132 g**; ilk tur 22.5 sa / 191 g) | |

Basılmayan v5 STL: `pivot_bracket` (artık kullanılmaz; sakla). Diğer 18 v5 STL aynen (md5 `cad/v5_dondurulmus_md5.json`).

## 2. Donanım farkı
| Kalem | Adet | Kaynak | Nerede |
|---|---|---|---|
| M4×16 havşa | 1 yeni | **SATIN AL** (envanterde M4 havşa yalnız 2× M4×10) | krank ucu, üstten (T1) |
| M4×16 havşa | 2 | v5 pivot_bracket'tan **sökülen** | sol_servo_braketi ↔ taban (T10) |
| M4 somun | 2 | v5 pivot_bracket ayağından sökülen | braket ayak cepleri |
| M4 somun | 1 | v5 sol pitch piminden (nut_fpivot) sökülen | krank alt emniyet somunu (T3) |
| M2×8 mercimek | 3 | envanter (30) | adaptör ↔ frame_L 2, sol servo kulağı 1 |
| M2 somun | 1 | envanter ("var", adet sayılmadı) | sol servo kulağı |
| SG90 kol vidası | 2 | servo kiti (sayılacak) | sol kol ↔ adaptör (≤ 5.4 mm) |
| SG90 | 1 | envanter: boşta (~6) | sol pitch, **D9**. Sağ pitch'te MG90S yoksa v5'in SG90'ı yerinde kalır (yeni SG90 gerekmez) |
| MG90S | 1 | envanterde YOK — kullanıcıya soruldu, cevap bekleniyor | sağ pitch, D5: **MG90S varsa onu, yoksa SG90**. Tork hesabı SG90 kapasitesiyle yapıldı (pay 2.01 değişmez) |
| Elektrolitik kondansatör 1000 µF 16 V | 1 | envanter (4) | servo 5V–GND, Uno pin başlığına yakın, uzun bacak + (MONTAJ T15; denetim D3) |
| SG90 çift kol | 1 | servo kiti | sol kol |
| Artan (v6'da kullanılmaz) | — | — | 2× M4×10 yuvarlak (eski krank ucu + sol pim), pivot_bracket |
| Kaldırılan plan kalemi | — | — | M4×25 havşa, ağırlık, ağırlık tutucu (SEÇENEK B, PLAN §11) |

## 3. SATIN ALINACAK
| Kalem | Adet | Amaç | ~TL |
|---|---|---|---|
| M4×16 havşa (DIN 7991 / ISO 10642) | 1 (+2 yedek) | krank ucu vidası (D7) | 10–20 |
| Vida sabitleyici, orta güç (Loctite 243 muadili) | 1 | krank ucu vidası + alt emniyet somunu, isteğe bağlı A pimi | 150–250 (elde varsa 0; yedek: şeffaf oje) |
| Kaymaz ayak, yapışkanlı silikon Ø10 (≈1.5 mm) | 4 | kama altı (izler 0.5 derin); kalınlığı ölç → `params_v6.AYAK_T` | 30–50 |
| *(isteğe bağlı)* Servo uzatma kablosu 15 cm | 2 | Uno kamanın arkasında ≤ 60 mm değilse sabit pitch servoları için (kablo kontrolü marjı 8–16 mm) | 30–60 |
| **Toplam** | | | **~190–320 TL** (vida sabitleyici elde ise ~40–130 TL) |
Plan §8'deki "M3/M4 pul (Z1 ayarı)" artık gerekmez: braket yarığı (±3) eksenel farkı karşılar.

## 4. Servo / kanal
| Kanal | Pin | Servo | Aralık (v6) |
|---|---|---|---|
| EYE_YAW | D3 | SG90 | v5 aynen |
| EYE_PITCH | D5 | **MG90S** (yoksa SG90) | 80…**108** (pitch −10…+18) |
| LIDS | D6 | SG90 | v5 aynen |
| **EYE_PITCH_L** | **D9** | **SG90** | **180 − EYE_PITCH** (100…72), ayrı ofset |
Firmware/settings bu işin dışında (MONTAJ "Firmware / ayar").
