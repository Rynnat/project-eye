# Project Eye v6 — Doğrulama (Agent Flow v2, aşama 4)

Tarih: 2026-10-12. Girdi: `denetim.md` (D1–D8). Betikler `dogrulama/D<n>.py`, `.venv-cad` ile çalıştırıldı, modelin kendi `eye_v6` / `kin_v6` / `analiz` fonksiyonlarını kullanır. Tasarım dosyalarına yazılmadı.
Envanter kopyası: `dogrulama/envanter_plan-v2.md` (gh api, `Rynnat/agentic-mechanism-design@plan-v2`).

| # | Önem | Sınıf | Ölçülen | Eşik | Betik |
|---|---|---|---|---|---|
| D1 | önemli | **ÖLÇÜLEMEZ** (hesap kısmı doğrulandı) | Tek servo payı: dolgu 0.6 / 0.75 / 1.0 için **1.42 / 1.25 / 1.05**. İki servoda 2.27 / 2.01 / 1.68. R14'te sayısal paylaşım ya da akım kriteri yok. | hedef 1.5 | `dogrulama/D1.py` |
| D2 | önemli | **DOĞRULANDI** | Durma 21.72°, komut sınırı +18 → pay **3.72°**. Yarım kademe SG90 (21T) **8.57°**, MG90S (25T) **7.20°**. Kalibrasyonsuz hatanın durma yönünde payı aşma olasılığı %28 / %24. İki servo arası göreli kaçıklık en fazla 15.8°. Yerçekimi durma yönüne itiyor: AM yüksekliği pitch −10 → +21 arasında 74.55 → 67.20 mm düşüyor. MONTAJ'ın Takma tablosunda kalibrasyon adımı yok; "kalibrasyon" yalnız firmware notunda (satır 62), R14 taraması satır 70. | yarım kademe ≥ 3.72° ise risk gerçek | `dogrulama/D2.py` |
| D3 | önemli | **ÖLÇÜLEMEZ** (BOM kısmı doğrulandı) | Envanterde 1000 µF ×4 "servo güç hattı için" var. PLAN, MONTAJ, BOM_FARK, DECISIONS, STATE içinde kondansatör geçişi **0**. R14 ölçü aracı multimetre; MCUSR/bandgap yok. | — | `dogrulama/D3.py` |
| D4 | önemli | **DOĞRULANDI** | Kama (yarı): mevcut 2.4/2.0/3 nervür **76 106 mm³ = 94.4 g**; 1.6/1.6/1 nervür 61.7 g; 1.2/1.2/1 nervür **40 097 mm³ = 49.7 g** (denetçiyle birebir). R5 α30 en kötü poz (p−13_k1): mevcut 30.3 mm / 30.8°; 1.6 → 28.2 mm / 27.3°; 1.2 → **27.4 mm / 26.0°**. Baskı süresi dilimlenmedi. | ≥ 25 mm / ≥ 24° (R5 hedefi 15/15) | `dogrulama/D4.py` |
| D5 | küçük | **ÖLÇÜLEMEZ** | Rapordaki zincir yeniden üretildi: en kötü −0.270, RSS 0.154. Yalnız MG90S cep düzeltmesi (datasheet 12.2 → ±0.25) ile en kötü −0.420, RSS 0.085 (RSS hâlâ geçer). Denetçinin iki yeni halkası (MG90S mil/gövde, yapıştırma kaçıklığı) ±0.15 / ±0.20 / ±0.30 alınınca RSS payı 0.034 / −0.002 / −0.094. | RSS payı ≥ 0 | `dogrulama/D5.py` |
| D6 | küçük | **DOĞRULANDI** | frame_L şeridinde Ø1.6 silindir malzeme boyu **5.60 mm** (iki delikte %100 dolu); 5.5 hedefi **0.10 mm** et bırakır. Şerit arkası 0–10 mm silindir ile 36 poz × 2 delikte (yaw −25/0/25 × pitch −13/0/18/21 × kapak 0/0.5/1) hiçbir parçaya kesişim yok, en büyük kesişim **0.0 mm³**. Konum hatası/salgı ölçülmedi. | kesişim 0 | `dogrulama/D6.py` |
| D7 | küçük | **DOĞRULANDI** (geometri) | v5 krank somun cebi köşe eti ölçüm **1.70 mm**, hesap 1.69 mm; cep derinliği 3.40 mm. Sıkma torkunun cebi çatlatıp çatlatmayacağı ölçülemez. | MIN_ET 1.6 | `dogrulama/D7.py` |
| D8 | küçük | **DOĞRULANDI** | envanter.md: SG90 boşta **~6**, MG90S geçen satır **0**. BOM_FARK: "SG90 elde (10)", "MG90S elde (1)". Belgeler çelişiyor. | belge = envanter | `dogrulama/D8.py` |

**Toplam: DOĞRULANDI 5 (D2, D4, D6, D7, D8) · ÇÜRÜTÜLDÜ 0 · ÖLÇÜLEMEZ 3 (D1, D3, D5).**

## Çürütülen yok. Ek notlar ve kısmi bulgular
- **D2 (kritik, doğrulandı).** Dikkat: diş sayıları (21T / 25T) modelde yok, TowerPro datasheet bilgisidir. Kullanıcı sayabilir. Yerçekimi yönü ve 3.72° pay modelden ölçüldü.
- **D4 (kritik, doğrulandı) — çelişen kısıt.** Önerilen duvar 1.2 mm, projenin kendi `MIN_ET = 1.6` [SPEC] kuralının altında. 1.6 mm duvarda kama 61.7 g (denetçinin ≤ 55 g beklentisi tutmuyor), devrilme 28.2 mm / 27.3° ile geçiyor. Tasarımcı karar vermeli: MIN_ET'i kama için gevşetmek ya da 1.6/1.6/1 ile ~%35 hafifleme. Baskı süresi (≈ yarı) dilimlenmedi, hacim oranına dayanıyor; Orca ile doğrulanmadı.
- **D5.** Denetçinin RSS 0.6–0.7 iddiası, eklenen iki halka ±0.3 olursa tutuyor; ±0.15 ile tutmuyor. SG90 sol cep boşluğu kullanıcı ölçümüne (12.5, v5 baskı testi 12.7 uygun / 12.9 bol) dayanıyor; denetçinin "cep ±0.25–0.45" iddiası yalnız MG90S için (datasheet 22.8×12.2) geçerli. Kesin sonuç için kumpas ölçümü gerekir.
- **D6.** Önerilen talimat ("delip geçmek zararsız") model açısından güvenli.
- **D7.** Köşe eti 1.70, MIN_ET'in (1.6) üstünde. Geometri açısından cep yeterli; "en fazla 1/8 tur" talimatı yine de ucuz bir önlem.
- **D8.** MG90S'in envanterde yokluğu yoksa-varlığı kanıtlamaz, yalnız belgenin tutarsızlığını gösterir. Kullanıcı çekmeceyi saymalı.

## Kullanıcı testleri (ÖLÇÜLEMEZ olanlar)
- **D1:** Pitch −10'da 30 s bekleterek her servonun kırmızı hattına seri multimetre (200 mA) bağla, iki akım dengeli ve toplam ≤ ~1.3 × tek-servo akımı olsun. Biri boşta akımdaysa gerçek pay 1.25'tir.
- **D3:** Firmware'e `MCUSR` + 2 ms bandgap `readVcc` ekleyip R14 taramasını ×10 kondansatörsüz ve 5V–GND'ye 1000 µF ile koştur; en düşük Vcc ≥ 4.6 V ve reset yok istenir.
- **D5:** Çerçeve takılmadan (T9 öncesi) iki milin y ve z merkezini kumpasla ölç; |Δy|, |Δz| ≤ 0.5 mm.
