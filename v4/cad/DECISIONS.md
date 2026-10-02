# Project Eye v4 — mekanik kararlar (v3 POC'nin revizyonu)

**v4 nedir:** v3 (`../../v3/`) olduğu gibi duruyor; bu klasör onun mekanik revizyonu. Şartname v3 SPEC.md + revizyon isteği:
(1) alt kapaklar geri, aynı LIDS servosunda, kapalıyken iris+bebek tamamen örtülü; (2) kapak–göz boşluğu ≤ 1.0 mm;
(3) ön maske; (4) yuvarlatılmış kenarlar, düzenli yerleşim; (5) ≤ 16 farklı STL, M3 ≤ 14. Malzeme kısıtları v3 ile aynı
(M4×10 yuvarlak, M4×16 havşa, M3×10 ≤ 14; M2/somun/pim/rotil/tel yok; her dönen eklem M4 vida = pim).
Servo ölçüsü kullanıcı ölçümüyle güncel (`SG_L 22.5`, `SG_W 12.5`, cep boyu +0.4, eni +0.2 — `params.py §1, §8`).

Etiketler: **[SEÇİM]** tasarım kararı · **[SAPMA]** istekten/v3'ten ayrılma · **[DOĞRULANMADI]** yalnız model/hesap.
Sayılar `params.py`'de. Doğrulama: `out/check_report.txt`. Değişince: `python eye_v4.py && python check.py && python render.py`.

## 1. Mimari (v3'ten farkı)

| | v3 | v4 |
|---|---|---|
| Göz tutucu | çerçeve kolu gözle kapak arasından geçiyordu (kapak göze 3.3 mm) | **kafes**: her gözün iki yanında dikey plaka + üstte/altta köprü. Hiçbir çerçeve parçası kapak kabuğunun (R 18.6–20.6) bandına girmez |
| Kapaklar | 2 üst kapak, sabit gövdede, R 21.3 | **4 kapak (2 üst + 2 alt), çerçevede**, küre kabuk dilimi R 18.6–20.6 → **gözden 0.6 mm** |
| Kapak mafsalı | taşıyıcı servo milinde | her gözün **iki yanında** (4 kutup) M4×10: vida kafası üst kapak göbeğinin cebinde, gövdesi alt kapak göbeğinden geçip kafes plakasına vidalı |
| LIDS | servo mili = kapak ekseni (direkt) | servo **çerçevede** (sağ gözün arkası), krankı (= yaw_crank ile aynı STL) ucundaki tek pime iki lama: üst lama ≈ paralelkenar (üst kapaklar), alt lama **çapraz** (alt kapaklar ters yönde) |
| Maske | yok | ön maske, badem açıklıklar (arkadan öne genişleyen pah) |
| Parça | 12 STL | 16 STL (sınır) |

- **Kapaklar pitch'i mekanik olarak izler [SEÇİM]:** kapaklar pitch çerçevesinde. Göz aşağı bakınca kapaklar da iner (doğal göz kapağı). Asıl neden: kapakları sabit gövdeye koyarsak pitch ±20'de dönen çerçeve (üst/alt pim göbekleri) kapak bandını keser. Sonuç: yazılımdaki `lid_follow_pitch` artık mekanik takibin ÜSTÜNE eklenir → v4'te 0'a yakın başlayın (kinematics.json → cross_coupling).
- **Göz = küre, kapak = eş merkezli küre kabuk:** göz hangi pozda olursa olsun kapak iç yüzü ile göz arası 0.6 mm sabit (check §9b: 0.579 ölçüldü; ~0.02 üçgenleme).
- **Kapalıyken örtme:** üst kapak kenarı −4°, alt kapak kenarı −5.4° (arada 1.4° ≈ 0.45 mm). Aradaki çizgiyi üst kapağın **dudağı** (R 21–23, −10°'ye iner, |x_rel| ≤ 17.5, ucu R1 tam yuvarlak) önden kapatır; dudak alt kapağın 0.4 mm önünden geçer. check §10: 9 göz pozu × 5 bakış açısı (±15° yaw, ±12° pitch eğik bakış dahil), maskeli ve maskesiz: **görünen iris+bebek = 0.00 mm²**.
- **Açıkken:** üst kapak +27° (dudak kenarı +17°) → irisin üst **1.60 mm**'sini örter (check §10, önden z-tamponu); alt kapak −29.2° (kenar −34.6°) → irisin altında.
- **Göz arkadan takılır:** gözün arkasında (x_rel ±18, y −13…16) kafes parçası yok; üst köprü göbeği y 16.4'te (göz üst düzlemi y 16). Kapak mafsal vidaları gözün tarafından takıldığı için **kapaklar gözden ÖNCE** takılır (BOM §4).

### LIDS dört-çubuk [SEÇİM]
YZ düzleminde (çerçeve koordinatı): servo mili S = (y 9.6, z −29.3), krank r = 20 (yaw_crank STL'i), krank ucu kapalıda φ −70.4°.
Üst pim U (ρ 14.5, φ 142.4), alt pim L (ρ 16.5, φ −72.1); lama boyları 21.2 / 28.4. Rastgele tarama (~60 bin aday) + geometrik kısıtlar
(lamalar üst çubuktan, yaw lamasından, yaw servosundan, maskeden uzak; tek yönlü/monoton; |krank| ≤ 40°) ile seçildi.
- Üst kapak 0 → 27°, alt kapak 0 → 29.2° (hafif doğrusal değil), krank 0 → −24.6°.
- İletim sapması (0 = ideal 90°): en kötü ~43° (krank–üst lama), yani iletim açısı ≥ ~47°. Kapaklar hafif; SG90 için yeterli sanılıyor **[DOĞRULANMADI]**.
- **LIDS servo aralığı DEĞİŞTİ:** kapalı **102.3°**, açık **77.7°** (v3: 112.5 / 67.5). Kontrol yazılımı `kinematics.json → channels.LIDS.table` (11 satır, lid 0..1) ve `settings_suggestion` ile güncellenmeli.
- Sol ve sağ kapak parçaları ortada **bindirmeli** (sol parça iç kat, sağ parça dış kat) radyal civatayla birleşir: üstte M4×10 + M3×10, altta 2× M4×10.

### Yaw paralelkenarı
v3 ile aynı (kollar r 20, düz lama, yaw = servo açısı). Değişenler: yaw servosu z −60'a alındı (LIDS krankına yer), lama ortada geriye kıvrık (LIDS alt laması önünden geçer); göz kolu kilidi artık **M3×10, pimin arkasında** (önde alt kapak var).

## 2. Sapmalar / sınırlar [SAPMA]
1. **Parça sayısı tam 16** (sınırda). LIDS krankı yaw_crank ile aynı STL olduğu için sığdı.
2. **Maske asimetrik:** x −98.5…+113.5 (212 mm, tabla 220). Sağdaki pitch servosu gövdesi x 111'e uzanıyor; simetrik maske 226 mm olurdu. Gözler maskenin ortasından 7.5 mm sola kayık.
3. **Kablo yarığı ↔ kulak vidası:** SG90'ın kablo ucundaki kulak deliği, korunması istenen kablo yarığının tam üstüne düşüyor (v3'te de aynı: o delikte plastik yok). v4'te her servo **tek M3** (mil tarafı kulak) + cep ile tutuluyor; kablo tarafı kulağa vida konmadı. Gerekirse o kulağa sıcak tutkal.
4. **Eğik bakışta maskenin arkası:** önden (±15°) yalnız göz ve kapaklar görünür; çok yandan (render 04) badem köşelerinden kafes plakası görülebilir.
5. **Kapak ucunda ince kenar:** dudak |x_rel| 17.5'te düz kesik; iç yüzle 34°'lik kama (min 0.1 mm, %1 1.6 mm). Maske arkasında, işlevsel değil.
6. Göz kolu kilit vidası M3 (v3'te M4×16 havşa) — alttan erişim alt köprüden geçemiyordu.

## 3. Baskı yönü ve kurallar
- **Dikey delik kuralı:** göz, göz kolu, lamalar, kranklar, yaw_mount delikleri baskıda dikey; tabladaki ağızlarda `BOTTOM_CHAMFER` 0.4 pah.
- **Kafesler (frame_L/R) ÖN YÜZÜ (z = +3.5) tablada:** plakalar, köprüler, kiriş z yönünde prizmatik → desteksiz. Yatay kalan delikler (+0.2 pay, `HORIZ_HOLE_EXTRA`): üst/alt göz pimi pilotları (Y), 4 kapak menteşe pilotu (X), sol pitch pimi pilotu, sağda pitch kolu vidaları ve LIDS servo kulağı M3. Pilotlar (vida sabit) → risk düşük. LIDS kanadının alt kenarı ~28 mm köprü (iki ucu dayalı) — hafif sarkabilir.
- **Kapaklar X ekseni dik** (dış göbek yüzü tablada): kutup çevresindeki kubbe ve dudak altı sarkma (check §7: 290–460 mm²) → **bu dört parçada destek (tree/normal) önerilir**; havada başlayan ada yok.
- **Maske ön yüzü tablada**, ayak ve cıvata pedleri dik duvar. Base: taban tablada.
- STL kontrolü (check §7): 16 STL'nin hepsi tek gövde, z_min = 0, 0.2 mm katmanlarda havada başlayan ada yok.

## 4. Kalite / estetik
- Görünen dış kenarlar: kutu parçalar `rbox` ile R1.0 yuvarlatılmış (kafes, köprüler, kiriş, yaw_mount, base R3, maske dış köşeler R6 + R1.2); alt kapak üst kenarı R0.9 fileto; üst kapak dudağı R1.0 tam yuvarlak; maske açıklıkları arkadan öne 2 mm genişleyen pah.
- Servolar: pitch sağ dışta (v3 gibi), yaw ve LIDS gözlerin arkasında toplu; önden bakınca maske hepsini gizler.
- Render: `out/renders/01–11` (önden açık/kapalı/bakış, 3/4 maskeli ve maskesiz, göz yakın açık/kapalı, arka 3/4, yan, LIDS sürücüsü, alt).

## 5. Doğrulama özeti (`out/check_report.txt`, son çalıştırma)
- 18 zorunlu + 57 ara poz (kapak 0 / 0.5 / 1): **çakışma 0**.
- **Eklem dışı en küçük boşluk 0.398 mm** (kafes plakası ↔ üst kapak çubuğu; tasarım 0.4, fark üçgenleme).
- **Kapak–göz 0.58 mm** (tasarım 0.60; hedef 0.4–1.0).
- **Kapalı pozda görünen iris+bebek 0.00 mm²** (maskeli ve maskesiz, eğik bakışlar dahil).
- Et kalınlığı %1 ≥ 1.6 bütün parçalarda. Tek-katı, geçerli katı, su geçirmez ağ: hepsi OK.
- Vida: M3 6/14, M4×10 13, M4×16 havşa 13.
- **Yöntem sınırları:** rijit, boşluksuz model; baskı toleransı, sürtünme, esneme, servo torku **[DOĞRULANMADI]**.

## 6. Riskler
1. **Kapak menteşesi:** M4×10 kafası üst kapak göbeğindeki cepte döner (kafa altı = eksenel yatak). Vida plakada pilotta sabit; çok sıkılırsa kapaklar sıkışır → "serbest dönene kadar geri al". Eksenel oynama kapakları 0.4 mm kaydırabilir; göz boşluğu 0.6 **[DOĞRULANMADI]**.
2. **Dört-çubuk iletim açısı ~47°** (en kötü uç) — sürtünmeyle SG90 zorlanabilir; kapaklar hafif olduğu için sorun beklenmiyor **[DOĞRULANMADI]**.
3. **Pitch servosu artık daha ağır bir çerçeve taşıyor** (+ LIDS servosu + 4 kapak). Ağırlık merkezi eksenin gerisinde; tork hesabı YAPILMADI **[DOĞRULANMADI]**.
4. **Sağ dış plakada pitch kolunun merkez vidası yok:** plaka merkezinde kapak menteşe vidasının pilotu var. Kol, 2 küçük vida + çerçevenin sol pim ile eksenel sıkıştırılmasıyla milde kalır.
5. İnce et: SG90 kulak deliği ↔ servo cebi arası ~1.0–1.1 mm (LIDS kanadı, yaw_mount) — SG90 geometrisi, v3'te de vardı.
6. Kafeslerin iki yarısı yalnız yaw_mount (4× M4×16) ile birleşiyor; ortada kiriş yüzleri temas eder.
7. `lid_follow_pitch` yazılım ayarı mekanik takiple üst üste biner (bkz. §1).
8. Kapak parçaları desteğe muhtaç (bkz. §3); uzun (~90 mm) ince çubuklu baskı → brim önerilir.

## 7. Dosyalar
`params.py` ölçüler · `kin.py` kinematik (yaw/pitch kapalı formül + LIDS dört-çubuk çözücü) · `eye_v4.py` parçalar + montaj +
`build()`, `pose_transforms(yaw, pitch, lid)`, `servo_angles()`, `tess()` (v3 API'si; yeni gruplar: `lids` = üst kapaklar, `lids_lo`,
`lidcrank`, `link_up`, `link_lo`; GLB düğümleri `G_<grup>`) · `kinematics_export.py` → `out/kinematics.json` (v3 formatı; LIDS tablosu
11 satır + üst/alt kenar açıları) · `check.py` → `out/check_report.txt` (yeni: §9b kapak–göz, §10 iris görünürlüğü) · `render.py` → `out/renders/`.
Not: v3'ün görüntüleyicisi (`v3/viewer`) grupları `pose_transforms`'tan genel olarak okur; v4 için henüz model.html üretilmedi.
