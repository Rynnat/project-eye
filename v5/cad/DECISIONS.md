# Project Eye v5 — somunlu bağlantılar (v4 + M3/M4 somun)

**v5 nedir:** v4 (`../../v4/`) olduğu gibi duruyor. Kullanıcının elinde M3 ve M4 **somunlar** da var. v5, v4'ün aynı mekanizması;
plastiğe kılavuzsuz vidalama (`M3_PILOT`/`M4_PILOT`) nerdeyse tamamen kalktı, yerine **gömülü (captive) altıgen somun cepleri** geldi.
Kinematik ve servo aralıkları v4 ile **birebir aynı** (`out/kinematics.json` → settings_suggestion ve LIDS tablosu v4 ile eşit; kontrol edildi).
Aşağıdaki §0 v5'e özgü; §1 ve sonrası v4 kararlarıdır (sayılar v5'e göre güncellendi).

## 0. Somun tasarımı [SEÇİM]
- Ölçü (ISO 4032): M3 AF 5.5 × 2.4, M4 AF 7.0 × 3.2. Cep: AF + `NUT_POCKET_CLR` 0.3, derinlik + `NUT_POCKET_DEPTH_CLR` 0.2 (`params.py §1`, [TOL]).
- **Cep kuralı:** somunun vida kafası tarafındaki yüzü plastiğe dayanır (yük oraya biner); cep uç tarafa ya da bir yüzeye açıktır.
  Kaydırmalı cepte kanal duvarları somunun iki düz yüzüne paraleldir → somun dönmez.
- **Baskı:** (a) cep ekseni baskıda dikeyse (göz kolu ucu, krank ucu, kapak tabı, göz, taban/maske ayakları) üstten ya da yandan açık düz altıgen;
  (b) kafeslerde (ön yüzü tablada basılır) eksen yatay → cep **ön yüze (tablaya) açık kanal**, tavanı altıgenin 60° köşesi → destek yok;
  (c) pivot_bracket'ta kanal yukarı açık.
- **Vida boyu kontrolü** (`check.py §8b`): 29 somunlu bağlantının hepsinde vida ucu somunun dış yüzünden ≥ 1 diş (M3 0.5, M4 0.7) çıkıyor
  (en az: bindirme M3 0.7, LIDS kulak M3 1.0, göz kolu ucu M4 1.3). Vidalar ve somunlar çarpışma taramasında katı olarak var → hiçbiri başka parçaya değmiyor.

| Bağlantı | Vida | Somun nerede | Tür |
|---|---|---|---|
| Göz üst pimi ×2 | M4×16 havşa (üst köprü, üstten) | köprü altındaki blokta, öne açık kanal | kafa+somun köprüyü sıkar, göz uçta döner |
| Göz alt pimi ×2 | M4×16 havşa (alttan) | alt köprünün üst yüzünde (kol altında), öne açık | aynı |
| Kapak menteşesi ×4 | **M4×16 havşa** (v4: M4×10), kafa üst kapak göbeğinin havşasında | kafes plakasında, öne açık kanal | kapaklar kafa ile plaka arasında döner |
| Pitch pimi (sol) | M4×10 | **pivot_bracket'ta** (üstten kanal) — vida artık yatağa kilitli, çerçeve göbeği RUN'da döner | kilitli |
| Göz kolu ucu ×2, yaw krank ucu | M4×10 (lamanın altından) | kolun / krankın uç göbeğinde, üstten açık | lama kafa ile parça arasında döner |
| LIDS krank ucu (A) | M4×16 havşa | krank uç göbeği (yaw_crank ile aynı STL) | lamalar arada döner |
| Alt kapak pimi (L) | M4×10 | alt kapak tabının göbeğinde (lama tarafına açık) | lama uçta döner |
| Göz kolu kilidi ×2 | M3×10 | **gözün içinde**, arka düzden kaydırmalı | sabit |
| Orta bindirmeler (4) | M3×10 radyal (v4: M4/M3 karışık) | sol parçanın iç katında, eksen tarafına açık | sabit |
| yaw_mount ↔ kiriş ×2 | M4×16 havşa (alttan) | bacakta, dışa açık kanal (v4: bacak başına 2 → 1) | sabit |
| Servo kulakları ×3 | M3×10 | tutucu plakanın arkasında serbest somun | sabit |
| pivot_bracket, maske ↔ base (2+2) | M4×16 havşa (alttan) | ayak / ped içinde, yandan / arkadan kanal | sabit |

- **Kalan kılavuzsuz delikler:** (1) **üst kapak lama pimi (scr_U, M4×10)** — tab 3.1 mm ince, bir yanında LIDS servo kolu (0.4 mm boşluk), öbür yanında üst lama; somun cebi ya da dışarıda somun sığmıyor. (2) SG90'ın kendi küçük kol vidaları (servoyla gelen sac vidaları, M3/M4 değil).
- **Menteşelerde gevşeme [DOĞRULANMADI]:** göz pimlerinde ve pitch piminde sabit parça kafa ile somun arasında sıkışır → vida kilitli. Kapak menteşesi, lama pimleri ve krank uçlarında ise dönen parça kafa ile somun arasında; vida sıkılırsa eklem kilitlenir. "Serbest dönene kadar geri al" gerekir. Düz somun + dönme titreşimi vidayı gevşetebilir → bu eklemlerde bir damla vida sabitleyici / oje, ya da varsa nyloc önerilir.

### v5'te düzeltilen v4 hataları (v4'e dokunulmadı; v4 kullanılacaksa bilinmeli)
1. **Göz kolu kilit vidası gözün dışına düşüyordu:** v4'te M3 pilotu z −10.2'de, gözün arka düzü z −10 → deliğin çoğu göz dışında. v5: arka düz z −14, kilit z −10.5, M3 somun gözün içinde.
2. **Alt göz pimi vidası takılamıyordu:** arka kiriş alt köprünün 1.1 mm altından geçiyor, 16 mm vidanın alttan girmesine yer yok. v5: kirişte her gözün altında Ø9.6 erişim deliği.

### v5 ölçü değişiklikleri
Üst köprü y 21.2…27.5 (+ somun bloğu y 18.7…), köprüler z −6'ya kadar, sağ dış plaka 8.6 mm (menteşe vidası ucu), sol dış göbek x_rel 41'e kadar,
pivot_bracket 5.0 mm, pitch servosu 3 mm dışarıda (mil x 84.9; açı ilişkisi aynı), base/maske x −101…+116.5 (**217.5 mm**, tabla 220'ye sığıyor ama sınırda),
yaw_mount bacakları x 23.5–34.5, göz kolu alt yüzü tablada basılır (uç göbeği yukarı).

# (v4 kararları) Project Eye v4 — mekanik kararlar (v3 POC'nin revizyonu)

> v5 notu: aşağıdaki metin v4'ten. Vida türü/boyu ve "pilot" geçen yerler §0 ile değişti (somunlu); güncel sayılar §0, `BOM.md` ve `out/check_report.txt`.

**v4 nedir:** v3 (`../../v3/`) olduğu gibi duruyor; bu klasör onun mekanik revizyonu. Şartname v3 SPEC.md + revizyon isteği:
(1) alt kapaklar geri, aynı LIDS servosunda, kapalıyken iris+bebek tamamen örtülü; (2) kapak–göz boşluğu ≤ 1.0 mm;
(3) ön maske; (4) yuvarlatılmış kenarlar, düzenli yerleşim; (5) ≤ 16 farklı STL, M3 ≤ 14. Malzeme kısıtları v3 ile aynı
(M4×10 yuvarlak, M4×16 havşa, M3×10 ≤ 14; M2/somun/pim/rotil/tel yok; her dönen eklem M4 vida = pim).
Servo ölçüsü kullanıcı ölçümüyle güncel (`SG_L 22.5`, `SG_W 12.5`, cep boyu +0.4, eni +0.2 — `params.py §1, §8`).

Etiketler: **[SEÇİM]** tasarım kararı · **[SAPMA]** istekten/v3'ten ayrılma · **[DOĞRULANMADI]** yalnız model/hesap.
Sayılar `params.py`'de. Doğrulama: `out/check_report.txt`. Değişince: `python eye_v5.py && python check.py && python render.py`.

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
- **Eklem dışı en küçük boşluk 0.400 mm** (v5; yaw lama ↔ servo kol vidası ve LIDS kolu ↔ üst kapak tabı). v4'teki 0.398 (kafes plakası ↔ üst kapak çubuğu) v5'te 0.498.
- **Kapak–göz 0.58 mm** (tasarım 0.60; hedef 0.4–1.0).
- **Kapalı pozda görünen iris+bebek 0.00 mm²** (maskeli ve maskesiz, eğik bakışlar dahil).
- Et kalınlığı %1 ≥ 1.6 bütün parçalarda. Tek-katı, geçerli katı, su geçirmez ağ: hepsi OK.
- Vida (v5): M3×10 9/14, M4×10 6, M4×16 havşa 15; somun M3 9, M4 20.
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
`params.py` ölçüler · `kin.py` kinematik (yaw/pitch kapalı formül + LIDS dört-çubuk çözücü) · `eye_v5.py` parçalar + montaj +
`build()`, `pose_transforms(yaw, pitch, lid)`, `servo_angles()`, `tess()` (v3 API'si; yeni gruplar: `lids` = üst kapaklar, `lids_lo`,
`lidcrank`, `link_up`, `link_lo`; GLB düğümleri `G_<grup>`) · `kinematics_export.py` → `out/kinematics.json` (v3 formatı; LIDS tablosu
11 satır + üst/alt kenar açıları) · `check.py` → `out/check_report.txt` (yeni: §9b kapak–göz, §10 iris görünürlüğü) · `render.py` → `out/renders/`.
Not: v3'ün görüntüleyicisi (`v3/viewer`) grupları `pose_transforms`'tan genel olarak okur; v4 için henüz model.html üretilmedi.

## 2026-09-29 kullanici geri bildirimleri
- Servo kolu delikleri r=6 (kullanicinin cift kolu olculdu; capraz kolda 5). Kol ucu yaricapi <=16.5 guvenli, 17.5'te LIDS kolu yaw_mount'a carpar -> uzun kol kesilir.
- Iris + bebek ayri basilir (tek renk baskida beyaz top olmasin): gozde O16.2 x 2 cep, iris O16 (on yuzu goz kuresi), ortasinda O6.15 x 1.9 bebek cebi. Paylar [TOL], gevsekse yapistirici.
- Kablo yarigi MILE YAKIN uca tasindi (kullanici gercek serVoda gordu; v3/v4'te uzak uctaydi). Kablo cikisi (cable_*) carpisma taramasina eklendi. Degisen STL: base, frame_R, yaw_mount (+ eye, iris, pupil).
- Kulak vidasi (M3 + somun, her servoda 1) MILE UZAK kulaga tasindi: yeni kablo yarigi mile yakin kulaktaki vidayi kesiyordu (olculdu, 0 mm). STL degismedi (iki kulak deligi zaten acik), yalniz montaj/goruntuleyici. Yarik-vida mesafesi artik > 2.5 mm.
- Kapak servosu kanadi LS_WING_Y 26 -> 32.5: kulak vidasi uzak kulaga tasininca havada kalmisti (kullanici buldu). check_havada.py eklendi (vida/somun en yakin basili parcaya <= 0.6 mm). Degisen: frame_R.
- Taban iki parca (kullanici, 2026-09-30): ortadan, 40 mm yatay bindirme; alt dil (3 mm) servolu base_R'ye, ust dil base_L'ye; yalniz yapistirma, dil ucu boslugu 0.2. base_L ters basilir (PR_YDOWN). Toplam hacim 117.28 (eski 117.38, fark = dil bosluklari).
- MONTAJ ERISIMI (kullanici 2026-09-30: "vida oraya isinlanmayacak"): check_montaj.py eklendi (her vidanin giris koridoru: kafa x vida boyu + 30 mm uc, MONTAJ SIRASINA gore; hepsi-takili modu --takili). Basilmis parcalar DONDURULDU (eye_lever, link_up, link_lo, yaw_crank, pivot_bracket, coupler, yaw_mount, frame_L, frame_R: md5 ayni). Cozumler: (1) scr_U: lid_up_L kolunda kafa+tornavida kanali (U0 ekseninde, 40 mm); lamalar GOZDEN ONCE. (2) scr_ltip: goz alt montaji (iki goz + kollar + coupler, ltip alttan) iki kafese birlikte arkadan kayar (supurme 0-100 mm serbest, kapak acik/kapali); kapak servosu gozlerden SONRA (servo yolu serbest, lamalar bosta cevrilir). (3) scr_lap_up1: ust bindirmede TEK civata x 10.3 (kanat kafa yoluna 0.7 mm biniyordu); bindirme vidalari gozlerden once, kapaklar KAPALI. Degisen STL: lid_up_L, lid_up_R. Sonuc: sirali 0 imkansiz / 0 engel; 3 uyari (M3 pan kafa SG90 govdesine ~0.15 mm, bilinen).
