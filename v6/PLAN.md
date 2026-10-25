# Project Eye v6 — PLAN (Agent Flow v2, aşama 1, G1 kapısı)

Tarih: 2026-10-12. Kod/CAD yok; bu belge onaylanınca tasarımcı başlar.
Girdiler: `v5/cad/params.py`, `v5/cad/eye_v5.py`, `v5/cad/DECISIONS.md`, `v5/MONTAJ.md`, `v5/cad/BOM.md`, `v5/settings.json`,
`agent-flow/AKIS.md`, `agent-flow/sablonlar/README.md`, `agent-flow/checks/README.md`, `epic-tasarim-prensipleri.md`.
Proje için `research_notes/` klasörü yok; sayıların kaynağı v5 modelinden yapılan hesap (§5.0'da yöntem).
**v5 dondurulmuş:** v5 klasörüne dokunulmaz. v6 CAD'i v5 geometrisini `v5/cad/eye_v5.build()`'den **import eder** (kopyalamaz). Yalnız yeni parçaları ve yeni vida yerleşimini ekler.
Etiketler: **[TAHMIN]** ölçülmemiş (yanında nasıl ölçüleceği yazılı) · **[ÖLÇÜM]** kullanıcı ölçer · **[KULLANICI ONAYI]** karar kullanıcıda · **[TASLAK]** şablonda doğrulanmamış.
Koordinat: v5 ile aynı **mekanizma ekseni** (orijin iki göz merkezinin ortası, +X robotun sağı, +Y mekanizmanın üstü, +Z ileri). v6'da bütün mekanizma masaya göre X ekseni etrafında α kadar **burnu yukarı** döner.

---

## 0. Sade özet (kullanıcı için)

1. **Hiçbir v5 parçası yeniden basılmıyor.** Yalnız 3 yeni küçük/orta parça basılır: 2 kama (tabanın altına) + 1 ağırlık tutucu.
2. **Yükseklik:** taban, önü 46 mm kalkık bir **kamanın** üstüne oturur. Göz düz bakarken 30° yukarı bakar. 1–1.5 m'deki ayakta insanların yüzüne yeter (hesap §1.3).
3. **Pitch (yukarı-aşağı):** SG90 yerine elindeki **MG90S** takılır. Tek başına yetmiyor. Çerçevenin önüne, altına **~75 g ağırlık** (madeni para ya da somun) asılır. Bu ağırlık arkadaki fazla ağırlığı dengeler. Göz nötrden en fazla 10° aşağı, 20° yukarı bakar. Bu, masaya göre 20°–50° yukarı demek.
4. **Düşen vida:** düşen vida büyük ihtimalle arkadaki **krank ucundaki** vida. Göz kollarındaki vidaların 0.5 mm altında kiriş var, düşemezler. Bu vida artık **yukarıdan** takılır ve kranktaki somuna kilitlenir. Altına bir emniyet somunu + vida sabitleyici gelir. Gevşese bile vida kafasının üstünde durur, aşağı düşmez.
5. **Satın alma ~250–350 TL:** vida sabitleyici, 2 adet M4×25, kaymaz ayak, gerekirse ağırlık.
6. **Senin kararın gereken 4 şey §9'da.** En önemlisi eğim açısı (önerim 30°).

---

## 1. Spec

### 1.1 Eksenler
| Eksen | v5 | v6 | Not |
|---|---|---|---|
| Yaw | ±25°, SG90 D3 | **aynen** | paralelkenar değişmiyor |
| Pitch | ±20°, SG90 D5 (şu an ölü, 90'da sabit) | **−10…+20°**, **MG90S** D5, servo **80…110** | aşağı sınır −20 → −10: tork payı için (§5). Masaya göre bakış 20°–50° |
| Kapak (LIDS) | SG90 D6, açık 80 / kapalı 115 | **aynen** | |
| Masa eğimi α | 0 | **30°** [KULLANICI ONAYI] | `params` içinde tek değişken `TILT_DEG`. Tasarım 20/30/40'ın hepsini üretebilmeli |
| İkinci pitch servosu | yok | **SEÇENEK B** (§5.4, §9) | varsayılan: yok |

### 1.2 Boyut / kütle
| Kalem | Değer | Kaynak |
|---|---|---|
| Pitch ile dönen kütle | **227 g** (dolgu 0.75) · 195 g (0.6) · 280 g (1.0) [TAHMIN] | v5 modeli: hacim × PLA 1.24 × dolgu; vida/somun hacim × 7.85; SG90 9 g. **Ölçüm:** §7-R1 tartı + kalem üstünde denge |
| Dönen grubun ağırlık merkezi (AM) | y −7.1, z −22.1 mm (eksenin 7 mm altında, **22 mm arkasında**), ρ 23.3 mm | aynı hesap |
| Sabit kısım (taban+maske+braket+servo) | 213 g [TAHMIN] | aynı |
| Kama ×2 | ~60 g toplam [TAHMIN] | kabuk + nervür; Orca gram tahmini ile güncellenir |
| Ağırlık | **75 g** (ayarlanabilir 50–130 g) | §5.3 |
| Toplam | ~575 g | |
| Taban izi | 217.5 × 89 mm (v5 tabanı) | kama yan rayları +3 mm/yan |
| Pitch ekseni masadan | v5: 63 mm → v6 (α30): **86 mm** | göz merkezi 75 cm masada ≈ **83.6 cm** |
| Kamanın ön yüksekliği | α20: 31 · **α30: 46** · α40: 59 mm (+3 mm taban kalınlığı) | 92 mm × sin α |

### 1.3 Eğim açısı tablosu (masa 75 cm, göz ~83.6 cm)
Gereken bakış (yataydan yukarı) = atan((yüz − 83.6)/mesafe):

| Mesafe \ yüz yüksekliği | 140 cm | 160 cm | 175 cm |
|---|---|---|---|
| 1.00 m | 29° | 37° | 42° |
| 1.25 m | 24° | 31° | 36° |
| 1.50 m | 21° | 27° | 31° |
→ **Gereken aralık 21°–42°.**

| α | Masaya göre bakış (pitch −10…+20) | Kapsama | Pitch payı MG90S + 75 g (dolgu 0.75 / 1.0) | Geri devrilme payı (AM arka kenarın önünde) | Arkanın masaya en yakın noktası (pitch +20) |
|---|---|---|---|---|---|
| 20° | 10°–40° | 175 cm'yi 1 m'den kaçırır | 1.33 / 1.10 ✗ | 34 mm ✓ | 7.7 mm |
| **30°** | **20°–50°** | **tamamı** | **1.66 / 1.36** (✗ ise ağırlık artırılır, §5.3) | **24 mm** ✓ (~22°) | **5.5 mm** |
| 40° | 30°–60° | 140 cm'yi 1.25 m'den uzakta kaçırır | 2.31 / 1.84 ✓ | 14 mm ✗ (<15) → arka/yan ayak uzatmak gerekir | 3.2 mm |

Not: Gözün 30° yukarı bakması dikeyle 60° demek. Bu, kullanıcının "60 bile yeterli" sözüyle uyumlu. **Varsayım: 30°. Kullanıcı onaylayacak.**

### 1.4 Değişmeyen kısıtlar
Ender 3 Neo 220×220, PLA, "Epic POC" profili. Toleranslar `sablonlar/toleranslar.py`: M4 4.3, somun AF+0.3/t+0.2, SG90 cebi 22.9×12.7.
Servolar **Uno 5 V'tan** beslenir, harici kaynak yok (kullanıcı kararı). MG996R elendi. Kamera laptopun kamerası, tablayla ilgisi yok.

---

## 2. Parça listesi

### 2.1 v5 basılı parçaları — yeniden kullanım tablosu (prensip 7)
| v5 STL | Adet | v6 kararı | Neden / dokunuş |
|---|---|---|---|
| `eye` | 2 | **aynen** | |
| `iris`, `pupil` | 2+2 | **aynen** | |
| `eye_lever` | 2 | **aynen** | Uç vidası alttan kalır. Kafanın 0.5 mm altında kiriş (RB_Y −24.4) var. Vida gevşese de düşemez (§3 notu) |
| `coupler` | 1 | **aynen** | Krank ucunda vida yönü değişir, delik aynı (4.3 RUN) |
| `yaw_crank` (yaw) | 1 | **aynen** | Vida ters yönden takılır. Mevcut üstten açık somun cebi kullanılır |
| `yaw_crank` (LIDS) | 1 | **aynen** | A pimine yalnız vida sabitleyici (isteğe bağlı) |
| `link_up`, `link_lo` | 1+1 | **aynen** | |
| `frame_L` | 1 | **aynen** | Ağırlık tutucu, kirişteki mevcut Ø9.6 erişim deliklerini kullanır |
| `frame_R` | 1 | **aynen** | MG90S kolunun delikleri r=6'ya uymazsa 2× Ø1.6 pilot delinir (koşullu, el matkabı/Dremel, §4 T2) |
| `yaw_mount` | 1 | **aynen** | |
| `lid_up_L/R`, `lid_lo_L/R` | 4 | **aynen** | |
| `pivot_bracket` | 1 | **aynen** | (Seçenek B'de yerine yeni braket) |
| `base_L`, `base_R` (yapıştırılmış) | 1 | **aynen** | Kamanın üstüne oturur. Delik, kesme yok |
| `mask` | 1 | **aynen** | |
**Yeniden basılan: 0.**

### 2.2 Yeni basılacak parçalar (3 baskı, 2 STL)
| Ad | Adet | Amaç | Şablon / yeni | Boyut [TAHMIN] | Baskı yönü |
|---|---|---|---|---|---|
| `kama` (L ve R) | 2 (tek STL, R aynalı basılır) | Tabanı α eğimle taşır: önü yüksek. Arkada **dudak** (taban arka kenarını tutar), yanlarda **ray** (taban üst kenarına biner, kalkmayı önler) | yeni. Kenar/et kuralları `toleranslar.py` | ~112 × 85 × 52 mm (α30), kabuk 2.4 + iç nervür | yan yüz tablada (eğim üçgeni tablada, prizmatik → desteksiz) |
| `agirlik_tutucu` | 1 | ~75 g ağırlığı çerçevenin önüne-altına taşır. 2× M4×25 ile kirişin altına, göz alt pimlerinin yerine | yeni. `serbest_delik("M4")`, havşa yuvası (`csk_seat`, v5'te basıldı) | ~100 × 14 × 17 mm, cepli | düz alt yüz tablada (delikler dikey) |

Toplam basılı parça v6'da 3 (hedef ≤ 10 ✓). Hepsi tablaya sığıyor (en büyüğü kama ~112 mm < 200).

**Kama ayrıntısı (tasarımcı için kısıtlar):**
- Taban altı düzlemi (mekanizmada y = −60) kama üst yüzüne tam oturur. Kamanın arka alt kenarı masada, ön yüzü dikey. Taban altında min et 3 mm (t0).
- **Arka dudak:** taban arka kenarını (z = −62, R3 yuvarlak) tutar. Dudak tepesi **y ≤ −57** olmalı (taban üstü −54'ün 3 mm altı). Nedeni: pitch +20'de krank ucu vida/somunu ve yaw_mount köşesi taban arka kenarının 1–4 mm arkasına, y ≈ −51…−49'a iner (§6 Z3). Orta kısımda (|x| ≤ 18) dudakta **kablo kertiği**.
- **Yan raylar** yalnız arka bölgede: sol x −101…−96, z −62…−24 (pivot_bracket ayağı z ±20'de). Sağ x 95…116.5, z −62…−12 (servo tutucu z ±9.5). Ray dudağı taban üstünden en fazla 3 mm yükselir. Çerçeve |x| ≤ 88.5 olduğundan raylara erişemez.
- İki yarı birbirine bağlanmaz. Tabanı taban birleştirir. Ortada ≤ 10 mm boşluk kalabilir (taban 6 mm plaka, orta bindirme yapıştırılmış).
- Kama altında 4 kaymaz ayak yeri (Ø10 × 0.5 mm sığ iz).

**Ağırlık tutucu ayrıntısı:**
- İki göz alt pim vidası (x ±47.5, z 0) kirişteki Ø9.6 erişim deliğinden geçer. v5'te M4×16 havşa, kafa alt köprünün havşasında. v6'da **M4×25 havşa**, kafa tutucunun havşasında. Somun v5'teki gibi alt köprünün üstünde. Göz yine vidanın ucunda döner. Uç gözde 5.0 mm (v5: 5.7).
- Ağırlık cebi: AM hedefi **y ≈ −38, z ≈ +3** (eksenin önü-altı). Cep 130 g'a kadar alır (madeni para yığını ya da M10 somun).
- Tutucu **|x| ≤ 52** içinde kalır: maske civata pedleri x ±(53…67), y −42'ye kadar, z 4…16'da. Tutucunun alt yüzü **y ≥ −46** (maske ayağı üstü −50).

---

## 3. Arayüz tablosu

| Parça çifti | Bağlantı | Vida / somun | Delik ve pay | Dönen |
|---|---|---|---|---|
| **yaw_crank ↔ coupler (DEĞİŞİYOR)** | vida-pim, vida **üstten**, kranka kilitli | **M4×16 havşa** (envanter, bol), krank cebindeki mevcut M4 somuna vidalanır. Kafa somunun üst yüzüne oturur ve sıkılır (baş–somun kilidi). Somun altıgen cepte dönemez, vida kranka kilitlenir. Coupler'ın altında **M4 somun + vida sabitleyici**, coupler'a 0.2 mm boşlukla | krank ucu 4.3 RUN + somun cebi (v5). Coupler 4.3 RUN. Krank–coupler arası 0.4 (v5). Vida ucu somundan 1.4 mm çıkar (≥ 0.7 ✓) | **coupler** vida gövdesi üstünde döner. Vida+krank birlikte döner |
| eye_lever ↔ coupler ×2 | v5 aynen | M4×10 alttan, kol cebinde somun | v5 | coupler. **Not:** vida kafası kiriş üstüne 0.5 mm. Gevşerse ≤ 0.5 mm eksenel oynar, düşemez |
| LIDS krank ↔ link_up/link_lo (A pimi) | v5 aynen | M4×16 havşa | v5 | lamalar. Aynı gevşeme türü. Eksen yatay olduğu için yerçekimiyle düşmez. **Vida sabitleyici** önerilir (§4 T13) |
| Kapak menteşeleri ×4, scr_U, scr_L | v5 aynen | v5 | v5 | aynı tür, yatay. Gözlem yok → dokunma |
| **base ↔ kama ×2 (YENİ)** | geçme: yerçekimi + arka dudak + yan ray | yok | ray altı ile taban üstü arası 0.3. Ray iç genişliği (iki yarı birlikte) taban +0.6 [TAHMIN, ilk baskıda bak]. Dudak 3 mm tutar | yok |
| kama ↔ masa | kaymaz ayak ×4 | — | — | yok |
| **agirlik_tutucu ↔ frame_L / frame_R kiriş (YENİ)** | vida + mevcut somun | 2× **M4×25 havşa** (satın al), somun alt köprüde (v5'teki) | tutucu 4.3 + havşa 8.6. Kiriş Ø9.6 (vidaya değmez). Sıkınca tutucu kirişin altına bastırır | göz vida ucunda döner (v5 gibi) |
| ağırlık ↔ tutucu | cep + bant/sıcak tutkal | — | cep +0.5 | yok |
| **MG90S ↔ base_R tutucu (DEĞİŞİYOR)** | v5 cebi | M3×10 + somun, milden uzak kulak. Gerekirse kulak altına M3/M4 pul (Z1 ayarı) | cep 22.9×12.7. MG90S 22.8×12.2 [TAHMIN, kumpas] | — |
| MG90S kolu ↔ frame_R dış plaka | v5 yöntemi (kol plakaya 2 küçük sac vida) | servoyla gelen vidalar | plakadaki pilotlar r=6, Ø1.6. MG90S kolunda r [ÖLÇÜM] | çerçeve = servo mili (direkt) |
| frame_L ↔ pivot_bracket | v5 aynen | M4×10 pim | v5. Eksenel boşluk 0.4 (Z1) | çerçeve |
| *(Seçenek B)* frame_L ↔ 2. servo | yeni sol braket + kol adaptörü (boss üstüne geçer, plaka şeridine 2 sac vida) | M3×10 kulak. 2 pilot frame_L şeridine delinir | — | çerçeve |

---

## 4. Montaj sırası

Yönler mekanizma ekseninde (+Y mekanizmanın üstü). "Alet" = tornavida/anahtar giriş yönü. Her adım tersten sökülebilir olmalı: `montaj_sokme` + `giris_yolu` ile denetlenir (§7).

### 4.1 v5'ten sökme (mevcut donanım)
| # | Adım | Parça hareketi | Alet |
|---|---|---|---|
| S1 | Uno'yu kapat, 3 servo fişini çek | — | — |
| S2 | Maske: alttan 2× M4×16 sök | maske **+Y** | alttan **+Y** |
| S3 | pivot_bracket: tabandan 2× M4×16 sök (braket pimle çerçevede kalır) | — | alttan **+Y** |
| S4 | Çerçeve + braket **−X** 5 mm (kol pitch servo milinden çıkar), sonra **+Y** | çerçeve −X, +Y | — |
| S5 | Sol pimi (M4×10) braketten sök, braketi ayır | braket −X | dıştan **+X** |
| S6 | SG90 pitch: M3×10 + somun sök, servoyu cepten çek | servo **−X** | −X yönünden |
| S7 | yaw_mount: kirişin altından 2× M4×16 havşa sök. yaw_mount + yaw servo **+Y** kalkar (mil krank kolundan çıkar) | +Y | alttan **+Y** |
| S8 | Krank ucu vidası (M4×10, alttan) sök. Krank **+Y** kaldır. Cebindeki somunu koru (cep üste açık) | +Y | alttan **+Y** |

### 4.2 v6 takma
| # | Adım | Takma yönü | Alet |
|---|---|---|---|
| T1 | **Krank alt montajı:** somun krank ucundaki cebe (**−Y**, üstten). M4×16 havşaya vida sabitleyici. Vidayı üstten (**−Y**) somuna tak, kafa somuna oturunca sıkıca sık (kilit). Servo kolu kranka v5'teki gibi vidalı kalır | vida −Y | üstten **−Y** (servo yokken serbest) |
| T2 | Krank → coupler: krankı **−Y** indir, vida gövdesi coupler'ın krank ucundaki deliğe girer | −Y | — |
| T3 | Alttan M4 somun (**+Y**) + sabitleyici. Coupler'a değince ~¼ tur geri al (≈0.2 mm). Coupler elle serbest dönmeli | +Y | alttan anahtar/pense **+Y** (z −80, kirişin arkası açık) |
| T4 | Yaw servosu 90° (`firmware/notr_yaw_d3`), gözler düz. yaw_mount + servo **−Y** indir, mil krank kolunun göbeğine girer | −Y | — |
| T5 | yaw_mount → kiriş: alttan 2× M4×16 havşa | +Y | alttan **+Y** |
| T6 | **Ağırlık tutucu:** ağırlığı cebe koy (bant). **Önce** sol göz alt pimi M4×16'yı sök (**−Y**), M4×25'i tutucudan geçirip tak (**+Y**). Sonra sağ göz için aynısını yap. İkisini aynı anda sökme: göz üst pimde asılı kalsın | tutucu +Y | alttan **+Y** (kirişteki Ø9.6 delik) |
| T7 | MG90S'i 90°'ye getir (montaj programı). Kumpasla Z1'i ölç (§6). Gerekirse kulak altına pul koy. Servoyu cebe **+X** sok. M3×10 + somun, milden uzak kulak | +X | −X tarafından **+X** |
| T8 | Kol: SG90 kolu MG90S miline oturursa aynen kalır. Oturmazsa frame_R'den sök, MG90S kolunu vidala. Delikler uymazsa plakaya 2× Ø1.6 pilot del (**servo yokken**) | — | **+X** (dış plakaya) |
| T9 | Çerçeve → taban: braketi sol pimle çerçeveye tak (M4×10, **+X**). Çerçeve+braketi **−Y** indir, sonra **+X** 5 mm: kol MG90S miline (pitch 0 = servo 90) | −Y, +X | — |
| T10 | Braket → taban: alttan 2× M4×16 havşa | +Y | alttan **+Y** |
| T11 | Maske: **−Y**, alttan 2× M4×16 | −Y | alttan **+Y** |
| T12 | Kamalar masaya, kaymaz ayaklar altta. Tüm mekanizmalı taban önden, eğim boyunca aşağı doğru (**−Z**) rayların altından kaydırılır, arka dudağa dayanır. Pitch bu sırada 0 olmalı (servo takılı) | −Z | — |
| T13 | *(İsteğe bağlı)* LIDS A pimine vida sabitleyici: vidayı ¼ tur gevşet, bir damla, geri sık. Kapak serbest dönmeli | — | v5 adım 13 yönü (−X tarafından) |
| T14 | Kablolar arka dudaktaki kertikten Uno'ya | — | — |

**Sökme kontrolü:** her adım tersten yapılabilir olmalı. Kritik olanlar: (a) T4 tersi. yaw_mount +Y kalkarken üstünde engel olmamalı. LIDS kanadı z −37…−21, yaw_mount z ≤ −48.6 → model doğrulayacak. (b) T12 tersi. Taban **+Z** kayarken çerçevenin arka çıkıntısı arka dudağın üstünden geçmeli (pitch 0'da en alçak nokta y −24.4, dudak −57). (c) T6 sırasında göz düşmemeli (üst pim takılı).
**Sıfırdan montaj** (ileride baştan kurulursa): v5 `MONTAJ.md` 1–19 aynen. Farklar: 15. adımdan sonra T1–T3, 17. adımda krank ucu vidası yok (T1–T3'te takıldı) ve T4–T5; maskeden önce T6; en son T12. Pitch servosu MG90S.

---

## 5. Tork bütçesi

### 5.0 Yöntem
v5 modelindeki her parça (`eye_v5.build()`): kütle = hacim × yoğunluk (PLA 1.24 × dolgu, çelik 7.85, SG90 9 g, MG90S 13.4 g, kol 1 g). Konumlar v5 montaj pozundan.
Mekanizma eğik değilken α = 0. Yerçekimi torku T(θ) = m·g·(AM'nin yatay kolu), θ = α + pitch.
**Kalibrasyon kanıtı:** v5 (α0, ±20) için gereken tork 0.53 + dinamik/sürtünme 0.08 = **0.61 kg·cm** çıkıyor. SG90'ın %35 × stall kuralıyla kapasitesi 0.63 → **pay 1.04**. v5'te "aşağı bakarken zorlanıyor" gözlemiyle uyumlu. Bu yüzden **%35 kuralı esas** alınır. %67 kuralı (proje kabulü 1.2) yalnız bilgi olarak verilir.
Pay hedefi ≥ 1.5. Yük, harici kaynak olmadığı için Uno 5 V'ta (≈4.8 V) ölçülür.

### 5.1 Tablo (pitch, dolgu 0.75 esas; parantezde dolgu 1.0)
| Durum | Yerçekimi T_max | + dinamik¹ + sürtünme² | Gereken | Kapasite (%35) | **Pay** |
|---|---|---|---|---|---|
| v5: α0, ±20, SG90 | 0.53 | 0.06 + 0.02 | 0.61 | 0.63 | 1.04 ✗ (gözlendi) |
| α30, ±20, MG90S | 0.47 | 0.08 | 0.55 | 0.63³ | 1.15 ✗ |
| α30, −10…+20, MG90S | 0.42 (0.50) | 0.08 | 0.50 (0.58) | 0.63 | 1.27 (1.08) ✗ |
| **α30, −10…+20, MG90S + 75 g** | **0.30 (0.38)** | 0.08 + 0.02⁴ | **0.40 (0.48)** | 0.63 | **1.58 (1.31)** ✓ (✗ → ağırlığı 130 g'a çıkar: **1.54**) |
| α30, −10…+20, MG90S + SG90 sol (Seç. B, ağırlıksız) | 0.42 (0.50) | 0.08 | 0.50 (0.58) | 2 × 0.63 × 0.8⁵ = 1.01 | 2.03 (1.73) ✓ |
| α20, −10…+20, MG90S + 75 g | 0.40 | 0.10 | 0.50 | 0.63 | 1.27 ✗ |
| α40, −10…+20, MG90S + 75 g | 0.19 | 0.10 | 0.29 | 0.63 | 2.2 ✓ (devrilme ✗, §1.3) |

¹ Dinamik: dönen atalet I ≈ 3.4·10⁻⁴ kg·m² (75 g ile 4.5·10⁻⁴). 10° hareket 0.1 s'de 0.24 kg·cm ister, **0.25 s'de 0.05**. → Firmware'de pitch hareketi ≥ 0.25 s / 10° ile sınırlanır (yaw ve kapak hızlı kalır). Bu sınır olmadan pay tutmaz.
² Sürtünme: sol pim μ0.3 × 1.1 N × 2 mm ≈ 0.007 kg·cm. Servo kabloları sürüklenmesi [TAHMIN] 0.02.
³ MG90S stall 1.8 kg·cm @4.8 V (TowerPro; bazı satıcılarda 2.0–2.2) → %35 = 0.63. **Kağıt üstünde MG90S, SG90'dan güçlü değil.** Kazancı metal dişli (dişli sıyırma yok) ve pratikte biraz daha yüksek tork [TAHMIN]. Asıl iyileşme eğim + ağırlık + aralık daraltmasından gelir.
⁴ Ağırlıkla atalet artar → dinamik 0.06 → 0.08.
⁵ İki servonun ayar farkıyla birbirine karşı çalışması için %80 verim [TAHMIN].

### 5.2 Neden eğim torku azaltıyor
AM eksenin arkasında ve hafif altında (φ ≈ −162°). Burnu yukarı eğmek AM'yi dikeye yaklaştırır. Kol 22 mm'den α30'da ~16 mm'ye iner (−%11).
Ağırlık eksenin **önü-altında** (y −38, z +3). Eğik mekanizmada bu nokta masaya göre öne düşer ve arka AM'yi dengeler. **Eğim olmadan ağırlık işe yaramaz:** α0'da payı 1.04 → 0.92'ye düşürür. Yani ağırlık eğimle birlikte gelen bir çözüm.
CG'yi eksene yaklaştırmanın diğer yolları, rakamlarla:
- Eksenin önüne-üstüne ağırlık: yer yok (göz, kapak, maske). Sol dış plakada 8 mm şerit var ama 337 g gerekir.
- Yay/lastik (tabandan yaw_mount'u iten ya da arkadan çeken): yerçekimi torku aşağı bakışta artarken yay kuvveti azalır. Ters eşleşiyor. ±19 mm yol ile ~±%50 kuvvet değişimi → reddedildi.
- yaw servosunu/yaw_mount'u öne almak: çerçeve ve paralelkenar yeniden basılır → prensip 7'ye aykırı, reddedildi.
- Pitch'e 1:1.7 kol-çubuk redüksiyonu: pay ~1.9 olur ama +3 parça, sağ yatak ve servo yeri değişir → seçenek dışı.

### 5.3 Ağırlık ayarı
Gerçek dönen kütle ölçülünce (§7-R1) ağırlık şu kurala göre seçilir: pay(%35) ≥ 1.5 sağlayan en küçük ağırlık. Dolgu 0.75 → 75 g, dolgu 1.0 → 130 g.
Sıfır torkun pitch'i (dengenin oturduğu yer) α30 + 75 g'da θ ≈ 47°, yani pitch ≈ +17. Elektrik kesilince göz yavaşça yukarı bakar. Bu güvenli (arka masaya inmez, §6 Z3).

### 5.4 Diğer eksenler
| Eksen | Yük | Gereken | Kapasite | Pay |
|---|---|---|---|---|
| Yaw (SG90) | Eğimle pim ekseni α+p kadar yatar. Coupler + kollar (~7 g) r 20'de, yerçekimi en kötü 0.004 kg·cm. Gözlerin AM'si pim ekseninde (z 0.2) | ~0.01 + sürtünme [TAHMIN 0.1] | 0.63 | > 5 (v5'te donanımda çalışıyor) |
| LIDS (SG90) | 4 kapak 14 g, AM'leri X ekseninden ≤ 12 mm → ≤ 0.007 kg·cm. Dört çubuk iletim açısı ≥ 47° | ~0.05 [TAHMIN] | 0.63 | > 5 (v5'te çalışıyor) |

**Seçenek B (ikinci pitch servosu, sol, D9, ters açı):** Ağırlığa gerek kalmaz (pay 2.03 / 1.73). Bedeli:
- yeni `sol_servo_braketi` (pivot_bracket'ın 2 deliğine oturur, servo gövdesi taban kenarından ~19 mm dışarı taşar)
- yeni `sol_kol_adaptoru` (frame_L dış göbeğine geçer, plaka şeridine 2 sac vida → frame_L'ye 2× Ø1.6 delik)
- firmware'de 4. kanal
- iki servonun ayarı 1–2° farklı olursa birbirine karşı çalışır, stall akımı çeker (2 × ~0.7 A [TAHMIN])
- Uno 5 V'ta brown-out riski (v5'te USB'de görüldü)
Öneri: **A (ağırlık)**. B yalnız A donanım testinde tutmazsa (§7-R11).

---

## 6. Kritik tolerans zincirleri

**Z1 — Pitch ekseni eksenel (sol göbek ↔ braket boşluğu), MG90S değişimi sonrası.** Hedef: boşluk 0…1.0 mm (< 0 → sol göbek brakete sürter, tork yer). v5'in kurulu hali taban kabul edilir; yalnız değişenler:
| Halka | Nominal | ± |
|---|---|---|
| v5 sol boşluk | +0.4 | [ÖLÇÜM: sentil/kağıt şerit] |
| MG90S mil ucu yüksekliği − SG90 (kulak alt yüzünden mil ucuna) | 0 [TAHMIN] | ±1.0 (kumpas: iki servoda da ölç) |
| Kol kalınlığı farkı (MG90S kolu kullanılırsa) | 0 | ±0.3 |
| Servonun cepte yeniden oturması (cep boyu payı 0.4) | 0 | ±0.2 |
| Braketin yeniden vidalanması (4.3 delik / M4) | 0 | ±0.15 |
En kötü: 0.4 ± 1.65 → **−1.25…+2.05**. RSS: 0.4 ± 1.07 → **−0.67…+1.47** → ölçmeden kabul edilmez.
Düzeltme: T7'de mil yüksekliği farkı ölçülür. MG90S mili SG90'dan Δ uzunsa kulak altına Δ kalınlıkta pul konur. Kısaysa pim gömülmesi 4.6 → 4.6−Δ (Δ ≤ 2'ye kadar yeterli).

**Z2 — Krank ucu vida kafası ↔ yaw servo gövdesi (y yönü).** Kafa, krankın r 16–24 bölgesinde. Servo gövdesinin ucu milden 16.7 mm'de, ±25° yaw'da kafa gövde izinin ~0.7–2.6 mm içine girer. Hedef ≥ 0.4.
| Halka | Nominal | ± |
|---|---|---|
| Servo gövde üst yüzü (mil ucundan 2.8 yukarıda) | y −9.1 [TAHMIN, SG90 VARSAYIM] | ±0.3 |
| Somun üst yüzü (cep tabanı −15.3 + somun 3.2) | −12.1 | somun 2.9–3.2 → −0.3/0 (boşluğu artırır); cep tabanı baskı ±0.1 |
| Havşa kafa yüksekliği (ISO 10642 maks 2.48) | +2.48 | −0.28/0 (boşluğu artırır) |
| Kol kalınlığı (krank–servo arası) | 1.5 [VARSAYIM] | ±0.2 |
Nominal boşluk **0.52**. Yalnız kötü yönler: en kötü **−0.08** (sürtme), RSS **+0.15** → hedefin altında.
Düzeltme: T1'den sonra krankı servo yokken ölç. Krank alt yüzünden kafa tepesine ≤ 7.0 mm olmalı. Fazlaysa kafa tepesi Dremel ile 0.5 mm taşlanır (metal vida, prensip 7 dışı). Kafanın somun pahına gömülmesi (+0.2…0.4) hesaba katılmadı, lehte.

**Z3 — Arkanın en alçak noktası ↔ kama arka dudağı ve masa (pitch +20, α30).** En alçak noktalar: krank ucu vida/somunu, r ≈ 84 mm (pitch +20'de y ≈ −51.4, z ≈ −66) ve yaw_mount alt-arka köşesi.
| Halka | Nominal | ± |
|---|---|---|
| Dudak tepesi y −57'ye boşluk | +5.6 | — |
| Pitch aşımı (kalibrasyon hatası ±2° × 84 mm) | 0 | ±2.9 |
| Vida ucu / somun konumu (Z3 vidası, T3 boşluğu) | 0 | ±0.5 |
| Kama baskı yüksekliği / açı (±0.5° × 80 mm) | 0 | ±0.7 |
| Taban kalınlığı | 0 | ±0.2 |
En kötü **+1.3**, RSS **+2.6** ✓ (hedef ≥ 0.4). Masaya: nominal 5.5 mm, aynı halkalarla en kötü +1.2 ✓.
Firmware'de pitch üst sınırı servo 110'u (pitch +20) geçmemeli. Kalibrasyonda +20'nin gerçek açısı telefon eğim ölçeriyle doğrulanır.

---

## 7. Kontrol kapsam matrisi

| # | Risk | Ölçen | Mevcut / YENİ | Ölçülemiyorsa: kim, nasıl |
|---|---|---|---|---|
| R1 | Dönen kütle ve AM tahmini (dolgu 0.6–1.0 → tork ±%25) | v6 `check` içinde kütle/AM tablosu | **YENİ `checks/yercekimi_tork.py`**: parça hacmi × yoğunluk × dolgu, AM, T(θ) = f(α, pitch aralığı, ağırlık), dinamik terim, iki kural (%35 esas, %67 bilgi), pay ≥ 1.5. KAPSAM: dolgu ve servo kapasitesi = girdi (ÖLÇÜLMEDİ). Agentic-mechanism-design L1 `check_tork.py` ile aynı iş, tek yerde | **Kullanıcı:** S4'ten sonra çerçeveyi (gözler, kapaklar, yaw servo, LIDS servo takılı) mutfak tartısında tart (±1 g). Sonra yatay bir kalemin üstünde iki konumda dengele (taban aşağı / ön yüz aşağı) → AM'nin z ve y'si ±3 mm. Alternatif: Orca CLI ile v5 STL'lerini "Epic POC" profiliyle dilimle, parça başı gram |
| R2 | Pitch payı < 1.5 | `yercekimi_tork` (yukarıdaki) | YENİ | Servonun gerçek torku → R11 |
| R3 | Çakışma: kama, ağırlık tutucu, ters vida + alt somun, tüm pozlar | `checks/cakisma.py`, `gruplar` ile. Pozlar: yaw {−25,0,25} × pitch {**−13**, −10, 0, 10, 20, **23**} (±3 aşım) × kapak {0, 0.5, 1} | mevcut | — |
| R4 | Eğik duruşta masaya / kama dudağına yakınlık | `checks/zemin.py`, zemin = masa düzlemi (α'ya göre). Kama dudağı ve raylar ayrıca çakışmada | mevcut | — |
| R5 | Geri devrilme | **YENİ `checks/devrilme.py`**: tüm parçaların AM'si (pitch ve kapak pozlarında). Masaya izdüşüm ile destek kenarı arası ≥ 15 mm ve devrilme açısı ≥ 15°. α20/30/40 için rapor | YENİ | Kullanıcı: kurulu halde arkadan parmakla hafif it, ~20° yatınca geri dönmeli |
| R6 | Montaj/sökme sırası (T1–T12), özellikle T4 yaw_mount iniş yolu, T12 kayma yolu, T6 | `checks/montaj_sokme.py` (T12 parçalı yol), `checks/giris_yolu.py` (T1 üstten alet servo yokken, T3 alttan anahtar, T6 alttan alet kirişteki Ø9.6'dan) | mevcut | Somunun cebe konma yolu ve elle tutma ölçülmez → kullanıcı |
| R7 | Vida boyları: M4×16 havşa (krank, ≥1 diş taşma), M4×25 (göz pimi uçta ≥ 4.5 mm) | `checks/vida_envanter.py`, `checks/vida_adet.py` | mevcut | — |
| R8 | Havada vida/somun (alt emniyet somunu, M4×25) | `checks/havada.py kontrol_donanim_temas` | mevcut | — |
| R9 | Yeni parçalar baskı: tabla ≤ 200, et ≥ 1.6, ada, ray/dudak konsolu | `checks/tabla.py`, `checks/havada.py` (tek gövde, ada), `checks/sarkma.py` | mevcut | — |
| R10 | Tolerans zincirleri Z1–Z3 | `checks/tolerans.py` (zincirler §6'dan aynen) | mevcut | Z1 ve Z2'nin [TAHMIN] halkaları → R12 ölçümleri |
| R11 | MG90S'in Uno 5 V'ta gerçek torku | ölçülemez (CAD) | — | **Kullanıcı tutma testi** (`checks/test_esik.py` ile eşik doğrulanır): MG90S'e 20 mm'lik basılı/kit kol. 315 g ağırlık = 0.63 kg·cm, 2 dk, Uno 5 V'tan, servo 90'da tutmalı. Sonra kurulu halde: pitch −10…+20 yavaş tarama ×10 + 5 dk idle, takılma/titreme/reset yok. Başarısızsa Seçenek B |
| R12 | MG90S ölçüleri: gövde boy/en, kulak delik aralığı, kulak altından mil ucuna, kol delik yarıçapı, dönüş yönü | ölçülemez | — | **Kullanıcı kumpas** (T7 öncesi). SG90 ile aynı ölçüler karşılaştırmalı. Dönüş yönü: 80→100 komutunda kol hangi yöne → `invert` |
| R13 | Vida zamanla gevşer mi (krank ucu, A pimi) | ölçülemez | — | **Kullanıcı:** vida ve somuna boya kalemiyle çizgi çek, 30 dk idle + stant sonrası çizgi kaymış mı bak |
| R14 | Uno 5 V düşmesi (brown-out) | ölçülemez | — | **Kullanıcı:** multimetre 5V–GND, pitch taraması sırasında ≥ 4.6 V; reset olmamalı |
| R15 | Basılı kamanın gerçek açısı | ölçülemez | — | **Kullanıcı:** telefon eğim ölçer taban üstünde, hedef α ±1° |
| R16 | v5 parçaları değişmemeli (prensip 7) | **YENİ küçük kontrol** (v6 check içinde): v6 build'deki v5 parçalarının hacim + sınır kutusu v5 STEP'leriyle aynı (0.01 tolerans). Yeni parça adları ayrı listede | YENİ | — |
| R17 | Kablo boyu (çerçeve → taban → Uno) pitch −13…+23 | `checks/kablo.py` | mevcut | Gerçek kablo yolu → kullanıcı |
| R18 | Yazılım: bakış eşlemesi (dünya yüksekliği → pitch = yükseklik − α), pitch hız sınırı, limit 80–110 | `v5/control` testlerinin v6 kopyası (ayrı iş, CAD dışı) | — | Donanımda kullanıcı yüz takibiyle |

---

## 8. Satın alma (envanter dışı)

| Kalem | Adet | ~TL | Not |
|---|---|---|---|
| Vida sabitleyici, orta güç (Loctite 243 ya da muadil) | 1 | 150–250 | Elde varsa 0. Yedek: şeffaf oje |
| M4×25 havşa (DIN 7991 / ISO 10642) | 2 (+2 yedek) | 10–20 | Ağırlık tutucu = göz alt pimi |
| Ağırlık 75–130 g | 1 set | 0–40 | Madeni para (1 TL ≈ 8.2 g [TAHMIN], 9–16 adet) ya da M10 somun 7–12 adet |
| Kaymaz ayak (yapışkanlı silikon/keçe, Ø10) | 4 | 30–50 | |
| M3/M4 pul (Z1 ayarı için) | 10'lu | 10 | Elde varsa 0 |
| *(Seçenek B)* 2. servo SG90 | 1 | 60–90 | Elde varsa 0 |
**A toplamı ~200–370 TL.** MG90S elde (1 adet). M4 somun (+1), M4×16 havşa (aynı adet, yön değişiyor) elde.

---

## 9. Açık kararlar (kullanıcı onayı — kısa)

1. **Eğim açısı 30° olsun mu?** (20° = daha az eğik ama uzun boyluya yakından yetmez; 40° = daha dik ama arkaya devrilmeye yakın)
2. **Düşen vida arkadaki (yaw servosunun altındaki) vida mıydı?** Plan bunu varsayıyor. Göz kollarındakiler düşemez.
3. **Göz en fazla 10° aşağı bakabilsin (şimdi 20°), karşılığında motor rahatlasın: tamam mı?** Masaya göre 20°–50° yukarı bakar.
4. **Denge için çerçevenin önüne ~75 g ağırlık (madeni para) mı, yoksa sola ikinci bir servo mu?** Önerim ağırlık: daha az parça, daha az akım.

Tasarımcıya kalan teknik notlar (kullanıcıya sorulmaz): MG90S kolunun r=6 delik uyumu (T8), Z2 için kafa yüksekliği ölçümü (T1).

---

## 10. CAD dışı işler (G1'den sonra, ayrı)
- `settings.json`: `pitch_dead: false`. Yeni `mount_tilt_deg: 30` (face_follow bakış yüksekliğini pitch'e çevirirken çıkarır). `EYE_PITCH` min/max 80/110, `invert` R12'ye göre.
- Firmware: pitch limiti 80–110. `IDLE_GAZE_V_MAX` 0 → 0.45 geri. **Pitch hız/ivme sınırı** (≥ 0.25 s / 10°). `kinematics.json` pitch tablosu −10…+20.
- Seçenek B seçilirse: D9 kanalı, ters açı (180 − a), ayrı kalibrasyon ofseti.

---

## 11. Kullanıcı kararları (2026-10-12)

1. Eğim **30°** — onaylandı.
2. Düşen vida **arkadaki krank ucu vidası** — onaylandı (plan varsayımı doğru).
3. Aşağı bakış **−10°** (pitch −10…+20, servo 80–110) — onaylandı.
4. Denge: **Seçenek B — ikinci pitch servosu (solda, D9, ters açı)**. Öneri A idi; kullanıcı B'yi seçti.
   Sonuç: `agirlik_tutucu` YOK, M4×25 gerekmez; yeni `sol_servo_braketi` + `sol_kol_adaptoru`, frame_L şeridine 2× Ø1.6 pilot delik (basılı parçaya tek dokunuş, matkapla).
   Açık risk: iki servo Uno 5V'tan → firmware'de iki pitch kanalı AYNI anda ve hız sınırlı sürülmeli, ayrı kalibrasyon ofseti; ilk testte reset/titreme izlenecek.
