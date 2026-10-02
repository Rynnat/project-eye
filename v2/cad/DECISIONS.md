# Project Eye v2 — Mekanik kararlar ve şartnameden sapmalar

Bu dosya yalnızca **mekanik iş kolunun** kararlarını içerir (`cad/`). Etiketler: **[SPEC]** şartnameye uygun,
**[SAPMA]** şartnameden ayrılır (gerekçeli), **[SEÇİM]** şartnamenin serbest bıraktığı yerde verilen karar,
**[DOĞRULANMADI]** yalnızca modelde/hesapta var, fiziksel olarak denenmedi.

Tüm sayılar `params.py`'dedir; buradaki değerler oradan kopyadır. Doğrulama: `check.py` → `out/check_report.txt`.

## 1. Genel mimari

| Eksen | Mekanizma | Neden |
|---|---|---|
| Göz yaw (kanal 0) | Kardan **göbeğine** üstten sıkılan tüp+kol; iki göz kolu + ortadaki servo kolu (hepsi r=10, eksenleri z=0 doğrusunda) tek çubukla **paralelkenar** | Göbek yalnızca yaw yapar → yaw = servo açısı, **birebir** ve pitch'ten tamamen bağımsız. İlk denemede (gözün arkasındaki topa itme çubuğu) pitch ±25'te yaw servosu ~10° kayıyordu. |
| Göz pitch (kanal 1) | Gözün **alt kutbundaki** top (yaw ekseni üzerinde) → basılı çubuk → ortak sallanan mil (rocker) → sol servo, mil eş eksenli | Top yaw ekseninde olduğundan pitch=0'da yaw hiç etkilemez. İki göz aynı geometri → sağ/sol pitch eşit. |
| Kapaklar (kanal 2–5) | Her kapak kendi 4-çubuk mekanizması (kapak kolu 12 → bağlantı kolu → servo kolu 10), servolar arka kirişin arkasında | Bağımsız kapaklar (SPEC §4); nötrde kol ve servo kolu bağlantıya dik (ölü nokta kuralı). |

## 2. Şartnameden sapmalar [SAPMA]

1. **Göz çatalı köprüsü** v1'de z=4.5–6.5 idi; pitch ±25'te göbeğin köşesi (4,4) z=5.32'ye süpürüyor → köprü **z=5.7–7.9**'a alındı, ön yüzde ±1.3'e pahlandı; kollar z>2.5'te ±2'ye daraltıldı. (SPEC'in sabit olarak verdiği göbek 8, M2×5, kol 2 mm, boşluk 0.5 aynen korundu.)
2. **Sabit çatal**: kollar ±3 yerine **±4.2** (üst kolda yaw tüpü için Ø4.85 yatak); köprü z=−8…−6 (v1 −9…−7'ye yakın); **direk Ø4 yuvarlak** (v1: Ø5). Köşe pozunda (yaw 30 + pitch 25) direk yönü göze göre 38.3° sapıyor; Ø4 direk + 52° arka açıklık 2.6° pay bırakıyor. Ø5 ile açıklığın ~54°'ye çıkması gerekiyordu, bu da alt pad'in kapak bandına girmesine yol açıyordu.
3. **Bebek Ø3.8** (SPEC Ø4): bebek, göz çatalını kabuğa tutturan **siyah M2 havşa başlı vidanın başıdır** (DIN 965, baş Ø3.8). İris (Ø12) modelde yalnızca görsel gövde; boya/çıkartma ile yapılır.
4. **Vida sabitleme şeması** v1'den farklı: M2 vidalar **çatal kollarına** kılavuzsuz vidalanır, göbek delikleri **geçiş** (Ø2.2) — göbek vida uçları üzerinde döner. v1 çiziminde hangisinin döndüğü belirsizdi.
5. **Göz kabuğunda erişim delikleri** (Ø4.4): iki yanda (yatay kardan vidaları) ve altta (sabit çatalın alt vidası). Montajda göbek kabuğun içinde vidalanmak zorunda; bu delikler olmadan kardan kurulamaz. Yan delikler uç yaw'da pencereden görülebilir → isterseniz boya/tapa.
6. **Kabuk üst yarığı**: yaw tüpü (Ø4.6) kabuğun tepesinden çıkar; kabukta pitch ±25 boyunca 5.8 mm genişlikte yarık var. Göz −25 aşağı bakarken yarığın ön kenarı dünya açısında ~+26°'ye gelir: üst kapak nominal açıkken (kenar +22) örtülür; üst kapak **tam açık + aşağı bakış** kombinasyonunda yarık görünebilir. [DOĞRULANMADI: görsel]
7. **Pitch çubuğu basılı**, uçlarında geçmeli küresel yuva (ağız yarı açısı 61°, top Ø4.8 metal RC rotil topu). Hazır RC rotillerin açılma açısı bilinmediği için yuva geometrisi bizde. Göz ucunda gereken eğim en fazla **20.8°**, yuvanın izni ~31°.

## 3. Serbest bırakılan yerlerde seçimler [SEÇİM]

- **Kapak genişlikleri**: üst 38°, alt 30°. Kapanma: kenarlar −5°'te buluşur, aralarında 1.5° (~0.35 mm) boşluk (sürtünme yok). Mekanik tam açık: üst kenar +38, alt kenar −45 (nominal açık +22/−28 + bakış takibi 0.6×25 payı). Üst kapak dilimi tam açıkta 76°'ye çıkar; yaw tüpü 90°±10°'de → 4° pay. Alt kapak dilimi −75°'e kadar iner; alt pad'in kapak bandını geçtiği dar şeritle (|x|≤2.5) köşe pozunda 3.6° pay.
- **Kapak dilimi kutuplarda koniyle kesilir** (düz kesim ince bıçak kenarı bırakıyordu). Göbekler eksenel yığın: alt kapak 13–15, üst kapak 18.4–20.4 mm (göz merkezinden), kanat (sabit pim taşıyıcı) 20.8–24.8. Kollar yalnız dış tarafta.
- **Alt pad + kanca kabukla tek parça** (kabuk kenar düzlemi tablada basılır; kanca paddan dik yükselir). Pad çerçevesinin kapak bandını (r 12.6–13.8) geçen kısmı yalnızca |x|≤2.5 şerit: yaw+pitch birleşince x≠0 noktaları öne döner ve alt kapağa girer. r≥14.5'teki kısımlar hiçbir pozda kapak kabuğuna giremez (dönme r'yi korur).
- **Gözler ve pitch kolları aynalı**: pad çerçevesinin geniş kısmı her gözde **iç tarafta** (dış tarafta kapak kolları/bağlantıları var; uç pozlarda çerçeve u≈19 mm'ye savruluyor). Sağ rocker kolu dış tarafta. Top x=0'da olduğundan kinematik değişmez.
- **Pitch çubuğu nötrde arkaya doğru 12° yükselir**, rocker kolu 16 mm: yaw×pitch ızgarasında sayısal tarama; göz ucu rotil eğimini 37° → 20.8°'ye indirir, servo −31…+32°.
- **Kapak servoları z=−48.5'te** (arka kirişin arkasında), milleri dışa bakar, iki servo tek tutucu plakada (üst kapak servosunun kulakları yükseltilmiş pedlerde).
- **Maske** z=16.5–19.5, göz pencereleri 27×14 elips. Göz yukarı bakınca pitch kancası z≈15.2'ye gelir.
- **Servo kolu delik yarıçapı r=10** (v1'de kullanılan; SPEC: delik aralığı 2 mm). Stok kolların bu deliği Ø1.6'ya büyütülüp M2 vidalanır.

## 4. Çapraz etki ve firmware sınırları (yazılım tarafı için önemli)

- **Yaw**: çapraz etki yok.
- **Pitch**: pitch=0'da yaw etkisi sıfır; köşelerde var. Yazılım yaw=0 tablosuyla (−25 → servo 56.7) sürerse, yaw ±30'da göz **−31.8°**'ye gider. O bölge doğrulanmadı ve orada yarık/direk çakışması var.
- Bu yüzden **firmware'in sabit pitch sınırı** yaw'dan bağımsız güvenli olacak şekilde önerildi: servo **63.2…121.6°** (her yaw'da göz ±25 içinde; yaw=0'da ulaşılan −20.3…+23.0). Tam ±25 isteyen yazılım `kinematics.json → cross_coupling.EYE_PITCH_servo_deg_grid` ile yaw'a bağlı sınır uygulamalı. `check.py` bu güvenli sınırları yaw ızgarasıyla da tarar.
- SPEC §6'nın doğrusal pitch modeli yaklaşık: +25 → 125.6, −25 → 56.7 (asimetrik). Tablo daha doğru.

## 5. Doğrulama yöntemi ve sınırları

- Çarpışma: CadQuery katıları 0.01 mm sapmayla üçgenlenir; kesişim hacmi manifold3d boolean, mesafe python-fcl. Eşik 0.01 mm³.
- Vidalar kök çapında (Ø1.6) modellendi: diş plastiğe gömülür. Pim Ø1.95 (sıkı geçme deliğiyle eş).
- Eklem çiftleri (pim-göbek, vida-link, kol-mil, top-yuva, tüp-yatak) tasarım gereği 0–0.15 mm boşlukla temas eder. Raporda **eklem dışı** en küçük boşluk ayrıca verilir.
- Et kalınlığı ışın yöntemiyle örneklenir (kenar/köşe örnekleri ayıklanır); 1 %'lik değer raporlanır.
- **Doğrulanmayanlar**: baskı ve montaj toleransları (tolerans test parçası sonrası `params.py [TOL]` güncellenecek), SG90'ın datasheet dışı ölçüleri (kubbe yarıçapı, mil yüksekliği, kulak kalınlığı, delik aralığı 27.8 — varsayım), servo kolu ölçüleri, basılı yuvanın geçme kuvveti, servo torku yeterliliği (hesaplanmadı), dişli boşluğu, sürtünme, esneme. Model rijit ve boşluksuz eklemlidir.

## 6. Dosyalar

- `params.py` tüm ölçüler · `kin.py` kinematik (numpy) · `eye_v2.py` parçalar + montaj + dışa aktarım · `check.py` tarama · `kinematics_export.py` → `out/kinematics.json` · `render.py` → `out/renders/`
- `kin.py`, `kinematics_export.py`, `render.py` SPEC §7 listesinde yok; `eye_v2.py`'yi sade tutmak için ayrıldı.

## 7. Tolerans sabitleri (`params.py` §1 ve ilgili yerler) — test baskısından sonra güncellenecek

Hepsi adlandırılmış sabit; `eye_v2.py`/`check.py` içinde gömülü tolerans yok (kalan ±0.1 değerleri yalnızca boolean kesim payı).
`M2_PILOT 1.7` · `M2_CLEAR 2.2` · `PIN_PRESS 1.95` · `PIN_RUN 2.25` · `AXIAL_GAP 0.4` · `SERVO_POCKET_CLR 0.3` ·
`CBORE_D 4.2` · `CSK_CLR 0.15` · `GLUE_GAP 0.05` · `YAW_COUPLER_HOLE 2.3` · `HORN_SCREW_PILOT 1.6` ·
`SOCKET_CAV_CLR 0.1` · `YAW_TUBE_BEARING_CLR 0.25` (→ `YAW_TUBE_BEARING`) · `YAW_SLOT_CLR 0.6` · `SCREW_HEAD_GAP 0.2`.
Kaldırılan: `NUT_POCKET`, `NUT_M2_*` (tasarımda somun kalmadı). Kapak yığını (`LID_*_U`) ve `YAW_COUPLER_Y` artık
`AXIAL_GAP`'ten türetiliyor. Bir tolerans değişince: `python eye_v2.py && python check.py && python render.py`.

## 8. Doğrulama özeti (son çalıştırma, `out/check_report.txt`)

- 18 zorunlu + 12 firmware-sınır + 87 ara poz = 117 poz: **kesişim ≥ 0.01 mm³ olan çift: 0**.
- Eklem dışı en küçük boşluk **0.173 mm** (göz çatalı ↔ sabit çatal, yaw ±30 + pitch −25 köşesi). Bu, baskı toleransı
  mertebesinde; ilk montajda köşe pozunda sürtünme olup olmadığına bakın. Sonrakiler: kapak↔kapak 0.18 (kapalı), kabuk↔direk 0.29.
- Geçersiz katı: 0; basılan parçaların hepsi tek katı. Et kalınlığı %1 yüzdelik ≥ 1.2 (lokal minimumlar havşa/pah kenarı).
