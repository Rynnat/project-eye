# Project Eye v3 (POC) — mekanik kararlar

Etiketler: **[SPEC]** şartnameye uygun · **[SAPMA]** şartnameden ayrılır · **[SEÇİM]** serbest bırakılan yerde verilen karar ·
**[DOĞRULANMADI]** sadece modelde/hesapta var, fiziksel olarak denenmedi. Tüm sayılar `params.py`'de. Doğrulama: `out/check_report.txt`.

## 1. Mimari (v2'den farkı: her bağlantı düzlemsel, 3B mafsal yok)

| Kanal | Mekanizma | Neden |
|---|---|---|
| EYE_PITCH (1) | İki göz ortak bir **pitch çerçevesinde** (frame_L + frame_R + yaw_mount). Çerçeve göz merkezlerinden geçen X ekseninde döner: **sağda pitch servosunun koluna vidalı (direkt)**, solda sabit yatakta M4 pim. | Eksen göz merkezinden geçtiği için pitch = servo açısı (birebir). Lama/rotil yok. |
| EYE_YAW (0) | Her göz çerçevede **dikey M4 pimlerle** (üst + alt) döner. Yaw servosu **çerçevenin üstünde taşınır**; iki göz kolu + servo krankı (r=20) tek düz lamayla paralelkenar. | Servo çerçeveyle birlikte eğildiği için paralelkenar her pitch'te düzlemsel kalır → 3B mafsal gerekmez (SPEC kural 2'nin "eksenleri paralel tut" seçeneği). Kol/krank nötrde lamaya dik (kural 6). |
| LIDS (2) | İki üst kapak aynı X ekseninde; ikisi tek **taşıyıcıya** (lid_carrier) 2'şer M4 ile bağlı. Taşıyıcı ortadaki servonun koluna vidalı (direkt). | Kapak ekseni = göz merkez ekseni → kapak-göz boşluğu her pozda sabit. |

- **Pim şeması [SPEC]:** vida sabit parçanın `M4_PILOT` deliğine vidalanır (kafa havşalı/yüzeyde), dönen parça `M4_RUN` deliğinde döner, arada `AXIAL_GAP` 0.4. Göz pimleri: vida çerçevede, göz döner. Sol pitch pimi: vida çerçeve yan plakasında (çerçeveyle döner), sabit yatakta (`base`) RUN deliğinde döner — kafa yatağın dış yüzüne sürtünür.
- **Direkt sürüş [SEÇİM]:** pitch ve kapaklarda lama yok → lamaya-dik kuralı bu iki kanalda geçersiz, ölü nokta yok. Kapak servosunda 90° = yarı açık (aralık ortada).
- **Parça sayısı:** 12 farklı STL (14 basılı adet): eye ×2, eye_lever ×2, coupler, yaw_crank, frame_L, frame_R, yaw_mount, lid_L, lid_R, lid_carrier, pivot_bracket, base.
- **Sol yatak ayrı parça (pivot_bracket) [SEÇİM]:** çerçevenin sağ yan plakası servo miline eksenel olarak geçirilmek zorunda; sabit bir sol yatak (0.4 boşluk) bunu engellerdi. Yatak çerçeve takıldıktan sonra alttan 2× M4×16 ile base'e vidalanır.

## 2. Şartnameden sapmalar [SAPMA]
1. **Kapak göze 3.3 mm uzakta** (kapak iç küresi R21.3, göz R18). Gözün üst pimini taşıyan çerçeve kolu gözle kapak arasından geçmek zorunda (kol küre kabuğu 18.4–20.8, 2.4 et). Görsel olarak kalın kapak gibi görünür.
2. **Alt kapak yok [SPEC]**, ayrıca **maske/yüz plakası yapılmadı** (zaman). Önden bakınca ortadaki kapak servosu ve direk iki gözün arasında görünüyor.
3. **Gözde düz yüzeyler:** üst/alt y=±13 düzlemleri (pim yüzleri, alt yüzey baskı tablası) ve arka z=−10 kesiği. Önden görünmez; göz tam aşağı bakınca alt düzlemin kenarı görülebilir.
4. **Yaw kolu kilit vidası gözün önünde** (z=+8.5): arkada yer yoktu (pim deliğiyle havşa çakışıyordu). Kol gözün altından ~15 mm öne taşar; maske yoksa alttan görünür.

## 3. Baskı yönü kuralları (orkestratör notu uygulandı)
- **Dikey delik kuralı:** göz, göz kolu, lama, krank, yaw_mount, kapak ve çerçevenin X-ekseni delikleri baskıda **dikey**. Tablaya değen delik ağızlarında `BOTTOM_CHAMFER` 0.4×45° pah var.
- **Yatay kalan delikler (+`HORIZ_HOLE_EXTRA` 0.2 mm verildi, test bunları ölçmüyor):**
  - `frame_L/R`: üst ve alt göz pimi pilotları (M4_PILOT 3.5 → 3.7), yaw_mount pilotları (→ 3.7). Çerçeve yan plakası tablada basıldığı için (küre kabuk kol başka yönde desteksiz basılamaz) Y-ekseni delikleri yatay kalır. Pilot olduklarından (vida sabit, dönmez) risk düşük; vida gevşek tutarsa bir damla CA.
  - `lid_carrier`: 4× kapak vidası RUN (4.3 → 4.5, rijit bağlantı, önemsiz), servo kolu vidaları ve orta delik.
  - `pivot_bracket`: **sol pitch yatağı M4_RUN (4.3 → 4.5)** — tek kritik yatay yatak (dönen eklem). Sıkıysa 4.6 matkapla açın.
  - `base`: 4× servo kulağı M3 pilotu (2.7 → 2.9).
- Teardrop profil kullanılmadı; basit +0.2 çap payı seçildi.
- STL kontrolü (`check.py §7`): her STL tek gövde, z_min = 0, 0.2 mm katmanlarda havada başlayan ada yok. Kabartma yazı yok.

## 4. Seçimler [SEÇİM]
- Göz Ø36, IPD 95 [SPEC]. Göz dolu basılır (alt düzlem y=−13'te küre eğimi 46° → desteksiz).
- Yaw ±25, pitch ±20, kapak kenarı −5° (kapalı) … +40° (açık).
- Yaw kolu/krank r=20 → servo 65…115°; pitch servo 70…110°; kapak servo 112.5 (kapalı) … 67.5 (açık).
- Yaw servosu çerçevede ters asılı (mil aşağı), gövde çerçevenin arka-üstünde; kapak taşıyıcısının süpürme alanının dışında (r > 31).
- Kapak servosu gövdesi ön-aşağı (phi 300°) yönlü: göz lamasının süpürme alanından uzak.
- Servo kolu ↔ basılı parça: SG90 **çift kollu** kol, göbeğe en yakın delikler (r≈7, `HORN_SCREW_R`), **servonun kendi küçük vidalarıyla** (≤5 mm; daha uzunu yaw krankında lamaya değer).
- SG90 kulakları **M3** ile (kulak delikleri 3.2 mm matkapla büyütülür).

## 5. Doğrulama özeti (`out/check_report.txt`, son çalıştırma)
- 18 zorunlu + 57 ara poz: **çakışma 0**.
- **Eklem dışı en küçük boşluk 0.400 mm** (yaw krankındaki servo kol vidası ucu ↔ lama; çerçeve kolu ↔ kapak — ikisi de tasarım gereği 0.4).
- Geçersiz katı 0; basılı parçalar tek katı; et kalınlığı %1 yüzdelik ≥ 1.74 (kapak), çoğu ≥ 2.
- M3: 6 / 14. M4×16 havşa 12, M4×10 yuvarlak 8.
- **Yöntem sınırları:** model rijit ve boşluksuz; baskı toleransı, sürtünme, esneme, servo torku, SG90 ölçü sapmaları **[DOĞRULANMADI]**.

## 6. Riskler / bilinen sorunlar
1. **M3 kafa ↔ SG90 gövdesi:** kulak deliği gövdeden ~2.5 mm; M3 yuvarlak kafa (Ø5.6) gövde duvarına ~0.3 mm biniyor (check §2'de 1.2 mm³). Kafa hafif eğik oturabilir; gerekirse kafanın kenarını eğeleyin.
2. **Servo tutucularda ince et:** M3 pilot ile servo cebi arası ~0.9–1.0 mm (`base` orta direk ve pitch tutucu, `yaw_mount`). SG90 kulak geometrisinden kaynaklı; vidayı aşırı sıkmayın.
3. **Pitch çerçevesi sağda servo mili üzerinde, solda M4 pimde:** iki eksen arasında hizasızlık sürtünme yapabilir [DOĞRULANMADI]. Sol pim sıkıysa yatağı 4.5–4.6 açın.
4. **Kapak taşıyıcısı yalnız kapak servosunun milinde (tek destek)** — hafif, ama eksenel boşluk kapakları hafif oynatabilir [DOĞRULANMADI].
5. Göz alt pimi gözün ağırlığını taşır; üst çerçeve kolu 2.4 mm kabuk — esneme [DOĞRULANMADI].
6. **Montaj erişimi:** pitch servosu kolunun merkez vidası çerçevenin içinden kısa tornavidayla sıkılır (gözler takılmadan önce). Kapak taşıyıcısı çerçeveden önce takılmalı.
7. Toleranslar (`params.py §1`) tolerans testinden sonra güncellenmeli; `M4_CSK_D` 8.6 varsayılan.

## 7. Dosyalar
`params.py` ölçüler · `kin.py` kinematik (kapalı formül) · `eye_v3.py` parçalar + montaj + `build()`, `pose_transforms(yaw, pitch, lid)`, `servo_angles()`, `tess()` + dışa aktarım · `kinematics_export.py` → `out/kinematics.json` · `check.py` → `out/check_report.txt` · `render.py` → `out/renders/`.
Değişiklikten sonra: `python eye_v3.py && python check.py && python render.py` (Windows'ta `PYTHONUTF8=1`).
