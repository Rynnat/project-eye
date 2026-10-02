# Project Eye v3 — POC Şartnamesi

**Amaç:** Kullanıcının elindeki malzemeyle, Ender 3 Neo'da güvenle basılıp hızlıca kurulabilen bir kavram kanıtı.
Kusursuzluk değil **basitlik ve sağlamlık**. Ölçüler v2'den serbestçe büyüyebilir. Sonra malzeme alınıp v2 çizgisine dönülecek.
Referans: v2 (`../v2/`, özellikle `v2/cad/` kodu ve `v2/cad/DECISIONS.md`). Bu tasarım v2'nin sadeleştirilmiş, büyütülmüş halidir.

## Eldeki malzeme (SABİT — başka bağlantı elemanı yok)
- **M4×10 yuvarlak kafa** (çok sayıda): eklem pimleri ve genel bağlantı.
- **M4×16 havşa (90° koni) kafa** (çok sayıda): taban ve kalın parçalar.
- **M3×10 yuvarlak kafa: sadece 14 adet.** Yalnızca M4'ün sığmadığı yerlerde.
- **M2 YOK. Somun YOK. Çelik pim YOK. Rotil/top YOK. Tel YOK.**
- SG90 servolar, kendi kol ve vidalarıyla (kol vidaları kol ↔ basılı parça bağlantısında kullanılabilir). Arduino Uno.
- Yazıcı: Creality Ender 3 Neo, 0.4 nozül, PLA, tabla 220×220.

## Kapsam
- **İki göz**, birbirine bağlı (şaşılık yok). Göz Ø**36**, göz merkezleri arası **95** (insan oranı ~2.6 × çap).
- **3 servo:**
  | Kanal | İsim | İşlev |
  |---|---|---|
  | 0 | `EYE_YAW` | İki göz birlikte sağ-sol |
  | 1 | `EYE_PITCH` | İki göz birlikte yukarı-aşağı |
  | 2 | `LIDS` | İki **üst** kapak birlikte (kırpma) |
- Alt kapaklar **yok** (maske/yüz plakası alt kapak şeklini sabit olarak verebilir).
- Hareket aralığı: yaw ±25°, pitch ±20°, kapak tam açık ↔ kapalı (göz kırpabilmeli).
- Koordinat sistemi v2 SPEC §1 ile aynı: mm, +X robotun sağı, +Y yukarı, +Z ileri, orijin iki göz merkezinin ortası.

## Basitlik kuralları (asıl amaç bunlar)
1. **Basılı top/rotil yuvası, sıkı geçme pim, kartal/snap-fit yok.** Her dönen eklem: **M4 vida = pim**.
   Vida sabit parçada `M4_PILOT` deliğine vidalanır, dönen parça `M4_RUN` deliğinde döner. İki parça arasında `AXIAL_GAP` boşluk.
2. **İtme çubukları basılı düz lama** (her iki ucunda M4_RUN delikli). 3B mafsal gerekiyorsa, dönme eksenleri
   hemen hemen paralel kalacak şekilde düzen kur ya da lamayı yeterince uzun ve esnek tut. Bunu `DECISIONS.md`'de gerekçelendir.
3. **Min. duvar 1.6 mm, min. detay 0.8 mm.** Destek gerektiren yerleri en aza indir, gerekiyorsa BOM'da belirt.
4. **Az parça:** hedef ≤ 15 farklı basılı parça. Her parça tablaya düz bir yüzle oturabilmeli.
5. Kardan mafsalı v2 mantığıyla kalabilir ama büyütülmüş: göbek ve çatal ölçüleri M4'e göre.
6. Servo kolları nötrde (90°) itme lamasına **dik** (ölü nokta dersi).

## Toleranslar (`params.py`'de adlandırılmış sabit, test sonucu gelince güncellenecek)
`M3_PILOT 2.7` · `M3_CLEAR 3.3` · `M4_PILOT 3.5` · `M4_RUN 4.3` · `M4_CSK_D 8.2` (90°) · `AXIAL_GAP 0.4` · `SERVO_POCKET_CLR 0.3`.
Tolerans testi: `cad/tolerance/` (kullanıcı basıyor).

## Çıktılar (v2 ile aynı düzen)
`cad/params.py`, `kin.py`, `eye_v3.py` (parçalar + montaj + `pose_transforms(yaw, pitch, lid)` + `servo_angles`),
`check.py` (çakışma taraması), `render.py`; `cad/out/{step,stl,glb,renders}/`, `glb/rig.json`, `kinematics.json`,
`check_report.txt`; `cad/BOM.md` (basılacak parçalar + vida sayıları: M3 ≤ 14 kontrolü!), `cad/DECISIONS.md`.

## Doğrulama
- Hareket aralığı taramasında çakışma 0; eklem dışı min. boşluk raporu (POC için ≥ 0.4 mm hedef, Ender 3 Neo).
- Bütün parçalar geçerli katı, duvar ≥ 1.6 mm.
- BOM'da M3 sayısı ≤ 14.
