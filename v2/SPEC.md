# Project Eye v2 — Tasarım Şartnamesi

v1: tek göz, kardan mafsallı, 2 SG90 (sadece 3D görselleştirme).
v2: **iki göz + dört kapak, 6 SG90, basılabilir parçalar, firmware ve kontrol yazılımı.**
Esin: InMoov "Advanced Eye Mechanism" (Dakota76: 6×SG90, kapaklı, gözler bağımsız sağ-sol)
ve Will Cogley'in göz mekanizması (6 servo: 2 göz ekseni + 4 kapak).

Bu belgedeki **SABİT** maddeler iki iş kolunun (mekanik / elektronik-yazılım) ortak sözleşmesidir.
Değiştirmek gerekirse gerekçesiyle `DECISIONS.md`'ye yazılır.

## 1. Koordinat sistemi (SABİT)
- Birim **mm**, derece.
- Orijin: iki göz merkezinin tam ortası.
- **+X = robotun sağı** (karşıdan bakana göre sol), **+Y = yukarı**, **+Z = ileri** (yüzün baktığı yön).
- Sol göz merkezi `(-31.5, 0, 0)`, sağ göz merkezi `(+31.5, 0, 0)` → göz bebekleri arası (IPD) **63**.
- Açılar: **yaw +** = robot kendi sağına bakar; **pitch +** = yukarı bakar.
  Kapak açısı: **0 = tam kapalı**, **1 = tam açık** (normalize).

## 2. Göz (v1'den devralınan, SABİT)
- Göz küresi Ø**24**, iris Ø12, bebek Ø4 (yetişkin ortalaması).
- Her göz kendi merkezinde **kardan mafsalı** ile döner:
  göbek 8×8×8, 4× **M2×5** (göbeğe 2.5 mm girer), çatal kolu 2 mm, göbek-kol boşluğu 0.5.
- Sabit çatal arkadan bir direkle çerçeveye bağlanır.
- Hareket aralığı: **yaw ±30°, pitch ±25°**.
- Göz kabuğu: v1'de yarım küreydi. v2'de kapakların arkasında görünmemesi için öne bakan kısım
  küre olmalı; arka açıklık serbest (mafsal ve itme bağlantısı için).

## 3. Kapaklar
- Göz başına **üst + alt** kapak. Toplam 4 kapak.
- Kapaklar **küre kabuk dilimi**dir ve **gözün pitch ekseni etrafında** (göz merkezinden geçen X ekseni) döner.
  - İç yarıçap = 12 + **0.6 boşluk**, kalınlık **1.2**.
  - Pim yerleri gözün iki yanında, çerçeveye bağlı.
- Kapalıyken üst ve alt kapak kenarları gözün yatay orta çizgisinin **hafif altında** buluşur
  (insan gözünde kapaklar ortanın biraz altında kapanır). Kapanma açısı parametrik olsun.
- Açıkken üst kapak irisin üstünden ~1–2 mm örter; bakış yukarı-aşağı gittikçe kapak onu takip edebilsin.

## 4. Servolar ve kanal haritası (SABİT)
6 × SG90 (Luxorparts datasheet: gövde 22.8×12.4, kulaklarla 32.0, kulak altı 15.6, gövde üstü 26.7,
mil gövde ucundan ≈5.8). Kol deliği aralığı 2 mm.

| Kanal | İsim | İşlev |
|---|---|---|
| 0 | `EYE_YAW` | İki gözün sağ-sol hareketi (bağlantı çubuğuyla birlikte) |
| 1 | `EYE_PITCH` | İki gözün yukarı-aşağı hareketi (birlikte) |
| 2 | `LID_UL` | Sol üst kapak |
| 3 | `LID_LL` | Sol alt kapak |
| 4 | `LID_UR` | Sağ üst kapak |
| 5 | `LID_LR` | Sağ alt kapak |

- Gözler birbirine **bağlıdır**: yaw ve pitch için birer ortak bağlantı çubuğu vardır, şaşılık yoktur.
- Kapaklar **bağımsızdır**: göz kırpma, uykulu bakış ve ifade üretilebilir.
- Arduino **Uno** pin haritası: D3, D5, D6, D9, D10, D11 → kanal 0..5.
- **Güç:** servolar harici **5 V ≥3 A** kaynaktan beslenir, GND Uno ile ortaktır.
  6 SG90'ın toplam zorlanma akımı USB'nin kaldıracağından fazladır.
- **Ölü nokta kuralı (v1 dersi):** nötr konumda her servo kolu itme çubuğuna **dik** olmalı.

## 5. Seri protokol (SABİT)
- **115200 baud**, satır sonu `\n`, ASCII.
- PC → Arduino:
  - `S a0 a1 a2 a3 a4 a5` — 6 kanalın **servo açısı** (derece, 0–180, ondalık kabul edilir).
    Kalibrasyon PC tarafında yapılır, Arduino ham açı alır.
  - `D` — detach: tüm servolar bırakılır.
  - `A 1|0` — Arduino'nun kendi başına "canlı" moda (idle) geçmesini aç/kapat.
  - `?` — durum sorgusu.
- Arduino → PC:
  - `OK` — geçerli komut.
  - `ERR <mesaj>` — geçersiz komut.
  - Açılışta `EYE v2 READY`.
  - `?` sorgusuna `STATE a0..a5 idle=<0|1>`.
- **Güvenlik:**
  - Açılar firmware içinde kanal başına **sabit mekanik limitlere** kırpılır.
  - Her kanal hedefe **hız sınırıyla** gider: en fazla 600°/s. Bu, SG90'ın datasheet hızına ve insan sakkadına yakın.
- **Idle:** Firmware **2 saniye** komut almazsa kendi başına yaşar: rastgele sakkadlar, 3–6 saniyede bir göz kırpma, yavaş nefes gibi kapak salınımı. İlk `S` komutunda PC kontrolü geri alır.

## 6. Kalibrasyon ve ayarlar: `settings.json` (SABİT şema)
```json
{
  "serial_port": "auto",
  "channels": {
    "EYE_YAW":   {"center": 90, "min": 60, "max": 120, "invert": false, "deg_per_unit": 30},
    "EYE_PITCH": {"center": 90, "min": 65, "max": 115, "invert": false, "deg_per_unit": 25},
    "LID_UL":    {"closed": 60, "open": 120, "invert": false},
    "LID_LL":    {"closed": 120, "open": 70, "invert": false},
    "LID_UR":    {"closed": 120, "open": 60, "invert": false},
    "LID_LR":    {"closed": 60, "open": 110, "invert": false}
  },
  "behaviour": {
    "blink_interval_s": [3, 6],
    "lid_follow_pitch": 0.6,
    "saccade_speed_deg_s": 500,
    "idle_after_s": 1.5
  }
}
```
- Gözler: `angle = center + clamp(u,-1,1) * deg_per_unit * (invert ? -1 : 1)`, ardından [min, max] aralığına kırpılır.
- Kapaklar: `angle = closed + openness * (open - closed)`.
- Değerler **örnek başlangıç değeridir**. Gerçek değerler montajdan sonra kalibrasyon aracıyla bulunur.

## 7. Dosya düzeni (SABİT)
```
modules/project-eye/v2/
  SPEC.md             bu belge
  DECISIONS.md        şartnameden sapmalar ve gerekçeleri (iki iş kolu da yazar)
  model.html          etkileşimli 3D görüntüleyici (panelin ana butonu bunu açar)
  settings.json       §6
  README.md           ürün sayfası: malzeme listesi (BOM), baskı ayarları, montaj, kablolama
  cad/                (MEKANİK iş kolu)
    eye_v2.py         CadQuery ile parametrik tüm parçalar + montaj
    params.py         tüm ölçüler tek yerde
    check.py          çakışma ve hareket taraması testleri
    out/step/*.step   Fusion 360'a aktarılabilir parçalar
    out/stl/*.stl     baskıya hazır parçalar (baskı yönüne döndürülmüş)
    out/glb/assembly.glb  (veya parça başına .glb) görüntüleyici için, hareketli parçalar ayrı düğüm
    out/renders/*.png kontrol görüntüleri
  firmware/           (ELEKTRONİK iş kolu)
    project_eye_v2/project_eye_v2.ino
  control/            (YAZILIM iş kolu)
    eye_control.py    seri sürücü + kalibrasyon + davranış (bakış, kırpma, kapak takibi)
    face_follow.py    webcam yüz takibi → bakış (Lunar'ın BlazeFace modelini kullanır)
    calibrate.py      kanal kanal kalibrasyon aracı (settings.json'a yazar)
    sim.py            donanım yokken sahte seri port (testler için)
    tests/
```

## 8. Doğrulama çıtası
- **Mekanik:**
  - Hareket aralığının köşelerinde (yaw ±30, pitch ±25) ve kapaklar açık/kapalıyken parça çiftleri arasında **çakışma hacmi 0**.
  - Minimum boşluk raporu.
  - Her parça tek parça **katı (valid solid)** olmalı.
  - Duvar kalınlıkları ≥1.2 mm.
- **Firmware:** `arduino-cli compile --fqbn arduino:avr:uno` hatasız geçmeli.
  Derleyici: `C:\Users\LENOVO\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`
- **Yazılım:** `sim.py` ile protokol testleri geçmeli. Donanım gerektiren kısımlar donanım olmadan da çökmeden devre dışı kalmalı.
