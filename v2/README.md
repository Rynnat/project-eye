# Project Eye v2

İki göz, dört bağımsız kapak, 6 × SG90, Arduino Uno. Esin kaynakları: InMoov "Advanced Eye Mechanism"
(Dakota76) ve Will Cogley'in 6 servolu göz mekanizması. Parçalar sıfırdan, parametrik olarak tasarlandı.

**Görmek için:** Epic panelde Project Eye → sürüm **v2** → Project Eye. `model.html` açılır: gerçek parçalar,
hareket kontrolü, canlı mod, kırpma ve servo açıları.

## Durum

| İş kolu | Durum | Doğrulama |
|---|---|---|
| Mekanik (`cad/`) | Tamam | 117 pozda çakışma **0**; en dar eklem dışı boşluk 0.173 mm; bütün katılar geçerli, duvar ≥1.2 mm |
| Firmware (`firmware/`) | Tamam | `arduino:avr:uno` derleniyor |
| Yazılım (`control/`) | Tamam | 89 unittest geçiyor (≈1900 pozda sınır aşımı yok), yüz takibi webcam'de denendi |
| Baskı toleransları | **Bekliyor** | Ender 3 Neo'da tolerans testi (`cad/tolerance/TEST.md`) |
| Gerçek donanım | **Denenmedi** | Servo yönleri, limitler, güç, montaj |

## Sıra

1. **Tolerans testi:** `cad/tolerance/out/*.stl` bas, `cad/tolerance/TEST.md`'ye göre dene ve sonucu raporla
   (`A3 B4 C2 ...`). Toleranslar `cad/params.py`'ye yazılır, bütün parçalar yeniden üretilir.
2. **Parçaları bas:** `cad/out/stl/` (23 parça, baskı yönüne çevrilmiş). Malzeme ve yön: `cad/BOM.md` §1.
3. **Malzemeleri al:** `cad/BOM.md` §2–4. 50 M2 vida, 4 × Ø2×12 pim, 4 RC rotil topu, 6 SG90, 5 V ≥3 A kaynak.
4. **Firmware'i yükle:** `control/README.md` → Firmware. Lunar gimbal'inin portuna yükleme.
5. **Montaj:** `cad/BOM.md` §5. Servolar 90°'deyken kollar **bağlantıya dik** takılır.
6. **Kalibrasyon:** `control/calibrate.py`. Başlangıç değerleri `cad/out/kinematics.json → settings_suggestion`.
7. **Çalıştır:** `control/eye_control.py` (demo) veya `control/face_follow.py` (yüz takibi).

## Dosyalar

| Yol | İçerik |
|---|---|
| `SPEC.md` | İki iş kolunun sözleşmesi: koordinatlar, kanal/pin haritası, seri protokol, ayar şeması |
| `cad/params.py` | Bütün ölçüler ve toleranslar tek yerde (`[TOL]` etiketliler test baskısıyla güncellenecek) |
| `cad/eye_v2.py` · `kin.py` · `check.py` | Parçalar + montaj · kinematik · çakışma taraması |
| `cad/out/step/` · `stl/` · `glb/` | Fusion için STEP (parça + `assembly.step`) · basılacak STL · görüntüleyici için GLB |
| `cad/out/kinematics.json` | Servo açısı ↔ göz/kapak açısı tabloları, köşe sınırları, ayar önerisi |
| `cad/out/check_report.txt` · `renders/` | Kontrol raporu · 9 kontrol görüntüsü |
| `cad/BOM.md` · `cad/DECISIONS.md` | Malzeme listesi + montaj sırası · tasarım kararları ve şartnameden sapmalar |
| `firmware/project_eye_v2/` | Uno firmware'i |
| `control/` | `eye_control` · `calibrate` · `face_follow` · `sim` · testler; ayrıntı `control/README.md` |
| `settings.json` | Kalibrasyon ve davranış ayarları (panelde ⚙) |
| `viewer/` | `model.html`'i üreten betik ve şablon. Model değişince: `build_viewer.py` |

## Bilinen riskler

- **Köşelerde yukarı-aşağı çapraz etkisi (çözüldü, donanımda denenmedi):** Firmware sınırları mekanik güvenli
  değerlere indirildi (pitch 63.17–121.62°). `eye_control` pitch servosunu `kinematics.json`'daki 2B tablodan
  hesaplıyor ve hiçbir yolda sınır dışı açı göndermiyor. Sabit sınır yüzünden göz ortada (yaw=0) −20…+23°'ye kadar iner.
- **0.173 mm boşluk** (göz çatalı ↔ sabit çatal, köşe pozunda) baskı toleransı mertebesinde. İlk montajda köşede sürtünme var mı bakılmalı.
- **Denenmemiş geçmeler:** Basılı rotil yuvası (pitch çubuğu) ve kapakların küçük destekli baskısı.
- **Tedarikte ölçülmesi gerekenler:** SG90'ın datasheet dışı ölçüleri, servo kolunda r=10 deliği, rotil topu ölçüleri.
- **Hesaplanmadı:** Servo torku. İris yalnızca görsel.
