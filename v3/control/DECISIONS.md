# Project Eye v3 (POC) — Elektronik/Yazılım kararları

v3 = v2 yazılımının 3 servoya sadeleştirilmiş hali. v2'nin protokol yorumları ve
güvenlik kuralları (`../../v2/control/DECISIONS.md`) aynen geçerli; burada yalnızca farklar var.

## Kanallar ve protokol
- Kanal 0 `EYE_YAW` D3, 1 `EYE_PITCH` D5, 2 `LIDS` D6 (iki üst kapak tek servoda).
- `S a0 a1 a2` (tam 3 açı; v2'nin 6 açılı satırı `ERR S needs 3 angles`), açılış `EYE v3 READY`,
  `STATE a0 a1 a2 idle=<0|1>`. D / A / ? ve tüm hata kuralları v2 ile aynı.
- **Port el sıkışması:** yalnızca `EYE v3 READY` ya da 3 açılı `STATE` kabul edilir.
  `READY` (Lunar gimbal) ve `EYE v2 READY` (v2 kartı: 6 kanal, farklı limitler) **reddedilir**,
  onlara S/D/A gönderilmez (testli).

## Firmware limitleri = `cad/out/kinematics.json → limits_checked`, pay yok

| Kanal | Min | Max | Not |
|---|---|---|---|
| EYE_YAW | 65.0 | 115.0 | servo = 90 − yaw (±25°) |
| EYE_PITCH | 70.0 | 110.0 | servo = 90 + pitch (±20°), direkt |
| LIDS | 67.5 | 112.5 | 112.5 kapalı, 67.5 açık |

Çapraz etki yok (mekanik rapor), bu yüzden v2'deki gibi yaw'a bağlı pitch sınırı gerekmedi.
Idle derleme-zamanı değerleri settings.json ile aynı (`test_firmware_sync` karşılaştırır).
Derleme: flash 9540 B (%29), RAM 552 B (%26).

## settings.json (v2 şemasının 3 kanallı hali)
- `EYE_YAW`, `EYE_PITCH`: v2 ile aynı alanlar. `LIDS`: `closed`, `open`, `invert` + **`min`, `max`**
  (settings_suggestion'da var; ham servo açısı, opsiyonel — varsa kapak açısı bunlara kırpılır).
  `_note` alanları alınmadı. Değerler settings_suggestion ile birebir (testli).
- calibrate.py `closed`/`open` kaydedince LIDS `min/max`'ı bunları kapsayacak şekilde günceller
  (yoksa yeni kalibrasyon eski min/max'a takılırdı).

## Eşleme
- **Tablo** (`kinematics.json` varsa): göz açısı yaw = u·25°, pitch = v·20°;
  `servo = center + yön · (T(açı) − T(0))`, **yön = ayar yönü × tablonun kendi yönü**.
  Yani `invert` doğrusal formüldeki anlamını korur (true: +u → servo azalır); tablo yalnızca
  eğrinin şeklini verir. Önerilen `invert: true` + tablo (90 − yaw) aynı sonucu verir; donanımda
  yön ters çıkarsa tek bayrak iki modda da düzeltir. v3 tabloları zaten doğrusal, iki mod eşit (testli).
- Kapak: v2 ile aynı "tablo kalibrasyona oturtulur" formülü, sonra LIDS min/max, sonra firmware limiti.
- Tablo yoksa/bozuksa SPEC §6 doğrusal formülü (`eye.mapping` = `"LINEAR"`), çökme yok.
- **Kapak pitch takibi yazılımda kaldı:** v3 kapakları gövdeye bağlı, gözle eğilmiyor; yukarı bakışta
  kapağın kalkması için `lid_follow_pitch` (üst kapak: `o + k·0.5·v·o`, 0..1). Alt kapak yok.
- **wink yok** (iki kapak tek servoda). face_follow'da `l/r` tuşları kaldırıldı.

## Güvenlik (v2'den aynen)
Her çıkan açı `fw_clamp` ile firmware limitinde; S açıları 2 ondalık (yuvarlama limiti aşamaz);
calibrate jog'u limitte durur. Test: ~2000 poz × 4 eşleme × 2 ayar setinde limit aşımı yok.

## Doğrulanmayanlar
Gerçek donanımda hiçbir şey denenmedi: servo yönleri (özellikle yaw invert), kapak açıları,
güç davranışı, Uno ile el sıkışma.
