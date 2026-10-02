# Project Eye v5 — Elektronik/Yazılım kararları

> **v5 notu:** v5 yalnız bağlantıları somunlu yaptı; kinematik v4 ile birebir aynı (`v5/cad/out/kinematics.json`). Bu klasör v4 yazılımının kopyası, banner `EYE v5 READY`. Aşağıdaki "v4" ifadeleri v5 için de geçerli.

v4 yazılımı, **v3'ün birebir kopyasıdır** (`../../v3/control`, v3'e dokunulmadı) ve v4 mekaniğine
uyarlanmıştır. v3'ün (ve v2'nin) protokol yorumları ve güvenlik kuralları aynen geçerlidir:
`../../v3/control/DECISIONS.md`. Aşağıda yalnızca v3'e göre farklar var.

## Kanallar
Aynı: 0 `EYE_YAW` D3, 1 `EYE_PITCH` D5, 2 `LIDS` D6. Fark: LIDS artık **dört kapağı** (2 üst + 2 alt)
tek servo ve dört-çubukla sürüyor (alt kapaklar ters yönde). Yazılım açısından hâlâ tek kanal,
`lids(openness)`; wink yok.

## Firmware limitleri = `v4/cad/out/kinematics.json → limits_checked`, pay yok

| Kanal | Min | Max | v3 |
|---|---|---|---|
| EYE_YAW | 65.0 | 115.0 | aynı (servo = 90 − yaw) |
| EYE_PITCH | 70.0 | 110.0 | aynı |
| LIDS | **77.71** | **102.29** | 67.5–112.5 |

LIDS: 102.29 kapalı, 77.71 açık. Dört-çubuk tablosu (11 satır) doğrusal değil; tablo modunda
kapak açısı tablodan (kalibrasyona oturtularak) hesaplanır, tablo satırları aynen üretilir (testli).
Derleme: flash 9538 B (%29), RAM 552 B (%26).

## `lid_follow_pitch` varsayılanı 0.0 [mekanik ajan uyarısı]
v4'te kapaklar **pitch çerçevesinde**: göz aşağı/yukarı bakınca kapaklar mekanik olarak birlikte
döner. Yazılımdaki pitch takibi bunun **üstüne** eklenirdi (çift takip). Bu yüzden:
- `settings.json` ve `DEFAULT_SETTINGS`: `behaviour.lid_follow_pitch = 0.0`.
- Firmware idle: `IDLE_LID_FOLLOW_PITCH = 0.0` (sim.py aynı; `test_firmware_sync` settings ile eşitliği kontrol eder).
- Formül ve kod yolu duruyor; ince ayar gerekirse küçük bir değer (ör. 0.1) settings.json'dan verilebilir.
  Test: k = 0 iken pitch değişse de LIDS servo açısı değişmiyor.

## Port el sıkışması: yalnızca banner [v3'ten fark]
- Kabul: yalnızca `EYE v5 READY`. Red: `READY` (Lunar gimbal), `EYE v2 READY`, `EYE v3 READY`.
- v2/v3'te banner gelmezse `?` gönderilip `STATE` kabul ediliyordu. v3 kartı da aynı biçimde
  3 açılı `STATE` döndürdüğü için bu yol v3 ile v4'ü **ayıramaz** → v4'te kaldırıldı.
  El sıkışma sırasında hiçbir porta hiçbir şey yazılmaz (testli).
- Bedeli: port açılınca resetlenmeyen bir kart (DTR otomatik reset kapalı klon) bulunamaz → sim'e düşer.
  Uno'lar pyserial ile açılınca resetlenir. Gerekirse kartın reset tuşuna portu açarken basın.
  Protokolü değiştirmemek için STATE'e sürüm alanı eklenmedi.

## settings.json
v3 şemasıyla aynı (3 kanal; LIDS'te `closed/open/min/max/invert`). Değerler v4
`settings_suggestion` ile birebir (`_note` alanları hariç), `lid_follow_pitch` 0.0.

## Doğrulanmayanlar
Hiçbir şey gerçek donanımda denenmedi: servo yönleri (yaw invert), dört-çubuk kapak açıları,
SG90'ın dört-çubuğu (iletim açısı ~47°) sürtünmeyle zorlanmadan çevirip çeviremediği, güç, Uno el sıkışması.

## D2 butonu (kullanıcı, 2026-09-30)
Buton D2 ↔ GND (dahili pull-up, 40 ms debounce). Açılışta **nötr** (idle kapalı; önceden 2 sn sessizlikten sonra kendiliğinden başlıyordu). Butona basınca doğal hareket (idle) hemen başlar, tekrar basınca nötr poza (gözler merkez, kapak %80) döner ve idle kapalı kalır. Karttaki LED (D13) idle açıkken yanar. `A 1`/`A 0` aynı anahtarı değiştirir (PC bağlanınca `A 1` gönderir). Buton seriye yazmaz. sim.py `press_button()` + `ButtonTests`.
