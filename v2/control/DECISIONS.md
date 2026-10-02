# Project Eye v2 — Elektronik/Yazılım kararları

SPEC.md §4–§8 SABİT sözleşmesine uyuldu. Aşağıdakiler şartnamenin açık bıraktığı
yerlerde verilen kararlar ve (varsa) küçük sapmalardır.

## Protokol yorumları (§5)

| Konu | Karar | Gerekçe |
|---|---|---|
| `?` cevabı | Yalnızca `STATE ...` döner, ayrıca `OK` gönderilmez. | STATE zaten onaydır; PC tarafında satır eşleştirmesi basit kalır. |
| `STATE` açıları | **Anlık** (hız sınırlı) konum, 1 ondalık. | Hedef değil, servonun o an nerede olduğu daha faydalı. |
| `idle=<0\|1>` | Firmware'in **şu an** canlı modda olup olmadığı (A bayrağı değil). | PC'nin "kontrol kimde" sorusuna doğrudan cevap. |
| Idle sayacı | Yalnızca `S` sıfırlar. `?` ve `A` sıfırlamaz. | Sadece durum soran bir PC gözleri dondurmasın; SPEC "ilk S'de kontrol PC'ye geçer" diyor. |
| `D` sonrası | Servolar bırakılır ve bir sonraki `S`'e kadar idle **başlamaz**. | `D` güvenlik komutudur; 2 sn sonra servoların kendiliğinden canlanması sürpriz olurdu. |
| 0–180 dışı açı | `ERR angle out of 0-180`, satır uygulanmaz. 0–180 içi değerler sessizce mekanik limite kırpılır (`OK`). | §5 "0–180" diyor; aralık dışı değer büyük olasılıkla bozuk veri. |
| Esneklik | `\r` yok sayılır, boşluk/tab ayırıcı, boş satır sessizce yok sayılır, komut harfi küçük de olabilir. | Seri monitörden elle deneme kolaylığı. PC her zaman büyük harf gönderir. |
| Bozuk satır | >64 karakterlik satır sonuna kadar atılır, tek `ERR line too long`; yazdırılamaz bayt → `ERR bad character`. | Tampon asla taşmaz, bir sonraki satırda toparlanır. |

## Firmware

- **Mekanik limitler = `cad/out/kinematics.json → limits_checked`, pay yok** (ilk sürümdeki
  geniş 45–135 / 30–150 zarfı kaldırıldı; o bölge çakışma taramasından geçmemişti):

  | Kanal | Min | Max | Kaynak |
  |---|---|---|---|
  | EYE_YAW | 60.00 | 120.00 | limits_checked (servo = 90 + yaw, pitch'ten bağımsız) |
  | EYE_PITCH | 63.17 | 121.62 | `EYE_PITCH_firmware_safe` (yaw'dan bağımsız güvenli) |
  | LID_UL / LID_UR | 64.38 / 64.37 | 115.63 / 115.62 | limits_checked |
  | LID_LL / LID_LR | 66.23 | 113.77 | limits_checked |

  **Pitch bedeli:** sabit sınır her yaw'da güvenli olmak zorunda; yaw ±30'da tam ±25'i
  verir ama yaw=0'da göz yalnızca **−20.3…+23.0°**'ye ulaşır. Yaw=0 tablosunun
  56.73–125.59 aralığı kullanılsaydı yaw ±30'da göz −31.8°'ye gidip yarık/direk
  çakışmasına girerdi. Yaw'a bağlı firmware sınırı (her iki servonun anlık konumuna
  bakan) daha geniş aralık verirdi, ama iki servo bağımsız hızlarla hareket ederken
  geçişlerde güvenlik garantisi zorlaşır; bu yüzden orkestratörün istediği sabit sınır uygulandı.
  Limitler 2 ondalıklı olduğundan PC `S` açılarını **2 ondalıkla** gönderir
  (1 ondalıkta 113.77 → 113.8 limit dışına yuvarlanıyordu).
- **Idle kalibrasyonu derleme zamanında:** firmware PC'siz yaşarken kapak açık/kapalı
  açılarını bilmek zorunda. Protokol SABİT olduğu için kalibrasyon yükleme komutu
  eklenmedi; `IDLE_LID_CLOSED/OPEN`, `IDLE_EYE_*` settings.json ile aynı (şu an
  kinematics.json önerisi; pitch idle'da doğrusal 34.43°/birim, ±0.45 birim → ±15.5°,
  sınırların içinde). Kalibrasyondan sonra elle güncellenmeli; `test_firmware_sync`
  settings.json ile farkı yakalar.
- Hız sınırı 600°/s, 5 ms tik, `millis()` tabanlı, bloklamaz. Alt-derece çözünürlük için
  `writeMicroseconds` (544–2400 µs = Servo.h varsayılanı) kullanılır.
- Açılışta gözler 90°, kapaklar %80 açık konuma **hız sınırı olmadan** gider (servo
  konumu bilinmez, kaçınılmaz). Darbe genişliği `attach`'tan önce yazılır ki servolar 90°'ye zıplamasın.
- Idle: %30 olasılıkla merkeze yakın, aksi halde ±0.6 / ±0.45 birim rastgele sakkad
  (0.4–2.5 sn aralık); 3–6 sn'de bir 120 ms kırpma; 5 sn periyotlu kapak nefesi
  (0.72 ± 0.12); kapaklar PC ile aynı formülle pitch'i takip eder.

## PC yazılımı

- **Kapaklarda `invert`:** §6 kapak formülü `invert`'i kullanmıyor ama şemada var.
  Anlamı: servo ayna takıldıysa ham açı `180 − açı` olarak aynalanır; `closed/open`
  mantıksal değer olarak kalır. calibrate.py bunu hesaba katarak kaydeder.
- **Kapak–pitch takibi:** `shift = lid_follow_pitch · 0.5 · v · aciklik`;
  üst = açıklık + shift, alt = açıklık − shift (0..1'e kırpılır). Açıklıkla orantılı
  olduğundan kapalı göz bakış yüzünden aralanmaz. Firmware idle'ı aynı formülü kullanır.
- **Sakkad hız sınırı** derece uzayında (yaw ve pitch birlikte, vektör uzunluğu ≤
  `saccade_speed_deg_s`·dt). Firmware'in 600°/s sınırı bunun üstünde ikinci emniyet.
- **Kırpma şekli:** Lunar ultimatum'daki 0.16 sn üçgen yerine 0.25 sn yamuk
  (kapan %30, kapalı %20, açıl %50). 600°/s'de ~60° kapak yolu ~100 ms sürdüğü için
  0.16 sn'lik kırpmada kapak fiziksel olarak tam kapanamazdı.
- **İki idle katmanı:** `behaviour.idle_after_s` (1.5 sn) PC tarafı "bakacak kimse yok →
  yavaş gezin" eşiğidir; bu sırada PC keepalive S göndermeye devam eder (0.5 sn'de bir),
  firmware idle'ına geçilmez. Firmware idle'ı yalnızca PC `release()`/`close()` ettiğinde
  ya da bağlantı koptuğunda devreye girer. `close()` varsayılan olarak `A 1` gönderir
  (program kapanınca göz kendi başına yaşamaya devam eder); `close(detach=True)` servoları bırakır.
- **S gönderimi:** en fazla 50 Hz, yalnızca ≥0.1° değişimde, 2 ondalık (satır ≤ 47 bayt).
  `compute_angles` her açıyı firmware limitine kırpar; calibrate.py'nin jog'u da limitte durur —
  PC hiçbir yolda limit dışı açı göndermez.
- **Kinematik tablolar (`cad/out/kinematics.json`):** varsa `EyeController` otomatik yükler
  (`eye.mapping == "TABLE"`), yoksa/bozuksa SPEC §6 doğrusal formülüne düşer (`"LINEAR"`), çökmez.
  - Göz: `u,v` → göz açısı yaw = u·30°, pitch = v·25° (tablo kapsamı; bu modda `deg_per_unit`
    kullanılmaz, yalnızca doğrusal yedekte). Yaw servosu 1B tablodan, pitch servosu
    `cross_coupling.EYE_PITCH_servo_deg_grid`'den **çift doğrusal enterpolasyonla** (köşe çapraz
    etkisi telafi edilir). `center` = kalibrasyon trimi (tablo notrundan kayma), `invert` =
    servo tepkisini center etrafında aynalar. Sonra settings `[min,max]` ve firmware limiti:
    ulaşılamayan poz (ör. yaw=0'da pitch −25) en yakın güvenli poza iner.
  - Kapak: 4-çubuk tablosu T(o) kalibrasyona oturtulur:
    `angle = closed + (T(o) − T(0))·(open − closed)/(T(1) − T(0))` — o=0/1'de kalibre edilen
    değer aynen korunur, eğri modelden gelir. Pitch takibinde açıklık tablonun payına kadar
    (üst 1.61, alt 1.76) 1'in üstüne çıkabilir; doğrusal modda 1'de kalır.
  - settings.json şeması **değişmedi**; değerler `settings_suggestion`'dan alındı (EYE_PITCH
    `_note` alanı alınmadı). `DEFAULT_SETTINGS` (eksik anahtar tamamlama) de aynı öneriye çekildi,
    çünkü SPEC §6 örnek kapak değerleri (60/120 …) yeni limitlerin dışında kalıyordu.
- **Port bulma:** Arduino/CH340 benzeri portlar önce, Bluetooth sanal portları atlanır.
  Her aday açılır ve `EYE v2 READY` beklenir; gelmezse (kart resetlenmediyse) 2 sn sonra
  zararsız `?` gönderilir ve `STATE` beklenir. `READY` gönderen cihaz (Lunar gimbal)
  anında reddedilir; ona hiçbir S/D/A gitmez (test ediliyor).
  **Not:** Bir Uno'nun portunu açmak onu resetler — Lunar gimbal'i kullanılmıyorken
  tarama onu yeniden başlatabilir (zararsız, merkeze döner). Bunu tamamen önlemek için
  settings.json'da `serial_port: "COMx"` ya da `EYE_EXCLUDE_PORTS=COM3` ortam değişkeni.
  Şemaya yeni anahtar eklememek için hariç tutma ortam değişkeniyle yapıldı.
- **Sim'e düşme:** pyserial yoksa ya da cihaz bulunamazsa `sim.SimSerial` (firmware'in
  Python eşdeğeri) kullanılır; üst katmanlar farkı görmez. Çalışırken bağlantı koparsa
  komut gönderimi durur, mod `OFFLINE` olur, program çökmez.
- **face_follow:** Ultimatum'daki momentum/öngörü (MomentumPredictor) taşınmadı; yerine
  gürültü için EMA + hız tabanlı "odak" kısılması kullanıldı. Öngörü gimbal gecikmesini
  telafi ediyordu; burada firmware hızı (600°/s) ve sakkad hızı yeterli görüldü — donanımda
  gecikme hissedilirse geri eklenebilir. Kamera robotla aynı yöne (izleyiciye) bakıyor
  varsayılır: ham karede sağdaki yüz robotun sağındadır (+u); ters ise `--mirror` / `m`.
  Tekil örnek kilidi kendi dosyasıdır (`control/.face_follow.lock`); Lunar aynı kamerayı
  kullanıyorsa kamera açılamaz ve açık bir hata mesajı verilir.
- **Testler `unittest`** (stdlib) ile yazıldı: makinede pytest kurulu değil ve testlerin
  pyserial/mediapipe olmadan da koşması istendi. `pytest` ile de keşfedilir.
