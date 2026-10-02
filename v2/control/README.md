# Project Eye v2 — Kontrol yazılımı ve firmware

İki göz + dört kapak, 6 × SG90, Arduino Uno. Sözleşme: `../SPEC.md` (§4 kanal/pin,
§5 seri protokol, §6 settings.json). Kararlar ve sapmalar: `DECISIONS.md`.

> **Durum:** Firmware derleniyor, PC yazılımı firmware simülatörüne karşı test edildi.
> **Gerçek donanımda henüz denenmedi** (servo yönleri, limitler, güç davranışı doğrulanmadı).

## Dosyalar

| Dosya | Ne yapar | Bağımlılık |
|---|---|---|
| `../firmware/project_eye_v2/project_eye_v2.ino` | Uno firmware: protokol, limitler, 600°/s hız sınırı, idle "canlı" mod | Servo (Arduino çekirdeği) |
| `eye_control.py` | Seri sürücü, port bulma + el sıkışma, kalibrasyon, `look/lids/blink/wink/update` | pyserial (yoksa sim) |
| `sim.py` | Firmware'in Python eşdeğeri + sahte seri port | stdlib |
| `calibrate.py` | Kanal kanal etkileşimli kalibrasyon → `../settings.json` | pyserial, Windows konsolu |
| `face_follow.py` | Webcam → BlazeFace → en büyük yüz → bakış | opencv, mediapipe (Lunar venv) |
| `tests/` | 89 unittest: protokol, formüller, kinematik tablo, davranış, uçtan uca, .ino↔sim↔kinematics senkronu | stdlib |

## Kurulum

```powershell
# eye_control / calibrate / sim / testler için (yalnızca pyserial):
python -m venv C:\Users\LENOVO\epic-project\.venv-ctl
C:\Users\LENOVO\epic-project\.venv-ctl\Scripts\python.exe -m pip install pyserial
# face_follow için Lunar'ın venv'i (mediapipe + opencv + pyserial hazır):
#   C:\Users\LENOVO\lunar-tracker\.venv\Scripts\python.exe
```

`.venv-ctl` bu makinede kuruldu (pyserial 3.5).

## Kablolama

| Kanal | İsim | Uno pini |
|---|---|---|
| 0 | EYE_YAW | D3 |
| 1 | EYE_PITCH | D5 |
| 2 | LID_UL (sol üst) | D6 |
| 3 | LID_LL (sol alt) | D9 |
| 4 | LID_UR (sağ üst) | D10 |
| 5 | LID_LR (sağ alt) | D11 |

- SG90 kablo: **turuncu** = sinyal → pin, **kırmızı** = +5 V (harici), **kahverengi** = GND.
- Servolar **harici 5 V ≥ 3 A** kaynaktan beslenir. Uno'nun 5 V pininden 6 servo
  **beslenmez** (USB ~500 mA; tek SG90 zorlanmada ~650 mA çeker).
- Harici kaynağın **GND'si Uno GND'sine bağlanır** (ortak toprak; yoksa sinyal referanssız kalır).
- Harici +5 V'u Uno'nun 5 V pinine bağlamayın; Uno USB'den beslenir.
- Öneri: servo besleme hattına 470–1000 µF elektrolitik kondansatör (ani akım titremesini keser).
- Ölü nokta kuralı (§4): nötr konumda her servo kolu itme çubuğuna dik olmalı —
  kolları takmadan önce firmware'i açıp servoları nötre getirin (açılışta gözler 90°).

## Firmware

```powershell
& "C:\Users\LENOVO\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" `
  compile --fqbn arduino:avr:uno C:\Users\LENOVO\epic-project\modules\project-eye\v2\firmware\project_eye_v2
# Yükleme (kart takılıyken, doğru portla! Lunar gimbal'inin portuna yüklemeyin):
#   ... upload -p COMx --fqbn arduino:avr:uno <aynı klasör>
```

Derleme: flash 9724 B (%30), RAM 651 B (%31), uyarı yok.

**Bakış eşlemesi:** `cad/out/kinematics.json` varsa göz servoları yaw×pitch 2B tablosundan
(çift doğrusal, köşe çapraz etkisi telafili), kapaklar 4-çubuk tablosundan hesaplanır;
yoksa SPEC §6 doğrusal formülü. Firmware limiti yüzünden yaw=0'da göz pitch'i −20…+23°,
yaw ±30'da tam ±25° (ayrıntı: DECISIONS.md).
Seri monitörden elle deneme (115200, satır sonu LF): `?`, `S 90 90 100 90 80 90`, `A 0`, `D`.

## Çalıştırma

`control` klasöründen:

```powershell
$py = "C:\Users\LENOVO\epic-project\.venv-ctl\Scripts\python.exe"
& $py eye_control.py            # donanımı bul (yoksa sim) ve kısa demo hareketi
& $py eye_control.py --sim      # donanımsız
& $py calibrate.py              # kalibrasyon (--sim ile prova, --port COM5)
& $py -m unittest discover -s tests -v

$env:PYTHONUTF8 = "1"
C:\Users\LENOVO\lunar-tracker\.venv\Scripts\python.exe face_follow.py          # --sim, --port COM5, --camera 1, --mirror
```

- **Port:** `settings.json` → `"serial_port": "auto"` her portu `EYE v2 READY`
  el sıkışmasıyla doğrular; Lunar gimbal'i (`READY`) reddedilir. Tarama bir Uno'yu
  resetleyebileceği için sabit port yazmak (`"COM5"`) ya da `EYE_EXCLUDE_PORTS=COM3` önerilir.
- **face_follow tuşları:** `q`/ESC çıkış, pencere X ile kapanır, `b` kırp, `l`/`r` wink,
  `m` ayna, `h` kontrolü firmware idle'ına bırak/geri al, `d` servoları bırak.
  Aynı anda tek kopya çalışır (kamera + COM kilidi).
- **calibrate tuşları:** `1..6` kanal, ←/→ ±1°, ↑/↓ ±5°, `,`/`.` ±0.2°; göz: `c` center,
  `n` min, `x` max; kapak: `k` closed, `o` open; `i` invert, `g` kayıtlılar arası gez,
  `t` test, `z` detach, `s` kaydet, `q` çık.

## Kod içinden

```python
from eye_control import EyeController
eye = EyeController()          # port: settings.json; bulamazsa sim
eye.look(0.5, -0.2)            # -1..1, +u robotun sağı, +v yukarı
eye.lids(0.8, left=0.4)        # taban açıklık, göz bazında ezilebilir
eye.blink(); eye.wink("left")
while True:
    eye.update()               # her karede: sakkad sınırı, kırpma, kapak takibi, S gönderimi
```

## Kalibrasyon sonrası yapılacaklar

1. `calibrate.py` ile altı kanalın gerçek değerlerini `settings.json`'a yaz.
2. Firmware `LIMIT_MIN/MAX` şu an `cad/out/kinematics.json` mekanik sınırları (pay yok;
   pitch 63.17–121.62 yaw'dan bağımsız güvenli aralık). Montajda sınırlar farklı çıkarsa
   yalnızca **daraltın**; `IDLE_LID_CLOSED/OPEN` ve `IDLE_EYE_*`'ı yeni settings.json'a çekin,
   aynı sabitleri `sim.py`'de güncelleyin (`tests/test_firmware_sync.py` ve
   `tests/test_kinematics.py` farkı yakalar).
3. Derle, yükle, `face_follow.py` ile dene.
