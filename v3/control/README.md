# Project Eye v3 (POC) — Firmware ve kontrol yazılımı

3 × SG90, Arduino Uno. v2 yazılımının sadeleştirilmiş hali (kararlar: `DECISIONS.md`).

> **Durum:** Firmware derleniyor (flash 9540 B %29, RAM 552 B %26); yazılım firmware
> simülatörüne karşı 44 testle doğrulandı. **Gerçek donanımda denenmedi.**

| Kanal | İsim | Pin | Servo aralığı (firmware limiti) |
|---|---|---|---|
| 0 | EYE_YAW | D3 | 65–115 (servo = 90 − yaw) |
| 1 | EYE_PITCH | D5 | 70–110 |
| 2 | LIDS (iki üst kapak) | D6 | 112.5 kapalı … 67.5 açık |

**Kablolama:** servolar **harici 5 V** (3 servo için ≥2 A, 3 A önerilir) kaynaktan; kaynak GND'si
**Uno GND'sine bağlı** (ortak toprak). Harici +5 V'u Uno'ya bağlamayın, Uno USB'den beslenir.
Turuncu = sinyal, kırmızı = +5 V, kahverengi = GND. Servo kolunu takmadan önce servoyu nötre getirin
(firmware açılışta gözleri 90°'ye alır).

## Kurulum ve çalıştırma
```powershell
# pyserial venv'i (v2 ile ortak): C:\Users\LENOVO\epic-project\.venv-ctl
$py = "C:\Users\LENOVO\epic-project\.venv-ctl\Scripts\python.exe"
cd C:\Users\LENOVO\epic-project\modules\project-eye\v3\control
& $py eye_control.py --sim           # demo (--sim olmadan donanımı arar, yoksa sim)
& $py calibrate.py                   # 1..3 kanal, ok tuşları, c/n/x göz, k/o kapak, s kaydet
& $py -m unittest discover -s tests -v
$env:PYTHONUTF8 = "1"
C:\Users\LENOVO\lunar-tracker\.venv\Scripts\python.exe face_follow.py    # --sim, --port COM5, --camera 1

# firmware derleme (yüklerken portu kontrol edin; Lunar gimbal ya da v2 kartına yüklemeyin)
& "C:\Users\LENOVO\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" `
  compile --fqbn arduino:avr:uno ..\firmware\project_eye_v3
```

Port `auto`: yalnızca `EYE v3 READY` veren kart kullanılır; Lunar gimbal (`READY`) ve v2 kartı reddedilir.
Tarama bir Uno'yu resetleyebilir; sabit port için settings.json'da `"serial_port": "COM5"` ya da
`EYE_EXCLUDE_PORTS=COM3`.

## Kod içinden
```python
from eye_control import EyeController
eye = EyeController()        # settings.json + cad/out/kinematics.json; donanım yoksa sim
eye.look(0.5, -0.2)          # -1..1 (+u robotun sağı, +v yukarı)
eye.lids(0.8); eye.blink()   # wink yok (iki kapak tek servo)
while True:
    eye.update()
```

Montajdan sonra: `calibrate.py` ile değerleri bul; firmware limitlerini yalnızca **daraltın**,
`IDLE_*` sabitlerini ve `sim.py`'yi yeni settings.json'a çekin (testler farkı yakalar).
