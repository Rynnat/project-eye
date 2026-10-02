# Project Eye v4 — Firmware ve kontrol yazılımı

3 × SG90, Arduino Uno. v3 yazılımının v4 mekaniğine uyarlanmış kopyası (kararlar: `DECISIONS.md`).

> **Durum:** Firmware derleniyor (flash 9538 B %29, RAM 552 B %26); yazılım firmware
> simülatörüne karşı 45 testle doğrulandı. **Gerçek donanımda denenmedi.**

| Kanal | İsim | Pin | Servo aralığı (firmware limiti) |
|---|---|---|---|
| 0 | EYE_YAW | D3 | 65–115 (servo = 90 − yaw) |
| 1 | EYE_PITCH | D5 | 70–110 |
| 2 | LIDS (2 üst + 2 alt kapak) | D6 | 102.29 kapalı … 77.71 açık |

**Kablolama:** servolar **harici 5 V** (≥2 A, 3 A önerilir) kaynaktan; kaynak GND'si **Uno GND'sine
bağlı** (ortak toprak). Harici +5 V'u Uno'ya bağlamayın. Turuncu = sinyal, kırmızı = +5 V,
kahverengi = GND. Servo kolunu takmadan önce servoyu nötre getirin.

## Çalıştırma
```powershell
$py = "C:\Users\LENOVO\epic-project\.venv-ctl\Scripts\python.exe"   # pyserial
cd C:\Users\LENOVO\epic-project\modules\project-eye\v4\control
& $py eye_control.py --sim           # demo (--sim olmadan donanımı arar, yoksa sim)
& $py calibrate.py                   # 1..3 kanal, ok tuşları, c/n/x göz, k/o kapak, s kaydet
& $py -m unittest discover -s tests -v
$env:PYTHONUTF8 = "1"
C:\Users\LENOVO\lunar-tracker\.venv\Scripts\python.exe face_follow.py    # --sim, --port COM5

& "C:\Users\LENOVO\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" `
  compile --fqbn arduino:avr:uno ..\firmware\project_eye_v4
```

- Port `auto`: yalnızca açılışta `EYE v4 READY` basan kart kullanılır; Lunar gimbal, v2 ve v3 kartları
  reddedilir. Sabit port için settings.json'da `"serial_port": "COM5"` ya da `EYE_EXCLUDE_PORTS=COM3`.
- `behaviour.lid_follow_pitch` = 0.0: kapaklar pitch'i zaten mekanik olarak izliyor.
- Montajdan sonra: `calibrate.py` ile değerleri bulun; firmware limitlerini yalnızca **daraltın**,
  `IDLE_*` sabitlerini ve `sim.py`'yi yeni settings.json'a çekin (testler farkı yakalar).
