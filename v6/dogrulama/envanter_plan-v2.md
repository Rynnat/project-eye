# Envanter

Elde olan parçalar. İki amacı var: (1) ne olduğunu bilmek, (2) AI'a tasarım yaptırırken **kısıt** olarak vermek ("yalnız bu cıvatalar var"). Tezdeki ölçülmüş parça kütüphanesinin (K3) ilk hali.

`?` = elde olup olmadığı ya da adedi doğrulanmadı. Güncelleme: 2026-10-06 (sıfırdan; cıvatalar başlangıç, diğer parçalar tek tek ekleniyor).

## Bağlantı elemanları

Ölçü notu: mercimek başlılarda boy = gövde boyu (baş hariç). **Havşa başlılarda boy = toplam boy (baş dahil).**

| Cıvata | Baş | Adet | Somun |
|---|---|---|---|
| M2×8 | mercimek, yıldız | 30 | var |
| M3×8 | mercimek, yıldız | 30 | var |
| M3×10 | yuvarlak | 6 | var |
| M3×14 | mercimek, yıldız | 30 | var |
| M3×15 | havşa | 20 | var |
| M4×8 | mercimek, yıldız | 30 | var |
| M4×10 | yuvarlak | çok (Project Eye'dan kalan) | var |
| M4×10 | havşa | 2 | var |

**Tasarım kısıtları (AI'a verilecek):**
- Elde **M3 için yalnız 8, 10, 14 mm** gövde boyu var; havşada yalnız 15 mm.
- **M4 havşa yalnız 2 adet** → tasarımda M4 havşa kullanma.
- Somunla kullanımda yaklaşık en fazla bağlanabilir kalınlık (somun + ~1 diş taşma düşülerek): M2×8 ≈ 6 mm · M3×8 ≈ 5 mm · M3×10 ≈ 7 mm · M3×14 ≈ 11 mm · M4×8 ≈ 4 mm · M4×10 ≈ 6 mm.

## Diğer parçalar

| Parça | Adet | Not |
|---|---|---|
| MG996R servo (set: kollar ve küçük vidalarıyla) | 6 | ~9,4 kg·cm @4,8 V; 55 g; yük altında 2,5 A'e kadar |
| Arduino Nano genişletme kartı (Nano I/O shield, Uno boyutunda) | 1 | Nano takılır; her pin için G-V-S üçlü uç, DC jak, I2C/UART uçları. MG996R gücü bu karttan verilmez |
| PCA9685 16 kanal servo sürücü | 2 | kondansatör kartta takılı; güç yeşil vidalı girişten (5–6 V); birinin A0 pedi lehimlenecek (adres 0x41) |
| L298N motor sürücü (çift H-köprü) | 1 | 2 DC motor (hız + yön) ya da 1 step motor sürer; servo için değil |
| IR kumanda seti: kumanda (17 tuş) + IR alıcı modül (KY-022) + IR verici LED + 3'lü dişi-dişi kablo | 1 set | robotu uzaktan komutla kontrol |
| Arduino Uno klon (SMD ATmega328P, CH340 USB çipi, USB-B) | 1 | boşta, yeni projeler için; Windows'ta CH340 sürücüsü gerekebilir |
| Erkek pin header şeridi (40 pin) | 1 | |
| HC-SR501 PIR hareket sensörü | 1 | insan hareketini algılar; 2 ayar vidası (mesafe, süre); 5 V, çıkış dijital |
| Arduino Nano klon (CH340, mini USB, pinler lehimli) | 1 | Nano genişletme kartına takılır |
| LM35 sıcaklık sensörü (TO-92) | 1 | analog çıkış, 10 mV/°C; 4–30 V |
| YD-ESP32 geliştirme kartı (muhtemelen ESP32-S3, iki USB-C) | 1 | Wi-Fi + Bluetooth; 3,3 V lojik |
| HC-05 Bluetooth modülü | 1 | 3,6–6 V; STATE/RXD/TXD/GND/VCC/EN |
| 1602A LCD ekran (16×2) | 1 | I2C modülü yok, paralel bağlantı (çok pin) |
| 0,96" OLED ekran (I2C) | 1 | 4 pin; muhtemelen SSD1306 128×64 |
| BMP280 basınç-sıcaklık sensörü (I2C) | 1 | nem ölçmez |
| HC-SR04 ultrasonik mesafe sensörü | 3 | |
| TP4056 Li-ion şarj modülü (USB-C, korumalı) | 2 ? | |
| 18650 Li-ion pil, 3500 mAh | 1 | kutulu |
| Döner enkoder modülü (KY-040) | 2 | |
| Potansiyometre | 1 | |
| Buzzer | 2 | |
| Direnç seti | 1 | |
| LED seti (karışık renk) | çok | |
| Buton (tactile) | ~12 | |
| Rocker anahtar | 1 | |
| Pin header şeridi (erkek + dişi) | 2 | |
| Breadboard | 1 | |
| Elektrolitik kondansatör 1000 µF 16 V | 4 | servo güç hattı için; uzun bacak + |
| SG90 servo (boşta) | ~6 | Project Eye'dakilerden ayrı |
| USB kablo (A – mini B, Nano için) | 1 | |
| 4'lü AA pil yuvası (kablolu) | 2 | biri robot araba kitinde |
| AA pil (Varta Super Heavy Duty) | 4 | çinko-karbon: yüksek akımda zayıf, servo için alkalin/NiMH daha iyi |
| AA pil paketi (sarı, 4'lü) | 1 | |
| 9 V pil | 2 | servolar için kullanılmaz |
| Arduino Uno kutusu (siyah) | 1 | toplam 2 Uno var: 1 boşta (klon) + 1 Project Eye'da (kilitli) |
| 5 V 5 A adaptör (MRW-2505) | 2 | her PCA9685'e bir tane |
| Dişi DC jak (kablolu) | 2 | |
| Kırmızı-siyah (+ turuncu) kablo | 1 makara | |
| Havya (Sunline) + stand (S line) | 1 | |
| Lehim teli (tüp) | 1 | |
| Lehim pastası (flux) | 1 | |
| Multimetre | 1 | |
| İğne eğe seti | ~10 | baskı parçalarını düzeltmek için |
| USB kablolar (çeşitli) | birkaç | |
| TT sarı redüktörlü DC motor | ~6 | robot araba için; L298N ile sürülür |
| Sarhoş teker (caster) | 1 | |
| Robot araba montaj parçaları (braket, vida) | 1 set | |

## 🔒 Kilitli: Project Eye v5 (takılı, kullanılamaz)

Bu parçalar Project Eye'da monte. Envanter sayılarına dahil değil; sökülmedikçe başka projede kullanılmaz. Kaynak: `project-eye/v5/cad/BOM.md`.

| Parça | Adet | Not |
|---|---|---|
| SG90 servo | 3 | yaw D3, pitch D5, kapaklar D6 |
| Arduino Uno | 1 | |
| Buton | 1 | D2 ↔ GND |
| Basılı parçalar (PLA) | 19 STL / 22 baskı | ~250–320 g |
| M4×16 havşa | 15 | |
| M4×10 yuvarlak | 6 | |
| M3×10 yuvarlak | 8 | |
| M4 somun | 20 | |
| M3 somun | 8 | |
| SG90 kol vidaları | 6 | servoyla gelen |
