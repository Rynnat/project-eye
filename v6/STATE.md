# Project Eye v6 — STATE (devir notu, en yeni en üstte)

## Nasıl çalıştırılır
```
cd modules/project-eye/v6/cad
set PYTHONUTF8=1
..\..\..\..\.venv-cad\Scripts\python.exe -B eye_v6.py [--only kama,sol_servo_braketi,sol_kol_adaptoru]   # STL/STEP/GLB; --only: parça + parça kontrolü (out/parca_<ad>.txt)
..\..\..\..\.venv-cad\Scripts\python.exe -B check.py [--quick]     # out/check_report.txt (~4 dk, 54 poz) + out/check_ozet.json
..\..\..\..\.venv-cad\Scripts\python.exe -B analiz.py              # out/analiz.json (kütle/tork, dolgu ve α taraması)
..\..\..\..\.venv-cad\Scripts\python.exe -B render.py              # out/renders/01..07
cd ..\viewer
..\..\..\..\.venv-cad\Scripts\python.exe -B build_viewer.py        # SONUC: TEMIZ değilse üretmez
node test_harness\run.js ..\model.html                             # GECTI
```
Sıra: eye_v6 → check → analiz → render → build_viewer → run.js. **v5'e yazma yok** (`-B`; render v5 modülünü import etmez).
Kütüphane testi: `cd agent-flow && python -B -m checks.test_regresyon && python -B -m checks.test_eye_v6_ekleri && python -B -m checks.test_eye_v6_tur1 && python -B -m sablonlar.test_sablonlar`.

## 2026-10-12 — aşama 5 düzeltme turu 1 BİTTİ (DOĞRULANDI: D2, D4, D6, D7, D8)
- `check_report.txt`: **SONUC: TEMIZ** (111 parça, 54 eğik poz; 44 OLCULDU / 176 OLCULMEDI satırı); run.js GECTI, model.html 3.7 MB. Yeni kalıcı kontroller: `R9_kama_hafiflik`, `MONTAJ_kalibrasyon_sirasi`,
  `MONTAJ_sikma_talimati`, `R6_frame_L_matkap`, `BOM_envanter_belge` (+ plan_sapmasi "kama kabugu"). Kütüphane: `agent-flow/checks/hafiflik.py`,
  `belge.py`, `test_eye_v6_tur1.py` [TASLAK]. Kanıt `cad/out/kanit_duzeltme.txt` (ÖNCE SORUN 9 → SONRA TEMİZ; `python -B kanit_duzeltme.py`, ~50 s).
- **Yeniden üretilen yalnız `kama`** (md5 452e96ce; eski 23a58031 `cad/out/tur0/kama.stl`). sol_servo_braketi / sol_kol_adaptoru değişmedi (md5 aynı).
  GLB (kama + frame_L delip geçen matkap) yeniden, render + model.html yenilendi. v5 ağacı md5 aynı.
- Kama: 1.6/1.6/1.6, 2 nervür → **65.4 g dolu / yarı**; Orca 0.2 **7 sa 42 dk / 60.5 g**, 0.28 **4 sa 09 dk / 59.6 g**. Devrilme α30 **28.6 mm / 27.8°**.
- MONTAJ: T1 "değince en fazla 1/8 tur"; A0 adaptör kılavuzuyla delip geç; T15 1000 µF; **K1–K5 kalibrasyon** (sağ pitch → D9 ofset → 80–100 → tam tarama);
  firmware notu writeMicroseconds + iki nokta; "Kullanıcı testleri" (D1, D3, D5). BOM_FARK: SG90 ~6, MG90S envanterde YOK (soruldu), 1000 µF.
- **Açık (kullanıcı):** MG90S var mı (cevap bekleniyor; yoksa sağda v5 SG90 kalır, S6/T7/T8 atlanır); D1/D3/D5 testleri; diğerleri aşağıda.
- Görsel kontrol bu turda yapılmadı (yalnız kama et/nervür değişti; parça kontrolü + render yenilendi).

## 2026-10-12 — aşama 2 (tasarım) BİTTİ, denetim bekliyor
- `check_report.txt`: **SONUC: TEMIZ** (112 parça, 54 eğik poz; 38 OLCULDU / 166 OLCULMEDI satırı, ~120'si gerekçeli muaf çift).
  run.js **GECTI**, `model.html` üretildi (4.4 MB). v5 ağacı md5 ile değişmedi (oturum başı/sonu karşılaştırması).
- Yeni STL (basılmadı, dondurulmadı; ilk baskıdan sonra md5 ile dondur): kama 23a58031, sol_servo_braketi d3618987, sol_kol_adaptoru b2393c0e.
- Tork (SEÇENEK B, 4.6 V ölçekli, %35): dolgu 0.75 **2.01**, 1.0 1.68, 0.6 2.27; tek MG90S 1.25. Z2 = 4.52 mm. Mekanik pitch durma +21.7°.
- Devrilme α30: arka kenara 30.3 mm / 30.8°.
- Görsel kontrol 2 tur yapıldı (tur 1: 01–07; tur 2: 07 + ortografik yan) — sorun yok, görsel düzeltme yapılmadı.

### Plan sapmaları (DECISIONS.md D5–D15)
pitch üst sınırı +20 → **+18** (v5 kirişi +21.7°'de tabana değer); aşım pozu +23 → +21; M4×16 havşa 1 satın; adaptör M2×8; sol kulak M2×8;
sağ ray ayna; kama kütlesi 2×94 g; pitch ekseni 88.6; kapasite 4.6 V; tutma testi 330 g; maske yolu önden kaydırma.

### Açık / kullanıcı
- Satın: M4×16 havşa (+yedek), kaymaz ayak ×4, vida sabitleyici (yoksa); isteğe bağlı 2× 15 cm servo uzatma.
- Ölç: MG90S + sol SG90 kulak altı → mil ucu (R12), kol vidası boyu (≤ 5.4), kaymaz ayak kalınlığı (`AYAK_T`), SG90 kablo boyu.
- Firmware/settings (AYRI İŞ): EYE_PITCH 80…108, EYE_PITCH_L (D9) = 180 − EYE_PITCH, hız sınırı ≥ 0.25 s/10°, mount_tilt_deg 30.
- Panel entegrasyonu (Epic) yapılmadı (istenmedi).
