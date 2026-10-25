# Project Eye v6 — Bağımsız denetim (Agent Flow v2, aşama 3)

Tarih: 2026-10-12. Denetlenen: `PLAN.md` (§11 geçerli), `DECISIONS.md`, `MONTAJ.md`, `BOM_FARK.md`, `STATE.md`,
`cad/out/check_report.txt` (SONUC TEMIZ, 54 poz, 38 OLCULDU / 166 OLCULMEDI), `cad/*.py`, `out/analiz.json`, v5 `BOM.md`/`MONTAJ.md`,
v3 `SPEC.md`/`BOM.md`, envanter (`Rynnat/agentic-mechanism-design` `envanter.md@plan-v2`, gh api ile okundu, 2026-10-06 sayımı).
Tasarım dosyalarına dokunulmadı. Bağımsız betikler scratchpad'de (`kama_an.py`, `kama_v.py`, `thin.py`, `drill.py`, `mask.py`), `.venv-cad` ile
modelin kendi `eye_v6.build()` / `kin_v6` / `checks` fonksiyonlarıyla çalıştı.

---

## 0. Ölçülmeyenlerin elle incelemesi (bulgu olmayanlar dahil)

| KAPSAM satırı / soru | Ne yaptım | Sonuç |
|---|---|---|
| ~105 muaf çift satırı (R3 rijit + pozlar) | `check.py eklem_ciftleri` + `RIJIT_MUAF` okundu. v5 JOINTS (pim-delik / 0.4 eksenel / kol-mil) eğimle değişmez (eğim rijit dönüşüm, bağıl hareket aynı); v5 donanımda çalışıyor. Yeni 3 muaf: sol mil-kol (spline teması), `scr_ctip6 × coupler` (pim-delik, v5'teki ile aynı tür), `nut_ctip_alt × coupler` (0.2, Z5 zinciri ölçüyor). Hepsinin kesişimi ayrıca `R3_cakisma_eklem_kesisim`'de ölçülmüş (0) | **Muafiyetler gerçek bir çakışmayı gizlemiyor.** Tek dikkat: `servo_pitch` MG90S yerine SG90 gövdesiyle muaf/ölçülü → MG90S boyutu R12 kumpasına bağlı (biliniyor) |
| `R6_giris_yolu` muaf: T3 lokma × vida ucu | Vida ucu somundan 1.42 taşar; 7 mm lokmanın iç boşluğu ≥ 4 mm | Muafiyet doğru |
| `R6_montaj_cerceve_alt` sıraya alınmayan 19 parça | Taban/maske/kama ana sırada ölçülmüş; MG90S'in cebe girişi (T7) hiçbir sırada yok ama v5 cebi ve yolu aynı | Kabul (R12 kumpası) |
| Maske yolu (soru 7) | `kontrol_montaj_sokme` ile maske + vidalar + çerçeve sırası 5 ayrı pozda yeniden koşturuldu: (yaw,pitch,kapak) = (0,0,0), (0,+21.7,0), (0,−13,0), (+25,0,1), (−25,+21.7,1) | **Hepsinde GEÇTİ.** Önde indir + 30 mm geri kaydır yolu, çerçeve elektriksizken mekanik durmaya düşmüş olsa bile mümkün. Bulgu yok |
| M4×16 havşa envanteri (soru 1) | `envanter.md@plan-v2` (2026-10-06, "sıfırdan" sayım): M4 havşa **yalnız 2× M4×10**; 15 M4×16 havşa "Kilitli: Project Eye v5". "Bol" notu v3 `SPEC.md` (v3 öncesi durum); v3 12, v5 15 adet kullandı. v6'da çıkan vidalar M4×10 (scr_fpivot, scr_ctip), M4×16 serbest kalmıyor. M4×10 yuvarlak (bol) boy yetmez (gereken ≥ 15.3) | **Tasarımcı haklı, bulgu yok.** PLAN §3 "envanter, bol" hatalıydı; D7 doğru düzeltmiş |
| Z2 = 4.52 (soru 6) | `check.py` z2 = `min_bosluk(scr_ctip6, servo_yaw)` yaw −25..25. Elle: somun üstü y −12.1 (krank üstü −13.4 + cep −1.9 + somun 3.2), kafa tepesi −12.1 + 2.48 = −9.62; servo gövde yüzü −5.1 → 4.52 ✓. Krank alt yüzü −16.9 → kafa tepesine 7.28 (MONTAJ T1 "~7.3" ✓). Kafa iç kenarı milden 15.5, SG90 gövde ucu ~16.9 → yalnız gövde ucunun altında; kapak (göbek, r 5.9) uzakta | Doğru. Coupler: krank–coupler 0.4 (v5) + alt somun 0.175 (Z5 en kötü 0.075 > 0) → serbest döner ✓. Montaj sırası T1→T2→T3→T4 `R6_montaj_cerceve_alt` ile doğrulanmış; T3 lokma koridoru (Ø10) yaw_mount yokken serbest ✓. Yalnız sıkma torku → D7 |
| Tork elle (soru 3) | AM (−0.1, −7.0, −21.8), 232 g, α30, pitch 0: kol = 7·0.5 − 21.8·0.866 = −15.4 mm → 0.232 × 1.54 = **0.357 kg·cm** (rapor 0.358 ✓). Dinamik: I 3.58e−4, 10°/0.25 s bang-bang a = 4θ/t² = 11.2 rad/s² → 0.041 ✓. Kapasite 1.8 × 0.35 × 4.6/4.8 = 0.604; ×2×0.8 = 0.966; gereken 0.482 → **2.00** ✓ | Aritmetik doğru. Varsayımın kendisi (iki servo yükü paylaşır) → D1. Not: kapasite MG90S = SG90 alınmış, yani MG90S hesapta hiçbir kazanç getirmiyor |
| Devrilme | AM (α30, p−13): arka destek kenarı (ayak y 19.66 + r 5 = 24.66) → AM y −5.6; yükseklik 30.3 / tan 30.8° = 50.8 mm | Rapor tutarlı; kama hafifletme etkisi → D4 |
| Kama et "0.27 mm" | STL'de en yakın yüzey normali (0,1,0) ve (−0.87,−0.5,0): 60° sivri profil köşesi | Kaba et kontrolünün köşe yanılgısı. Bulgu yok |
| frame_L matkap (soru 5) | Şerit dolu mu, arkasında ne var: Ø1.6 silindir şerit içinde 5.6 mm boyunca %100 malzeme; 5→15 mm derinlikte, kapak 0/0.5/1 × yaw −25/0/25 (9 poz) hiçbir parçaya değmiyor | Delip geçmek zararsız → D6 |
| PLAN kapsam matrisi ↔ kontrol | R1–R18 hepsi bir kontrole ya da kullanıcı testine bağlı. **Bağlanmamış risk:** PLAN §5.4 "iki servo birbirine karşı çalışır, stall akımı" — yalnız R14'te nitel ("vızıldamıyor"); yük paylaşımını ölçen sayısal bir kriter yok → D1 | Bulgu |

---

## Bulgular

### D1 [önemli] Pay 2.01, iki servonun yükü paylaştığı varsayımına bağlı ve bu varsayımı hiçbir şey ölçmüyor
Yer: `cad/params_v6.py:105` (`IKI_SERVO_VERIM = 0.8 [TAHMIN]`), `cad/analiz.py:144`, DECISIONS D13, MONTAJ "Testler" R14
İddia: İki servo rijit biçimde bağlı ve her biri kendi potansiyometresine göre çalışıyor. Bu durumda yükü kimin taşıyacağını ayar farkı belirler. Yalnız biri taşırsa pay dolgu 0.75'te **1.25**, dolgu 1.0'da **1.05** olur (< 1.5). MONTAJ'da paylaşımı doğrulayan ya da ayarlayan bir adım yok. T-R11 her servoyu tek başına sınar; R14 ise yalnız gerilime ve sese bakar.
Ölçme yöntemi: Kurulu haldeyken pitch −10'da (en kötü tork) 30 s bekletilir. Multimetre 200 mA DC kademesinde, sırayla D5 ve D9 servolarının kırmızı hattına seri bağlanır (araya servo uzatma kablosu konur). Her servonun akımı not edilir. Sonra D9 fişi çekilip sağ servonun tek başına çektiği akım ölçülür.
Beklenen sonuç (iddia doğruysa): iki servonun da boştaki akımın (~10 mA) belirgin üstünde akım çekmesi beklenir; oran 0.5–2 arasında olmalı ve toplam, sağ servonun tek başına çektiği akımın ≤ ~1.3 katını geçmemeli. Biri ~boşta akımdaysa paylaşım yok demektir, gerçek pay 1.25 olur. Toplam tek servonun çok üstündeyse servolar birbirine karşı çalışıyor demektir.
Öneri: MONTAJ'a bir "paylaşım ayarı" adımı eklenmeli: D9 ofseti µs adımlarıyla değiştirilir, toplam akımın en düşük ve iki akımın dengeli olduğu nokta seçilir. Firmware (ayrı iş) için iki not: D9'da `writeMicroseconds` kullanılmalı, çünkü `write()` tam derece çözünürlüğündedir ve ±0.5° kalan fark tek başına itişmeye yetebilir. MG90S ile SG90'ın darbe→açı eğimi farklı olabileceği için tek ofset yerine iki noktalı (ofset + eğim) kalibrasyon yapılmalı. Pay hesabı da "en kötü tek servo: 1.25" satırıyla birlikte raporlanmalı.

### D2 [önemli] İlk enerji anında, kalibrasyondan önce, spline kademesi hatası mekanik durma payını (3.7°) aşabilir; MONTAJ'da kalibrasyon adımı test taramasından önce gelmiyor
Yer: MONTAJ.md:46–50 (B1, T7, T9 "servo 90°'de kola oturt"), :70 (R14 −10…+18 tarama), :72 (Durma testi en sonda); DECISIONS D5; `check_report` R3_pitch_mekanik_durma (+21.7°)
İddia: Kol spline'a ancak diş kademesinde oturur. Bu yüzden servo 90°'deyken çerçevenin gerçek açısı yarım kademeye kadar sapar (20–25 diş → ±7–9°). Bu sapma 108 komutu ile +21.7° durma arasındaki 3.7° payından büyük. Ayrıca yerçekimi torku tüm aralıkta + yönde, yani durmaya doğru (`T(aci)` hep pozitif). Bu da servonun yük altındaki sapmasını durma tarafına ekler. Kalibre edilmemiş iki servo ayrıca birbirinden ±9°'ye kadar farklı açı ister ve ilk anda itişir. Sonuç stall akımı, gerilim çökmesi ve reset olabilir; SG90'da plastik dişli sıyırma riski de var.
Ölçme yöntemi: (a) SG90 ve MG90S kolundaki diş sayısı N sayılır; yarım kademe = 180/N derece. (b) T9'dan sonra, kalibrasyondan önce, iki servo da 90 komutundayken telefon eğim ölçeri taban ve çerçeve üstüne konur ve gerçek pitch okunur. Ayrıca sol kol vidaları gevşekken sol kolun adaptör pilotlarına göre ne kadar kaçık olduğuna bakılır.
Beklenen sonuç (iddia doğruysa): yarım kademe ≥ 7°; ölçülen pitch hatası tipik olarak 2–9°. Hata 3.7°'yi geçerse, kalibrasyonsuz 108 komutu çerçeveyi tabana dayar.
Öneri: MONTAJ'da sıra şöyle olmalı: (1) önce D9 fişi takılı değilken sağ servo kalibre edilir: telefonla pitch 0 bulunur, sonra +18'e 1°'lik adımlarla çıkılır ve kiriş–taban arası ≥ 1 mm görülür; (2) sonra D9 takılıp D1'deki paylaşım ayarı yapılır; (3) R14 taraması en son yapılır. Firmware'de sınır ofset uygulandıktan sonra kesilmeli (108 = kalibre pitch +18).
[gözlem] Elektrik kesildiğinde ya da reset olduğunda çerçeve yerçekimiyle +21.7° durmasına düşer. Reset döngüsünde bu tekrar tekrar olur. Kalıcı hasar beklenmez (v5 basılı parça v5 basılı parçaya değiyor), ama kullanıcı bunu bilmeli.

### D3 [önemli] R14'teki gerilim testi asıl riski göremiyor; dört servo Uno'dan besleniyor ama envanterdeki kondansatör BOM'da yok
Yer: MONTAJ.md:70 (R14 "multimetre 5V–GND ≥ 4.6 V"), PLAN §5.4 (v5'te USB'de brown-out görüldü), BOM_FARK §2–3
İddia: Multimetre saniyede 2–3 ortalama okur. Reset ve titreme yaptıran 1–10 ms'lik çökmeleri göremez. v6 Uno 5 V hattına bir servo daha ekliyor (dört servo; USB'de 500 mA polyfuse). Buna karşın envanterde olan **1000 µF 16 V kondansatör (×4, "servo güç hattı için")** BOM'da önerilmemiş. Kondansatörü eklemek kullanıcının "harici kaynak yok" kararıyla çelişmez.
Ölçme yöntemi: Firmware'e geçici bir test eklenir. Açılışta `MCUSR` okunur ve seri porta yazılır (PORF/EXTRF/BORF/WDRF ile reset nedeni görülür). Tarama sırasında her 2 ms'de 1.1 V bandgap üzerinden Vcc okunur (`readVcc`) ve en düşük değer tutulur. R14 taraması (×10) önce kondansatörsüz, sonra 5V–GND'ye 1000 µF eklenmiş halde koşturulur.
Beklenen sonuç (iddia doğruysa, yani multimetre yeterli olsaydı): en düşük Vcc ≥ 4.6 V ve beklenmeyen reset olmaz. Multimetre 4.6 V üstü gösterirken en düşük Vcc 4.3 V'un altına iniyorsa, R14 kriteri asıl riski kaçırıyor demektir.
Öneri: BOM_FARK'a "1000 µF (envanter), servo 5V–GND, Uno pin başlığına yakın" eklenmeli. R14 kriteri "en düşük Vcc (bandgap) ≥ 4.6 V, MCUSR'de beklenmeyen reset yok" olarak yazılmalı.

### D4 [önemli] Kama gereğinden ağır; kullanıcının "en az baskı" önceliğine aykırı ve ağırlık kararlılık için gerekmiyor
Yer: `cad/params_v6.py:76-79` (KAMA_DUVAR 2.4, KAMA_DIS_DERI 2.0, 3 nervür), DECISIONS D11 ("1.6 kabuk → daha hafif; plan 2.4 dediği için korunmadı"), BOM_FARK §1 (2 × 10 sa 15 dk, toplam baskının ~%91'i)
İddia: Kamanın dolu prizması 229 cm³; kabuk + nervür 76 cm³ (%33, 94 g). Duvar ve deri 1.2 mm'ye, nervür sayısı 1'e inerse model **40 cm³ / 50 g** olur ve baskı süresi kabaca yarıya düşer. Kalan devrilme payı hedefin çok üstünde kalır. Yük ~460 g ve kamanın 93 × 110 mm alana yayılıyor, yani dayanım için 2.4 mm duvar gerekmiyor.
Ölçme yöntemi: (a) Hacim: `eye_v6.make_kama()` KAMA_DUVAR 1.2, KAMA_DIS_DERI 1.2, KAMA_NERVUR_Z (−15,) ile çalıştırılır. Ben ölçtüm: 40 097 mm³ = 49.7 g. 1.6/1.6/tek nervürde 61.7 g. (b) Devrilme: kama AM'si (masa) y −21.6, z 17.2 mm; kama dışı kütle 458 g, AM y +0.94, z 64.7. İki yarı toplam 99 g olunca AM y = (458·0.94 − 99·21.6)/557 = −3.1 → arka kenara **27.7 mm**, h = 56.2 → **26°** (şimdi 30.3 mm / 30.8°; hedef 15/15). (c) Süre: Orca CLI (Epic POC 0.2 ve önerilen 0.28) ile yeni STL dilimlenir.
Beklenen sonuç (iddia doğruysa): yarı başına ≤ ~55 g; 0.2'de ≲ 5–6 sa, 0.28'de ≲ 3 sa; `R5_devrilme_alfa30` ≥ 25 mm / ≥ 24°.
Öneri: Duvar/deri 1.2 (3 çevre) ve tek orta nervür (dudak ve ray bölgesindeki et korunur). Daha da az baskı için tam genişlikte prizma yerine 3–4 profil plakası ve üst deri yapılabilir; bu ayrı bir tasarım kararı. Sonra `check.py` (R5, et, sarkma) yeniden koşturulmalı.

### D5 [küçük] Z8 eş eksenlilik zincirinde halka eksik; RSS ile geçiyor, en kötü durumda zaten kalıyor
Yer: `cad/check.py` tolerans_zincirleri Z8; rapor satırı: nominal 0.5, **en kötü −0.27**, RSS 0.154 (ölçüt "rss"; diğer zincirlerin hepsi "en_kotu")
İddia: Zincirde üç halka yok: (i) MG90S milinin gövdeye göre konumu (gövde SG90 modeliyle temsil ediliyor, D20), (ii) taban yarılarının yapıştırma kaçıklığı (y/z; yalnız Z1-B'de x olarak var), (iii) gerçek cep boşluğu (zincir SG90'ı 12.5 alıyor, TowerPro verisi 11.8–12.2 → 12.7 cepte ±0.25–0.45). Bunlar eklenince en kötü ~1.2–1.6 mm, RSS ~0.6–0.7 mm çıkar; 0.5 izni RSS ile de aşılır. Braket yarığı yalnız x'te ayar veriyor, y/z kaçıklığı ayarlanamıyor. Kaçıklık çerçeveyi ve servo millerini yanal yükler; bu da D1'deki %80 verimi düşürür.
Ölçme yöntemi: Çerçeve takılmadan (T9 öncesi) MG90S cepte ve sol braket+SG90 tabana vidalı haldeyken, kumpasla taban üst yüzünden iki milin alt ve üst kenarı ölçülür (y merkezi). Ön-arka konum (z merkezi) taban ön kenarından ölçülür.
Beklenen sonuç (iddia doğruysa, yani izin 0.5 tutuyorsa): |Δy| ve |Δz| ≤ 0.5 mm.
Öneri: Zincire üç halka eklenmeli ve ölçüt en_kotu yapılmalı. Ölçüm 0.5'i aşarsa braket ayağına y için pul, z için delik genişletme (±1) öngörülmeli.

### D6 [küçük] frame_L matkap talimatı gereksiz katı, delik yeri hatasına karşı ise önlem yok
Yer: MONTAJ.md:37 (A0 "Ø1.6 × 5.5, uca bant, delip geçme"), `params_v6.py:48` (ADP_PILOT_DERIN 5.5, şerit 5.6)
İddia: 5.5 hedefi 0.1 mm et bırakır; elde bantla bu doğrulukta durmak mümkün değil. Delip geçmek de zararsız: delik ekseninde şeridin 5–15 mm arkasında 9 kapak/yaw pozunun hiçbirinde parça yok (ölçtüm), M2×8 ucu da şeridin 0.6 mm içinde kalır. Asıl hata kaynağı konum: flanş deliği Ø2.3, M2 Ø2.0 → işaretle-çıkar-del yönteminde izin ±0.15 mm. Elde işaretleme genelde 0.3–0.5 mm sapar; sapma olursa vida eğik girer ya da adaptörü göbek üstünde kaydırır (Z7 payı 0.25). Ayrıca şerit baskıda dik duvar; pilot 2 çevre katmanı ve aradaki seyrek dolgudan geçiyor, kavrama yalnız çevrelerde.
Ölçme yöntemi: A0'dan sonra adaptör takılır ve M2'ler elle girdirilir. Vida başının flanşa eğik oturup oturmadığına bakılır. Adaptör uç diskinin servo milinden radyal salgısı da ölçülür: kumpas, dört yönden r 10 diskinin kenarı.
Beklenen sonuç (talimat yeterliyse): vidalar eksenel girer, salgı ≤ 0.25 mm.
Öneri: Talimat "adaptörü göbeğe tak, flanş deliklerini kılavuz kullanarak Ø1.6 ile şeridin içinden geç (delip geçmek zararsız)" olarak değiştirilmeli. Ya da flanş deliği Ø2.6'ya büyütülmeli (adaptör henüz basılmadı).

### D7 [küçük] T1 "sıkıca sık": kafa–somun kilidinin torku, köşe eti 1.69 mm olan v5 krank cebine biniyor
Yer: MONTAJ.md:38 (T1), v5 `eye_v5.crank_local()` (uç diski r 5.9, somun cebi AF 7.0 + 0.3)
İddia: Somun cepte dönmüyor, sıkma torkunun tepkisi altıgen cebin köşelerine geliyor. Köşe eti 5.9 − 7.3/√3 = **1.69 mm**, cep derinliği 3.4 mm. Tork sayısız ve "sıkıca" diye tarif edilmiş; cep yarılırsa basılı bir v5 parçası (yaw_crank) yeniden basılmak zorunda kalır. Kilit için yüksek tork gerekmiyor, çünkü yanında vida sabitleyici ve alt emniyet somunu var.
Ölçme yöntemi: Hesap yukarıda (v5 parametreleri: uç diski 5.9, M4 AF 7.0, cep payı 0.3). Donanımda T1'den sonra krank ucu, cep köşelerinde beyazlama ya da çatlak için büyüteçle incelenir.
Beklenen sonuç (talimat güvenliyse): beyazlama/çatlak yok.
Öneri: "Kafa somuna değince en fazla 1/8 tur, küçük tornavidayla parmak gücü; kilidi vida sabitleyici sağlar" yazılmalı.

### D8 [küçük] Envanter satırları belgelerle uyuşmuyor (MG90S, SG90 adedi)
Yer: BOM_FARK.md:27–28 ("SG90 elde (10)", "MG90S elde (1)"), PLAN §0 "elindeki MG90S"
İddia: `envanter.md@plan-v2` (2026-10-06) MG90S'i hiç listelemiyor; SG90 için "~6 boşta" diyor. "10 SG90 + 1 MG90S" bir alışveriş listesinden geliyor (lunar-tracker notu: "siparişin bir kısmı/tamamı geldi"). MG90S yoksa hesap bozulmaz (kapasite SG90 ile eşit alınmış, pay 2.01 aynı kalır), ama BOM yanlış olur.
Ölçme yöntemi: Kullanıcı çekmecede MG90S'i bulur (metal dişli, gövdede "MG90S" yazar) ve boştaki SG90'ları sayar.
Beklenen sonuç (belge doğruysa): MG90S 1 ve SG90 ≥ 1 boşta.
Öneri: BOM_FARK'ta adetler "envanter.md: SG90 ~6, MG90S listede yok, kullanıcı doğrulayacak" olarak düzeltilmeli. MG90S yoksa sağda SG90 kullanılır (hesap değişmez).

---

## Sorulara kısa cevap
1. M4×16 havşa: tasarımcı haklı (güncel sayım). Bulgu yok.
2. Kama: gereğinden ağır → D4. Devrilme payı hafif kamada da 27.7 mm / 26° (hedef 15/15).
3. Seçenek B: aritmetik doğru, paylaşım varsayımı ölçülmüyor → D1. Brown-out testi yetersiz → D3. Eksen kaçıklığı → D5.
4. +18 / 108: model payı doğru (0.7° aşım + 3° izin), ama kalibrasyondan önce spline hatası bu payı aşar → D2.
5. Matkap: et 0.1 mm hedefi gereksiz, delip geçmek zararsız; konum hatası asıl risk → D6.
6. Krank vidası: sıra yapılabilir, Z2 4.52 doğru, coupler serbest. Yalnız sıkma torku → D7.
7. Maske yolu: 5 pozda yeniden ölçüldü, mümkün. Bulgu yok.
