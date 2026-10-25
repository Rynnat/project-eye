# Project Eye v6 — kararlar (neden + alternatif)

Girdi: `PLAN.md` (§11 kullanıcı kararları geçerli: eğim 30°, pitch −10…+20, düşen vida = krank ucu, denge = SEÇENEK B).
Kontrol: `cad/out/check_report.txt`. "PLAN SAPMASI" etiketli bölümler `check.py plan_sapmasi` ile denetlenir (anahtar kelime parantez içinde).

## D1 — v5 geometrisi kopyalanmaz, import edilir
`cad/eye_v6.py` → `v5/cad/eye_v5.build()` çağırır, çıkan/yeni parçaları ekler. v5 klasörüne hiçbir şey yazılmaz
(`sys.dont_write_bytecode`, numba önbelleği v5'e yazmasın diye render v5 modülünü import etmez, dışa aktarma `v6/cad/out`).
Kanıt: `R16_v5_stl_md5` (19 v5 STL md5'i `cad/v5_dondurulmus_md5.json` ile aynı) + `R16_v5_parca_geometri` (v6 montajındaki 18 v5
parçası v5 STL'iyle hacim/kutu olarak aynı). Alternatif: v5 kodunu kopyalamak → iki kopya ayrışır, prensip 7 denetlenemez.

## D2 — parametre dosyası `params_v6.py` (params.py değil)
v5 modülleri `from params import *` yapar; v6'da `params.py` olsaydı sys.path sırası yüzünden v5 yerine v6 dosyası yüklenirdi.

## D3 — SEÇENEK B geometrisi (plan ayrıntılandırmamıştı)
- `sol_kol_adaptoru` (frame grubu): frame_L dış göbeğine (r 6.5, z ≤ 3.5 düz) **D-delikle** geçer (radyal pay 0.25, Z7) — düz kenar
  dönmeyi tutar; şerit boyunca 3 mm flanş 2× M2×8 ile frame_L'ye (matkapla Ø1.6 × 5.5 pilot, MONTAJ A0). Uç duvarı 5.5 mm:
  kol vidası pilotu 3.9 + D-deliğin üstünde 1.6 et (ilk deneme 4.0'da parça kontrolü 0.5 mm et yakaladı). SG90 çift kolu uç yüzüne
  r = 6'daki 2 pilotla (v5 frame_R yöntemi). Baskı: flanş temas yüzü tablada.
- `sol_servo_braketi` (static): v5 pivot_bracket'ın 2 taban havşasına (x −91.4, z ±14) oturur, **aynı 2× M4×16 havşa** kullanılır.
  Sol SG90 sağ servonun aynası (mil +X, gövde aşağı), tutucu plaka kulakların altında (base_R tutucusu gibi), plaka 3.5 + 2 nervür.
  Ayak deliklerinde **x yönünde ±3 mm yarık**, somunlar üstten açık uzun altıgen cepte (dönmez): montajda sol mil kola tam
  oturtulur, sonra sıkılır → iki milin eksenel uyumu ayarla sağlanır (Z1-B: en kötü +0.3). Alternatif: sabit delik → MG90S mil
  yüksekliği farkı (±1.0 [TAHMIN]) doğrudan sol kavramaya biner (en kötü −2.7).
- Sol servo gövdesi taban kenarından 24 mm dışarı taşar (servo alt ucu x −125.4). Kama rayları z ≤ −24, braket z ±20: çakışmaz.
- Neden sol kulak M2: bkz. D9.

## D4 — krank ucu vidası (kullanıcı kararı uygulandı)
M4×16 havşa **üstten**, kafa kranktaki mevcut somunun üst yüzüne oturur ve sıkılır (kafa–somun kilidi; somun altıgen cepte
dönemez → vida kranka kilitli). Coupler vida gövdesi üstünde döner (4.3 delik). Altta M4 emniyet somunu coupler'a 0.2
(¼ tur geri, Z5), vida sabitleyici. Vida ucu alt somundan 1.42 mm taşar (≥ 0.7). Kafa ISO 10642 maks yükseklikte (2.48) modellendi.
Montaj sırası: krank alt montajı **servo takılmadan** (T1), sonra coupler'a (T2), sonra alt somun (T3), sonra yaw servo (T4).
Neden düşmez: gevşese bile kafa krankın üstünde, vida aşağı yalnız alt somun çıkarsa iner; iki kat sabitleyici.

## D5 — PLAN SAPMASI: pitch üst sınırı +20 → +18 (anahtar: pitch ust siniri)
**Neden:** v5 çerçevesinin arka kirişi (frame_L/R, y −29, z −74) pitch **+21.7°**'de tabanın üst yüzüne çarpıyor
(`R3_pitch_mekanik_durma`, ikiye bölme). Bu v5 geometrisi (basılı); plan Z3 yalnız krank ucunu ve kama dudağını hesaplamıştı.
Plandaki ±3° kalibrasyon aşımıyla +20 → +23 imkânsız (çerçeve × taban 92 mm³ kesişim @ +23).
**Karar:** üst sınır +18 (sağ servo 108, sol 72); aşım pozu +21 → 0.7° mekanik pay, en dar boşluk 0.70 mm.
Masaya göre bakış 20°–48°; plan §1.3'te gereken 21°–42° yine kapsanıyor. Tork payı da biraz artar.
**Alternatif:** +20'de kalıp kalibrasyonu ±1.5°'ye zorlamak (telefonla doğrulanır) — kullanıcı isterse firmware'de 110'a çıkabilir,
risk: durma noktasında servo stall akımı. Firmware/settings bu işin dışında: **EYE_PITCH max 108, EYE_PITCH_L min 72** yazılmalı.

## D6 — PLAN SAPMASI: çarpışma aşım pozları −13/+23 → −13/+21 (anahtar: asim pozu)
D5'in sonucu. Pozlar: yaw {−25, 0, 25} × pitch {−13, −10, 0, 10, 18, 21} × kapak {0, 0.5, 1} = 54 poz, masa eğik.

## D7 — PLAN SAPMASI: M4×16 havşa satın alınacak (anahtar: M4x16 havsa)
Plan "M4×16 havşa (envanter, bol)" diyordu. Envanter (`agentic-mechanism-design/envanter.md`): M4 havşa **yalnız 2× M4×10**;
v5'teki 15 M4×16 havşa takılı/kilitli. SEÇENEK B'de pivot_bracket'ın 2 M4×16'sı yeni brakete geçer (sökülen = kullanılan).
Krank ucu için **1 adet satın alınır** (+2 yedek, ~10 TL). M4×10 havşa yetmez (gereken 15.3 mm). Satın alınmadan T1 yapılamaz.

## D8 — PLAN SAPMASI: adaptör vidası sac vida yerine M2×8 (anahtar: adaptor vidasi)
Plan "plaka şeridine 2 sac vida" diyordu (servoyla gelen). Envanterde M2×8 30 adet var (kesin), SG90 kit vidalarının adedi ve boyu
bilinmiyor. M2×8 Ø1.6 pilota kılavuzsuz: flanş 3 + şerit 5 (pilot 5.5, şerit 5.6).

## D9 — PLAN SAPMASI: sol servo kulak vidası M3 yerine M2×8 + M2 somun (anahtar: sol servo kulak vidasi)
Plan arayüz tablosu (B) "M3×10 kulak" diyordu. v5'te bilinen sorun: SG90 uzak kulak deliği gövde ucundan 2.65 mm, M3 kafa/somun
gövdeye ~0.15 mm biniyor (v5 §2) ve kulak 3.2'ye delinmeli. M2 kafa (Ø4) gövdeye 0.65 mm boşluk bırakır, delme gerekmez
(kulak deliği ~Ø2; sıkıysa Ø2.2). Plaka 3.5: kulak 2.5 + plaka 3.5 + somun 1.6 + 1 diş 0.4 = 8.0 (tam).

## D10 — PLAN SAPMASI: sağ ray tek STL için ayna (anahtar: sag ray)
Plan sağ ray "x 95…116.5, z −62…−12", sol "x −101…−96, z −62…−24". İki kama yarısı **tek STL** (sağ = X aynası, plan §2.2)
olunca raylar simetrik olmalı: sağ dudak x 111.5…116.5, z −66…−24. Sağda bu bölgede taban üstünde engel yok (servo tutucu z ±9.5),
taban arka kenarı z −62 dudakla tutulur, ray yalnız kalkmayı önler — 5 mm yeterli.

## D11 — PLAN SAPMASI: kama kütlesi ~60 g → 2 × 65 g (tur 1; aşama 2'de 2 × 94 g) (anahtar: kama kutlesi)
**Tur 1 güncellemesi (D21):** 1.6 duvar/deri/nervür + 2 nervür → 52.7 cm³/yarı = 65.4 g dolu; Orca 0.2: **7 sa 42 dk / 60.5 g**, 0.28 öneri: **4 sa 09 dk / 59.6 g** yarı başına. Aşağıdaki metin aşama 2 kaydıdır.
Plan tahmini [TAHMIN] düşük. Profil (yan yüz) x boyunca 109.6 mm prizmatik: 2.4 kabuk + 3 nervür + dış deri 2.0 → 76 cm³/yarı
(dolgu 1.0: duvarlar 2–2.4 mm, dilimleyicide dolu basılır). Sonuç: toplam ~650 g; devrilme açısından lehte (AM öne/alçağa).
Orca CLI (Epic POC, 0.2): **10 sa 15 dk / 89 g yarı başına**; kama hassas geçme içermez (yalnız ray payı 0.3) → 0.28 katman + dış 50/iç 70 mm/s ile **5 sa 22 dk / 88 g** (öneri). Alternatif: 1.6 kabuk → daha hafif; plan "kabuk 2.4" dediği için korunmadı.

## D12 — PLAN SAPMASI: pitch ekseni masadan 86 → 88.6 mm (anahtar: pitch ekseni yuksekligi)
Kama arka dudağı 3.7 + dudak payı 0.3 (arka alt köşe z −66) ve kaymaz ayak 1.0 mm taşma [TAHMIN 1.5 − 0.5 iz] eklendi.
Göz merkezi 75 cm masada ≈ 83.9 cm. Bakış açısı tablosu (§1.3) ±0.3° değişir.

## D13 — PLAN SAPMASI: tork kapasitesi 4.6 V'a ölçeklendi (anahtar: tork kapasitesi)
Plan 0.63 kg·cm @4.8 V (%35 kuralı). PLAN R14 Uno hattını ≥ 4.6 V kabul ediyor → kapasite 4.6/4.8 ile ölçeklendi: 0.604 / servo
(Quad Spider v1 D4 dersi). SEÇENEK B: (0.604 + 0.604) × 0.8 = 0.966 kg·cm.
**Sonuç (dolgu 0.75 esas):** gereken 0.482 (yerçekimi 0.421 @ −10° + dinamik 0.041 + sürtünme 0.02) → **pay 2.01**; dolgu 1.0 → 1.68;
dolgu 0.6 → 2.27. Tek MG90S (B olmasaydı) 1.25 — B'nin gerekçesi.

## D14 — PLAN SAPMASI: tutma testi 315 g → 330 g (anahtar: tutma testi)
315 g × 20 mm @ Uno 5 V, 4.6 V eşdeğeri 0.60 < kapasite varsayımı 0.604 → test varsayımı kanıtlamaz (`test_esik`). 330 g → 0.632 ✓.
Kanıtlanan pay servo başına 2.10.

## D15 — PLAN SAPMASI: maske takma yolu −Y → önde indir + 30 mm geri kaydır (anahtar: maske takma yonu)
Plan T11 "maske −Y". `montaj_sokme`: maske ayağı (z −4…24.4) çerçevenin altına uzanır; düz −Y yolunda 22 mm'de frame_R'ye
çarpar (523 mm³). Önde (30 mm ileride) indirip geri kaydırınca yol serbest. v5'te maske yolu hiç denetlenmemişti (yalnız vidalar).

## D16 — kama tasarımı
Profil mekanizma YZ düzleminde: taban altı (y −60) kama üst yüzü, arka alt köşe (y −63, z −66), ön yüz dünyada dikey (ön yükseklik
49.1). Arka dudak tepesi y −57 (plan), orta 36 mm'de kablo kertiği. Raylar arka bölgede (z ≤ −24), dudak taban üstünden 0.3 + 2.7.
Cepler iç yüze (ortaya) açık, dış yüzde 2 mm deri; tüm duvarlar x boyunca prizmatik → dış yüz tablada **desteksiz** basılır.
4 kaymaz ayak izi (Ø10 × 0.5) dış kenarlardan 8 mm, alt yüz kenarlarından 6 mm. Devrilme (α30): AM arka destek kenarından
30.3 mm, devrilme açısı 30.8° (hedef 15/15); α40 21.2 mm (bilgi), α20 38.3 mm.

## D17 — Z2 sonucu plandan iyi
Plan Z2'yi servo üst yüzünü her yerde göbek seviyesinde (y −9.1) varsayıp 0.52 (en kötü −0.08) bulmuştu. v5 modelinde göbek yalnız
mil çevresinde (r 5.9); vida kafası milden ≥ 16 mm'de, gövde üst yüzü y −5.1 → **model boşluğu 4.52 mm**, zincir en kötü 3.92.
Kullanıcı kontrolü: T4'ten sonra yaw'ı elle ±25° çevir, kafa servoya değmemeli (SG90 üst kapağında mil dışında çıkıntı yoksa doğru).

## D18 — kablo ve Uno yeri
SG90/MG90S kablosu 250 mm [KAYNAK, ölç]. Uno kamanın hemen arkasında (kertikten ~60 mm) [SEÇİM]. Hareketli kablolar (yaw, kapak)
kiriş arka ucuna bağlanıp kertikten iner: en uzun 178 + 30 servis = 208 ✓. Sabit pitch servoları yanlardan doğrudan Uno'ya:
224 / 232 mm + 10 bükülme payı (hareket yok, servis halkası gerekmez) → marj 8–16 mm. Uno daha uzaksa **15 cm servo uzatma** (BOM).

## D19 — kontrol eşikleri ve muaflar
- Min boşluk 0.4, ölçümde 0.395: v5 AXIAL_GAP 0.4 düz yüzeylerde üçgenlemede 0.3999 çıkıyor.
- Kesişim eşiği 0.05 mm³: havşa kafa konisi yuvasıyla aynı yüzey (v5 §2'de 0.012–0.014 mm³ "üçgenleme").
- Muaf çiftler (raporda OLCULMEDI): servo × kendi kablo modeli; v5'te bilinen M3 kafa/somun × SG90 gövdesi (3 servo); scr_U × lid_up_L
  (kılavuzsuz M4); lid_lo_L × lid_lo_R (v5 bindirme); tasarım gereği temas/0.4 eklemleri (eye_v5.JOINTS + sol mil-kol, krank vidası-coupler,
  alt somun-coupler 0.2) — bunların kesişimi ayrıca `R3_cakisma_eklem_kesisim`'de ölçülür.

## D20 — MG90S modeli
MG90S gövdesi SG90 modeliyle temsil edildi (22.8×12.2 vs 22.5×12.5 [TAHMIN]); v5 cebi 22.9×12.7. Kulak altı → mil ucu farkı Z1-B'de
±1.0. Kumpas ölçümü (PLAN R12) T7'den önce; fark braket yarığıyla (±3) karşılanır, pul gerekmez.

---
# Düzeltme turu 1 (2026-10-12) — girdi `dogrulama.md`: DOĞRULANDI D2, D4, D6, D7, D8 · ÇÜRÜTÜLDÜ yok · ÖLÇÜLEMEZ D1, D3, D5
Her düzeltme için hata sınıfını yakalayan kontrol `agent-flow/checks/` altına yeni dosya olarak eklendi (`hafiflik.py`, `belge.py`;
sentetik test `checks/test_eye_v6_tur1.py`). Düzeltme ÖNCESİ belgeler/parametreler `cad/out/tur0/`; kanıt `cad/kanit_duzeltme.py` →
`cad/out/kanit_duzeltme.txt`: **ÖNCE SORUN (9) → SONRA TEMİZ**. Aynı kontroller `check.py`'de kalıcı.

## D21 — PLAN SAPMASI: kama kabuğu 2.4 → 1.6 (MIN_ET), 3 → 2 nervür (anahtar: kama kabugu) — denetim D4
**Neden:** kullanıcı önceliği "en az baskı"; kama toplam baskının ~%91'iydi. Yük ~460 g, 93 × 110 mm alana yayılıyor; dayanım için 2.4 gerekmiyor.
Denetimin önerdiği 1.2 mm **uygulanmadı**: proje kuralı `MIN_ET = 1.6` [SPEC] (koordinatör talimatı: 1.2 KULLANMA).
**Karar:** duvar/dış deri/nervür = MIN_ET 1.6; tabana dik 2 nervür z −35 / −3 (üst yüz açıklıkları 31 / 32 / 30 mm; ince duvar için
tek nervürün 51 mm açıklığı yerine). Taban altı et T0 3.0 ve ray/dudak ölçüleri aynı.
**Sonuç:** yarı 76 106 → **52 730 mm³** (94.4 → **65.4 g** dolu); Orca Epic POC 0.2: 10 sa 15 dk / 89 g → **7 sa 42 dk / 60.5 g**;
0.28 öneri: 5 sa 22 dk → **4 sa 09 dk / 59.6 g** (yarı başına). Toplam baskı 22.5 → ~17.4 sa (öneriyle ~10.3 sa).
Devrilme α30 en kötü (p−13, kapak 1): 30.3 mm / 30.8° → **28.6 mm / 27.8°** (hedef 15 / 15). Parça kontrolü (tek gövde, ada, et, tabla,
sarkma) GEÇTİ; kaba et kontrolünde %0.4 alan < 1.6 = 60° sivri profil köşesi (aşama 2'deki ile aynı yanılgı).
**Kontrol:** `R9_kama_hafiflik` (`checks.hafiflik`): hacim / aynı geometrinin tüm etleri MIN_ET varyantı ≤ 1.15. Önce 1.36 (yakaladı), şimdi 1.00.
Alternatif (yapılmadı): profil plakaları + üst deri ya da nervür pencereleri — sarkma/köprü riski, ayrı tasarım kararı.

## D22 — denetim D2: kalibrasyon sırası MONTAJ'a adım olarak (K1–K5)
Spline yarım kademesi (SG90 8.57°, MG90S 7.20°) mekanik durma payından (3.72°) büyük; kalibrasyon yalnız firmware notundaydı.
MONTAJ'a T14'ten sonra: T15 kondansatör, **K1** sağ pitch kalibrasyonu (D9 fişi çıkık, kol en yakın dişte, eğim ölçerle gerçek açı,
|fark| > ~4° ise kolu 1 diş kaydır), **K2** 1°'lik adımlarla +18'e, iki noktalı µs kaydı, **K3** D9 ofseti (D9 = 180 − a + ofset),
**K4** daraltılmış ilk tarama 80–100, **K5** tam tarama (R14). Firmware notu (ayrı iş): `writeMicroseconds`, iki noktalı kalibrasyon,
sınır ofsetten sonra kesilir. Elektrik kesilince +21.7° durmaya düşme gözlemi not edildi.
**Kontrol:** `MONTAJ_kalibrasyon_sirasi` (`checks.belge.kontrol_adim_sirasi`): 4 adım tablo satırında ve sırayla. Önce 3 bulgu (yalnız R14 vardı).

## D23 — denetim D6: frame_L matkabı delip geçer
5.6 şeritte 5.5 kör delik 0.10 mm et bırakıyordu; elde tutturulamaz. Delik ekseninde şeridin arkası 0–10 mm, 54 pozda tüm parçalara
(frame_L matkapsız dahil) kesişim **0**. `ADP_PILOT_DERIN = ADP_SERIT_T + 1` (montaj modelinde delik şeridi geçer; v5 STL aynı, R16 geçer).
A0: adaptörü göbeğe tak, **flanş deliklerini kılavuz kullanarak** Ø1.6 ile del, derinlik hedefi yok. Flanş deliğini Ø2.6'ya büyütme
seçeneği uygulanmadı (doğrulanmadı; konum hatası ÖLÇÜLEMEZ kısım, kılavuzlu delme ile karşılandı).
**Kontrol:** `R6_frame_L_matkap` (`checks.belge.kontrol_kor_delik`). Önce: kalan et 0.10 < 0.5 (yakaladı).

## D24 — denetim D7: T1 sıkma sayısal
"sıkıca sık" → "değince en fazla 1/8 tur" (cep köşe eti 1.70 mm ölçüldü; kilidi vida sabitleyici sağlar).
**Kontrol:** `MONTAJ_sikma_talimati` (`checks.belge.kontrol_sikma_talimati`). Önce T1 satırı (yakaladı).

## D25 — denetim D8 (+ D3 BOM kısmı): BOM_FARK envanterle uyumlu
SG90 "elde (10)" → envanter boşta ~6; MG90S "elde (1)" → **envanterde YOK, kullanıcıya soruldu**; sağ pitch "MG90S varsa onu, yoksa
SG90" (yoksa v5 SG90 yerinde kalır, S6/T7/T8 atlanır). Tork hesabı zaten SG90 kapasitesiyle (KAP_MG90S = KAP_SG90) → pay değişmez.
D3'ün doğrulanan kısmı: envanterdeki 1000 µF 16 V (×4) BOM'a ve MONTAJ T15'e eklendi (1 adet). Bu satın alma değil.
**Kontrol:** `BOM_envanter_belge` (`checks.belge.kontrol_bom_belge_envanter`). Önce 3 bulgu (SG90 10≠~6, MG90S, kondansatör yok).

## D26 — ÖLÇÜLEMEZ (D1, D3 gerilim, D5): kullanıcı testi
Modelle ölçülemez; MONTAJ "Kullanıcı testleri" bölümüne doğrulama.md'deki tek cümlelik testler yazıldı, `check.py` KAPSAM'da
OLCULMEDI satırı. D1: pay hesabı `IKI_SERVO_VERIM 0.8 [TAHMIN]` korunuyor; tek servo en kötü 1.25 (dolgu 0.75) raporda bilgi.
D5 zinciri değiştirilmedi (kumpas ölçümü bekleniyor).
