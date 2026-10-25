# Project Eye v6 — montaj (v5'ten farklar + sıra)

**Özet:** v5 parçalarının hiçbiri yeniden basılmaz. Yeni: `kama` ×2 (sağ = X aynası), `sol_servo_braketi`, `sol_kol_adaptoru`,
sol pitch servosu (SG90, **D9**), sağ pitch servosu **MG90S** (D5). Krank ucu vidası ters yönden (üstten) ve kilitli.
Bu sıra `cad/check.py` ile denetlenir: `R6_montaj_cerceve_alt` (T2–B1), `R6_montaj_ana` (T9–T12) sökerek doğrulama, `R6_giris_yolu`
(13 vida/somun için alet koridoru). Yönler mekanizma ekseninde: +X robotun sağı, **+Y mekanizmanın üstü**, +Z ileri.

## v5'ten farklar
| v5 | v6 |
|---|---|
| sol pitch: `pivot_bracket` + M4×10 pim | `sol_servo_braketi` + SG90 (D9, ters açı) + `sol_kol_adaptoru` (pivot_bracket rafa) |
| krank ucu: M4×10 yuvarlak **alttan**, kranktaki somuna | M4×16 havşa **üstten**, kafa kranktaki somuna kilitli + altta emniyet somunu (coupler'a 0.2) + vida sabitleyici |
| pitch servosu SG90 | MG90S (aynı cep) |
| pitch ±20 | **−10…+18** (servo 80…108; sol 100…72). +21.7°'de v5 kirişi tabana değer (DECISIONS D5) |
| taban masada | taban 30° kama üstünde, raylar + arka dudak |
| frame_L | aynı STL; şeride **2× Ø1.6 × 5.5 mm matkap** (A0) |

## Gereken alet / malzeme
Yıldız tornavida, küçük tornavida (M2, kol vidaları), 7 mm lokma ya da pense (alt somun), Ø1.6 matkap ucu + el matkabı/Dremel
(uca 5.5 mm bant), vida sabitleyici (orta güç), kumpas, telefon eğim ölçer, multimetre.

## Sökme (mevcut v5 donanımı)
| # | Adım | Hareket / alet |
|---|---|---|
| S1 | Uno'yu kapat, 3 servo fişini çek | — |
| S2 | Maske: alttan 2× M4×16 sök; maskeyi **30 mm öne kaydır, sonra kaldır** (düz yukarı çerçeveye takılır) | +Z, sonra +Y |
| S3 | pivot_bracket: tabandan 2× M4×16 havşa sök (**sakla: yeni brakette kullanılır**), braket ayağındaki 2 somunu sakla | alttan +Y |
| S4 | Çerçeve + braket **−X 5 mm** (kol pitch servosu milinden çıkar), sonra **+Y** kaldır | |
| S5 | Sol pimi (M4×10) ve somununu braketten sök (**somunu sakla: alt emniyet somunu olacak**); pivot_bracket artık kullanılmaz | dıştan |
| S6 | Sağ SG90 pitch: M3×10 + somun sök, servoyu cepten çek | −X |
| S7 | yaw_mount: kirişin altından 2× M4×16 havşa sök; yaw_mount + yaw servo **+Y** kalkar | alttan +Y |
| S8 | Krank ucu vidasını (M4×10, alttan) sök; krankı **+Y** kaldır. Cebindeki somunu cepte bırak | alttan +Y |

## Takma
| # | Adım | Takma yönü | Alet |
|---|---|---|---|
| **A0** | **frame_L matkap:** `sol_kol_adaptoru`'nu sol dış göbeğe geçir (D-delik göbeğin düz yüzüne), flanştaki 2 delikten şeride işaretle, adaptörü çıkar, **Ø1.6 × 5.5 mm** del (uca bant; şerit 5.6 kalın, delip geçme) | — | matkap, +X yönünde |
| **T1** | **Krank alt montajı (servo YOKKEN):** somun krank ucundaki cepte (üstten). M4×16 havşaya sabitleyici sür, **üstten (−Y)** somuna vidala; kafa somuna oturunca **sıkıca sık** (kafa–somun kilidi). Kontrol: krank alt yüzünden kafa tepesine ~7.3 mm | vida −Y | üstten |
| T2 | Krank → coupler: krankı **−Y** indir, vida gövdesi coupler'ın krank ucundaki deliğe girer | −Y | — |
| T3 | Alttan M4 emniyet somunu (S5'ten) **+Y**, sabitleyici; coupler'a değince **¼ tur geri** (~0.2). Coupler elle serbest dönmeli | +Y | 7 mm lokma alttan (kirişin arkası açık; vida ucu lokmanın içine girer) |
| T4 | Yaw servosu 90°, gözler düz. yaw_mount + servo **−Y** indir, mil krank kolunun göbeğine girer | −Y | — |
| T5 | yaw_mount → kiriş: alttan 2× M4×16 havşa | +Y | alttan |
| A1 | `sol_kol_adaptoru` sol göbeğe **+X** (D-delik, flanş şeride oturur) | +X | — |
| A2 | 2× M2×8 flanştan şerit pilotlarına (kılavuzsuz, aşırı sıkma) | +X | küçük tornavida −X tarafından |
| A3 | SG90 çift kolunu adaptör uç yüzüne 2 kol vidasıyla (r = 6 pilotlar; kol dikey) — **servo yokken**. Vida ≤ 5.4 mm | +X | küçük tornavida |
| B0 | (tezgâhta) Sol SG90 → `sol_servo_braketi` cebine, **gövde altı önde −X** (kablo plakadaki yarıktan geçer); kulaklar plakaya oturur. **M2×8 + M2 somun, milden uzak kulak** (kulak deliği sıkıysa Ø2.2). 2× M4 somunu (S3'ten) ayaktaki **üstten açık uzun ceplere** koy | −X | küçük tornavida +X tarafından |
| B1 | Sol servoyu **90° = pitch 0** konumuna getir (D9; sol servo ters: pitch 0 → 90, +18 → 72). Braket+servo **+X**: mil kola **tam** otursun | +X | — |
| T7 | MG90S'i 90°'ye getir. **Kumpasla** MG90S ve SG90'da kulak alt yüzü → mil ucu ölç (PLAN R12; fark braket yarığıyla karşılanır). MG90S'i base_R cebine **+X**, M3×10 + somun, milden uzak kulak | +X | −X tarafından |
| T8 | Sağ kol: SG90 kolu MG90S miline oturursa aynen kalır; oturmazsa frame_R'den sök, MG90S kolunu vidala (delik uymazsa 2× Ø1.6 pilot, servo yokken) | — | +X |
| T9 | Çerçeve (+ sol braket + sol servo, kolda) **−Y** indir, sonra **+X 5 mm**: sağ kol MG90S miline (pitch 0). Braket ayağı tabandaki havşaların üstüne gelir (yarık ±3) | −Y, +X | — |
| T10 | **Önce iki milin de kollara tam oturduğunu gör** (çerçeveyi eksenel itmeden), sonra alttan 2× M4×16 havşa (S3'tekiler) braket somunlarına | +Y | alttan (kama YOKKEN) |
| T11 | Maske: önde (≈30 mm ileride) indir, **30 mm geri kaydır**; alttan 2× M4×16 | −Y, −Z | alttan |
| T12 | Kamalar masaya, kaymaz ayaklar altta (Ø10 izlere). Taban önden, eğim boyunca aşağı (**−Z**) rayların altından kaydır, arka dudağa dayanır. Pitch 0 | −Z | — |
| T13 | *(isteğe bağlı)* LIDS A pimine sabitleyici: ¼ tur gevşet, damla, geri sık; kapak serbest dönmeli | — | v5 adım 13 |
| T14 | Kablolar: yaw (D3) + kapak (D6) kiriş arka ucuna bağla, arka dudaktaki kertikten Uno'ya; pitch servoları (D5 sağ, D9 sol) yanlardan doğrudan. **Uno kamanın hemen arkasında (kertikten ≤ 60 mm)**; uzaksa 15 cm uzatma | — | — |

**Sökme kontrolü:** her adım tersten yapılabilir (`check.py`). T3 tersi: lokma alttan, kirişin arkası açık. T9 tersi: −X 5 sonra +Y.
T12 tersi: taban +Z kayar (arka çıkıntılar dudağın üstünden geçer). Ölçülmeyen: somunların cebe konma yolu, elle tutma, kablo yolu.

## Firmware / ayar (bu işin DIŞINDA — ayrı iş, PLAN §10 + DECISIONS D5)
`EYE_PITCH` 80…**108** (plan 110 değil), **`EYE_PITCH_L` (D9) = 180 − EYE_PITCH**, iki pitch kanalı aynı anda ve hız sınırlı
(≥ 0.25 s / 10°), ayrı kalibrasyon ofseti; `mount_tilt_deg: 30`; `pitch_dead: false`.

## Testler (kullanıcı)
| Test | Nasıl | Geçme |
|---|---|---|
| T-R11 tutma | MG90S ve sol SG90'a 20 mm kol, **330 g** asılı, Uno 5 V'tan, servo 90°'de 2 dk | kaymaz, titremez, reset yok |
| Z2 | T4'ten sonra yaw'ı elle ±25° çevir | M4 havşa kafası yaw servo gövdesine değmez (model 4.5 mm) |
| R13 | krank ucu vidası + A pimi + alt somuna boya kalemi çizgisi; 30 dk idle + stant sonrası | çizgi kaymamış |
| R14 | multimetre 5V–GND, pitch −10…+18 yavaş tarama ×10 | ≥ 4.6 V, reset yok, iki servo birbirine karşı vızıldamıyor |
| R15 | telefon eğim ölçer taban üstünde | 30 ± 1° |
| Durma | pitch 108'de çerçeve kirişi ile taban arası | göz kararı ≥ 1 mm (model 1.5+) |
