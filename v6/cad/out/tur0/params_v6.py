"""Project Eye v6 - YENI olculer (v5 olculeri v5/cad/params.py'den import edilir, burada TEKRAR YAZILMAZ).

Dosya adi `params_v6.py`: v5 modulleri (`eye_v5`, `kin`) `from params import *` yapar; v6'da `params.py` olsaydi
sys.path sirasina gore v5 yerine v6 dosyasi yuklenirdi (DECISIONS D2).

Koordinat: v5 MEKANIZMA ekseni (orijin goz merkezlerinin ortasi, +X sag, +Y mekanizmanin ustu, +Z ileri).
Masa: mekanizma X ekseni etrafinda TILT_DEG kadar BURUN YUKARI doner (kin_v6.masa_M).
Etiketler: [SPEC] PLAN/kullanici | [TOL] baski toleransi | [KAYNAK] datasheet/standart | [TAHMIN] olculmedi (nasil olculur yaninda)
           [OLCUM] kullanici olcer | [SECIM] tasarim karari | [v5] v5'ten aynen
"""
import math

# ------------------------------------------------------------------------------------------ 1. Spec (PLAN §1, §11)
TILT_DEG = 30.0          # [SPEC] kullanici 2026-10-12 onayi. TEK degisken: kama, masa, devrilme hepsi bundan turer (20/30/40 uretilebilir)
PITCH_MIN, PITCH_MAX = -10.0, 18.0     # [SPEC->DUSURULDU] PLAN §11.3: -10..+20 (servo 80..110). PLAN SAPMASI (DECISIONS D5):
                                       # v5 kiris ucu (frame_L/R, y -29 z -74) pitch ~+21.5 deg'de TABAN USTUNE carpar (v5 geometrisi,
                                       # basili). +20 + 3 asim = +23 imkansiz -> ust sinir +18 (servo 108), asim pozu +21.
                                       # Masaya gore bakis 20..48 deg; gereken 21..42 (PLAN §1.3) yine kapsaniyor.
PITCH_ASIM = 3.0         # [SPEC] PLAN §7 R3: kalibrasyon asimi +-3 deg -> carpisma pozlari -13 ve +23
YAW_MAX = 25.0           # [v5]
CHECK_YAWS = (-25.0, 0.0, 25.0)
CHECK_PITCHES = (PITCH_MIN - PITCH_ASIM, PITCH_MIN, 0.0, 10.0, PITCH_MAX, PITCH_MAX + PITCH_ASIM)
CHECK_LIDS = (0.0, 0.5, 1.0)
MIN_BOSLUK = 0.4         # [SPEC] hareketli gruplar arasi
MIN_BOSLUK_OLCUM = 0.395 # 0.4 tasarim boslugu (AXIAL_GAP) duz yuzeylerde ucgenlemede 0.3999 cikar -> 0.005 ucgenleme payi
MIN_ET = 1.6             # [SPEC]

# ------------------------------------------------------------------------------------------ 2. Toleranslar (sablonlar/toleranslar.py ile ayni sayilar)
M2_SERBEST = 2.3         # [TASLAK] sablonlar.toleranslar.M2_SERBEST (M2 basilip denenmedi)
M2_PILOT = 1.6           # [v5 HORN_SCREW_PILOT] kilavuzsuz M2 / SG90 sac vidasi pilotu (v5 frame_R'de kullanildi)
YATAY_EK = 0.2           # [TASLAK] sablonlar.toleranslar.YATAY_DELIK_EK
GECME_PAY = 0.25         # [TOL] sol_kol_adaptoru D-deligi radyal pay: gobek r 6.5 -> 6.75 (Z7: dikey delik -0.15, yatay basilan gobek +-0.1)

# ------------------------------------------------------------------------------------------ 3. Vidalar (yeni)
M4_CSK_K = 2.48          # [KAYNAK] ISO 10642 M4 kafa yuksekligi maks (Z2 icin kotu durum)
M4_CSK_DK = 8.96         # [KAYNAK] ISO 10642 teorik kafa capi (kafa konisi 90 deg; 4.0 -> 8.96)
M4_JAM_GAP = 0.2         # [SPEC] PLAN §3/T3: alt emniyet somunu coupler'a 0.2 (1/4 tur geri al)
M2_HEAD_D, M2_HEAD_H = 4.0, 1.6     # [KAYNAK] ISO 7045 M2 mercimek dk 4.0, k 1.6
M2_NUT_AF, M2_NUT_T = 4.0, 1.6      # [KAYNAK] ISO 4032 M2
M2_SHANK_MODEL = 1.5     # kok cap ~1.57: pilot 1.6 ile cakismasin (v5 kurali: dis plastige gomulur)

# ------------------------------------------------------------------------------------------ 4. Sol pitch servosu (SECENEK B)
# sol_kol_adaptoru: frame_L dis gobegine (x -79.9..-88.5, r 6.5, z<=3.5 duz) gecer, plaka seridine 2x M2x8.
ADP_FLANS_T = 3.0        # [SECIM] flans kalinligi (x -80.4 -> -83.4); M2x8: 3 flans + 5 serit (serit 5.6)
ADP_FLANS_Y = (-20.0, 20.0)         # [SECIM] serit boyunca
ADP_VIDA_Y = (-14.0, 14.0)          # [SECIM] M2 vida y; serit icinde kapak pimi (y 0) ve kopruler (|y| >= 17.3) disinda
ADP_VIDA_Z = -0.75                  # serit ortasi (STRIP_Z -5..3.5)
ADP_PILOT_DERIN = 5.5    # [SECIM] frame_L'ye MATKAPLA acilan O1.6 pilot derinligi (plaka 5.6; uca bant isareti)
ADP_KAP_DUVAR = 2.0      # [SECIM] gobek kapagi duvari (r 6.7 -> 8.7)
ADP_UC_T = 5.5           # [SECIM] uc duvari: pilot 3.9 + 1.6 et D-deliginin ustunde (ilk deneme 4.0 -> et 0.5, parca kontrolu yakaladi)
ADP_UC_R = 10.0          # [SECIM] uc diski (kol oturma yuzu)
ADP_EKSENEL = 0.4        # [v5 AXIAL_GAP] gobek ucu (x -88.5) ile uc duvari arasi
KOL_VIDA_DERIN = 3.9     # kol sac vidasi pilot derinligi: vida boyu <= 1.5 kol + 3.9 = 5.4 [OLCUM: servo ile gelen kol vidasi boyu]

# sol_servo_braketi: v5 pivot_bracket'in 2 taban deligine (x -91.4, z +-14) oturur.
BRK_AYAK_Y = (-54.0, -47.0)         # [v5] taban ustu -54; ayak 7 (v5 BRK_FOOT_T)
BRK_AYAK_X = (None, -81.4)          # ic uc: maske ayagi x -80'e 1.4 (x0 plakanin dis yuzunden baslar)
BRK_AYAK_Z = (-20.0, 20.0)          # [v5] kama raylari z <= -24
BRK_VIDA_X = -91.4                  # [v5] (pivot_bracket ayak delikleri; base havsalari burada)
BRK_VIDA_Z = (-14.0, 14.0)          # [v5]
BRK_AYAR = 3.0           # [SECIM] ayak deliklerinde +-3.0 x yarigi: sol mil kola tam oturtulup sonra sikilir (Z1-B zinciri; 2.5 en kotu -0.2 cikti)
BRK_PLAKA_T = 3.5        # [SECIM] servo tutucu plaka (M2x8 + M2 somun: kulak 2.5 + plaka 3.5 + somun 1.6 + 1 dis 0.4 = 8.0)
BRK_PLAKA_Y1 = 13.5      # [v5] base_R tutucu ust ucu ile ayni
BRK_PLAKA_Z = 9.5        # [v5] base_R tutucu +-9.5
BRK_NERVUR_Z = (7.0, 9.0)           # servo govdesi z +-6.25 -> 0.75 bosluk; plaka kenar yuvarlatmasinin (r1) icinde kalir
BRK_NERVUR_UST_Y = 0.0   # [SECIM] nervur plakada y 0'a kadar, ayaga x -97'de iner
BRK_NERVUR_X1 = -97.0
SOL_KOL_UC_X = None      # turetilir: adaptor uc yuzu - kol kalinligi (kin_v6)

# ------------------------------------------------------------------------------------------ 5. Kama (PLAN §2.2) ve masa
KAMA_T0 = 3.0            # [SPEC] taban altinda min et (arka)
KAMA_DUDAK_T = 3.7       # [SECIM] arka dudak kalinligi (z)
KAMA_DUDAK_UST = -57.0   # [SPEC] dudak tepesi y <= -57 (PLAN §2.2, Z3)
KAMA_DUDAK_PAY = 0.3     # [TOL] dudak ic yuzu ile taban arka yuzu (z -62) arasi
KAMA_ON_Z = 27.0         # [v5] taban on yuzu
KAMA_DUVAR = 2.4         # [SPEC] kabuk (profil cevresi)
KAMA_DIS_DERI = 2.0      # [SECIM] dis yuzde kapali deri (cepler ice / ortaya acik)
KAMA_NERVUR = 2.0        # [SECIM] ic nervur kalinligi
KAMA_NERVUR_Z = (-40.0, -15.0, 8.0)   # [SECIM] tabana dik nervurler
KAMA_ORTA_X = (-101.0 + 116.5) / 2    # 7.75 taban ortasi = ayna duzlemi (tek STL, R aynali)
KAMA_ORTA_BOSLUK = 5.0   # [SPEC <= 10] iki yari arasi
RAY_PAY = 0.3            # [SPEC] ray ic yuzu - taban yan yuzu, ray dudagi alti - taban ustu (PLAN §3)
RAY_T = 3.0              # [SECIM] ray duvari (x)
RAY_DUDAK_IC = 5.0       # [SPEC] ray dudagi taban ustune 5 mm biner (sol x -101..-96)
RAY_DUDAK_UST = -51.0    # [SPEC] dudak taban ustunden en fazla 3 mm yukselir (-54 + 3)
RAY_Z = (None, -24.0)    # [SPEC] arka bolge: z (arka) .. -24 (braket ayagi z +-20, servo tutucu z +-9.5)
KERTIK_X = 18.0          # [SPEC] |x - orta| <= ~18: arka dudakta kablo kertigi (dudak kaldirilir)
AYAK_D, AYAK_T = 10.0, 1.5   # [TAHMIN] kaymaz ayak O10 x 1.5 (yapiskanli silikon; aldiginda kumpasla olc, AYAK_T guncelle)
AYAK_IZ = 0.5            # [SPEC] sig iz derinligi
AYAK_KENAR = 6.0         # [SECIM] ayak merkezi kama alt yuzunun arka/on kenarindan (egim boyunca)
AYAK_X_KENAR = 8.0       # [SECIM] ayak merkezi kamanin dis kenarindan (x)

# ------------------------------------------------------------------------------------------ 6. Kutle / tork (PLAN §5.0)
PLA_RHO = 1.24e-3        # g/mm3 [KAYNAK]
CELIK_RHO = 7.85e-3      # g/mm3 [KAYNAK]
DOLGU_ESAS = 0.75        # [TAHMIN] efektif dolgu (kabuk + %15-20 dolgu); PLAN esas
DOLGU_TARAMA = (0.6, 0.75, 1.0)
SG90_G, MG90S_G, KOL_G = 9.0, 13.4, 1.0     # [KAYNAK] TowerPro
AYAK_G = 0.3             # [TAHMIN]
RAY_MIN_V, KAP_V = 4.6, 4.8   # [SPEC] PLAN R14: Uno 5 V hatti >= 4.6 V; stall degerleri 4.8 V icin -> kapasite 4.6 V'a olceklenir (tork ~ gerilim [TAHMIN])
KAP_SG90 = 1.8 * 0.35 * RAY_MIN_V / KAP_V    # kg*cm 0.604 [KAYNAK 1.8 @4.8 V] x %35 kurali (PLAN §5.0) x 4.6/4.8
KAP_MG90S = 1.8 * 0.35 * RAY_MIN_V / KAP_V   # [KAYNAK TowerPro 1.8 @4.8 V] (bazi saticilar 2.0-2.2; esas dusuk deger)
TEST_G, TEST_KOL_CM = 330.0, 2.0    # [SPEC] tutma testi (PLAN R11 315 g -> 330 g: 4.6 V esdegeri kapasiteyi kanitlasin)
KAP_BILGI_ORAN = 0.67    # %67 kurali (yalniz bilgi, PLAN §5.0)
IKI_SERVO_VERIM = 0.8    # [TAHMIN] PLAN §5.1 dipnot 5: iki servo ayar farkiyla birbirine karsi
SURTUNME_KGCM = 0.02     # [TAHMIN] PLAN §5.1 dipnot 2 (sol pim yok -> yalniz kablo surukleme)
HAREKET_DEG, HAREKET_S = 10.0, 0.25    # [SPEC] firmware pitch hiz siniri >= 0.25 s / 10 deg (PLAN §5.1 dipnot 1)
TORK_HEDEF = 1.5         # [SPEC]
DEVRILME_MESAFE = 15.0   # [SPEC] PLAN R5
DEVRILME_ACI = 15.0      # [SPEC] PLAN R5

# ------------------------------------------------------------------------------------------ 7. Kablo
SG90_KABLO = 250.0       # [KAYNAK] TowerPro SG90 ~25 cm (kumpas/metre ile dogrula)
UNO_UZAKLIK = 60.0       # [SECIM/TAHMIN] Uno kamanin hemen arkasinda: kertikten pin basligina ~60 mm (kullanici yerlestirir; MONTAJ T14)
KABLO_SERVIS = 30.0      # [SECIM] hareketli (cerceve) kablolar
KABLO_PAY_SABIT = 10.0   # [SECIM] sabit servolar (pitch): hareket yok, yalniz bukulme payi


def rad(d):
    return math.radians(d)
