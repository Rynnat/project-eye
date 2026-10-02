"""Project Eye v3 (POC) - tum olculer tek yerde (mm, derece).  REVIZYON 2 (kapaklar + maske + kafes cerceve)

Koordinatlar (v2 SPEC §1): orijin iki goz merkezinin ortasi, +X robotun sagi, +Y yukari, +Z ileri.
Acilar X ekseni etrafinda: phi = atan2(y, z)  (0 = ileri, +90 = yukari, -90 = asagi, 180 = geri).
Etiketler: [SPEC] v3 SPEC.md sabiti | [TOL] baski toleransi (cad/tolerance/ testiyle guncellenecek) |
           [SECIM] tasarim karari | [VARSAYIM] dogrulanmamis tedarik olcusu | [KAYNAK] datasheet/standart |
           [OLCUM]/[TEST] kullanicinin olcumu / baski testi
Degistir -> `python eye_v4.py && python check.py && python render.py`.

MIMARI (ayrinti DECISIONS.md):
  * Her goz bir KAFESTE (cerceve yarisi): goz merkezinin iki yaninda dikey plaka (dis + ic), ustte ve altta kopru.
    Iki kafes arka kiris + yaw_mount ile tek PITCH CERCEVESI olur; X ekseni (goz merkezleri) etrafinda doner.
    Sagda pitch servosu direkt (kol dis plakaya vidali), solda M4 pim (pivot_bracket).
  * Goz kafeste dikey M4 pimlerle (ust kopru + alt kopru) yaw yapar; yaw paralelkenari v3 ile ayni.
  * KAPAKLAR CERCEVEDE: ust + alt kapak goz merkezi etrafinda donen kure kabuk dilimleri (R 18.6-20.6, gozden
    0.6 mm). Mentese pimleri her gozun iki yaninda (4 kutup), M4x10 vida gozun tarafindan, kafasi ust kapak
    gobeginin havsa/cep yuvasinda. Kapaklar pitch'i takip eder (dogal goz kapagi davranisi).
  * LIDS servosu cercevede (sag gozun arkasi), yaw_crank ile AYNI krank parcasi; ucundaki tek M4 pime iki lama:
    ust lama (hemen hemen paralelkenar -> ust kapak) + alt lama (capraz -> alt kapak TERS yonde).
  * On MASKE (sabit): yalniz badem gozler gorunur.
"""
import math

# ----------------------------------------------------------------------------------------------
# 1. Baski toleranslari [TOL] (SPEC v3; tolerans testi sonucu gelince guncelle)
# ----------------------------------------------------------------------------------------------
M3_PILOT = 2.7          # M3 vidanin plastige kilavuzsuz vidalandigi delik
M3_CLEAR = 3.3          # M3 gecis (test 3.2-3.6)
M4_PILOT = 3.5          # M4 kilavuzsuz vidalanan delik (vida bu parcada SABIT)
M4_RUN = 4.3            # M4 vida-pim eklemi: donen parca bu delikte doner (test 4.2-4.6)
M4_CSK_D = 8.6          # M4 havsa bas icin koni agiz capi (90 deg). Test 8.0-9.2; ISO 10642 kafa ~8.5-9.0
M4_CBORE_D = 8.8        # M4 yuvarlak kafa cep capi (kafa O8.0 + pay); kapak mentesesinde kafa cepte doner
AXIAL_GAP = 0.4         # yan yana donen iki duz parca arasi eksenel bosluk
SERVO_POCKET_CLR = 0.4  # [OLCUM] servo govdesi cep boslugu, BOY yonu (toplam) -> cep 22.9
SERVO_POCKET_CLR_W = 0.2  # [TEST] EN yonu (toplam) -> cep 12.7. 12.9 bol cikti (kullanici 2026-09-29)
HORIZ_HOLE_EXTRA = 0.2  # [TOL] BASKIDA YATAY (ekseni tablaya paralel) deliklere eklenen pay: ust yuzey ~0.2 sarkar.
                        #       Tolerans testi yalniz dikey delikleri olcer -> bu delikler DECISIONS.md'de listeli.
BOTTOM_CHAMFER = 0.4    # [TOL] tablaya degen delik agzina 0.4x45 pah (elephant foot comp. 0)
HORN_SCREW_PILOT = 1.6  # [VARSAYIM] SG90 ile gelen kucuk sac vidalari (~M2) icin kilavuzsuz delik
MIN_WALL = 1.6          # [SPEC] min duvar
EDGE_R = 1.0            # [SECIM] gorunen dis kenarlarda yuvarlatma / pah (hedef >= 1 mm)

# ----------------------------------------------------------------------------------------------
# 2. Vidalar [VARSAYIM - kumpasla olc]
# ----------------------------------------------------------------------------------------------
M4_SHANK_MODEL = 3.4    # carpisma modelinde vida govdesi (kok cap ~3.24; pilot 3.5 ile cakismasin)
M4_HEAD_D, M4_HEAD_H = 8.0, 3.1    # yuvarlak kafa (ISO 7045 pan: 8.0x3.1; ISO 7380 daha basik) -> kotu durum
M4_CSK_HEAD_D = 8.0     # havsa kafa modeli (ISO 10642 teorik 8.96, gercek ~7.5-8.5); koni 90 deg
M3_SHANK_MODEL = 2.4
M3_HEAD_D, M3_HEAD_H = 5.6, 2.4

# ----------------------------------------------------------------------------------------------
# 3. Goz [SPEC]
# ----------------------------------------------------------------------------------------------
EYE_D = 36.0
EYE_R = EYE_D / 2
IPD = 95.0
EYE_X = IPD / 2
EYE_TOP_Y = 16.0        # [SECIM] ust duzlem (ust pim yuzu). Ust kopru buna 0.4 kalir -> goz arkadan kayarak takilir
EYE_FLAT_Y = 13.0       # [SECIM] alt duzlem (alt pim yuzu, baski tablasi): asin(13/18)=46 deg -> desteksiz
EYE_BACK_Z = -10.0      # [SECIM] arka duz kesim
IRIS_R, PUPIL_R = 8.0, 3.0

PIN_TOP_RUN_DEPTH = 8.0        # gozde ust RUN deligi (y 16 -> 8)
PIN_BOT_RUN_DEPTH = 7.0        # alt (y -13 -> -6)
LEVER_LOCK_Z = -10.2           # [SECIM] yaw kolu kilidi: M3x10, alttan, pimin ARKASINDA (onde alt kapak var)
LEVER_LOCK_PILOT_DEPTH = 8.0

# ----------------------------------------------------------------------------------------------
# 4. Kapaklar (kure kabuk dilimleri, X ekseni = goz merkezi ekseni; cercevede)
# ----------------------------------------------------------------------------------------------
LID_R_IN = EYE_R + 0.6          # 18.6  -> kapak-goz boslugu 0.6 (hedef <= 1.0)
LID_T = 2.0
LID_R_OUT = LID_R_IN + LID_T    # 20.6
LIP_R_IN = LID_R_OUT + AXIAL_GAP    # 21.0 ust kapak dudagi (alt kapagin ONUNDEN gecer -> kapaliyken bindirme)
LIP_R_OUT = LIP_R_IN + 2.0          # 23.0 (dudak ucu R1.0 tam yuvarlak)
LIP_DROP = 6.0                  # dudak ust kapak kenarindan 6 deg asagi iner
LIP_HALF_X = 17.5               # dudak yalniz badem acikliginin gordugu yerde (|x_rel| <= 17.5, duz kesik)
UP_EDGE_CLOSED = -4.0           # [SECIM] kapali: ust kapak kenari -4 deg, dudak -10 deg'e kadar
UP_BACK_CLOSED = 47.0           # ust kapak arka kenari (kapali); acikta 75 -> ust pim gobeginden 1.2 mm
UP_OPEN = 27.0                  # [SECIM] ust kapak acilma acisi -> acik kenar +23, dudak +17 (irisin ustunu ~1.5 mm orter; check §10)
LO_EDGE_CLOSED = -5.4           # alt kapak ust kenari (kapali): ust kenara 1.4 deg (~0.45 mm)
LO_BACK_CLOSED = -44.0          # alt kapak alt kenari (kapali); acikta -72 -> alt kopruden uzak
# alt kapak acilma acisi dort-cubuktan gelir (kin.LO_OPEN_ACT ~30 deg): acik kenar ~-35.5, alt kenar ~-74
# mentese yigini (kutup basina, x_rel = goz merkezinden disari): [goz 18] [ust gobek] [alt gobek] [plaka]
HUB_R = 6.0
UP_HUB_X = (19.1, 23.9)         # ust kapak gobegi; goz tarafinda M4x10 yuvarlak kafa cebi (O8.8 x 3.2)
LO_HUB_X = (UP_HUB_X[1] + AXIAL_GAP, UP_HUB_X[1] + AXIAL_GAP + 2.6)      # 24.3 .. 26.9
PLATE_X = (LO_HUB_X[1] + AXIAL_GAP, LO_HUB_X[1] + AXIAL_GAP + 5.6)      # 27.3 .. 32.9 kafes yan plakalari
LID_PIN_HEAD_X = UP_HUB_X[0] + 0.1                                      # kafa ust yuzu (goz tarafi) 19.2
UP_CBORE_FLOOR = LID_PIN_HEAD_X + M4_HEAD_H                            # 22.3 kafa alti = cep tabani
LO_EAR_RHO = (HUB_R + AXIAL_GAP, 10.5)   # alt kapak kulagi: ust gobegin disindan dolanir
LO_EAR_PHI = (-40.0, -18.0)              # (kapali) kulak dilimi; kutba kadar uzanan dudaktan (<= -7.4) aci payi
# ic kutupta kapaklari ortaya baglayan kollar + cubuklar (ortada bindirme, 2x M4x10 radyal)
UP_BAR_RHO = (6.9, 10.1)        # lama duzlemlerinde ince (lama 10.5'ten gecer); bindirmede 6.9..14
UP_BAR_RHO_FULL = (6.9, 14.0)   # lama duzlemleri disinda
UP_LAP = dict(rho=(8.5, 13.0, 18.0), phi=(140.0, 190.0), x=(3.4, 18.9), bolts=((7.7, 'M4'), (14.9, 'M3')))
UP_BAR_PHI = (144.0, 176.0)     # kapali; acikta +28
LO_BAR_RHO = (12.0, 18.0)
LO_LAP = dict(rho=(12.0, 15.0, 18.0), phi=(-44.0, -8.0), x=(-2.4, 13.8), bolts=((1.9, 'M4'), (10.3, 'M4')))
LO_BAR_PHI = (-38.0, -14.0)     # kapali; acikta -28
# ortadaki bindirme: sol parca ic kat x[a,b], sag parca dis kat x[a+0.4, b+0.4]; radyal civatalar (x, tip)

# ----------------------------------------------------------------------------------------------
# 5. Kafes (frame_L / frame_R). x_rel disari pozitif.
# ----------------------------------------------------------------------------------------------
STRIP_Z = (-5.0, 3.5)           # yan plakalarin dikey seridi (kapak cubuklari onunden/arkasindan gecer)
FRONT_Z = 3.5                   # kafesin on duzlemi: kafes on yuzu tablada basilir (her sey z yonunde prizmatik)
PLATE_DISK_R = 6.5
TB_Y = (EYE_TOP_Y + 5.2, EYE_TOP_Y + 9.7)      # ust kopru y 21.2..25.7 (kapak dis yaricapi 20.6 + 0.6)
TB_Z = (-5.0, FRONT_Z)
TOP_BOSS_R = 3.6                # ust pim gobegi (kopruden goz ust yuzune 0.4 kalana dek)
BB_Y = (-23.3, -17.3)           # alt kopru (pim altinda 6 mm: havsa + pilot)
BB_Z = (-5.5, FRONT_Z)          # alt kapak (acik alt kenar -74) on kosesinden ~5 deg uzak
RB_Y = (-29.0, -24.4)           # arka kiris: lamanin (ve alt vida kafalarinin) ALTINDA
RB_Z = (-74.0, FRONT_Z)
YM_LEG_Z = (-72.0, -55.0)       # yaw_mount bacaklari (LIDS servosunun ve lamanin arkasinda)
YM_SCREW_Z = (-68.0, -59.0)     # alttan (kiris) M4x16 havsa, bacakta pilot
YM_SCREW_X = 27.75

# ----------------------------------------------------------------------------------------------
# 6. Yaw paralelkenari (cercevede; hepsi goz merkezlerinin altinda, duz)
# ----------------------------------------------------------------------------------------------
LEVER_Y = (-(EYE_FLAT_Y + AXIAL_GAP) - 3.5, -(EYE_FLAT_Y + AXIAL_GAP))   # (-16.9, -13.4) goz kolu ve servo krank
COUPLER_Y = (LEVER_Y[0] - AXIAL_GAP - 3.5, LEVER_Y[0] - AXIAL_GAP)       # (-20.8, -17.3) lama
LEVER_R = 20.0                  # [SECIM] goz kolu = servo krank boyu -> paralelkenar, yaw = servo acisi
LEVER_W = 9.0
COUPLER_W = 9.0
YAW_SERVO_Z = -60.0             # yaw servo mili (0, *, -60); krank ucu z -80 (LIDS krankindan uzak)
COUPLER_PATH = ((-47.5, -20.0), (-30.0, -20.0), (-12.0, -50.0), (12.0, -50.0), (30.0, -20.0), (47.5, -20.0))  # (x, z): ortada geriye kivrik (LIDS alt lamasina yer)
HORN_T = 1.5                    # [VARSAYIM] SG90 kol kalinligi
HORN_R = 15.0                   # [VARSAYIM] cift kollu SG90 kolu, uc yaricapi
HORN_SCREW_R = 7.0              # [VARSAYIM] cift kolun GOBEGE EN YAKIN delikleri (~r 7)
HORN_HUB_R = 3.5
YM_PLATE_T = 4.0
YM_LEG_X = (24.0, 31.5)
YM_PLATE_Z = (-84.0, -48.6)     # arka ucta kablo yarigi acik kalir

# ----------------------------------------------------------------------------------------------
# 7. LIDS surucusu (cercevede): servo mili -X'e bakar, sag gozun arkasinda. Krank = yaw_crank ile ayni STL.
#    (y, z) duzleminde; kin.py dort-cubuk cozer.
# ----------------------------------------------------------------------------------------------
LS_S = (9.6, -29.3)            # servo mili (y, z) -- dort-cubuk taramasi (DECISIONS §1)
LS_SPLINE_X = 2.2               # mil ucu x; kol 0.7..2.2, krank -2.8..0.7, ust lama -6.7..-3.2, alt lama -11.6..-7.1
LS_WING_X = (16.1, 20.2)        # sag ic plakadaki servo kanadi (kulaklar x=16.1 yuzunde)
LS_WING_Y = (-14.5, 26.0)      # servo govdesi YUKARI (mil tarafi kulak y=1.2'de); alt uc -14.5..-12.5 destek ayagi
LS_WING_Z = (-37.4, -21.0)     # govde cebini saran kisim; ust bag y>=15.2 (cubuk supurme alaninin ustunden) seride

LS_A_PHI0 = -70.4               # krank ucunun kapali pozdaki yonu (phi)
LS_U = (14.5, 142.4)            # ust kapak lama pimi (rho, phi) kapali
LS_L = (16.5, -72.1)            # alt kapak lama pimi (rho, phi) kapali
LINK_T = 3.5
LINK_LO_T = 4.5                 # alt lama: A pimindeki M4x16 havsa bu lamada
LINK_W = 8.0

# ----------------------------------------------------------------------------------------------
# 8. SG90 [KAYNAK/VARSAYIM] (boy/en kullanici olcumu)
# ----------------------------------------------------------------------------------------------
SG_L, SG_W, SG_H = 22.5, 12.5, 22.7   # [OLCUM] kullanicinin servosu, yaklasik; datasheet 22.8x12.4 idi
SG_EAR_SPAN = 32.0
SG_EAR_Z0 = 15.6
SG_EAR_T = 2.5
SG_TOP = 26.7
SG_SHAFT_OFF = 5.8
SG_BOSS_R = 5.9
SG_SPLINE_TOP = 29.5
SG_HOLE_SPACING = 27.8           # [VARSAYIM] kulak delik araligi
CABLE_SLOT_W = 6.0             # SG90 kablosu govdenin mile UZAK ucundan, alta yakin cikar: cebin o uc duvarinda
CABLE_SLOT_OUT = 8.0           # alttan uste acik yarik (servo kablosuyla birlikte yukaridan kayarak girer).
                               # Kullanici tolerans testinde yakaladi: kapali cep kabloyu engelliyordu.
SG_EAR_HOLE_DRILL = 3.2         # SG90 kulak delikleri (~O2) M3 icin 3.2 matkapla buyutulur

# ----------------------------------------------------------------------------------------------
# 9. Sabit govde (base + pivot_bracket + pitch servo tutucu) ve maske
# ----------------------------------------------------------------------------------------------
BASE_Y = (-60.0, -54.0)         # 6 mm: alttan M4x16 havsa (koni 4.3 + 1.7)
BASE_X = (-98.5, 113.5)
BASE_Z = (-62.0, 27.0)
BRACKET_T = 4.6
BRK_FOOT_T = 6.0
PITCH_PIN_BOSS_X = (PLATE_X[1], PLATE_X[1] + 5.4)     # sol dis plakada pitch pimi gobegi (x_rel 32.9..38.3)
MOUNT_T = 4.0
PITCH_SPLINE_X = EYE_X + PLATE_X[1] + HORN_T          # 81.9 (mil -X'e bakar)
MASK_Z = (23.4, 26.4)           # maske plakasi: kapak dudagi R 23.0 -> maske arka yuzu 23.4 (her pozda >= 0.4)
MASK_Y = (BASE_Y[1], 38.0)      # pitch -20'de LIDS servosunun ustu y~37
MASK_OPEN = (16.5, 11.0)        # badem aciklik yari eksenleri (x, y) maske ARKA yuzunde
MASK_OPEN_FRONT = (18.5, 13.0)  # on yuzde (pah/egim: goz cukuru hissi)
MASK_R = 6.0                    # maske dis kose yaricapi
MASK_X = BASE_X
MASK_FOOT_Z = (-4.0, MASK_Z[0] + 0.01)  # maske ayagi (tabana yatar, 2x M4x16 alttan)
MASK_FOOT_T = 4.0
MASK_SCREW_X = (-60.0, 60.0)

# ----------------------------------------------------------------------------------------------
# 10. Hareket araliklari ve kontrol
# ----------------------------------------------------------------------------------------------
YAW_RANGE = 25.0
PITCH_RANGE = 20.0
CHECK_YAWS = (-25.0, 0.0, 25.0)
CHECK_PITCHES = (-20.0, 0.0, 20.0)
INTERFERENCE_TOL_MM3 = 0.01
TARGET_CLEARANCE = 0.4
LID_EYE_GAP_MAX = 1.0
