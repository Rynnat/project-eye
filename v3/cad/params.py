"""Project Eye v3 (POC) - tum olculer tek yerde (mm, derece).

Koordinatlar (v2 SPEC §1): orijin iki goz merkezinin ortasi, +X robotun sagi, +Y yukari, +Z ileri.
Etiketler: [SPEC] v3 SPEC.md sabiti | [TOL] baski toleransi (cad/tolerance/ testiyle guncellenecek) |
           [SECIM] tasarim karari | [VARSAYIM] dogrulanmamis tedarik olcusu | [KAYNAK] datasheet/standart
Degistir -> `python eye_v3.py && python check.py && python render.py`.

MIMARI (ayrinti DECISIONS.md):
  * Iki goz ortak bir PITCH CERCEVESINDE (frame_L + frame_R + yaw_mount) durur; cerceve goz merkezlerinden gecen
    X ekseni etrafinda doner. Sagda pitch servosu ciftlesik (direkt), solda M4 pim.
  * Her goz cercevede DIKEY M4 pimlerle (ust+alt) yaw yapar. Yaw servosu CERCEVENIN USTUNDE tasinir ->
    yaw paralelkenari (goz kollari + servo krank + duz lama) her pozda duzlemsel. 3B mafsal yok.
  * Iki ust kapak ayni X ekseni etrafinda doner; ortadaki kapak servosu tasiyiciyi (lid_carrier) direkt dondurur.
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
AXIAL_GAP = 0.4         # yan yana donen iki duz parca arasi eksenel bosluk
SERVO_POCKET_CLR = 0.3  # servo govdesi cep boslugu (toplam)
HORIZ_HOLE_EXTRA = 0.2  # [TOL] BASKIDA YATAY (ekseni tablaya paralel) deliklere eklenen pay: ust yuzey ~0.2 sarkar.
                        #       Tolerans testi yalniz dikey delikleri olcer -> bu delikler DECISIONS.md'de listeli.
BOTTOM_CHAMFER = 0.4    # [TOL] tablaya degen delik agzina 0.4x45 pah (elephant foot comp. 0)
HORN_SCREW_PILOT = 1.6  # [VARSAYIM] SG90 ile gelen kucuk sac vidalari (~M2) icin kilavuzsuz delik
MIN_WALL = 1.6          # [SPEC] min duvar

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
EYE_FLAT_Y = 13.0       # [SECIM] ust/alt duzlem (pim yuzleri). Alt duzlem baski tablasi: kure y=-13'te
                        #  asin(13/18)=46 deg -> sarkma yalniz 0.27 mm'lik bantta (destek gerekmez)
EYE_BACK_Z = -10.0      # [SECIM] arka duz kesim
IRIS_R, PUPIL_R = 8.0, 3.0

# Goz yaw pimleri (goz-yerel): ust ve alt dikey M4. Vida CERCEVEDE sabit (pilot), goz M4_RUN'da doner.
PIN_TOP_RUN_DEPTH = 9.0        # gozde ust RUN deligi derinligi (y 13 -> 4)
PIN_BOT_RUN_DEPTH = 7.0        # alt (y -13 -> -6)
LEVER_LOCK_Z = 8.5             # yaw kolu kilit vidasi (havsa, alttan) - goz-yerel z. Onde: pim deligiyle arasinda 2 mm et
LEVER_LOCK_PILOT_DEPTH = 12.5

# ----------------------------------------------------------------------------------------------
# 4. Cerceve (frame_L / frame_R): yan plaka + ust kol + alt kol + arka kiris yarisi. x_rel = goz merkezinden
#    DISARI dogru pozitif (sol gozde -X).
# ----------------------------------------------------------------------------------------------
FT_IN = EYE_R + AXIAL_GAP              # 18.4 ust kol ic kuresi
FT_OUT = 20.8                          # ust kol dis kuresi (et 2.4); kapak ic kuresi bundan 0.4 buyuk
FT_FLAT_Y = EYE_FLAT_Y + AXIAL_GAP     # 13.4 ust kolun pim altindaki duz yuzu
FT_HALF_Z = 7.0                        # ust kol yarim eni (z)
FT_XREL = (-6.0, 15.4)                 # kure kabuk bolumu (x_rel); 15.4..18.4 arasi blok (kapak disinda)
FT_BLOCK_R = 22.0
TOP_CSK_TOP_Y = 20.2                   # ust pim havsa kafasi ust yuzu (kure tepesi 20.8'in 0.6 alti)
SIDE_X = (EYE_R + AXIAL_GAP, EYE_R + AXIAL_GAP + 5.0)   # yan plaka x_rel 18.4..23.4 (pilot 5 mm)
SIDE_R = 24.0                          # yan plaka dilimi yaricapi
SIDE_PHI = (40.0, 295.0)               # yan plaka dilimi (phi = atan2(y, z)); 40: ust kol blogu plakanin ustunde basilsin
FB_Y = (-23.3, -17.3)                  # alt kol (pim altinda 6 mm: havsa 2.1 + 3.9 dis)
FB_HALF_Z = 6.0
FB_XREL = (-6.0, SIDE_X[0])
XM_Y = (-11.0, -3.0)                   # arka kiris (iki yari x=0'da bulusur, yaw_mount ile baglanir)
XM_Z = (-35.0, -23.0)
XM_PAD_X = (9.5, 19.5)                 # |x| >= 9.5'ten yan plakaya kadar kiris genis (z -44..-23); ortada servo kubbesi icin dar
XM_PAD_Z = (-44.0, XM_Z[1])
YM_SCREW_Z = (-27.0, -40.0)            # yaw_mount -> kiris vidalari (x = +-YM_SCREW_X)
YM_SCREW_X = 14.5

# ----------------------------------------------------------------------------------------------
# 5. Yaw paralelkenari (cercevede; hepsi goz merkezlerinin y-altinda, duz)
# ----------------------------------------------------------------------------------------------
LEVER_Y = (-(EYE_FLAT_Y + AXIAL_GAP) - 3.5, -(EYE_FLAT_Y + AXIAL_GAP))   # (-16.9, -13.4) goz kolu ve servo krank
COUPLER_Y = (LEVER_Y[0] - AXIAL_GAP - 3.5, LEVER_Y[0] - AXIAL_GAP)       # (-20.8, -17.3) lama
LEVER_R = 20.0                  # [SECIM] goz kolu = servo krank boyu -> paralelkenar, yaw = servo acisi
LEVER_W = 9.0
COUPLER_W = 9.0
YAW_SERVO_Z = -42.0             # yaw servo mili (0, *, -42); krank ucu z -62 (kubbe kirisin 1.1 arkasinda)
HORN_T = 1.5                    # [VARSAYIM] SG90 kol kalinligi
HORN_R = 15.0                   # [VARSAYIM] cift kollu SG90 kolu, uc yaricapi
HORN_SCREW_R = 7.0              # [VARSAYIM] cift kolun GOBEGE EN YAKIN delikleri (~r 7). Farkliysa bu degeri olcup guncelleyin.
HORN_HUB_R = 3.5
YM_PLATE_T = 4.0                # yaw_mount plaka (kulaklarin ustunde)
YM_LEG_X = XM_PAD_X             # bacaklar |x| (kol vidasi kafalarina pay)
YM_PLATE_Z = (-67.0, -28.0)     # plaka z araligi (on kenar: kapak tasiyicisina pay)

# ----------------------------------------------------------------------------------------------
# 6. Kapaklar (iki ust kapak + tasiyici, hepsi X ekseni etrafinda birlikte)
# ----------------------------------------------------------------------------------------------
LID_R_IN = FT_OUT + 0.5         # 21.3 (ucgenleme payi dahil >= 0.4 bosluk)
LID_T = 2.0
LID_R_OUT = LID_R_IN + LID_T    # 23.2
LID_HALF_X = 15.0               # kapak bandi x_rel +-15 (kenarlari duz kesik)
LID_EDGE_CLOSED = -5.0          # [SECIM] kapali: alt kenar goz merkezinin 5 deg altinda
LID_BACK = 110.0                # kapak bandinin arka kenari (kapali)
LID_OPEN = 45.0                 # [SECIM] tam acik: kenar +40 deg
LID_SERVO_MID = LID_OPEN / 2    # servo 90 = yari acik (direkt surus, olu nokta yok; aralik ortalandi)
LID_BOSS_XREL = (-15.0, -6.5)   # kapak ic ucu takviyesi (tasiyiciya 2xM4)
LID_BOSS_PHI = (66.0, 114.0)
LID_BOSS_R = 27.0
LID_SCREW_YZ = ((22.6, 4.5), (22.6, -4.5))   # kapak-tasiyici vidalari (y, z), X yonunde
CAR_TAB_T = 3.5                 # tasiyici kulagi (x_rel -18.5..-15)
CAR_TAB_Y = (16.0, 31.0)
CAR_TAB_HALF_Z = 8.5
CAR_BAR_Y = (27.0, 31.0)        # ust kopru (x -32.5..32.5)
CAR_BAR_HALF_Z = 4.0
CAR_HUB_X = (2.5, 6.5)          # tasiyici gobek plakasi (servo koluna vidali)
CAR_HUB_R = 9.0
CAR_HUB_HALF_Z = 5.0
LID_SPLINE_X = 1.0              # kapak servosu mil ucu (mil +X)

# ----------------------------------------------------------------------------------------------
# 7. SG90 [KAYNAK/VARSAYIM] (v2 ile ayni)
# ----------------------------------------------------------------------------------------------
SG_L, SG_W, SG_H = 22.8, 12.4, 22.7
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
# 8. Sabit govde (base: plaka + sol yatak + orta direk (kapak servosu) + sag pitch servo tutucu)
# ----------------------------------------------------------------------------------------------
BASE_Y = (-54.0, -50.0)
BASE_X = (-87.0, 94.0)
BASE_Z = (-30.0, 22.0)
BRACKET_T = 4.6                 # sol yatak (M4 RUN, vida kafasi disarida)
BRK_FOOT_X = (-85.0, -63.0)     # sol yatak AYRI parca (pivot_bracket): cerceve takildiktan sonra vidalanir
BRK_FOOT_T = 6.0
BRK_SCREW_X = (-81.0, -67.0)    # tabandan (alttan havsa) ayaga M4x16
MOUNT_T = 4.0                   # servo tutucu plakalari
PITCH_SPLINE_X = EYE_X + SIDE_X[1] + HORN_T     # 72.4 (mil -X'e bakar)

# ----------------------------------------------------------------------------------------------
# 9. Hareket araliklari ve kontrol
# ----------------------------------------------------------------------------------------------
YAW_RANGE = 25.0
PITCH_RANGE = 20.0
CHECK_YAWS = (-25.0, 0.0, 25.0)
CHECK_PITCHES = (-20.0, 0.0, 20.0)
INTERFERENCE_TOL_MM3 = 0.01
TARGET_CLEARANCE = 0.4
