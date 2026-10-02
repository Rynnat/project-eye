"""Project Eye v2 - tum olculer tek yerde (mm, derece).

Koordinatlar SPEC §1: orijin iki goz merkezinin ortasi, +X robotun sagi, +Y yukari, +Z ileri.
Etiketler:
  [SPEC]   SPEC.md'de SABIT olan deger
  [KAYNAK] datasheet / standart (ISO, DIN) - kaynak belirtildi
  [SECIM]  tasarim secimi (gerekcesi yaninda)
  [TOL]    baski toleransi (FDM, 0.4 nozul, PLA/PETG icin tipik; kendi yaziciniza gore ayarlayin)
  [VARSAYIM] dogrulanmamis tedarik olcusu - parcayi elinize alinca olcup buradan guncelleyin
Tum parcalar bu dosyadan turetilir; bir degeri degistirip `python eye_v2.py` + `python check.py` calistirin.
"""
import math

# ----------------------------------------------------------------------------------------------
# 1. Baski toleranslari
# ----------------------------------------------------------------------------------------------
M2_PILOT = 1.7      # [TOL] M2 vidanin plastige kilavuzsuz vidalanacagi delik. M2 dis cap 2.0, kok cap ~1.57;
                    #       1.7 FDM'de ~1.6'ya kapanir -> vida kendi disini acar, catlatmaz.
M2_CLEAR = 2.2      # [TOL] M2 gecis / yatak deligi. 2.0 + 0.2: FDM delikleri ~0.1-0.15 kucuk cikar,
                    #       vida serbest doner ama sallanmaz (pim/yatak olarak kullanilan yerlerde de bu).
PIN_PRESS = 1.95    # [TOL] O2 celik pimin (DIN 7 / ISO 2338 m6) sikica gececegi delik.
PIN_RUN = 2.25      # [TOL] O2 pim uzerinde donen parca (kapak gobegi) deligi: 0.25 cap boslugu.
AXIAL_GAP = 0.4     # [TOL] yan yana donen iki duz parca arasi eksenel bosluk (surtunmesiz, tek katman ~0.2x2)
MIN_WALL = 1.2      # [SPEC §8] minimum duvar
CBORE_D = 4.2       # [TOL] M2 silindir bas vida kafasi havsasi (kafa 3.8 + 0.4)
CSK_CLR = 0.15      # [TOL] havsa bas (bebek) vida konisi etrafinda radyal bosluk
GLUE_GAP = 0.05     # [TOL] yapistirilan yuzeyler arasi (goz catali sapi - kabuk ici): yapistirici payi
YAW_COUPLER_HOLE = 2.3  # [TOL] paralelkenar cubugu delikleri: 3 paralel eksen -> hafif bol (sikismasin)
HORN_SCREW_PILOT = 1.6  # [TOL] SG90 kol vidalari (servo ile gelen ~M2 sac vidasi) icin rocker flans deligi
SOCKET_CAV_CLR = 0.1    # [TOL] basili rotil yuvasi: O4.8 top icin kovuk O4.9 (PETG, hafif oynak)
SERVO_POCKET_CLR = 0.3  # [TOL] servo govdesi icin cep boslugu (her yandan 0.15)

# ----------------------------------------------------------------------------------------------
# 2. Goz (SPEC §2)
# ----------------------------------------------------------------------------------------------
IPD = 63.0                          # [SPEC]
EYE_X = IPD / 2                     # [SPEC] sol goz (-31.5,0,0), sag goz (+31.5,0,0)
EYE_R = 12.0                        # [SPEC] O24
EYE_WALL = 1.6                      # [SECIM] v1'den; >=1.2 kurali, kurenin baskida duzgun cikmasi icin 4 perimetre
EYE_R_IN = EYE_R - EYE_WALL         # 10.4
IRIS_D = 12.0                       # [SPEC]
PUPIL_D = 4.0                       # [SPEC] bebek; bu tasarimda siyah M2 havsa bas vida kafasi (O3.8) - DECISIONS
# Arka aciklik: sabit direk tum hareket araliginda kabuga degmemeli.
EYE_OPEN_HALF = 52.0                # [SECIM] arka aciklik yari acisi (-Z ekseninden, ic yuzeyde).
                                    # kose pozunda (yaw 30 + pitch 25) direk yonu 38.3 deg sapar; O4 direk +11.1 deg
                                    # -> 49.4 < 52 (2.6 deg ~ 0.47 mm pay)
EYE_RIM_Z = -EYE_R_IN * math.cos(math.radians(EYE_OPEN_HALF))   # -6.40: kabuk bu duzlemde biter (baski tablasi)
ACCESS_HOLE_D = 4.4                 # [SECIM] kardan vidalarina tornavida erisim delikleri (kafa O3.8 + 0.6)

# Kardan mafsali (SPEC §2, v1)
HUB = 8.0                           # [SPEC] gobek 8x8x8
FORK_T = 2.0                        # [SPEC] catal kolu kalinligi
FORK_GAP = 0.5                      # [SPEC] gobek-kol boslugu
FORK_C = HUB / 2 + FORK_GAP + FORK_T / 2      # 5.5 kol orta duzlemi
FORK_OUT = HUB / 2 + FORK_GAP + FORK_T        # 6.5 kol dis yuzu
SCREW_M2_HEAD_D = 3.8               # [KAYNAK] ISO 4762 M2 kafa O3.8 x 2.0
SCREW_M2_HEAD_H = 2.0
SCREW_MINOR_D = 1.6                 # [KAYNAK] M2 kok capi ~1.567: vidalar carpisma modelinde bu capla (dis plastige gomulur)
HUB_HOLE_DEPTH = 3.0                # [SECIM] M2x5: 2 kol + 0.5 bosluk + 2.5 gobek; delik 3 derin (0.5 pay)
# Goz catali (gozle doner): kollar x=+-5.5 (yatay vidalar), onde kopru, kopruden kabugun on icine sap.
EYE_FORK_ARM_Y = 3.0                # [SECIM] kol yarim yuksekligi (vida etrafinda 3-1.1 = 1.9 et)
EYE_FORK_ARM_Z = (-2.1, 5.7)        # [SECIM] kol z araligi; z>2.5'te kol yuksekligi +-2'ye iner (pitch 25'te
EYE_FORK_ARM_Y2 = 2.0               #          sabit catal koluna (y>=4.5) girmesin)
EYE_FORK_BRIDGE_Z = (5.7, 7.9)      # [SECIM] v1'de 4.5: pitch 25'te gobek kosesi (4,4) z=5.32'ye supurur -> kopru 5.7'de
EYE_FORK_BRIDGE_Y = 2.0
EYE_FORK_BRIDGE_Y_FRONT = 1.3       # kopru on yuzde +-1.3'e daralir (pah): pitch 25'te kose y=4.43 < 4.5
EYE_FORK_STEM_D = 4.4               # [SECIM] icinde M2 pilot (1.7) -> et (4.4-1.7)/2 = 1.35 >= 1.2
# Sabit catal (hareketsiz): kollar y=+-5.5 (dikey vidalar), arkada kopru, kopruden direk.
FIX_FORK_BRIDGE_Z = (-8.0, -6.0)    # [SECIM] kopru; kose pozunda goz catali kol ucu z=-5.3'e iner. Koseler
                                    #  FIX_FORK_TRIM_R kuresiyle kirpilir
POST_D = 4.0                        # [SECIM] yuvarlak direk (aciklik hesabi yukarida)
FIX_FORK_TRIM_R = 10.1              # sabit catal basi bu kureyle kirpilir (kabuk ici 10.4 -> 0.3 bosluk)
POST_FLANGE = (13.0, 10.0, 5.0)     # [SECIM] arka kirise oturan flans (x, y, z); 2xM2x8 arkadan
FIX_ARM_CHAMFER = (0.8, 1.0)        # sabit catal kolu ic-on kenar basamagi (y, z): pitch+yaw'da goz catali
                                    #  koprusu kosesi buraya supuruyor

# Alt pad cercevesi + pitch kancasi: kabukla TEK PARCA, kenar duzleminde (baski tablasi) ve oradan dik yukselir.
PAD_T = 2.2                         # [SECIM] kapak bolgesine girmemesi icin ince (DECISIONS: kapak-pad acisi)
# Kapak bandini (r 12.6-13.8) gecen kisim DAR ve x=0 civarinda olmali: yaw+pitch birlesince x!=0 noktalar one doner.
# r>=14.5'teki kisimlar hic kapak kabuguna giremez (donus r'yi korur).
# SOL goz icin (+X = ic taraf). Sag goz kabugu bunun aynasidir: cerceve her gozde IC tarafta; dis tarafta kapak kollari
# ve baglantilari var, uc pozlarda (yaw 30 + pitch -25) alttaki cerceve u~19'a kadar savrulur.
BOTTOM_PAD_A0 = ((-2.5, 2.5), (-15.2, -7.0))    # kenardan asagi dar serit (bandi burada, |x|<=2.5'te gecer)
BOTTOM_PAD_C = ((-2.5, 12.0), (-15.2, -13.5))   # pencerenin ust kenari (r>=14.1)
BOTTOM_PAD_A1 = ((9.0, 12.0), (-31.7, -13.5))   # pencerenin ic yan kenari
BOTTOM_PAD_B = ((-4.5, 12.0), (-31.7, -26.1))   # pencerenin alt kenari, kanca blogu bunun ustunde
# pencere (x -4.5..9, y -26.1..-15.2): pitch cubugu buradan gecer; kose pozlarinda cubuk goz cercevesinde x~7.7'ye kayar

# ----------------------------------------------------------------------------------------------
# 3. Kapaklar (SPEC §3)
# ----------------------------------------------------------------------------------------------
LID_R_IN = EYE_R + 0.6              # [SPEC] 12.6
LID_T = 1.2                         # [SPEC]
LID_R_OUT = LID_R_IN + LID_T        # 13.8
LID_MEET = -5.0                     # [SPEC/SECIM] kapaklar orta cizginin 5 deg altinda bulusur (parametrik)
LID_CLOSE_GAP = 1.5                 # [SECIM] kapali iken iki kenar arasi 1.5 deg (~0.35 mm) - temas/surtunme yok
LID_UP_W = 38.0                     # [SECIM] ust kapak dilim genisligi (deg); tam acikta arka kenar 76 deg,
                                    #          yaw tupu (phi=90, +-10 deg) ile 4 deg pay
LID_LO_W = 30.0                     # [SECIM] alt kapak dilim genisligi; tam acikta arka kenar -75 deg
                                    #          (kapaliyken -35.75'e kadar orter; maske penceresi alti -30)
LID_UP_EDGE_OPEN_MAX = 38.0         # [SECIM] mekanik tam acik: ust kenar +38 deg (normal 22 + takip 0.6*25=15 -> 37)
LID_LO_EDGE_OPEN_MAX = -45.0        # [SECIM] alt kenar -45 deg
LID_UP_EDGE_NOMINAL = 22.0          # [SPEC §3] irisin ustunu ~1.5 mm orter: 12*sin(22)=4.5 = 6-1.5
LID_LO_EDGE_NOMINAL = -28.0         # [SECIM] alt kapak iris altina yakin
LID_TRUNC = 11.0                    # [SECIM] dilim kutuplarda r=13.2'de |x|=11'e karsilik gelen KONI ile kesilir
                                    #          (duz kesim ince bicak kenari birakiyordu)
LID_EAR_RHO = 8.8                   # kulak koprusunun dis yaricapi (eksenden)
LID_EAR_RHO_IN = 4.2                # kulak koprusu ic yaricapi (diger kapagin gobegini atlar)
LID_EAR_HALF = 11.0                 # kulak dilimi yarim acisi (deg)
LID_HUB_R = 3.6                     # gobek yaricapi (pim O2 icin et 2.5)
# Eksenel yigin (goz merkezinden |x| mesafesi, her iki yanda):
LID_PLATE_T = 2.0                   # kapak gobek/kol plakasi kalinligi
LINK_T = 2.0                        # kapak baglanti kolu kalinligi
LID_SCREW_TIP_ROOM = 0.6            # alt kol vidasinin (M2x5) ucu ile ust kapak plakasi arasi
LID_LO_HUB_U = (13.0, 13.0 + LID_PLATE_T)                                    # alt kapak gobek/kol plakasi
LID_LO_LINK_U = (LID_LO_HUB_U[1] + AXIAL_GAP, LID_LO_HUB_U[1] + AXIAL_GAP + LINK_T)   # alt baglanti (dis yan)
LID_UP_HUB_U = (LID_LO_LINK_U[1] + AXIAL_GAP + LID_SCREW_TIP_ROOM,
                LID_LO_LINK_U[1] + AXIAL_GAP + LID_SCREW_TIP_ROOM + LID_PLATE_T)     # ust kapak gobek/kol plakasi
LID_UP_LINK_U = (LID_UP_HUB_U[1] + AXIAL_GAP, LID_UP_HUB_U[1] + AXIAL_GAP + LINK_T)   # ust baglanti (dis yan)
LID_FIN_U = (LID_UP_HUB_U[1] + AXIAL_GAP, LID_UP_HUB_U[1] + AXIAL_GAP + 4.0)           # sabit pim kanadi (iki yanda)
# (AXIAL_GAP=0.4 ile: 13-15 | 15.4-17.4 | 18.4-20.4 | 20.8-22.8 ; kanat 20.8-24.8)
LID_PIN_L = 12.0                    # [KAYNAK] O2x12 DIN 7 pim; u=24.8 -> 12.8
LID_ARM_R = 12.0                    # [SECIM] kapak kolu boyu (servo kolu 10 -> ~1.2:1, servo +-27 deg)
LID_ARM_W = 5.0
LINK_W = 5.0

# ----------------------------------------------------------------------------------------------
# 4. SG90 (SPEC §4, Luxorparts datasheet; v1 model.html)
# ----------------------------------------------------------------------------------------------
SG_L, SG_W, SG_H = 22.8, 12.4, 22.7 # [KAYNAK] govde boy, en, yukseklik (kulak altina degil govde ustune)
SG_EAR_SPAN = 32.0                  # [KAYNAK]
SG_EAR_Z0 = 15.6                    # [KAYNAK] kulak alti (tabandan)
SG_EAR_T = 2.5                      # [VARSAYIM] kulak kalinligi (v1)
SG_TOP = 26.7                       # [KAYNAK] govde ustu (mil etrafindaki kubbe)
SG_SHAFT_OFF = 5.8                  # [KAYNAK/cizim] mil, govde ucundan
SG_BOSS_R = 5.9                     # [VARSAYIM] mil etrafindaki kubbe yaricapi (v1)
SG_SPLINE_TOP = 29.5                # [VARSAYIM] mil ucu (v1)
SG_HOLE_SPACING = 27.8              # [VARSAYIM] kulak delik aralik (yaygin cizimler 27.5-28); montajda olcun
HORN_T = 1.5                        # [VARSAYIM] kol kalinligi
HORN_HUB_R = 3.5
HORN_R_LID = 10.0                   # [SPEC] delik araligi 2 mm; r=10 v1'de kullanildi

# ----------------------------------------------------------------------------------------------
# 5. Rotil (top mafsal) - goz baglantilari
# ----------------------------------------------------------------------------------------------
# [VARSAYIM] Satin alinan: RC rotil TOPU (saplama) - top O4.8, M2 x 4 dis, yakadan top merkezine 4.9 mm, boyun O2.4.
#            Yuva (soket) satin alinmaz; basili pitch cubugunun ucunda. Tedarik edilen topa gore guncelleyin.
BALL_D = 4.8
BALL_H = 4.9          # montaj yuzeyinden top merkezine
STUD_COLLAR_D, STUD_COLLAR_H = 4.0, 1.2
STUD_NECK_D = 2.4
# Pitch cubugu BASILI (PETG) ve iki ucunda gecmeli kuresel yuva var (RC rotil yerine; eklem acisi bizim kontrolumuzde):
SOCKET_R = 4.0                      # [SECIM] yuva dis kure yaricapi -> et 4 - 2.45 = 1.55
SOCKET_MOUTH_HALF = 61.0            # [SECIM] agiz konisi yari acisi: agiz capi 2*2.45*sin61 = 4.29 < 4.8 -> top
                                    #          yuvaya bastirilarak takilir (0.25 mm/yan gecme) ve cikmaz.
                                    #          O2.4 boyunla izinli egim ~ 61 - asin(1.2/2.4) = 31 deg (gereken <= 25)
PITCH_LINK_T = (4.0, 3.0)           # cubuk kesiti (saplama-A yonunde, dik yonde)

# ----------------------------------------------------------------------------------------------
# 6. Goz baglantilari - goz-yerel koordinatlar (goz merkezi orijin, notr poz)
# ----------------------------------------------------------------------------------------------
# YAW: kardan gobegi SADECE yaw yapar. Gobege ustten bir tup+kol (yaw kolu) M2x16 ile sikilir; tup sabit catalin
# ust kolundaki yataktan ve goz kabugunun ust yarigindan gecer. Iki gozun kolu + ortadaki servo kolu (hepsi r=10,
# eksenleri z=0 dogrusunda) tek bir cubukla (paralelkenar) baglanir -> yaw = servo acisi, birebir, pitch'ten bagimsiz.
YAW_LEVER_W = 6.6                   # kol eni: havsa O4.2 -> et 1.2
YAW_LEVER_R = 10.0                  # [SECIM] = SG90 kolunda r=10 deligi (v1'de kullanilan)
YAW_LEVER_Y = (15.0, 18.0)          # kol plakasi (goz merkezine gore y); ust kapak r<=13.8 -> 1.2+ bosluk
YAW_TUBE_D = 4.6                    # tup dis cap; ic 2.2 -> et 1.2
YAW_TUBE_BEARING_CLR = 0.25         # [TOL] yaw tupu yatak cap boslugu (sabit catal ust kolu)
YAW_TUBE_BEARING = YAW_TUBE_D + YAW_TUBE_BEARING_CLR
YAW_SLOT_CLR = 0.6                  # [TOL] kabuk yarigi, tupun her yanina
YAW_CLAMP_SEAT = 0.8                # kol ustunde vida kafasi havsasi derinligi (M2x16 -> gobege 2.8 mm)
YAW_COUPLER_Y = (YAW_LEVER_Y[1] + AXIAL_GAP, YAW_LEVER_Y[1] + AXIAL_GAP + 2.0)   # cubuk kollarin ustunde
YAW_COUPLER_W = 6.0
YAW_SERVO_X = 0.0                   # servo mili iki gozun tam ortasinda (0, y, 0)
FIX_ARM_HALF_X = 4.2                # sabit catal kol yarim genisligi (yatak deligi O4.85 -> et 1.8);
                                    # kafa kesiti 8.4x13 (yari kosegen 7.74) -> kabuk arka acikligindan (O16.4) gecer
# PITCH: top gozun ALT KUTBUNDA (0,-20.7,0) (yaw ekseni uzerinde -> pitch=0'da yaw pitch'i hic etkilemez).
# Kanca kabukla tek parca basilir (pad'den dik yukselir).
PITCH_BALL = (0.0, -20.7, 0.0)      # yuva (R4) en ic noktasi r=16.7 > kapak dis yaricapi 13.8
PITCH_HOOK_X = (-4.0, 4.5)          # kanca blogu topun ALTINDA; saplama +Y (goz Y ekseni = yaw ekseni:
PITCH_HOOK_Y = (-31.2, PITCH_BALL[1] - BALL_H)   #  yaw saplamayi egmez, sadece pitch +-25 eger)
PITCH_HOOK_Z = (EYE_RIM_Z, 2.2)     # z<=2.2 (pilot ustu et 1.35); pitch +25'te blok z~15.2 -> maske 16.5'te
PITCH_ROD_L = 20.0                  # [SECIM] cubuk boyu (top merkezleri arasi)
PITCH_ROD_INCL = -12.0              # [SECIM] notr cubuk egimi: arkaya dogru 12 deg YUKSELIR. Sayisal tarama
                                    #  (yaw x pitch izgarasi) goz ucu rotil egimini 37 -> 20.8 deg.e indiriyor (izin 31)
PITCH_ARM_R = 16.0                  # [SECIM] rocker kolu: goz +-25 -> servo -31..+32
HOOK_CHAMFER_Z = -2.0               # kanca blogunun arka-ust kosesi 45 deg pahli (goz asagi bakinca cubuk gecsin)

# ----------------------------------------------------------------------------------------------
# 7. Cerceve / servo yerlesimi (dunya koordinati)
# ----------------------------------------------------------------------------------------------
BASE_Y = (-47.0, -43.0)             # taban plakasi (4 mm: havsa 2.2 + 1.8 et)
BASE_X = (-79.0, 79.0)
BASE_Z = (-74.0, 19.0)
BASE_CBORE_DEPTH = 2.2              # tabandaki ve kiristeki vida kafasi havsasi derinligi
SCREW_HEAD_GAP = 0.2                # donen vidanin kafasi ile yatak yuzu arasi (rocker yatak vidasi)
FOOT_T = 6.0                        # tutucu ayaklari: M2x6 -> 4.2 mm gecer, pilot 4.4, ustte 1.6 et
FOOT_PILOT_DEPTH = 4.4
MOUNT_T = 3.0                       # servo tutucu plaka kalinligi (kulak alti)
BEAM_Z = (-36.0, -30.5)             # arka kiris (direk flanslari ve kanatlar bunun onune baglanir)
BEAM_Y = (-5.0, 5.0)
BEAM_X = (-68.0, 68.0)
PILLAR_XS = (-66.0, 0.0, 66.0)      # kiris ayaklari (merkez x); genislik 6
PILLAR_W = 6.0
LID_SERVO_UP_YZ = (-1.0, -48.5)     # ust kapak servo mili (y, z) - arka kirisin arkasinda
LID_SERVO_LO_YZ = (-17.0, -48.5)    # alt kapak servo mili (y, z); link pitch servosunun ustunden gecer
PITCH_ROCKER_FLANGE_X = (-44.0, -41.0)   # sallanan milin servo koluna oturan flansi (sol)
BEARING_TOP = 3.5                   # rocker yatak plakasi eksenin 3.5 mm ustunde biter
ROCKER_END_X = 41.0                 # sag uc; sag rocker kolu (dis tarafta, x 36.4..40.4) disinda yatak plakasi
MASK_Z = (16.5, 19.5)
MASK_Y = (-46.0, 24.0)
MASK_X = (-64.0, 64.0)
MASK_OPEN_HALF = (13.5, 7.0)        # goz acikligi elips yari eksenleri (x, y); kapali ust kapak y=7.7'ye kadar orter

# ----------------------------------------------------------------------------------------------
# 8. Hareket araliklari (SPEC §2) ve kontrol pozlari
# ----------------------------------------------------------------------------------------------
YAW_RANGE = 30.0
PITCH_RANGE = 25.0
CHECK_YAWS = (-30.0, 0.0, 30.0)
CHECK_PITCHES = (-25.0, 0.0, 25.0)
INTERFERENCE_TOL_MM3 = 0.01          # [SPEC gorev] >= 0.01 mm3 kesisim hata
