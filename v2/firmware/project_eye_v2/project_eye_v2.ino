/*
 * Project Eye v2 - firmware (Arduino Uno)
 * ==========================================================================
 * Iki goz + dort kapak, 6 x SG90. Sozlesme: ../../SPEC.md §4 ve §5.
 *
 * Kanal haritasi (SPEC §4, SABIT):
 *   kanal 0  EYE_YAW    D3      kanal 3  LID_LL  D9
 *   kanal 1  EYE_PITCH  D5      kanal 4  LID_UR  D10
 *   kanal 2  LID_UL     D6      kanal 5  LID_LR  D11
 *
 * GUC: servolar HARICI 5 V >= 3 A kaynaktan beslenir, GND Uno ile ORTAK.
 * Uno'nun 5 V pininden 6 servo beslemeyin (USB ~500 mA, SG90 zorlanma
 * akimi servo basina ~650 mA).
 *
 * Seri protokol (115200 baud, satir sonu '\n', ASCII; '\r' yok sayilir):
 *   PC -> Arduino
 *     S a0 a1 a2 a3 a4 a5   6 kanalin ham servo acisi (0-180, ondalik olur)
 *     D                     tum servolari birak (detach)
 *     A 1 | A 0             kendi basina "canli" (idle) modu ac / kapat
 *     ?                     durum sorgusu
 *   Arduino -> PC
 *     EYE v2 READY          acilista bir kez
 *     OK                    gecerli S / D / A komutu
 *     ERR <mesaj>           gecersiz komut (satir islenmez)
 *     STATE a0 .. a5 idle=<0|1>   '?' cevabi (a = anlik konum, idle = su an
 *                                 canli modda mi)
 *
 * Guvenlik:
 *   - Her aci kanal basina SABIT mekanik limite kirpilir (LIMIT_MIN/MAX).
 *   - Her kanal hedefe en fazla MAX_SPEED_DEG_S hizla gider. Hareket
 *     millis() tabanli ve bloklamayandir; seri okuma hic durmaz.
 *   - Bozuk satir, asiri uzun satir, sayi olmayan/eksik/fazla arguman ve
 *     0-180 disi deger -> ERR, hicbir servo hareket etmez.
 *
 * Idle: IDLE_TIMEOUT_MS boyunca S gelmezse (ve idle acik, servolar bagli
 * ise) firmware kendi yasar: rastgele sakkadlar, 3-6 sn'de bir kirpma,
 * yavas nefes gibi kapak salinimi, kapaklar bakisin pitch'ini takip eder.
 * Ilk S komutunda PC kontrolu geri alir. '?' ve 'A' idle sayacini
 * SIFIRLAMAZ (sadece durum sorgulayan bir PC gozleri dondurmasin).
 *
 * NOT: Kalibrasyon PC tarafindadir (settings.json). Idle modu PC'siz
 * calistigi icin kapak acik/kapali acilarini asagidaki derleme-zamani
 * varsayilanlarindan alir (settings.json ornek degerleriyle ayni). Montaj ve
 * kalibrasyondan sonra IDLE_* sabitlerini ve LIMIT_* degerlerini gercek
 * degerlere gore guncelleyin.
 *
 * control/sim.py bu dosyanin Python esdegerini (FirmwareModel) icerir;
 * control/tests/test_firmware_sync.py iki taraftaki sabitlerin ayni
 * kaldigini kontrol eder. Buradaki bir sabiti degistirirseniz sim.py'yi de
 * guncelleyin.
 */

#include <Servo.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// --------------------------------------------------------------------------
// Sabitler
// --------------------------------------------------------------------------
#define FW_BANNER "EYE v2 READY"

const uint8_t NUM_CH = 6;
const uint8_t PINS[NUM_CH] = {3, 5, 6, 9, 10, 11};

// Sabit mekanik limitler (derece) - cad/out/kinematics.json "limits_checked"
// degerleri, PAY YOK: check.py bu araliklarin koselerinde carpisma taradi,
// disi DOGRULANMADI (ve pitch'te gercek carpisma var).
// EYE_PITCH: "EYE_PITCH_firmware_safe" = her yaw icin goz pitch'i +-25 icinde
// kalan yaw'dan BAGIMSIZ aralik. Bedeli: yaw=0'da goz yalnizca -20.3..+23.0
// dereceye iner/cikar. (yaw=0 tablosunun 56.73..125.59'u yaw +-30'da gozu
// -31.8'e goturur -> carpisma.) Kinematik tablo degisirse burayi, sim.py'yi
// guncelleyin; tests/test_firmware_sync.py karsilastirir.
//                                YAW     PITCH    UL       LL       UR       LR
const float LIMIT_MIN[NUM_CH] = { 60.0f,  63.17f,  64.38f,  66.23f,  64.37f,  66.23f};
const float LIMIT_MAX[NUM_CH] = {120.0f, 121.62f, 115.63f, 113.77f, 115.62f, 113.77f};

const float MAX_SPEED_DEG_S = 600.0f;       // SPEC §5: <= 600 deg/s
const uint16_t SERVO_US_MIN = 544;          // Servo.h varsayilanlari:
const uint16_t SERVO_US_MAX = 2400;         //   0 deg -> 544 us, 180 -> 2400
const unsigned long TICK_MS = 5;            // hareket guncelleme periyodu
const unsigned long IDLE_TIMEOUT_MS = 2000; // SPEC §5: 2 sn komutsuzluk
const uint8_t LINE_MAX = 64;                // en uzun kabul edilen satir

// Idle icin derleme-zamani kalibrasyon (settings.json = kinematics.json onerisi)
const float IDLE_EYE_CENTER[2] = {90.0f, 90.0f};
const float IDLE_EYE_DEG_PER_UNIT[2] = {30.0f, 34.43f};
//                                  UL      LL      UR      LR
const float IDLE_LID_CLOSED[4] = { 64.38f, 113.77f, 115.62f,  66.23f};
const float IDLE_LID_OPEN[4]   = { 96.15f,  86.85f,  83.85f,  93.15f};
const float IDLE_LID_FOLLOW_PITCH = 0.6f;
const float IDLE_GAZE_U_MAX = 0.6f;         // sakkad genligi (birim)
const float IDLE_GAZE_V_MAX = 0.45f;
const float IDLE_BREATH_BASE = 0.72f;       // kapak nefesi: taban aciklik
const float IDLE_BREATH_AMP = 0.12f;        //   +/- genlik
const float IDLE_BREATH_PERIOD_S = 5.0f;
const unsigned long IDLE_SACCADE_MIN_MS = 400;
const unsigned long IDLE_SACCADE_MAX_MS = 2500;
const unsigned long IDLE_BLINK_MIN_MS = 3000;
const unsigned long IDLE_BLINK_MAX_MS = 6000;
const unsigned long IDLE_BLINK_CLOSED_MS = 120;  // kapaklar bu sure kapaliya gider

// Baslangic konumu: gozler ortada, kapaklar ~%80 acik
const float BOOT_LID_OPENNESS = 0.8f;

// --------------------------------------------------------------------------
// Durum
// --------------------------------------------------------------------------
Servo servos[NUM_CH];
float pos[NUM_CH];      // anlik (hiz sinirli) konum, derece
float target[NUM_CH];   // hedef, derece (limitlere kirpilmis)
uint16_t lastUs[NUM_CH];

bool attached = false;
bool idleEnabled = true;
bool idleActive = false;
unsigned long lastSMs = 0;     // son S komutu (ya da acilis)
unsigned long lastTickMs = 0;

// idle alt durumu
float idleU = 0.0f, idleV = 0.0f;
unsigned long idleNextSaccadeMs = 0;
unsigned long idleNextBlinkMs = 0;
unsigned long idleBlinkStartMs = 0;
bool idleBlinking = false;

// seri satir tamponu
char lineBuf[LINE_MAX + 1];
uint8_t lineLen = 0;
bool lineOverflow = false;
bool lineBadChar = false;

// --------------------------------------------------------------------------
// Yardimcilar
// --------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float clampToLimit(uint8_t ch, float deg) {
  return clampf(deg, LIMIT_MIN[ch], LIMIT_MAX[ch]);
}

// Alt-derece cozunurluk icin write() yerine mikrosaniye kullanilir.
static uint16_t degToUs(float deg) {
  float us = SERVO_US_MIN + (deg / 180.0f) * (float)(SERVO_US_MAX - SERVO_US_MIN);
  return (uint16_t)(us + 0.5f);
}

static void writeServo(uint8_t ch, bool force) {
  uint16_t us = degToUs(pos[ch]);
  if (force || us != lastUs[ch]) {
    servos[ch].writeMicroseconds(us);
    lastUs[ch] = us;
  }
}

static void attachAll() {
  if (attached) return;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    // attach'tan ONCE darbe genisligini yaz: servo acilista 90'a ziplamasin
    writeServo(i, true);
    servos[i].attach(PINS[i], SERVO_US_MIN, SERVO_US_MAX);
  }
  attached = true;
}

static void detachAll() {
  for (uint8_t i = 0; i < NUM_CH; i++) servos[i].detach();
  attached = false;
}

static float lidAngle(uint8_t lid, float openness) {
  openness = clampf(openness, 0.0f, 1.0f);
  return IDLE_LID_CLOSED[lid] + openness * (IDLE_LID_OPEN[lid] - IDLE_LID_CLOSED[lid]);
}

static void sendErr(const char *msg) {
  Serial.print(F("ERR "));
  Serial.println(msg);
}

// --------------------------------------------------------------------------
// Hareket: kanal basina hiz sinirli yaklasma (bloklamaz)
// --------------------------------------------------------------------------
static void stepMotion(float dt) {
  float maxStep = MAX_SPEED_DEG_S * dt;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    float d = target[i] - pos[i];
    if (d > maxStep) d = maxStep;
    else if (d < -maxStep) d = -maxStep;
    pos[i] += d;
    if (attached) writeServo(i, false);
  }
}

// --------------------------------------------------------------------------
// Idle ("canli") mod
// --------------------------------------------------------------------------
static void idleEnter(unsigned long now) {
  idleActive = true;
  idleNextSaccadeMs = now;  // hemen bir sakkad
  idleNextBlinkMs = now + (unsigned long)random(IDLE_BLINK_MIN_MS, IDLE_BLINK_MAX_MS + 1);
  idleBlinking = false;
}

static void idleUpdate(unsigned long now) {
  // 1) Sakkad: rastgele bir noktaya sicra, sonra bir sure dur.
  if ((long)(now - idleNextSaccadeMs) >= 0) {
    if (random(100) < 30) {  // %30 ihtimalle merkeze yakina don
      idleU = random(-15, 16) / 100.0f;
      idleV = random(-10, 11) / 100.0f;
    } else {
      idleU = random(-(long)(IDLE_GAZE_U_MAX * 100), (long)(IDLE_GAZE_U_MAX * 100) + 1) / 100.0f;
      idleV = random(-(long)(IDLE_GAZE_V_MAX * 100), (long)(IDLE_GAZE_V_MAX * 100) + 1) / 100.0f;
    }
    idleNextSaccadeMs = now + (unsigned long)random(IDLE_SACCADE_MIN_MS, IDLE_SACCADE_MAX_MS + 1);
  }

  // 2) Kirpma: kapaklar IDLE_BLINK_CLOSED_MS boyunca kapaliya gider, sonra
  //    nefes acikligina doner (hiz siniri acilisi dogal olarak yumusatir).
  if (!idleBlinking && (long)(now - idleNextBlinkMs) >= 0) {
    idleBlinking = true;
    idleBlinkStartMs = now;
  }
  if (idleBlinking && now - idleBlinkStartMs >= IDLE_BLINK_CLOSED_MS) {
    idleBlinking = false;
    idleNextBlinkMs = now + (unsigned long)random(IDLE_BLINK_MIN_MS, IDLE_BLINK_MAX_MS + 1);
  }

  // 3) Nefes: yavas sinuzoidal kapak salinimi
  float phase = (float)(now % (unsigned long)(IDLE_BREATH_PERIOD_S * 1000.0f)) /
                (IDLE_BREATH_PERIOD_S * 1000.0f);
  float openness = IDLE_BREATH_BASE + IDLE_BREATH_AMP * sinf(phase * 2.0f * (float)M_PI);
  if (idleBlinking) openness = 0.0f;

  // 4) Kapaklar pitch'i takip eder: yukari bakinca ust kapak kalkar, alt
  //    kapak da yukari (kapanma yonune) gelir. eye_control.py ile ayni formul.
  float shift = IDLE_LID_FOLLOW_PITCH * 0.5f * idleV * openness;
  float upper = openness + shift;
  float lower = openness - shift;

  target[0] = clampToLimit(0, IDLE_EYE_CENTER[0] + idleU * IDLE_EYE_DEG_PER_UNIT[0]);
  target[1] = clampToLimit(1, IDLE_EYE_CENTER[1] + idleV * IDLE_EYE_DEG_PER_UNIT[1]);
  target[2] = clampToLimit(2, lidAngle(0, upper));  // UL
  target[3] = clampToLimit(3, lidAngle(1, lower));  // LL
  target[4] = clampToLimit(4, lidAngle(2, upper));  // UR
  target[5] = clampToLimit(5, lidAngle(3, lower));  // LR
}

static void idleStop() {
  idleActive = false;
  idleBlinking = false;
  for (uint8_t i = 0; i < NUM_CH; i++) target[i] = pos[i];  // oldugu yerde dur
}

// --------------------------------------------------------------------------
// Komut isleme
// --------------------------------------------------------------------------
// Tam bir ondalik sayi ayristirir; token'in tamami sayi olmali.
static bool parseNumber(const char *tok, float &out) {
  if (tok == NULL || *tok == '\0') return false;
  char *end = NULL;
  double v = strtod(tok, &end);
  if (end == tok || *end != '\0') return false;
  if (isnan(v) || isinf(v)) return false;
  out = (float)v;
  return true;
}

static void cmdState() {
  Serial.print(F("STATE"));
  for (uint8_t i = 0; i < NUM_CH; i++) {
    Serial.print(' ');
    Serial.print(pos[i], 1);
  }
  Serial.print(F(" idle="));
  Serial.println(idleActive ? 1 : 0);
}

static void handleLine(char *line, unsigned long now) {
  const char *delim = " \t";
  char *cmd = strtok(line, delim);
  if (cmd == NULL) return;  // bos satir: sessizce yok say
  if (cmd[1] != '\0') { sendErr("unknown command"); return; }

  char c = cmd[0];
  if (c >= 'a' && c <= 'z') c -= 32;  // seri monitorde elle yazmaya tolerans

  if (c == 'S') {
    float v[NUM_CH];
    for (uint8_t i = 0; i < NUM_CH; i++) {
      char *tok = strtok(NULL, delim);
      if (tok == NULL) { sendErr("S needs 6 angles"); return; }
      if (!parseNumber(tok, v[i])) { sendErr("bad number"); return; }
      if (v[i] < 0.0f || v[i] > 180.0f) { sendErr("angle out of 0-180"); return; }
    }
    if (strtok(NULL, delim) != NULL) { sendErr("S needs 6 angles"); return; }
    for (uint8_t i = 0; i < NUM_CH; i++) target[i] = clampToLimit(i, v[i]);
    idleActive = false;
    idleBlinking = false;
    lastSMs = now;
    attachAll();
    Serial.println(F("OK"));
  } else if (c == 'D') {
    if (strtok(NULL, delim) != NULL) { sendErr("D takes no args"); return; }
    idleActive = false;
    detachAll();
    for (uint8_t i = 0; i < NUM_CH; i++) target[i] = pos[i];
    Serial.println(F("OK"));
  } else if (c == 'A') {
    char *tok = strtok(NULL, delim);
    if (tok == NULL || strtok(NULL, delim) != NULL || tok[1] != '\0' ||
        (tok[0] != '0' && tok[0] != '1')) {
      sendErr("A needs 1 or 0");
      return;
    }
    idleEnabled = (tok[0] == '1');
    if (!idleEnabled && idleActive) idleStop();
    Serial.println(F("OK"));
  } else if (c == '?') {
    if (strtok(NULL, delim) != NULL) { sendErr("? takes no args"); return; }
    cmdState();
  } else {
    sendErr("unknown command");
  }
}

// Bloklamadan seri okur. Asiri uzun satirlar sonuna ('\n') kadar atilir ve
// tek bir ERR ile cevaplanir; tampon asla tasmaz.
static void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (lineOverflow) {
        sendErr("line too long");
      } else if (lineBadChar) {
        sendErr("bad character");
      } else {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf, now);
      }
      lineLen = 0;
      lineOverflow = false;
      lineBadChar = false;
      continue;
    }
    if (lineOverflow) continue;
    if ((ch < 32 || ch > 126) && ch != '\t') lineBadChar = true;
    if (lineLen >= LINE_MAX) {
      lineOverflow = true;
      continue;
    }
    lineBuf[lineLen++] = (char)ch;
  }
}

// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  randomSeed(((unsigned long)analogRead(A0) << 16) ^ micros());

  for (uint8_t i = 0; i < NUM_CH; i++) pos[i] = 90.0f;
  pos[0] = clampToLimit(0, IDLE_EYE_CENTER[0]);
  pos[1] = clampToLimit(1, IDLE_EYE_CENTER[1]);
  for (uint8_t l = 0; l < 4; l++) pos[2 + l] = clampToLimit(2 + l, lidAngle(l, BOOT_LID_OPENNESS));
  for (uint8_t i = 0; i < NUM_CH; i++) {
    target[i] = pos[i];
    lastUs[i] = 0;
  }
  attachAll();

  unsigned long now = millis();
  lastSMs = now;
  lastTickMs = now;
  Serial.println(F(FW_BANNER));
}

void loop() {
  unsigned long now = millis();
  pollSerial(now);

  if (idleEnabled && attached && !idleActive && now - lastSMs >= IDLE_TIMEOUT_MS) {
    idleEnter(now);
  }

  unsigned long elapsed = now - lastTickMs;
  if (elapsed >= TICK_MS) {
    lastTickMs = now;
    float dt = elapsed / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;  // uzun bir takilma sonrasi ani sicrama olmasin
    if (idleActive) idleUpdate(now);
    stepMotion(dt);
  }
}
