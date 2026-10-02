/*
 * Project Eye v3 (POC) - firmware (Arduino Uno)
 * ==========================================================================
 * Iki goz (bagli) + iki UST kapak (tek servo), 3 x SG90. Sozlesme: ../../SPEC.md
 * v2 firmware'inin (../../../v2/firmware) 3 kanalli hali; protokol ayni aile.
 *
 * Kanal haritasi:
 *   kanal 0  EYE_YAW    D3
 *   kanal 1  EYE_PITCH  D5
 *   kanal 2  LIDS       D6   (iki ust kapak birlikte)
 *
 * GUC: servolar HARICI 5 V (>= 2 A yeterli, 3 A onerilir) kaynaktan beslenir,
 * GND Uno ile ORTAK. Uno'nun 5 V pininden servo beslemeyin.
 *
 * Seri protokol (115200 baud, satir sonu '\n', ASCII; '\r' yok sayilir):
 *   PC -> Arduino
 *     S a0 a1 a2            3 kanalin ham servo acisi (0-180, ondalik olur)
 *     D                     tum servolari birak (detach)
 *     A 1 | A 0             kendi basina "canli" (idle) modu ac / kapat
 *     ?                     durum sorgusu
 *   Arduino -> PC
 *     EYE v3 READY          acilista bir kez
 *     OK                    gecerli S / D / A komutu
 *     ERR <mesaj>           gecersiz komut (satir islenmez)
 *     STATE a0 a1 a2 idle=<0|1>   '?' cevabi (anlik konum; idle = su an canli modda mi)
 *
 * Guvenlik:
 *   - Her aci kanal basina SABIT mekanik limite kirpilir (LIMIT_MIN/MAX =
 *     cad/out/kinematics.json "limits_checked", PAY YOK: disi DOGRULANMADI).
 *   - Her kanal hedefe en fazla 600 deg/s ile gider (millis tabanli, bloklamaz).
 *   - Bozuk / asiri uzun satir, eksik/fazla arguman, 0-180 disi -> ERR, hareket yok.
 *
 * Idle: 2 sn S gelmezse (idle acik ve servolar bagli ise) rastgele sakkadlar,
 * 3-6 sn'de bir kirpma, yavas kapak nefesi; kapak bakisin pitch'ini takip eder.
 * Ilk S'de PC kontrolu geri alir. '?' ve 'A' idle sayacini sifirlamaz.
 * Idle, PC'siz calistigi icin kalibrasyonu asagidaki derleme-zamani IDLE_*
 * sabitlerinden alir (settings.json ile ayni tutun).
 *
 * control/sim.py bu dosyanin Python esdegeridir; tests/test_firmware_sync.py
 * sabitlerin ayni kaldigini kontrol eder.
 */

#include <Servo.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define FW_BANNER "EYE v3 READY"

const uint8_t NUM_CH = 3;
const uint8_t PINS[NUM_CH] = {3, 5, 6};

//                                YAW     PITCH   LIDS
const float LIMIT_MIN[NUM_CH] = { 65.0f,  70.0f,  67.5f};
const float LIMIT_MAX[NUM_CH] = {115.0f, 110.0f, 112.5f};

const float MAX_SPEED_DEG_S = 600.0f;
const uint16_t SERVO_US_MIN = 544;
const uint16_t SERVO_US_MAX = 2400;
const unsigned long TICK_MS = 5;
const unsigned long IDLE_TIMEOUT_MS = 2000;
const uint8_t LINE_MAX = 64;

// Idle kalibrasyonu (settings.json = kinematics.json onerisi)
const float IDLE_EYE_CENTER[2] = {90.0f, 90.0f};
const float IDLE_EYE_DEG_PER_UNIT[2] = {25.0f, 20.0f};  // yaw yonu idle'da onemsiz (simetrik)
const float IDLE_LID_CLOSED = 112.5f;
const float IDLE_LID_OPEN = 67.5f;
const float IDLE_LID_FOLLOW_PITCH = 0.6f;
const float IDLE_GAZE_U_MAX = 0.6f;
const float IDLE_GAZE_V_MAX = 0.45f;
const float IDLE_BREATH_BASE = 0.72f;
const float IDLE_BREATH_AMP = 0.12f;
const float IDLE_BREATH_PERIOD_S = 5.0f;
const unsigned long IDLE_SACCADE_MIN_MS = 400;
const unsigned long IDLE_SACCADE_MAX_MS = 2500;
const unsigned long IDLE_BLINK_MIN_MS = 3000;
const unsigned long IDLE_BLINK_MAX_MS = 6000;
const unsigned long IDLE_BLINK_CLOSED_MS = 120;
const float BOOT_LID_OPENNESS = 0.8f;

// --------------------------------------------------------------------------
Servo servos[NUM_CH];
float pos[NUM_CH];
float target[NUM_CH];
uint16_t lastUs[NUM_CH];

bool attached = false;
bool idleEnabled = true;
bool idleActive = false;
unsigned long lastSMs = 0;
unsigned long lastTickMs = 0;

float idleU = 0.0f, idleV = 0.0f;
unsigned long idleNextSaccadeMs = 0;
unsigned long idleNextBlinkMs = 0;
unsigned long idleBlinkStartMs = 0;
bool idleBlinking = false;

char lineBuf[LINE_MAX + 1];
uint8_t lineLen = 0;
bool lineOverflow = false;
bool lineBadChar = false;

// --------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float clampToLimit(uint8_t ch, float deg) {
  return clampf(deg, LIMIT_MIN[ch], LIMIT_MAX[ch]);
}

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
    writeServo(i, true);  // attach'tan once darbe: servo 90'a ziplamasin
    servos[i].attach(PINS[i], SERVO_US_MIN, SERVO_US_MAX);
  }
  attached = true;
}

static void detachAll() {
  for (uint8_t i = 0; i < NUM_CH; i++) servos[i].detach();
  attached = false;
}

static float lidAngle(float openness) {
  openness = clampf(openness, 0.0f, 1.0f);
  return IDLE_LID_CLOSED + openness * (IDLE_LID_OPEN - IDLE_LID_CLOSED);
}

static void sendErr(const char *msg) {
  Serial.print(F("ERR "));
  Serial.println(msg);
}

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
// Idle
// --------------------------------------------------------------------------
static void idleEnter(unsigned long now) {
  idleActive = true;
  idleNextSaccadeMs = now;
  idleNextBlinkMs = now + (unsigned long)random(IDLE_BLINK_MIN_MS, IDLE_BLINK_MAX_MS + 1);
  idleBlinking = false;
}

static void idleUpdate(unsigned long now) {
  if ((long)(now - idleNextSaccadeMs) >= 0) {
    if (random(100) < 30) {
      idleU = random(-15, 16) / 100.0f;
      idleV = random(-10, 11) / 100.0f;
    } else {
      idleU = random(-(long)(IDLE_GAZE_U_MAX * 100), (long)(IDLE_GAZE_U_MAX * 100) + 1) / 100.0f;
      idleV = random(-(long)(IDLE_GAZE_V_MAX * 100), (long)(IDLE_GAZE_V_MAX * 100) + 1) / 100.0f;
    }
    idleNextSaccadeMs = now + (unsigned long)random(IDLE_SACCADE_MIN_MS, IDLE_SACCADE_MAX_MS + 1);
  }

  if (!idleBlinking && (long)(now - idleNextBlinkMs) >= 0) {
    idleBlinking = true;
    idleBlinkStartMs = now;
  }
  if (idleBlinking && now - idleBlinkStartMs >= IDLE_BLINK_CLOSED_MS) {
    idleBlinking = false;
    idleNextBlinkMs = now + (unsigned long)random(IDLE_BLINK_MIN_MS, IDLE_BLINK_MAX_MS + 1);
  }

  float phase = (float)(now % (unsigned long)(IDLE_BREATH_PERIOD_S * 1000.0f)) /
                (IDLE_BREATH_PERIOD_S * 1000.0f);
  float openness = IDLE_BREATH_BASE + IDLE_BREATH_AMP * sinf(phase * 2.0f * (float)M_PI);
  if (idleBlinking) openness = 0.0f;

  // ust kapak pitch'i takip eder (yukari bakinca kalkar) - eye_control.py ile ayni
  float upper = openness + IDLE_LID_FOLLOW_PITCH * 0.5f * idleV * openness;

  target[0] = clampToLimit(0, IDLE_EYE_CENTER[0] + idleU * IDLE_EYE_DEG_PER_UNIT[0]);
  target[1] = clampToLimit(1, IDLE_EYE_CENTER[1] + idleV * IDLE_EYE_DEG_PER_UNIT[1]);
  target[2] = clampToLimit(2, lidAngle(upper));
}

static void idleStop() {
  idleActive = false;
  idleBlinking = false;
  for (uint8_t i = 0; i < NUM_CH; i++) target[i] = pos[i];
}

// --------------------------------------------------------------------------
// Komutlar
// --------------------------------------------------------------------------
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
  if (cmd == NULL) return;
  if (cmd[1] != '\0') { sendErr("unknown command"); return; }

  char c = cmd[0];
  if (c >= 'a' && c <= 'z') c -= 32;

  if (c == 'S') {
    float v[NUM_CH];
    for (uint8_t i = 0; i < NUM_CH; i++) {
      char *tok = strtok(NULL, delim);
      if (tok == NULL) { sendErr("S needs 3 angles"); return; }
      if (!parseNumber(tok, v[i])) { sendErr("bad number"); return; }
      if (v[i] < 0.0f || v[i] > 180.0f) { sendErr("angle out of 0-180"); return; }
    }
    if (strtok(NULL, delim) != NULL) { sendErr("S needs 3 angles"); return; }
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

  pos[0] = clampToLimit(0, IDLE_EYE_CENTER[0]);
  pos[1] = clampToLimit(1, IDLE_EYE_CENTER[1]);
  pos[2] = clampToLimit(2, lidAngle(BOOT_LID_OPENNESS));
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
    if (dt > 0.1f) dt = 0.1f;
    if (idleActive) idleUpdate(now);
    stepMotion(dt);
  }
}
