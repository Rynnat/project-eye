// Project Eye v5 - montaj yardimcisi: YAW servosunu (D3) modelin notr acisinda (90 derece) sabit tutar.
// Kullanim: yukle, servo 90'a gelir ve orada kalir; krank/kolu gozler DUZ KARSIYA bakacak sekilde tak.
// Model: yaw 0 -> EYE_YAW 90, sinirlar 65..115 (v5/cad/out/kinematics.json).
#include <Servo.h>

const uint8_t YAW_PIN = 3;
const int NOTR = 90;

Servo yaw;

void setup() {
  Serial.begin(115200);
  yaw.attach(YAW_PIN);
  yaw.write(NOTR);
  Serial.println(F("EYE v5 MONTAJ: YAW D3 = 90 (notr). Krank/kolu gozler duz karsiya bakacak sekilde tak."));
}

void loop() {
  yaw.write(NOTR);   // surekli ayni aci: elle itilse bile geri doner
  delay(200);
}
