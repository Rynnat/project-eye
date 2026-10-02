// Project Eye v5 - montaj yardimcisi: LIDS servosu (gecici olarak D3'te).
// Acilista servo 172.7 dereceye gider: modelde kapak kranki bu acida TAM YATAY, ucu robotun ONUNE bakar
// (kin.py: phi = LS_A_PHI0 + b = 0; servo = 90 + (b - B_MID) = 90 + (70.4 + 12.29)).
// Kranki bu anda yere paralel, ucu one bakacak sekilde tak. Sonra seri porttan "A 90" -> tasarim notru
// (krank asagi, dikeyden ~7 derece one). Aci esleme ana firmware ile ayni (544..2400 us).
// DIKKAT: lamalar (link_up/link_lo) krank'a BAGLI DEGILKEN kullan; 172.7 kapak sinirinin (77.7..102.3) cok disinda.
#include <Servo.h>
const uint8_t PIN = 3;
const float YATAY = 172.7f;
Servo s;
float aci = YATAY;
void yaz(float d) { s.writeMicroseconds((int)(544 + d / 180.0f * (2400 - 544) + 0.5f)); }
void setup() {
  Serial.begin(115200);
  s.attach(PIN, 544, 2400);
  yaz(aci);
  Serial.println(F("EYE v5 MONTAJ LIDS: D3 = 172.7 (krank yatay, ucu one). 'A <derece>' ile degistir; 'A 90' tasarim notru."));
}
void loop() {
  static char buf[24]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (n > 1 && buf[0] == 'A') { float d = atof(buf + 1); if (d >= 0 && d <= 180) { aci = d; Serial.print(F("OK ")); Serial.println(aci, 1); } else Serial.println(F("ERR 0-180")); }
      n = 0;
    } else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  yaz(aci);
  delay(20);
}
