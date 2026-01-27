#include <Wire.h>
#define I2C_ADDR 0x12

struct CalibrationData {
  uint8_t currentPWM;
  uint8_t minPWM_Pos;
  uint8_t minPWM_Neg;
  bool isTesting;
} incoming;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial.println("--- PWM DEADZONE DETECTOR ---");
  Serial.println("Ketik 'f' (Maju) atau 'b' (Mundur) untuk mulai tes");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(cmd);
    Wire.endTransmission();
  }

  Wire.requestFrom(I2C_ADDR, sizeof(incoming));
  if (Wire.available() == sizeof(incoming)) {
    Wire.readBytes((uint8_t*)&incoming, sizeof(incoming));
    
    if (incoming.isTesting) {
      Serial.print("Testing PWM: ");
      Serial.println(incoming.currentPWM);
    } else {
      Serial.print("RESULT -> Deadzone Maju: "); Serial.print(incoming.minPWM_Pos);
      Serial.print(" | Deadzone Mundur: "); Serial.println(incoming.minPWM_Neg);
    }
  }
  delay(200);
}
