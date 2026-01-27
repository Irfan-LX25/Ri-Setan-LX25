#include <Wire.h>
#define I2C_ADDR 0x12

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial.println("--- PPR VERIFIER PG45 ---");
  Serial.println("Ketik 'r' untuk RESET ke nol");
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'r') {
      Wire.beginTransmission(I2C_ADDR);
      Wire.write('r');
      Wire.endTransmission();
      Serial.println(">> Ticks Reset to 0 <<");
    }
  }

  long ticks = 0;
  Wire.requestFrom(I2C_ADDR, sizeof(ticks));
  if (Wire.available() == sizeof(ticks)) {
    Wire.readBytes((uint8_t*)&ticks, sizeof(ticks));
    
    Serial.print("Current Ticks: ");
    Serial.print(ticks);
    Serial.print(" | Progress: ");
    Serial.print((abs(ticks) / 326.0) * 100.0);
    Serial.println("% dari 1 putaran");
  }

  delay(150);
}
