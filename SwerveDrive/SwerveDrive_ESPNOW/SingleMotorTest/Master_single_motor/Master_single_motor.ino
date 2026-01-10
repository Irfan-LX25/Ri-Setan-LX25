#include <Wire.h>
#include <Arduino.h>

// Simple ESP32 master: sends speed commands to one slave (address 10)
// Adjust pins below if you wired SDA/SCL differently
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const uint8_t SLAVE_ADDR = 10; // 7-bit address

unsigned long lastSend = 0;
unsigned long period = 1000; // send every 1s
int testSequence[] = {0, 600, 0, -100}; // speeds to send (positive/negative)
int seqLen = sizeof(testSequence) / sizeof(testSequence[0]);
int seqIdx = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 Simple I2C Master - Single Motor Test");
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.print("I2C initialized on SDA="); Serial.print(I2C_SDA);
  Serial.print(" SCL="); Serial.println(I2C_SCL);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= period) {
    lastSend = now;
    int speed = testSequence[seqIdx];
    seqIdx = (seqIdx + 1) % seqLen;

    // Format: speed#degree (slave will parse both; degree unused here)
    String payload = String(speed) + "#0";
    int len = payload.length();
    char buf[len + 1];
    payload.toCharArray(buf, len + 1);

    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write((uint8_t *)buf, len);
    int status = Wire.endTransmission();

    Serial.print("Sent to "); Serial.print(SLAVE_ADDR);
    Serial.print(" -> "); Serial.print(payload);
    Serial.print("  I2C status:"); Serial.println(status);

    // Wait a bit to avoid flooding
    delay(20);
  }
}
