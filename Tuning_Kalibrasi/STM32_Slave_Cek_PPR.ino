#include <Arduino.h>
#include <Wire.h>

#define ENC1_A PB7
#define ENC1_B PB6
#define I2C_SLAVE_ADDR 0x12

volatile long encoderTicks = 0;
unsigned long last_micros = 0;
const uint32_t DEBOUNCE_TIME = 150; 

void readEncoder() {
  uint32_t current_micros = micros();

  if (current_micros - last_micros > DEBOUNCE_TIME) {
    if (digitalRead(ENC1_B) == HIGH) encoderTicks++;
    else encoderTicks--;
    last_micros = current_micros;
  }
}

void requestEvent() {
  Wire.write((uint8_t*)&encoderTicks, sizeof(encoderTicks));
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    if (c == 'r') encoderTicks = 0; 
  }
}

void setup() {
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder, RISING);

  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {
}
