#include <Arduino.h>
#include <Wire.h>

#define MOTOR1_PWMA PA6
#define MOTOR1_PWMB PA7
#define ENC1_A PB7
#define ENC1_B PB6
#define I2C_ADDR 0x12

volatile long encoderTicks = 0;
long lastTicks = 0;

struct CalibrationData {
  uint8_t currentPWM;
  uint8_t minPWM_Pos;
  uint8_t minPWM_Neg;
  bool isTesting;
} data;

enum State { IDLE, TEST_FORWARD, TEST_BACKWARD };
State currentState = IDLE;

unsigned long lastUpdate = 0;

void readEncoder() {
  if (digitalRead(ENC1_B) == HIGH) encoderTicks++;
  else encoderTicks--;
}

void setMotor(int pwm) {
  if (pwm > 0) {
    analogWrite(MOTOR1_PWMA, pwm);
    analogWrite(MOTOR1_PWMB, 0);
  } else if (pwm < 0) {
    analogWrite(MOTOR1_PWMA, 0);
    analogWrite(MOTOR1_PWMB, abs(pwm));
  } else {
    analogWrite(MOTOR1_PWMA, 0);
    analogWrite(MOTOR1_PWMB, 0);
  }
}

void setup() {
  pinMode(MOTOR1_PWMA, OUTPUT);
  pinMode(MOTOR1_PWMB, OUTPUT);
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder, RISING);

  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  Wire.begin(I2C_ADDR);
  Wire.onRequest([]() { Wire.write((uint8_t*)&data, sizeof(data)); });
  Wire.onReceive([](int len) {
    char cmd = Wire.read();
    if (cmd == 'f') { currentState = TEST_FORWARD; data.isTesting = true; data.currentPWM = 0; }
    if (cmd == 'b') { currentState = TEST_BACKWARD; data.isTesting = true; data.currentPWM = 0; }
    if (cmd == 's') { currentState = IDLE; data.isTesting = false; setMotor(0); }
  });
}

void loop() {
  if (data.isTesting && millis() - lastUpdate > 100) {
    lastUpdate = millis();
    
    long delta = abs(encoderTicks - lastTicks);
    lastTicks = encoderTicks;

    if (delta >= 2) {
      if (currentState == TEST_FORWARD) data.minPWM_Pos = data.currentPWM;
      else if (currentState == TEST_BACKWARD) data.minPWM_Neg = data.currentPWM;
      
      data.isTesting = false;
      currentState = IDLE;
      setMotor(0);
    } else {
      data.currentPWM++;
      if (data.currentPWM > 255) {
        data.isTesting = false;
        setMotor(0);
      } else {
        setMotor(currentState == TEST_FORWARD ? data.currentPWM : -data.currentPWM);
      }
    }
  }
}
