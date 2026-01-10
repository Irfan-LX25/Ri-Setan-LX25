#include <Wire.h>
#include <Arduino.h>

// Simple STM32 slave: listens on address 10 and drives one motor PWM
// This sketch supports two control modes:
//  - DUAL_PWM mode (two complementary PWM pins: FWD and REV)
//  - DIR_PWM mode (one PWM pin + one DIR digital pin)
// Default is DIR_PWM because many motor drivers expect PWM+DIR.

// --- CONFIGURATION ---
#define I2C_SDA PB11
#define I2C_SCL PB10
#define SLAVE_ADDR 10

// Choose control mode: 0 = DUAL_PWM (two PWM pins), 1 = DIR_PWM (one PWM + DIR)
#define CONTROL_MODE_DIR_PWM 1
#define CONTROL_MODE_DUAL_PWM 0
#define CONTROL_MODE CONTROL_MODE_DIR_PWM

// Pins for DUAL_PWM mode (two PWM outputs to H-bridge inputs)
#define MOTOR_PWM_FWD PA6
#define MOTOR_PWM_REV PA7

// Pins for DIR_PWM mode (PWM + DIR)
#define MOTOR_PWM_PIN PA6
#define MOTOR_DIR_PIN PB7

#define DATA_SIZE 20
char receiveData[DATA_SIZE + 1];

int currentSpeed = 0; // range -255..255

static inline void setDualPwm(int s) {
  // s: -255..255
  int v = constrain(s, -255, 255);
  if (v > 0) {
    analogWrite(MOTOR_PWM_FWD, v);
    analogWrite(MOTOR_PWM_REV, 0);
  } else if (v < 0) {
    analogWrite(MOTOR_PWM_FWD, 0);
    analogWrite(MOTOR_PWM_REV, abs(v));
  } else {
    analogWrite(MOTOR_PWM_FWD, 0);
    analogWrite(MOTOR_PWM_REV, 0);
  }
}

static inline void setDirPwm(int s) {
  // s: -255..255
  int v = constrain(s, -255, 255);
  if (v > 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH); // forward
    analogWrite(MOTOR_PWM_PIN, v);
  } else if (v < 0) {
    digitalWrite(MOTOR_DIR_PIN, LOW); // reverse
    analogWrite(MOTOR_PWM_PIN, v));
  } else {
    analogWrite(MOTOR_PWM_PIN, 0);
  }
}

void applySpeed(int speed) {
  currentSpeed = constrain(speed, -255, 255);
#if (CONTROL_MODE == CONTROL_MODE_DUAL_PWM)
  setDualPwm(currentSpeed);
#else
  setDirPwm(currentSpeed);
#endif
}

void receiveEvent(int howMany) {
  memset(receiveData, 0, DATA_SIZE + 1);
  int idx = 0;
  while (Wire.available() && idx < DATA_SIZE) {
    receiveData[idx++] = Wire.read();
  }
  receiveData[idx] = '\0';

  // parse "speed#degree" — we only use speed here
  char *token = strtok(receiveData, "#");
  if (token != NULL) {
    int recvSpeed = atoi(token);
    int pwm = constrain(recvSpeed, -255, 255);
    applySpeed(pwm);
  }
}

void setup() {
  // No Serial because user can't monitor STM32; keep minimal
  // Configure I2C pins then start as slave
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

#if (CONTROL_MODE == CONTROL_MODE_DUAL_PWM)
  pinMode(MOTOR_PWM_FWD, OUTPUT);
  pinMode(MOTOR_PWM_REV, OUTPUT);
  analogWrite(MOTOR_PWM_FWD, 0);
  analogWrite(MOTOR_PWM_REV, 0);
#else
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);
#endif
}

void loop() {
  // All work done in receiveEvent; keep loop light
  delay(50);
}
