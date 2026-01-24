/*---------------------------------------------------------------------------------------------------------*/
/*----------------------REAL-TIME PID PARAMETER TUNING & MOTOR CALIBRATION TOOL----------------------------*/
/*--------------------INCLUDING INTERNAL ENCODER FEEDBACK WITH PID CLOSED LOOP FEEDBACK--------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-25 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-25 TEAM---------------------------------*/
/*----------------------------------------------------V1.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 16:50:00, 23 JAN 26------------------------------------*/
/*
 * STM32 SLAVE - ALL IN ONE (CALIBRATION + PID)
 * I2C: 0x0A | SDA: PB11, SCL: PB10
 * Encoder: TIM4 Direct Access (PB6 & PB7)
 */

#include <Wire.h>

// --- PINOUT ---
#define MOTOR_A PA_6
#define MOTOR_B PA_7

// --- SETTINGS ---
#define SLAVE_ADDR 0x10
#define SAMPLE_TIME 20 // ms

// --- VARIABLES ---
volatile long encoderCount = 0;
long lastEncoderCount = 0;

float currentSpeed = 0; // Ticks per SAMPLE_TIME
float targetValue = 0;  // Bisa berupa Target Speed (PID) atau Target PWM (Manual)
float pwmOutput = 0;

// PID Variables
float kp = 0.0, ki = 0.0, kd = 0.0;
float error = 0, lastError = 0, integral = 0;

// System Mode
// 0 = PID Mode (Target = Speed)
// 1 = Manual/Calibration Mode (Target = PWM -255 to 255)
int mode = 0; 

unsigned long lastTime = 0;

// Data to send
struct DataPacket {
  float speed;  // Kecepatan (Ticks per 20ms)
  long pos;     // Posisi Encoder Total
} sendData;

void setup() {
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  stopMotor();

  // --- SETUP ENCODER TIM4 (DIRECT REGISTER) ---
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  pinMode(PB6, INPUT_PULLUP);
  pinMode(PB7, INPUT_PULLUP);
  TIM4->PSC = 0;
  TIM4->ARR = 0xFFFF;
  TIM4->CCMR1 = 0x0101; 
  TIM4->SMCR = 0x0003; // Encoder Mode 3
  TIM4->CNT = 0;
  TIM4->CR1 |= 1;      // Enable

  // --- SETUP I2C ---
  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastTime >= SAMPLE_TIME) {
    // 1. BACA ENCODER
    int16_t count = TIM4->CNT; 
    TIM4->CNT = 0; 
    long delta = count; // Handle overflow otomatis krn int16_t
    
    encoderCount += delta;       // Posisi Total (Untuk Cek PPR)
    currentSpeed = (float)delta; // Kecepatan (Ticks per 20ms)

    // 2. KONTROL MOTOR
    if (mode == 1) { 
      // --- MODE MANUAL (Untuk Cek Max RPM & Arah) ---
      pwmOutput = targetValue; // Target dianggap PWM langsung
    } 
    else {
      // --- MODE PID (Untuk Tuning) ---
      error = targetValue - currentSpeed;
      
      // Anti-windup
      if (abs(pwmOutput) < 255) integral += error;
      
      float derivative = error - lastError;
      pwmOutput = (kp * error) + (ki * integral) + (kd * derivative);
      lastError = error;
    }

    // 3. EKSEKUSI KE DRIVER
    setMotor(pwmOutput);
    
    // 4. PREPARE DATA
    sendData.speed = currentSpeed;
    sendData.pos = encoderCount;
    
    lastTime = now;
  }
}

void setMotor(float pwm) {
  int p = (int)constrain(pwm, -255, 255);
  
  if (p > 0) {
    analogWrite(MOTOR_A, p);
    analogWrite(MOTOR_B, 0);
  } else if (p < 0) {
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, abs(p));
  } else {
    stopMotor();
  }
}

void stopMotor() {
  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
}

// Format Perintah I2C: [Char Command] [Float Value]
// P=Kp, I=Ki, D=Kd, S=Setpoint/Target, M=Mode, R=Reset
void receiveEvent(int howMany) {
  if (Wire.available() >= 5) {
    char cmd = Wire.read();
    union { byte b[4]; float f; } data;
    for(int i=0; i<4; i++) data.b[i] = Wire.read();

    switch(toupper(cmd)) {
      case 'P': kp = data.f; break;
      case 'I': ki = data.f; break;
      case 'D': kd = data.f; break;
      case 'S': targetValue = data.f; break;
      case 'M': mode = (int)data.f; targetValue=0; integral=0; break; // Ganti Mode
      case 'R': encoderCount = 0; break; // Reset Posisi
    }
  }
}

void requestEvent() {
  Wire.write((byte*)&sendData, sizeof(sendData));
}
