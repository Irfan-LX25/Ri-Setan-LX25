/*---------------------------------------------------------------------------------------------------------*/
/*----------------------REAL-TIME PID PARAMETER TUNING & MOTOR CALIBRATION TOOL----------------------------*/
/*--------------------INCLUDING INTERNAL ENCODER FEEDBACK WITH PID CLOSED LOOP FEEDBACK--------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-25 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-25 TEAM---------------------------------*/
/*----------------------------------------------------V1.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 16:50:00, 23 JAN 26------------------------------------*/
#include <Wire.h>
#define MOTOR1_PWMA PA6
#define MOTOR1_PWMB PA7
volatile long encoderCount = 0;
float input = 0, output = 0, setpoint = 0;
float kp = 1.0, ki = 0.0, kd = 0.0; 
float lastError = 0, integral = 0;
unsigned long lastTime = 0;
const int sampleTime = 20; // 20ms sampling
struct DataPacket {
  float rpm;    
  long enc;     
};

DataPacket sendData;

void setup() {
  pinMode(MOTOR1_PWMA, OUTPUT);
  pinMode(MOTOR1_PWMB, OUTPUT);
  stopMotor();
  setupEncoderTim4();
  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  Wire.begin(0x10); // Address 0x10
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {
    int16_t count = TIM4->CNT; 
    TIM4->CNT = 0;
    long delta = count;

    encoderCount += delta; 
    input = (float)delta;  
    float error = setpoint - input;
    if (abs(output) < 255) {
       integral += (error);
    }
    if(integral > 2000) integral = 2000;
    if(integral < -2000) integral = -2000;
    float derivative = (error - lastError);
    output = (kp * error) + (ki * integral) + (kd * derivative);
    lastError = error
    setMotorSpeed(output);
    lastTime = now;
    sendData.rpm = input; 
    sendData.enc = encoderCount;
  }
}

void setupEncoderTim4() {
  // 1. Enable Clock untuk TIM4 (Bit 2 di APB1ENR)
  // RCC->APB1ENR |= 0x00000004; // Versi Hardcode
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Versi Macro CMSIS (Lebih aman)
  pinMode(PB6, INPUT);
  pinMode(PB7, INPUT);
  TIM4->CR1 = 0;       // Matikan Timer dulu untuk setting
  TIM4->PSC = 0;       // Prescaler 0 (Hitung setiap tick)
  TIM4->ARR = 0xFFFF;  // Auto Reload Register (Max 16 bit)
  
  // Set CCMR1 (Capture/Compare Mode Register)
  // Kita set CC1S = 01 (Input TI1) dan CC2S = 01 (Input TI2)
  // 0x0101 dalam hex = biner ...0000 0001 0000 0001
  TIM4->CCMR1 = 0x0101; 
  
  // Set SMCR (Slave Mode Control Register)
  // SMS (Slave Mode Selection) bit 0-2 di set ke 011 (Encoder Mode 3)
  TIM4->SMCR = 0x0003; 
  
  // 4. Reset dan Enable Timer
  TIM4->CNT = 0;
  TIM4->CR1 |= 1; // Set Bit 0 (CEN - Counter Enable)
}

void setMotorSpeed(float pwmVal) {
  int pwm = (int)pwmVal;
  
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  if (pwm > 0) {
    analogWrite(MOTOR1_PWMA, pwm);
    analogWrite(MOTOR1_PWMB, 0);
  } else if (pwm < 0) {
    analogWrite(MOTOR1_PWMA, 0);
    analogWrite(MOTOR1_PWMB, abs(pwm));
  } else {
    stopMotor();
  }
}

void stopMotor() {
  analogWrite(MOTOR1_PWMA, 0);
  analogWrite(MOTOR1_PWMB, 0);
}

void receiveEvent(int howMany) {
  if (Wire.available() >= 5) { 
    char cmd = Wire.read();
    union { byte b[4]; float f; } data;
    for(int i=0; i<4; i++) data.b[i] = Wire.read();

    switch(cmd) {
      case 'P': kp = data.f; break;
      case 'I': ki = data.f; break;
      case 'D': kd = data.f; break;
      case 'S': setpoint = data.f; break; 
      case 'R': encoderCount = 0; break;  
    }
  }
}

void requestEvent() {
  Wire.write((byte*)&sendData, sizeof(sendData));
}
