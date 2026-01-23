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
#define SLAVE_ADDR 0x10  //Alamat I2C tetap desimal

struct DataPacket {
  float speed;
  long pos;
};
DataPacket data;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL
  delay(1000);
  Serial.println("=== SYSTEM READY ===");
  Serial.println("Commands:");
  Serial.println(" m1   -> Masuk MODE MANUAL (Cek MaxRPM)");
  Serial.println(" m0   -> Masuk MODE PID (Tuning)");
  Serial.println(" r    -> Reset Encoder (Cek PPR)");
  Serial.println(" s100 -> Set Target (PWM di Mode 1, Speed di Mode 0)");
  Serial.println(" p1.0 -> Set Kp, i0.1 -> Ki, d0.5 -> Kd");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    parseCommand(s);
  }
  Wire.requestFrom(SLAVE_ADDR, (int)sizeof(DataPacket));
  if (Wire.available()) {
    Wire.readBytes((byte*)&data, sizeof(DataPacket));
  }
  Serial.print("Target:"); Serial.print(getTarget()); // Placeholder
  Serial.print(" ,Speed:"); Serial.print(data.speed);
  Serial.print(" ,Posisi:"); Serial.println(data.pos); // Untuk Cek PPR
  
  delay(50);
}

float _lastTarget = 0;
float getTarget() { return _lastTarget; }

void parseCommand(String cmd) {
  cmd.trim(); if(cmd.length()==0) return;
  char c = cmd.charAt(0);
  float v = cmd.substring(1).toFloat();
  if(c=='s' || c=='S') _lastTarget = v;
  sendI2C(c, v);
}

void sendI2C(char c, float v) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(c);
  union { float f; byte b[4]; } d;
  d.f = v;
  Wire.write(d.b, 4);
  Wire.endTransmission();
}
