#ifndef XDRIVEMASTER_H
#define XDRIVEMASTER_H

#include <Wire.h>
#include "datareadfunc.h"

// Alamat I2C Slave
#define SLAVE_ADDR_FRONT 0x08 // Mengontrol Motor FL & FR
#define SLAVE_ADDR_REAR  0x09 // Mengontrol Motor RL & RR

// Konfigurasi Robot
const int MAX_RPM_ROBOT = 200; // Sesuai spek PG45
const int JOYSTICK_MAX = 4095; // Resolusi ADC ESP32
const int JOYSTICK_DEADZONE = 100;

// Variabel Target RPM
int16_t targetRPM_FL, targetRPM_FR, targetRPM_RL, targetRPM_RR;

// Inisialisasi I2C
void xDriveSetup() {
    Wire.begin(); // Join bus I2C sebagai Master
    Serial.println("I2C Master Initialized");
}

// Fungsi Map Joystick dengan Deadzone
int mapJoystick(int input) {
    if (input > JOYSTICK_DEADZONE) {
        return map(input, JOYSTICK_DEADZONE, JOYSTICK_MAX, 0, MAX_RPM_ROBOT);
    } else if (input < -JOYSTICK_DEADZONE) {
        return map(input, -JOYSTICK_DEADZONE, -JOYSTICK_MAX, 0, -MAX_RPM_ROBOT);
    }
    return 0;
}

// Kirim data ke Slave STM32 (2 Motor per Slave)
void sendToSlave(uint8_t address, int16_t motor1_rpm, int16_t motor2_rpm) {
    Wire.beginTransmission(address);
    
    // Pecah int16_t menjadi 2 byte (High Byte & Low Byte)
    Wire.write((motor1_rpm >> 8) & 0xFF);
    Wire.write(motor1_rpm & 0xFF);
    
    Wire.write((motor2_rpm >> 8) & 0xFF);
    Wire.write(motor2_rpm & 0xFF);
    
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("I2C Error at addr 0x"); Serial.println(address, HEX);
    }
}

// Kalkulasi Kinematika X-Drive dan Kirim ke Slave
void calcXDrive() {
    // 1. Ambil data joystick (Mapping -4095 s/d 4095 ke -200 s/d 200 RPM)
    // Asumsi: joyData[0]=LX, joyData[1]=LY, joyData[2]=RX(Rotate)
    // Sesuaikan indeks array joystick ini dengan urutan kirim ESP32 Anda
    int inputX = mapJoystick(recvData.joyData[0]); 
    int inputY = mapJoystick(recvData.joyData[1]); 
    int inputRot = mapJoystick(recvData.joyData[2]); 

    // 2. Rumus X-Drive (Sesuai Permintaan User)
    // FL = Y + X + Rotation
    // FR = Y - X - Rotation
    // RL = Y - X + Rotation
    // RR = Y + X - Rotation
    
    int rawFL = inputY + inputX + inputRot;
    int rawFR = inputY - inputX - inputRot;
    int rawRL = inputY - inputX + inputRot;
    int rawRR = inputY + inputX - inputRot;

    // 3. Normalisasi (Jika ada nilai melebihi MAX_RPM_ROBOT)
    int maxVal = max(abs(rawFL), max(abs(rawFR), max(abs(rawRL), abs(rawRR))));
    if (maxVal > MAX_RPM_ROBOT) {
        rawFL = (rawFL * MAX_RPM_ROBOT) / maxVal;
        rawFR = (rawFR * MAX_RPM_ROBOT) / maxVal;
        rawRL = (rawRL * MAX_RPM_ROBOT) / maxVal;
        rawRR = (rawRR * MAX_RPM_ROBOT) / maxVal;
    }

    targetRPM_FL = (int16_t)rawFL;
    targetRPM_FR = (int16_t)rawFR;
    targetRPM_RL = (int16_t)rawRL;
    targetRPM_RR = (int16_t)rawRR;

    // 4. Kirim ke Slave via I2C
    // Slave 1 (Depan): Motor 1 = FL, Motor 2 = FR
    sendToSlave(SLAVE_ADDR_FRONT, targetRPM_FL, targetRPM_FR);
    
    // Slave 2 (Belakang): Motor 1 = RL, Motor 2 = RR
    sendToSlave(SLAVE_ADDR_REAR, targetRPM_RL, targetRPM_RR);
}

#endif