/*---------------------------------------------------------------------------------------------------------*/
/*--------------------ARDUINO MEGA PARSING DATA FROM ESP32, X-DRIVE WITH JOYSTICK--------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-25 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-25 TEAM---------------------------------*/
/*----------------------------------------------------V1.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 20:30:00, 08 JAN 26-----------------------------------*/


#include "datareadfunc.h"
#include "xdrivemaster.h"

unsigned long lastKinematicsTime = 0;
const int KINEMATICS_INTERVAL = 20; // Update motor setiap 20ms

void setup() {
    // Inisialisasi Komunikasi
    recvStart();   // Serial ke ESP32 & PC
    xDriveSetup(); // I2C Master
    
    Serial.println("--- ROBOT 1 MASTER START ---");
}

void loop() {
    // 1. Baca data dari ESP32 terus menerus
    checkData();

    // 2. Hitung Kinematika & Kirim I2C secara periodik
    unsigned long currentMillis = millis();
    if (currentMillis - lastKinematicsTime >= KINEMATICS_INTERVAL) {
        calcXDrive();
        lastKinematicsTime = currentMillis;
        
        // Debugging Opsional (Bisa di-comment saat lomba)
        // Serial.print("FL:"); Serial.print(targetRPM_FL);
        // Serial.print(" FR:"); Serial.print(targetRPM_FR);
        // Serial.println();
    }
}