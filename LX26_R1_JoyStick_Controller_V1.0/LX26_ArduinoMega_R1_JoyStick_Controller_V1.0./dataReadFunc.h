#ifndef DATAREADFUNC_H
#define DATAREADFUNC_H

#include <Arduino.h>

// Struktur data untuk menampung data dari ESP32
// Sesuai dengan urutan pengiriman dari ESP32
typedef struct struct_message {
    bool stat[15] = {true}; // Status tombol (Default true/HIGH karena pull-up internal biasanya)
    int joyData[4] = {0, 0, 0, 0}; // Data Analog: LX, LY, RX, RY
} struct_message;

struct_message recvData;

// Inisialisasi Serial untuk komunikasi dengan ESP32
void recvStart() {
    // Serial0 untuk Debugging ke Laptop
    Serial.begin(115200);
    // Serial1 untuk menerima data dari ESP32
    Serial1.begin(115200); 
    Serial.println("System Initialized: Master Ready");
}

// Fungsi parsing data CSV dari ESP32
// Format: joy0,joy1,joy2,joy3,btn0,btn1...btn14,index
void parseData(String line) {
    int index = 0;
    // Menggunakan strtok untuk memecah string berdasarkan koma
    char *token = strtok((char *)line.c_str(), ",");
    while (token != nullptr) {
        if (index < 4) {
            // 4 data pertama adalah Joystick
            recvData.joyData[index] = atoi(token); 
        } else if (index < 19) {
            // 15 data berikutnya adalah Button
            recvData.stat[index - 4] = atoi(token); 
        }
        index++;
        token = strtok(nullptr, ",");
    }
}

// Fungsi utama untuk mengecek buffer Serial1
void checkData() {
    if (Serial1.available()) {
        String receivedLine = Serial1.readStringUntil('\n'); 
        if (receivedLine.length() > 0) {
            parseData(receivedLine);
        }
    }
}

#endif