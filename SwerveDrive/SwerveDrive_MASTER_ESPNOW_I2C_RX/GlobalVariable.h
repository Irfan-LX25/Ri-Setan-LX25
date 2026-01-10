#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
TaskHandle_t mainTask;
TaskHandle_t I2CTask;
void mainApplicationTask(void *parameter); 
void I2CCommunicationSendTask(void *parameter);
int SLAVE_ADDR_MODULE[4] = {10, 11, 12, 13};
int Drive_SP[4];
int Steer_SP[4];
short int lx;
short int ly;
short int rx;
#define pinBuzzer 25
#define EEPROM_SIZE 512
#define addsDegree 0
int8_t zeroDegree;
int8_t eepromVal;
typedef struct struct_message {
    bool stat[15];
    int joyData[4];
} struct_message;
struct_message recvData;
bool espnow_connected = false;
unsigned long lastDataTime = 0;
const unsigned long TIMEOUT_MS = 1000;
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void setupESP_NOW();
