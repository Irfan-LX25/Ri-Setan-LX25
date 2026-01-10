// pemanggilan library yang dibutuhkan
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <esp_now.h>

//inisialisasi Task yang digunakan
TaskHandle_t mainTask;
TaskHandle_t I2CTask;

//inisialisasi fungsi Task
void mainApplicationTask(void *parameter);
void I2CCommunicationSendTask(void *parameter);

//variabel swerve drive
int SLAVE_ADDR_MODULE[4]= {10, 11, 12, 13}; //alamat I2C untuk masing-masing module
int Drive_SP[4];
int Steer_SP[4];

// variabel data stick (diisi dari ESPNOW remote)
short int LX = 0, LY = 0, RX_Joy = 0;

// ================= ESPNOW receiver data structures (harus sama dengan remote) ================
typedef struct struct_message {
    bool stat[15];
    int joyData[4];
    uint8_t remoteIndex;
} struct_message;

volatile struct_message rxMessage;
volatile bool espNowNew = false; // flag: data baru diterima

// (button flags removed — mapping-only mode)
//insialisasi dan variabel EEPROM
#define EEPROM_SIZE 512
#define addsDegree 0
int8_t zeroDegree;
int8_t eepromVal;

// Mapping constants
#define JOY_DEADZONE 500      // deadzone for combined magnitude (0..4095)
#define MAX_RPM 600           // maximum speed (RPM) to send to slave
#define JOY_THRESHOLD 30      // threshold untuk mendeteksi joystick analog
#define RX_THRESHOLD 40       // threshold untuk rotation override

// helper to apply joystick -> steer/speed mapping dengan button override
void mapJoyToSetpoints() {
    long lx = LX;
    long ly = LY;
    long rx = RX_Joy;
    
    // Button logic: INPUT_PULLUP means LOW (0) = PRESSED, HIGH (1) = NOT PRESSED
    // stat[4]=UP, stat[5]=LEFT, stat[6]=DOWN, stat[7]=RIGHT
    bool buttonUp = (rxMessage.stat[4] == LOW);      // UP button PRESSED
    bool buttonDown = (rxMessage.stat[6] == LOW);    // DOWN button PRESSED
    bool buttonLeft = (rxMessage.stat[5] == LOW);    // LEFT button PRESSED
    bool buttonRight = (rxMessage.stat[7] == LOW);   // RIGHT button PRESSED

    // Priority 1: Check button overrides first (HIGHEST PRIORITY)
    if (buttonUp) {
        Serial.println("Button UP: Degree 0");
        int degreeval[4] = {0, 0, 0, 0};
        int speedval[4] = {100, 100, 100, 100};
        addValue(degreeval, speedval);
        return;
    }
    else if (buttonDown) {
        Serial.println("Button DOWN: Degree 180");
        int degreeval[4] = {180, 180, 180, 180};
        int speedval[4] = {100, 100, 100, 100};
        addValue(degreeval, speedval);
        return;
    }
    else if (buttonLeft) {
        Serial.println("Button LEFT: Degree -90");
        int degreeval[4] = {-90, -90, -90, -90};
        int speedval[4] = {100, 100, 100, 100};
        addValue(degreeval, speedval);
        return;
    }
    else if (buttonRight) {
        Serial.println("Button RIGHT: Degree 90");
        int degreeval[4] = {90, 90, 90, 90};
        int speedval[4] = {100, 100, 100, 100};
        addValue(degreeval, speedval);
        return;
    }

    // Priority 2: Check rotation override (right stick X)
    if (abs(rx) > 1000) {
        if (rx < 0) {
            Serial.println("Rotate Left");
            int degs[4] = {-45, -135, 45, 135};
            int spd[4] = {200, 200, 200, 200};
            for (int i = 0; i < 4; i++) { Steer_SP[i] = degs[i]; Drive_SP[i] = spd[i]; }
        } else {
            Serial.println("Rotate Right");
            int degs[4] = {135, 45, -135, -45};
            int spd[4] = {200, 200, 200, 200};
            for (int i = 0; i < 4; i++) { Steer_SP[i] = degs[i]; Drive_SP[i] = spd[i]; }
        }
        return;
    }

    // Priority 3: Analog joystick mapping (LEFT STICK)
    double mag = sqrt((double)lx * (double)lx + (double)ly * (double)ly);
    
    if (mag < JOY_DEADZONE) {
        // no translation input
        for (int i = 0; i < 4; i++) Drive_SP[i] = 0;
    } else {
        // compute angle so that forward (LY negative on remote) -> 0 degrees
        double angleRad = atan2((double)lx, -(double)ly); // atan2(x, -y)
        double angleDeg = angleRad * 180.0 / PI; // -180..180
        int angleInt = (int)round(angleDeg);

        // normalize magnitude (max 4095)
        double norm = mag / 4095.0;
        if (norm > 1.0) norm = 1.0;
        int speedVal = (int)round(norm * MAX_RPM);

        for (int i = 0; i < 4; i++) {
            Steer_SP[i] = angleInt;
            Drive_SP[i] = speedVal;
        }
        
        Serial.print("Angle: "); Serial.print(angleInt);
        Serial.print("° Speed: "); Serial.print(speedVal);
        Serial.print(" RPM (Magnitude: "); Serial.print((int)mag); Serial.println(")");
    }
}

/*
    program untuk memindahkan data setpoint ke variabel sementara sebelum
    dikirim ke masing-masing module swerve drive melalui I2C
*/

void addValue(int degreeVal[4], int speedVal[4]) {
  for (int i = 0; i < 4; i++) {
    Steer_SP[i] = degreeVal[i];
    Drive_SP[i] = speedVal[i];
  }
}

// onConnect removed (PS3 indicator not used in ESPNOW mode)

void setup() {
    Serial.begin(115200);
//pembacaan data hadap roda saat terkahir kali digerakkan di memori EEPROM
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialise EEPROM");
        Serial.println("Restarting...");
        delay(1000);
        ESP.restart();
    }
    // pinBuzzer removed; no hardware buzzer configuration
// setup ESPNOW receiver (replace PS3 controller usage)
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error initializing ESP-NOW");
        } else {
            Serial.println("ESP-NOW initialized");
            esp_now_register_recv_cb([](const uint8_t * mac, const uint8_t *incomingLocal, int len){
                // copy into volatile struct safely
                if (len == sizeof(struct_message)) {
                    memcpy((void*)&rxMessage, incomingLocal, sizeof(struct_message));
                    espNowNew = true;
                }
            });
        }
        Serial.println("Ready to receive ESPNOW data");
    zeroDegree = EEPROM.read(addsDegree);
    for (int i = 0; i < 4; i++) {
        Steer_SP[i] = zeroDegree;
    }
/*
    Menjalankan task RTOS untuk program utama
    xtaskCreatePinnedToCore(
        fungsi yang akan dijalankan,
        nama task,
        ukuran stack memory untuk task,
        parameter yang akan dikirim ke task,
        prioritas task,
        handle untuk task,
        core CPU tempat task dijalankan
    ESP32 memiliki 3 core CPU (core 0 dan core 1 dan ULP core)
    sebisa mungkin prioritas task disamakan untuk menghindari crash task
    yang paling sering dijalankan diberikan prioritas lebih tinggi
*/ 
 // Membuat Task untuk komunikasi I2C
    xTaskCreatePinnedToCore(
        I2CCommunicationSendTask,
        "I2C Communication Task",
        10000,
        NULL,
        1,
        &I2CTask,
        0
    );
    xTaskCreatePinnedToCore(
        mainApplicationTask,
        "Main Application Task",
        10000,
        NULL,
        1,
        &mainTask,
        1
    );
}

void loop() {
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

/*
    Program Task utama untuk membaca data stick PS3
    dan mengirimkan data setpoint ke masing-masing module
    swerve drive melalui I2C
*/
void mainApplicationTask(void *parameter) {
    pinMode(2, OUTPUT); // Pin LED indikator task berjalan
    for (;;){
        // Jika ada data ESPNOW baru, salin ke variabel lokal yang dipakai program
        if (espNowNew) {
            // copy joy data
            LX = rxMessage.joyData[0];
            LY = rxMessage.joyData[1];
            RX_Joy = rxMessage.joyData[2];
            
            espNowNew = false;
            // map joystick values to steer/speed setpoints (mapping-only source)
            mapJoyToSetpoints();
        }
        if (Steer_SP[0]>0){
            eepromVal = 1;
        }
        else {
            eepromVal = -1;
        }
        EEPROM.write(addsDegree,eepromVal);
        EEPROM.commit();
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay untuk mengatur frekuensi pembacaan stick
    }
}
/*
    Program Task untuk mengirimkan data setpointsudut dan kecepatan
    ke masing-masing module swerve drive melalui I2C
*/
void I2CCommunicationSendTask(void *parameter) {
    Wire.begin(); // Inisialisasi I2C sebagai Master
    unsigned long lastDebugTime = 0;
    
    for (;;){
        for (int i = 0; i <4; i++) {
            // Format: Drive_SP#Steer_SP (contoh: 100#45)
            String dataModule = String(Drive_SP[i]) + "#" + String(Steer_SP[i]);
            int dataSize = dataModule.length();
            
            // Konversi string ke char array
            char dataToSend[dataSize + 1];
            dataModule.toCharArray(dataToSend, dataSize + 1);
            
            // Kirim ke slave dengan alamat I2C
            Wire.beginTransmission(SLAVE_ADDR_MODULE[i]);
            Wire.write((uint8_t *)dataToSend, dataSize);
            //Wire.endTransmission();
            int i2cStatus = Wire.endTransmission();
            
            // Debug output setiap 500ms (lebih informatif)
            if (millis() - lastDebugTime >= 500) {
                int displayIndex = i + 1; // tampilkan 1..4 untuk manusia
                Serial.print("Module "); Serial.print(displayIndex);
                Serial.print(" (Addr:"); Serial.print(SLAVE_ADDR_MODULE[i]);
                Serial.print(") <- "); Serial.print(dataModule);
                Serial.print("  I2C status:"); Serial.print(i2cStatus);
                if (i2cStatus == 0) {
                    Serial.println(" (OK)");
                } else if (i2cStatus == 1) {
                    Serial.println(" (Error: data too long)");
                } else if (i2cStatus == 2) {
                    Serial.println(" (NACK on address - no device responded)");
                } else if (i2cStatus == 3) {
                    Serial.println(" (NACK on data)");
                } else {
                    Serial.println(" (Other error)");
                }
            }
            
            vTaskDelay(30 / portTICK_PERIOD_MS); // Delay antar pengiriman ke masing-masing module
        }
        
        if (millis() - lastDebugTime >= 500) {
            lastDebugTime = millis();
        }
    }
}
