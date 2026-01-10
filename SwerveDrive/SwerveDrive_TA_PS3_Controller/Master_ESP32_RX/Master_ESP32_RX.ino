#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Ps3Controller.h>

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

// variabel data stick PS3
short int LX, LY, RX;

#define pinBuzzer 25 
//insialisasi dan variabel EEPROM
#define EEPROM_SIZE 512
#define addsDegree 0
int8_t zeroDegree;
int8_t eepromVal;

// program untuk menyimpan nilai analog joystick PS3
void readStick() {
  LX = Ps3.data.analog.Stick.lX();
  LY = Ps3.data.analog.Stick.lY();
  RX = Ps3.data.analog.Stick.rX();
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

/*
    program untuk memberikan indikator melalui buzzer saat
    ps3 sudah terkoneksi dengan ESP32 master
*/
void onConnect(){
    for (int i = 0; i < 3; i++) {
        digitalWrite(pinBuzzer, HIGH);
        delay(100);
        digitalWrite(pinBuzzer, LOW);
        delay(100);
    }
    Serial.println("PS3 Connected");
}

void setup() {
    Serial.begin(115200);
//pembacaan data hadap roda saat terkahir kali digerakkan di memori EEPROM
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialise EEPROM");
        Serial.println("Restarting...");
        delay(1000);
        ESP.restart();
    }
    pinMode(pinBuzzer, OUTPUT);
// setup koneksi PS3 controller
    Ps3.attach(readStick);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("01:02:03:04:05:06"); // ganti dengan MAC Address PS3 controller Anda
    Serial.println("Ready to connect to PS3 controller");
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
        if(LY <= -30 && (LX >= -40 && LX <= 40) || Ps3.data.button.up){
            Serial.println("Speed 600 dan Degree 0");
            int degreeval[4] = {0, 0, 0, 0};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LY >= 30 && (LX >= -50 && LX <= 50) || Ps3.data.button.down){
            Serial.println("Speed 600 dan Degree 180");
            int degreeval[4] = {180, 180, 180, 180};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LX <= -30 && (LY >= -50 && LY <= 50) || Ps3.data.button.left){
            Serial.println("Speed 600 dan Degree -90");
            int degreeval[4] = {-90, -90, -90, -90};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LX >= 30 && (LY >= -50 && LY <= 50) || Ps3.data.button.right){
            Serial.println("Speed 600 dan Degree 90");
            int degreeval[4] = {90, 90, 90, 90};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LY <= -30 && (LX >= 31 && LX <= 127)){
            Serial.println("Speed 600 dan Degree 45");
            int degreeval[4] = {45, 45, 45, 45};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LY >= 30 && (LX>=31 && LX <=127)){
            Serial.println("Speed 600 dan Degree 135");
            int degreeval[4] = {135, 135, 135, 135};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LY <= -50 && (LX<=-36 && LX >=-128)){
            Serial.println("Speed 600 dan Degree -45");
            int degreeval[4] = {-45, -45, -45, -45};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (LY >= 35 && (LX<=-36 && LX >=-128)){
            Serial.println("Speed 600 dan Degree -135");
            int degreeval[4] = {-135, -135, -135, -135};
            int speedval[4] = {100, 100, 100, 100};
            addValue(degreeval, speedval);
        }
        else if (RX <= -40){
            Serial.println("Speed 500");
            int degreeval[4] = {-45, -135, 45, 135};
            int speedval[4] = {200, 200, 200, 200};
            addValue(degreeval, speedval);
        }
        else if (RX >= 40){
            Serial.println("Speed 500");
            int degreeval[4] = {135, 45, -135, -45};
            int speedval[4] = {200, 200, 200, 200};
            addValue(degreeval, speedval);
        }
        else {
            Serial.println("Speed 0 dan Degree 0");
            for (int i = 0; i < 4; i++) {
                Drive_SP[i] = 0;
            }
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
    for (;;){
        String dataModule[4];
        for (int i = 0; i < 4; i++) {
            dataModule[i] = String(Drive_SP[i]) + "#" + String(Steer_SP[i]);
            int dataSize = dataModule[i].length();
            char dataToSend[dataSize + 1];
            dataModule[i].toCharArray(dataToSend, dataSize + 1);
            Wire.beginTransmission(SLAVE_ADDR_MODULE[i]);
            Wire.write((const uint8_t *)dataToSend, dataSize);
            Wire.endTransmission();
            vTaskDelay(50 / portTICK_PERIOD_MS); // Delay antar pengiriman ke masing-masing module
        }
    }
}
