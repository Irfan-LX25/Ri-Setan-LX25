#include <GlobalVariable.h>
int speedVal[4];
int degreeVal[4];
String dataModule[4];
void readRecvData(){
  lx = map(recvData.joyData[0], -4095, 4095, -128, 127);
  ly = map(recvData.joyData[1], -4095, 4095, -128, 127);
  rx = map(recvData.joyData[2], -4095, 4095, -128, 127);
}
void addValue(int degreeVal[4], int speedval[4])
{
  for (int i = 0; i < 4; i++){
    Steer_SP[i] = degreeVal[i];
    Drive_SP[i] = speedVal[i];
  }
}
void onConnect()
{
  for (int i = 0; i < 3; i++);{
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
    delay(100);
  }
  Serial.println("ESP-NOW Connected.");
}
void onDisconnect()
{
  for (int i = 0; i < 5; i++){
    digitalWrite(pinBuzzer, HIGH);
    delay(50);
    digitalWrite(pinBuzzer, LOW);
    delay(50);
  }
  Serial.println("ESP-NOW Disconnected.");
  for (int i = 0; i < 4; i++){
    Drive_SP[i] = 0;
  }
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvData, incomingData, sizeof(recvData));
  lastDataTime = millis();
  
  if (!espnow_connected) {
    espnow_connected = true;
    onConnect();
  }
}
void setupESP_NOW() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW Ready");
}
void checkConnection() {
  if (espnow_connected && (millis() - lastDataTime > TIMEOUT_MS)) {
    espnow_connected = false;
    onDisconnect();
  }
}
void setup() {
  Serial.begin(9600);
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  pinMode(pinBuzzer, OUTPUT);
  setupESP_NOW();
  zeroDegree = EEPROM.read(addsDegree);
  for (int i = 0; i < 4; i++){
    Steer_SP[i] = zeroDegree;
  }
  xTaskCreatePinnedToCore(
    I2CCommunicationSendTask,
    "12CCommunicationSendTask",
    10000,
    NULL,
    1,
    &I2CTask,
    0
  );
  xTaskCreatePinnedToCore(
    mainApplicationTask,
    "mainApplicationTask",
    10000,
    NULL,
    1,
    &mainTask,
    1);
}
void loop() {
  checkConnection();
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
void mainApplicationTask(void *parameter){
  pinMode(2, OUTPUT);
  for(;;)
  {
    if (espnow_connected) {
      readRecvData();
      bool btnUP = recvData.stat[4];   
      bool btnDOWN = recvData.stat[6];  
      bool btnLEFT = recvData.stat[5];  
      bool btnRIGHT = recvData.stat[7]; 
      if (ly <= -30 && (ly >= -40 && lx <= 40) || btnUP)
      {
        Serial.println("Speed 600 dan degree 0");
        int degreeVal[4] = {0, 0, 0, 0};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 30 && (lx >= -50 && lx <=50) || btnDOWN)
      {
        Serial.println("Speed 600 dan degree 180");
        int degreeVal[4] = {180, 180, 180, 180};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 30 && (lx >= -50 && lx <=50) || btnRIGHT){
        Serial.println("Speed 600 dan degree 90");
        int degreeVal[4] = {90, 90, 90, 90};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly <= -30 && (lx >= -50 && lx <=50) || btnLEFT)
      {
        Serial.println("Speed 600 dan degree -90");
        int degreeVal[4] = {-90, -90, -90, -90};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly <= -30 && (lx >= 31 && lx <=127))
      {
        Serial.println("Speed 600 dan degree 45");
        int degreeVal[4] = {45, 45, 45, 45};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 30 && (lx >= 31 && lx <=127))
      {
        Serial.println("Speed 600 dan degree 135");
        int degreeVal[4] = {135, 135, 135, 135};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly <= -50 && (lx <= -36 && lx >=-128))
      {
        Serial.println("Speed 600 dan degree -45");
        int degreeVal[4] = {-45, -45, -45, -45};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 35 && (lx <= -36 && lx <=-128))
      {
        Serial.println("Speed 600 dan degree -135");
        int degreeVal[4] = {-135, -135, -135, -135};
        int speedVal[4] = {100, 100, 100, 100};
        addValue(degreeVal, speedVal);
      }
      else if (rx <= -40)
      {
        Serial.println("Speed 500");
        int degreeVal[4] = {-45, -135, 45, 135};
        int speedVal[4] = {200, 200, 200, 200};
        addValue(degreeVal, speedVal);
      }
      else if (rx >= 40)
      {
        Serial.println("Speed 500");
        int degreeVal[4] = {135, 45, -135, -45};
        int speedVal[4] = {200, 200, 200, 200};
        addValue(degreeVal, speedVal);
      }
      else 
      {
        Serial.println("Speed 0 dan degree 0");
        for (int i = 0; i < 4; i++)
        {
          Drive_SP[i]=0;
        }
      }
    } else {
      for (int i = 0; i < 4; i++) {
        Drive_SP[i] = 0;
      }
    }
    if (Steer_SP[0] > 0)
    {
      eepromVal = -1;
    }
    EEPROM.write(addsDegree, eepromVal);
    EEPROM.commit();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void I2CCommunicationSendTask(void *parameter){
  Wire.begin(); 
  for (;;)
  {
    String dataModule[4];
    for (int i = 0; i < 4; i++)
    {
      dataModule[i] = String(Drive_SP[i]) + "#" + String(Steer_SP[i]);
      int dataSize = dataModule[i].length();
      char dataToSend[dataSize + 1];
      dataModule[i].toCharArray(dataToSend, dataSize + 1); 
      Wire.beginTransmission(SLAVE_ADDR_MODULE[i]);
      Wire.write((const uint8_t *) dataToSend, dataSize);
      Wire.endTransmission();
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}
