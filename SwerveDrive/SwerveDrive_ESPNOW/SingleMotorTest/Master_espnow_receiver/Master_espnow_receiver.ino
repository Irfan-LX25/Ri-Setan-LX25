#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

// Simple ESP32 ESPNOW receiver (master) -> sends PWM commands via I2C to STM32 slave
// Adjust I2C pins if you wired differently
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const uint8_t SLAVE_ADDR = 10; // target STM32 slave 7-bit address

// Toggle if joystick Y is inverted on your remote (true if pushing forward gives negative values)
// Set to true because your test showed forward produced reverse — invert the sign.
const bool INVERT_Y = false;
// Deadzone (joystick small jitter around center will be treated as zero)
const int JOY_DEADZONE = 400; // adjust if needed (0..4095)

// ESPNOW message format must match remote
typedef struct struct_message {
  bool stat[15];
  int joyData[4];
  uint8_t remoteIndex;
} struct_message;

volatile struct_message rxMessage;
volatile bool espNowNew = false;

void onDataRecv(const uint8_t * mac, const uint8_t *incomingLocal, int len) {
  if (len == sizeof(struct_message)) {
    memcpy((void*)&rxMessage, incomingLocal, sizeof(struct_message));
    espNowNew = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 ESPNOW Receiver -> I2C Master (single motor)");

  // I2C init
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.print("I2C init SDA="); Serial.print(I2C_SDA); Serial.print(" SCL="); Serial.println(I2C_SCL);

  // ESPNOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  } else {
    Serial.println("ESP-NOW initialized");
    esp_now_register_recv_cb(onDataRecv);
  }
}

unsigned long lastSent = 0;

void loop() {
  if (espNowNew) {
    // copy volatile data locally using memcpy to avoid volatile assignment issues
    struct_message m;
    noInterrupts();
    memcpy((void*)&m, (const void*)&rxMessage, sizeof(struct_message));
    espNowNew = false;
    interrupts();

    int joyY = m.joyData[1]; // left stick Y (per remote mapping)

    // Apply deadzone on raw joystick value
    if (abs(joyY) < JOY_DEADZONE) {
      joyY = 0;
    }

    // Map joystick (-4095..4095) to PWM (-255..255)
    int pwm = 0;

    if (joyY > JOY_DEADZONE) {
      pwm = 255;     // MAJU
    }
    else if (joyY < -JOY_DEADZONE) {
      pwm = -255;    // MUNDUR
    }
    else {
      pwm = 0;       // DIAM
    }

    // Build payload as "pwm#0"
    String payload = String(pwm) + "#0";
    int len = payload.length();
    char buf[len + 1];
    payload.toCharArray(buf, len + 1);

    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write((uint8_t *)buf, len);
    int status = Wire.endTransmission();

    Serial.print("JoyY: "); Serial.print(joyY);
    Serial.print(" -> PWM: "); Serial.print(pwm);
    Serial.print("  Sent to "); Serial.print(SLAVE_ADDR);
    Serial.print(" payload="); Serial.print(payload);
    Serial.print(" I2C status="); Serial.println(status);

    lastSent = millis();
  }

  // optional: send zero when no joystick activity for safety
  // implement if desired

  delay(10);
}
