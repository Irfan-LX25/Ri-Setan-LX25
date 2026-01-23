// alamat tetap pakai nilai desimalnya
#include <Wire.h>
#define I2C_SDA 21
#define I2C_SCL 22

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("\nI2C Scanner Ready!");
  Serial.print("SDA Pin: "); Serial.println(I2C_SDA);
  Serial.print("SCL Pin: "); Serial.println(I2C_SCL);
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Ditemukan device I2C di alamat: 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error di alamat 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0)
    Serial.println("Tidak ada device I2C yang ditemukan\n");
  else
    Serial.println("Selesai.\n");

  delay(2000);
}
