<div align="center">

# 🤖 RI-SETAN LX25  

<img src="docs/KRAI.png" width="180"/>

### LX25 PROGRAMMING RESEARCH AND DEVELOPMENT

![GitHub repo size](https://img.shields.io/github/repo-size/Irfan-LX25/Ri-Setan-LX25)
![GitHub last commit](https://img.shields.io/github/last-commit/Irfan-LX25/Ri-Setan-LX25)
![Robotics](https://img.shields.io/badge/Robotics-Research-blue)
![Embedded](https://img.shields.io/badge/Embedded-System-green)

</div>

---

## 📘 Dokumentasi Riset Program LX24

🔗 **Repository Referensi LX24**  
https://github.com/HabibMuhammad05/KRAI-ESPNOW-DS2-CONTROLLER-PROJECT  

**Deskripsi Singkat:**
- DS2 Controller menggunakan **ESP32**
- Komunikasi wireless berbasis **ESP-NOW**
- Mendukung fitur lengkap **Transmitter & Receiver**
- Digunakan pada **R1 & R2 Abu Robocon 2025**
- Riset **PID Control Motor PG45** menggunakan **internal encoder**

---

## 📙 Dokumentasi Riset Program LX25

🔗 **Repository Utama LX25**  
https://github.com/Irfan-LX25/Ri-Setan-LX25

**Deskripsi & Alur program :**

#### 🔹 R1 – Manual Control (DS2 Controller ESP-NOW)

```mermaid
graph TD
    DS2[DS2 Controller<br/>ESP32]
    RX[Receiver<br/>ESP32]
    MEGA[Arduino Mega]

    STM1[STM32 Addr1]
    STM2[STM32 Addr2]

    FL[FL Motor]
    FR[FR Motor]
    RL[RL Motor]
    RR[RR Motor]

    SENS[Sensor IMU, Limit, Encoder ]
    ACT[Actuator Servo, Solenoid ]

    DS2 -->|ESP-NOW| RX
    RX -->|Serial| MEGA

    RX -->|I2C| STM1
    RX -->|I2C| STM2

    STM1 --> FL
    STM1 --> RL

    STM2 --> FR
    STM2 --> RR

    MEGA -->|GPIO / I2C / SPI| SENS
    MEGA -->|GPIO / PWM| ACT

```
### 📂 Struktur Folder & Penjelasan

- **LX26_R1_JoyStick_Controller_V1.0**  
  Versi awal dengan Mega sebagai Master I2C STM32 Slave

- **LX26_R1_JoyStick_Controller_V1.1**  
  Revisi dengan ESP32 RX sebagai Master I2C STM32 Slve

#### 🔹 R2 – DualMode PnP (Manual ↔ Autonomous)

```mermaid
graph TD
    DS2[DS2 Controller<br/>ESP32]
    RX[Receiver<br/>ESP32]
    MEGA[Arduino Mega]
    RPI[Raspberry Pi 3B]

    STM1[STM32 Addr1]
    STM2[STM32 Addr2]

    FL[FL Motor]
    FR[FR Motor]
    RL[RL Motor]
    RR[RR Motor]

    SENS[Sensor IMU, Limit, Encoder]
    ACT[Actuator Servo, Solenoid]

    DS2 -->|ESP-NOW| RX
    RX -->|Serial| MEGA

    DS2 -->|Mode Switch| RPI
    RPI -->|USB Serial| MEGA

    MEGA -->|I2C| STM1
    MEGA -->|I2C| STM2

    STM1 --> FL
    STM1 --> RL

    STM2 --> FR
    STM2 --> RR

    MEGA -->|GPIO / I2C / SPI| SENS
    MEGA -->|GPIO / PWM| ACT


```
### 📂 Struktur Folder & Penjelasan

- **LX26_R1_JoyStick_Controller_V1.0**  
  Versi awal pengembangan joystick controller sebagai antarmuka utama operator robot.

- **LX26_R1_JoyStick_Controller_V1.1**  
  Penyempurnaan joystick controller dengan optimasi respons, stabilitas input, dan struktur kode.

#### 🔹 R2 – Autonomous PnP

```mermaid
graph TD
    RPI[Raspberry Pi 3]
    MEGA[Arduino Mega]
    CAM[ESP32-CAM]

    STM1[STM32 Addr1]
    STM2[STM32 Addr2]

    FL[FL Motor]
    FR[FR Motor]
    RL[RL Motor]
    RR[RR Motor]

    SENS[Sensor IMU, Limit, Encoder]
    ACT[Actuator Servo, Solenoid]

    CAM -->|USB Serial| RPI
    RPI -->|USB Serial| MEGA

    MEGA -->|I2C| STM1
    MEGA -->|I2C| STM2

    STM1 --> FL
    STM1 --> RL

    STM2 --> FR
    STM2 --> RR

    MEGA -->|GPIO / I2C / SPI| SENS
    MEGA -->|GPIO / PWM| ACT


```
### 📂 Struktur Folder & Penjelasan

- **LX26_R1_JoyStick_Controller_V1.0**  
  Versi awal pengembangan joystick controller sebagai antarmuka utama operator robot.

- **LX26_R1_JoyStick_Controller_V1.1**  
  Penyempurnaan joystick controller dengan optimasi respons, stabilitas input, dan struktur kode.


Repositori ini digunakan sebagai **basis riset internal dan pengembangan lanjutan** sistem LX25.
