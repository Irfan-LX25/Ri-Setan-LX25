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

## 📙 Dokumentasi Riset Program LX25 (Repository Ini)

🔗 **Repository Utama LX25**  
https://github.com/Irfan-LX25/Ri-Setan-LX25  

Repositori ini merupakan **pengembangan lanjutan dan riset internal** sistem kontrol berbasis **LX25 Controller**, dengan fokus pada modularitas, kestabilan, dan fleksibilitas penggunaan pada robot kompetisi.

### 📂 Modul yang Tersedia

#### 🕹️ JoyStick Controller
- `LX26_R1_JoyStick_Controller_V1.0`
- `LX26_R1_JoyStick_Controller_V1.1`

**Fungsi:**
- Pembacaan input joystick
- Mapping nilai analog → sinyal kontrol
- Optimasi respons kontrol operator
- Struktur kode modular untuk pengembangan lanjutan

---

#### 🔁 Dual Mode Plug & Play
- `LX26_R2_DualMode_PnP_V1.0`

**Fungsi:**
- Mode operasi ganda (Manual / PnP)
- Switching mode tanpa reset sistem
- Digunakan untuk fleksibilitas strategi robot di lapangan

---

#### 🤖 Motion Control
- `SwerveDrive`

**Fungsi:**
- Implementasi logika **swerve drive**
- Pengaturan arah dan kecepatan roda
- Cocok untuk robot omni-directional

---

#### ⚙️ PID Control Motor DC
- `Tuning_PID_DC_MOTOR`

**Fungsi:**
- Riset tuning **PID motor DC**
- Menggunakan feedback **encoder internal**
- Analisis respon sistem (overshoot, settling time, error)
- Digunakan sebagai dasar pengembangan motion control

---

## 🎯 Tujuan Riset LX25

- Mengembangkan sistem kontrol robot yang **presisi & stabil**
- Menyediakan basis riset untuk **kompetisi robotika**
- Menjadi referensi internal pengembangan controller LX-series
- Mempermudah debugging dan pengujian lapangan

---

## 🧪 Catatan

⚠️ Seluruh kode dalam repositori ini bersifat:
- **Research-oriented**
- Digunakan untuk eksperimen dan pengujian
- Dapat berubah sesuai kebutuhan riset dan kompetisi

---

<div align="center">

### 🔧 Built for Robotics Research & Competition

</div>
