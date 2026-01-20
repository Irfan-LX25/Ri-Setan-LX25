/*---------------------------------------------------------------------------------------------------------*/
/*----------------------REAL-TIME PID PARAMETER TUNING & MOTOR CALIBRATION TOOL----------------------------*/
/*--------------------INCLUDING INTERNAL ENCODER FEEDBACK WITH PID CLOSED LOOP FEEDBACK--------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-24 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-24 TEAM---------------------------------*/
/*----------------------------------------------------V1.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 22:50:00, 6 JAN 26------------------------------------*/

/* * PETUNJUK PENGGUNAAN (TUNING GUIDE):
 * 1. Upload program ini ke Arduino Mega.
 * 2. Pastikan robot digantung (roda tidak menyentuh lantai).
 * 3. Buka 'Serial Plotter' di Arduino IDE (Ctrl+Shift+L) dengan Baudrate 115200.
 * 4. Grafik Biru = Target RPM, Grafik Merah = Actual RPM.
 * 5. Ketik perintah di bawah ini pada kolom Serial untuk mengubah nilai secara real-time:
 * - "P1.5"  -> Mengubah Kp menjadi 1.5
 * - "I0.2"  -> Mengubah Ki menjadi 0.2
 * - "D0.05" -> Mengubah Kd menjadi 0.05
 * - "M1"    -> Pindah tuning ke Motor 1
 */

// ===================================================================================
// 1. DEFINISI PIN & MAPPING HARDWARE (UPDATED)
// ===================================================================================
// Konfigurasi Pin Motor (3 Pin Control: IN_A, IN_B, PWM) & Encoder
// Sesuai request user:

// --- Motor 1 (Front Right / Kanan Depan) ---
// User Label: FR
uint8_t M1_IN_A = 43;
uint8_t M1_IN_B = 45;
uint8_t M1_PWM  = 5;
uint8_t M1_ENC_A = 21; // Interrupt Pin
uint8_t M1_ENC_B = 24;

// --- Motor 2 (Front Left / Kiri Depan) ---
// User Label: FL
uint8_t M2_IN_A = 49;
uint8_t M2_IN_B = 47;
uint8_t M2_PWM  = 4;
uint8_t M2_ENC_A = 20; // Interrupt Pin
uint8_t M2_ENC_B = 22;

// --- Motor 3 (Rear Left / Kiri Belakang) ---
// User Label: RL
uint8_t M3_IN_A = 41;
uint8_t M3_IN_B = 39;
uint8_t M3_PWM  = 6;
uint8_t M3_ENC_A = 2;  // Interrupt Pin
uint8_t M3_ENC_B = 26;

// --- Motor 4 (Rear Right / Kanan Belakang) ---
// User Label: RR
uint8_t M4_IN_A = 35;
uint8_t M4_IN_B = 37;
uint8_t M4_PWM  = 7;
uint8_t M4_ENC_A = 3;  // Interrupt Pin
uint8_t M4_ENC_B = 28;

// ===================================================================================
// 2. PARAMETER KONFIGURASI SISTEM
// ===================================================================================

// [PENTING] Kalibrasi Pulsa Encoder
// Rumus: (PPR Encoder Murni) x (Rasio Gearbox)
const float PULSES_PER_REV = 326.0; //PG45 = 17PPR @1:19.2 Ratio = 17x19.2 ~ 326

// ID Motor yang aktif saat start-up (Default: Motor 2 / FL)
uint8_t activeMotor = 2;

// Variabel PID Global (Nilai awal 0 agar aman saat start)
float Kp = 0.0; 
float Ki = 0.0;
float Kd = 0.0;

// Variabel Kontrol Uji Coba
float targetRPM = 0;              // Target kecepatan yang diminta
unsigned long intervalSwitch = 3000; // Durasi ganti target (ms) - Setiap 3 detik ganti speed

// ===================================================================================
// 3. VARIABEL OPERASIONAL (JANGAN UBAH BAGIAN INI)
// ===================================================================================
volatile long enc1=0, enc2=0, enc3=0, enc4=0; 

// Variabel memori untuk perhitungan PID
long prevEnc = 0;
unsigned long lastTime = 0;
unsigned long lastSwitch = 0;
float integral = 0;
float prevErr = 0;
unsigned long sampleTime = 50; // Sampling rate PID: 50ms (20Hz)

// ===================================================================================
// 4. SETUP PROGRAM
// ===================================================================================
void setup() {
  // Inisialisasi komunikasi serial untuk plotter & command
  Serial.begin(115200);
  
  // Konfigurasi Mode Pin Motor (3 Pin per Motor: A, B, PWM)
  pinMode(M1_IN_A, OUTPUT); pinMode(M1_IN_B, OUTPUT); pinMode(M1_PWM, OUTPUT);
  pinMode(M2_IN_A, OUTPUT); pinMode(M2_IN_B, OUTPUT); pinMode(M2_PWM, OUTPUT);
  pinMode(M3_IN_A, OUTPUT); pinMode(M3_IN_B, OUTPUT); pinMode(M3_PWM, OUTPUT);
  pinMode(M4_IN_A, OUTPUT); pinMode(M4_IN_B, OUTPUT); pinMode(M4_PWM, OUTPUT);

  // Konfigurasi Mode Pin Encoder
  pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
  pinMode(M3_ENC_A, INPUT_PULLUP); pinMode(M3_ENC_B, INPUT_PULLUP);
  pinMode(M4_ENC_A, INPUT_PULLUP); pinMode(M4_ENC_B, INPUT_PULLUP);

  // Mengaktifkan Interrupt Hardware
  // Pin 2, 3, 20, 21 semuanya support interrupt di Arduino Mega
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_ENC_A), isr4, RISING);

  // Pesan awal
  Serial.println("=== LEXARGA-24 PID TUNING MODE (UPDATED PINOUT) ===");
  Serial.println("System Ready. Open Serial Plotter now.");
  delay(2000); 
}

// ===================================================================================
// 5. LOOP UTAMA (MAIN PROGRAM)
// ===================================================================================
void loop() {
  
  // --- BAGIAN A: GENERATOR SINYAL UJI (STEP INPUT) ---
  if (millis() - lastSwitch > intervalSwitch) {
    if (targetRPM == 0) {
      targetRPM = 80.0; // Set Target ke 80 RPM
    } else {
      targetRPM = 0;    // Set Target ke 0 (Berhenti)
    }
    lastSwitch = millis();
    if (targetRPM == 0) integral = 0; 
  }

  // --- BAGIAN B: PARSING DATA SERIAL (INTERAKTIF) ---
  if (Serial.available()) {
    char cmd = Serial.read();      
    float val = Serial.parseFloat(); 
    
    if (cmd == 'P' || cmd == 'p') { Kp = val; integral=0; } 
    if (cmd == 'I' || cmd == 'i') { Ki = val; integral=0; } 
    if (cmd == 'D' || cmd == 'd') { Kd = val; integral=0; } 
    if (cmd == 'M' || cmd == 'm') { 
      activeMotor = (int)val;       
      stopAll();                    
      integral=0; prevErr=0;        
    }
  }

  // --- BAGIAN C: KALKULASI PID & KONTROL MOTOR ---
  if (millis() - lastTime >= sampleTime) {
    long currentTick;
    
    // Mengambil data encoder dari motor yang sedang aktif dipilih
    switch(activeMotor) {
      case 1: currentTick = enc1; break;
      case 2: currentTick = enc2; break;
      case 3: currentTick = enc3; break;
      case 4: currentTick = enc4; break;
      default: currentTick = 0;
    }

    // 1. Hitung Kecepatan Aktual (RPM Real-time)
    long delta = currentTick - prevEnc; 
    prevEnc = currentTick;
    float currentRPM = ((float)delta / PULSES_PER_REV) * (60000.0 / sampleTime);

    // 2. Algoritma PID
    float error = targetRPM - currentRPM; 
    
    integral += error * (sampleTime / 1000.0);
    integral = constrain(integral, -255, 255); 
    
    float derivative = (error - prevErr) / (sampleTime / 1000.0);
    prevErr = error; 

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // 3. Eksekusi ke Driver Motor
    driveMotor(activeMotor, output);

    // --- BAGIAN D: OUTPUT VISUALISASI DATA ---
    Serial.print("Target_RPM:"); Serial.print(targetRPM);
    Serial.print(","); 
    Serial.print("Actual_RPM:"); Serial.print(currentRPM);
    Serial.println(); 

    lastTime = millis(); 
  }
}

// ===================================================================================
// 6. FUNGSI PENDUKUNG (DRIVER 3 PIN & INTERRUPT)
// ===================================================================================

// Fungsi untuk menggerakkan motor (Support L298N / Driver 3 Pin)
void driveMotor(uint8_t id, int pwm) {
  pwm = constrain(pwm, -255, 255);
  uint8_t pinA, pinB, pinPWM;

  // Mapping ID motor ke Pin fisik
  if(id==1) { 
    pinA = M1_IN_A; pinB = M1_IN_B; pinPWM = M1_PWM; 
  } else if(id==2) { 
    pinA = M2_IN_A; pinB = M2_IN_B; pinPWM = M2_PWM; 
  } else if(id==3) { 
    pinA = M3_IN_A; pinB = M3_IN_B; pinPWM = M3_PWM; 
  } else { 
    pinA = M4_IN_A; pinB = M4_IN_B; pinPWM = M4_PWM; 
  }

  // Logika arah putaran 3 Pin (A, B, PWM)
  if(pwm > 0) { 
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, pwm);
  } else if(pwm < 0) { 
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    analogWrite(pinPWM, abs(pwm));
  } else { 
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, 0);
  }
}

// Fungsi darurat untuk menghentikan semua motor
void stopAll() {
  driveMotor(1, 0); driveMotor(2, 0); driveMotor(3, 0); driveMotor(4, 0);
}

// Interrupt Service Routines (ISR)
void isr1() { if(digitalRead(M1_ENC_B)==LOW) enc1++; else enc1--; }
void isr2() { if(digitalRead(M2_ENC_B)==LOW) enc2++; else enc2--; }
void isr3() { if(digitalRead(M3_ENC_B)==LOW) enc3++; else enc3--; }
void isr4() { if(digitalRead(M4_ENC_B)==LOW) enc4++; else enc4--; }
