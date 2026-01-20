// ==========================================
// 1. KONFIGURASI AWAL (USER SETTING)
// ==========================================
// Pilih Motor yang mau dites (1=FR, 2=FL, 3=RL, 4=RR)
const uint8_t ACTIVE_MOTOR = 2; 

// Masukkan estimasi PPR (Hanya dipakai untuk hitung RPM di Menu 2)
// Ganti nilai ini setelah Anda tahu nilai pastinya dari Menu 1.
float RUMUS_PPR = 326.0; 

// ==========================================
// 2. DEFINISI PIN (HEMAT MEMORI - uint8_t)
// ==========================================
// FR (Motor 1)
const uint8_t M1_A=43; 
const uint8_t M1_B=45; 
const uint8_t M1_PWM=5;
const uint8_t M1_EA=21;
const uint8_t M1_EB=24;
// FL (Motor 2)
const uint8_t M2_A=49; 
const uint8_t M2_B=47; 
const uint8_t M2_PWM=4;
const uint8_t M2_EA=20; 
const uint8_t M2_EB=22;
// RL (Motor 3)
const uint8_t M3_A=41; 
const uint8_t M3_B=39; 
const uint8_t M3_PWM=6;
const uint8_t M3_EA=2;  
const uint8_t M3_EB=26;
// RR (Motor 4)
const uint8_t M4_A=35; 
const uint8_t M4_B=37; 
const uint8_t M4_PWM=7;
const uint8_t M4_EA=3;  
const uint8_t M4_EB=28;

// ==========================================
// 3. VARIABEL GLOBAL
// ==========================================
volatile long encoderTicks = 0;
long prevTicks = 0;
unsigned long lastLoop = 0;
char mode = '0'; // 0=Stop, 1=Cek PPR, 2=Cek RPM

void setup() {
  Serial.begin(115200);

  // Setup Pin Motor (Output)
  pinMode(M1_A, OUTPUT); pinMode(M1_B, OUTPUT); pinMode(M1_PWM, OUTPUT);
  pinMode(M2_A, OUTPUT); pinMode(M2_B, OUTPUT); pinMode(M2_PWM, OUTPUT);
  pinMode(M3_A, OUTPUT); pinMode(M3_B, OUTPUT); pinMode(M3_PWM, OUTPUT);
  pinMode(M4_A, OUTPUT); pinMode(M4_B, OUTPUT); pinMode(M4_PWM, OUTPUT);

  // Setup Pin Encoder (Input Pullup)
  pinMode(M1_EA, INPUT_PULLUP); pinMode(M1_EB, INPUT_PULLUP);
  pinMode(M2_EA, INPUT_PULLUP); pinMode(M2_EB, INPUT_PULLUP);
  pinMode(M3_EA, INPUT_PULLUP); pinMode(M3_EB, INPUT_PULLUP);
  pinMode(M4_EA, INPUT_PULLUP); pinMode(M4_EB, INPUT_PULLUP);

  // Aktifkan Interrupt sesuai motor yang dipilih
  switch(ACTIVE_MOTOR) {
    case 1: attachInterrupt(digitalPinToInterrupt(M1_EA), isr, RISING); break;
    case 2: attachInterrupt(digitalPinToInterrupt(M2_EA), isr, RISING); break;
    case 3: attachInterrupt(digitalPinToInterrupt(M3_EA), isr, RISING); break;
    case 4: attachInterrupt(digitalPinToInterrupt(M4_EA), isr, RISING); break;
  }

  printMenu();
}

void loop() {
  // --- A. BACA INPUT SERIAL ---
  if (Serial.available()) {
    char input = Serial.read();
    
    // Abaikan karakter enter/spasi
    if(input == '\n' || input == '\r' || input == ' ') return;

    if (input == '1') {
      mode = '1';
      stopAll(); encoderTicks = 0;
      Serial.println("\n[MODE 1: CEK PPR AKTIF]");
      Serial.println("-> Silakan putar roda 1x putaran penuh dengan tangan.");
    }
    else if (input == '2') {
      mode = '2';
      stopAll(); encoderTicks = 0; prevTicks = 0;
      Serial.println("\n[MODE 2: CEK RPM AKTIF]");
      Serial.println("-> Motor akan berputar MAX SPEED dalam 2 detik...");
      delay(2000); // Safety delay
    }
    else {
      mode = '0';
      stopAll();
      Serial.println("\n[STOP]");
      printMenu();
    }
  }

  // --- B. EKSEKUSI SESUAI MODE ---
  
  // Update data setiap 500ms agar Serial tidak banjir
  if (millis() - lastLoop > 500) {
    
    // --- MODE 1: CEK PPR (MANUAL) ---
    if (mode == '1') {
      Serial.print("PULSA TERBACA: ");
      Serial.println(encoderTicks);
    }

    // --- MODE 2: CEK RPM (AUTO) ---
    else if (mode == '2') {
      driveMotor(ACTIVE_MOTOR, 255); // Jalankan Full Speed

      // Hitung RPM
      long currentDelta = encoderTicks - prevTicks;
      prevTicks = encoderTicks;
      
      // Rumus: (DeltaPulsa / PPR) * (60000ms / IntervalWaktu)
      float rpm = ((float)currentDelta / RUMUS_PPR) * (60000.0 / 500.0);
      
      Serial.print("PWM: 255 | RPM: ");
      Serial.println(abs(rpm));
    }

    lastLoop = millis();
  }
}

// ==========================================
// FUNGSI PENDUKUNG
// ==========================================

void printMenu() {
  Serial.println("\n=== MENU DIAGNOSTIK ===");
  Serial.print("Motor Terpilih: "); Serial.println(ACTIVE_MOTOR);
  Serial.println("Ketik '1' -> Cek PPR (Putar Manual)");
  Serial.println("Ketik '2' -> Cek RPM (Putar Otomatis)");
  Serial.println("Ketik 'x' -> STOP / Kembali ke Menu");
  Serial.println("=======================");
}

void stopAll() {
  // Matikan semua driver (Logic 3 Pin: A=LOW, B=LOW, PWM=0)
  digitalWrite(M1_A, 0); digitalWrite(M1_B, 0); analogWrite(M1_PWM, 0);
  digitalWrite(M2_A, 0); digitalWrite(M2_B, 0); analogWrite(M2_PWM, 0);
  digitalWrite(M3_A, 0); digitalWrite(M3_B, 0); analogWrite(M3_PWM, 0);
  digitalWrite(M4_A, 0); digitalWrite(M4_B, 0); analogWrite(M4_PWM, 0);
}

void driveMotor(uint8_t id, int pwm) {
  uint8_t pA, pB, pPWM;
  if(id==1) { pA=M1_A; pB=M1_B; pPWM=M1_PWM; }
  else if(id==2) { pA=M2_A; pB=M2_B; pPWM=M2_PWM; }
  else if(id==3) { pA=M3_A; pB=M3_B; pPWM=M3_PWM; }
  else { pA=M4_A; pB=M4_B; pPWM=M4_PWM; }

  // Arah Maju saja untuk tes RPM
  digitalWrite(pA, HIGH);
  digitalWrite(pB, LOW);
  analogWrite(pPWM, pwm); // PWM tidak perlu constrain disini karena input kita fix 255
}

// Interrupt Service Routine (ISR)
void isr() {
  uint8_t pinB;
  // Cek pin B motor yg aktif untuk arah (Quadrature decoding simpel)
  if(ACTIVE_MOTOR==1) pinB = M1_EB;
  else if(ACTIVE_MOTOR==2) pinB = M2_EB;
  else if(ACTIVE_MOTOR==3) pinB = M3_EB;
  else pinB = M4_EB;

  if (digitalRead(pinB) == LOW) encoderTicks++; else encoderTicks--;
}
