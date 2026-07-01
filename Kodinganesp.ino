/**
 * ===================================================================================
 * FILE 1 (REAL): ESP32 PEMBACA METER (Modbus RTU) - VERSI INTEGRASI UTAMA
 * Membaca parameter listrik asli dari Meteran melalui protokol Modbus RTU (RS485)
 * lalu meneruskannya ke LoRa Aurora Transmitter via Serial1 (UART).
 * ===================================================================================
 * WIRING ESP32 ↔ MAX485 (Modbus ke Meter):
 * MAX485    ESP32
 * ──────    ─────
 * RO    →   GPIO16 (RX2)
 * DI    ←   GPIO17 (TX2)
 * RE+DE →   GPIO4
 * VCC   →   5V (Sangat disarankan memakai 5V luar agar tegangan IC RS485 stabil)
 * GND   →   GND
 * A     →   Terminal A meteran listrik
 * B     →   Terminal B meteran listrik
 *
 * WIRING ESP32 ↔ LoRa Aurora Transmitter (via Serial1):
 * ESP32 Meter        Aurora TX Board
 * ───────────        ───────────────────────────
 * GPIO33 (TX1)  →   Pin 1  = RX0 (GPIO3 Aurora)
 * GPIO32 (RX1)  ←   Pin 3  = TX0 (GPIO1 Aurora)  ← Opsional
 * GND           →   GND Aurora
 * ===================================================================================
 */

#include <Arduino.h>

// ────────── PIN MODBUS MAX485 ──────────
#define RO_PIN    16    // RX2 ESP32 (Menerima data dari MAX485)
#define DI_PIN    17    // TX2 ESP32 (Mengirim perintah ke MAX485)
#define DIR_PIN   4     // DE+RE MAX485 (Kontrol arah komunikasi Half-Duplex)

// ────────── PIN SERIAL KE AURORA TX ──────────
#define LORA_RX_PIN  32   // RX1 ESP32 (Opsional)
#define LORA_TX_PIN  33   // TX1 ESP32 → terhubung ke RX0 Aurora TX (Pin 1)

// ────────── PARAMETER MODBUS METERAN ──────────
#define SLAVE_ID    1     // Slave ID Meteran (Default: 1, sesuaikan setting di alat)
#define BAUD_RATE   9600  // Baud rate komunikasi Modbus (Default: 9600)

// ────────── INTERVAL ──────────
#define READ_INTERVAL_MS   2000  // Membaca meteran setiap 2 detik
#define SEND_INTERVAL_MS   2000  // Mengirim data ke LoRa setiap 2 detik

// ────────── REGISTER ADDRESS METERAN (MODBUS MAP) ──────────
#define REG_UA     9
#define REG_UB     11
#define REG_UC     13
#define REG_IA     15
#define REG_IB     17
#define REG_IC     19
#define REG_IAVG   21
#define REG_PS     23
#define REG_QS     25
#define REG_PFS    27
#define REG_HZ     29
#define REG_T1     31
#define REG_T2     33
#define REG_PA     39
#define REG_PB     41
#define REG_PC     43
#define REG_PFA    59
#define REG_PFB    61
#define REG_PFC    63
#define REG_UAB    65
#define REG_UBC    67
#define REG_UCA    69
#define REG_SS     57

// ────────── STRUKTUR DATA PARAMETER METERAN ──────────
struct MeterData {
  float Ua, Ub, Uc;
  float Uab, Ubc, Uca;
  float Ia, Ib, Ic, Iavg;
  float Pa, Pb, Pc;
  float PFa, PFb, PFc;
  float Ps, Qs, Ss;
  float PFs;
  float Hz;
  float T1, T2;
  bool  valid;
  int   errors;
};

MeterData meter = {0};
unsigned long lastRead = 0;
unsigned long lastSend = 0;

// ============================================================
//  FUNGSI HITUNG CRC16 (MODBUS STANDARD)
// ============================================================
uint16_t calcCRC(uint8_t *d, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;
}

// ============================================================
//  FUNGSI BACA 1 DATA FLOAT (2 REGISTERS/32-BIT) DENGAN FC 0x03
// ============================================================
float readFloat(uint16_t wordAddr) {
  // Bersihkan buffer sisa di port serial
  while (Serial2.available()) Serial2.read();

  // Konstruksi Request Frame Modbus RTU
  uint8_t req[8];
  req[0] = SLAVE_ID;
  req[1] = 0x03;                       // Function Code 0x03 (Read Holding Registers)
  req[2] = (wordAddr >> 8) & 0xFF;     // Alamat Register Tinggi
  req[3] = wordAddr & 0xFF;            // Alamat Register Rendah
  req[4] = 0x00;                       // Jumlah Register Tinggi
  req[5] = 0x02;                       // Jumlah Register Rendah (2 registers = 32-bit float)
  uint16_t crc = calcCRC(req, 6);
  req[6] = crc & 0xFF;                 // CRC Low
  req[7] = (crc >> 8) & 0xFF;          // CRC High

  // Aktifkan mode KIRIM pada IC MAX485 (DE + RE = HIGH)
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(200);
  Serial2.write(req, 8);
  Serial2.flush();
  delayMicroseconds(200);
  
  // Kembalikan ke mode TERIMA pada IC MAX485 (DE + RE = LOW)
  digitalWrite(DIR_PIN, LOW);

  // Tunggu respon dengan timeout maksimal 1000ms
  uint8_t rx[16];
  int len = 0;
  uint32_t t = millis();
  while (millis() - t < 50 && len < 16) {
    if (Serial2.available()) {
      rx[len++] = Serial2.read();
      t = millis();
    }
  }

  // Validasi panjang data minimum (9 bytes untuk respon 2 registers)
  if (len < 9)                             return NAN;
  if (rx[0] != SLAVE_ID || rx[1] != 0x03)  return NAN;
  if (rx[2] != 0x04)                       return NAN; // Byte count harus 4 bytes data

  // Validasi keamanan checksum CRC16
  uint16_t rxCRC  = (uint16_t)rx[len-1] << 8 | rx[len-2];
  uint16_t calCRC = calcCRC(rx, len - 2);
  if (rxCRC != calCRC)                     return NAN;

  // Konversi byte array ke tipe data float asli
  union { uint8_t b[4]; float f; } u;
  u.b[3] = rx[3]; u.b[2] = rx[4];
  u.b[1] = rx[5]; u.b[0] = rx[6];
  return u.f;
}

// ============================================================
//  FUNGSI SEKUENSAL BACA SEMUA PARAMETER DARI METERAN
// ============================================================
void readAll() {
  meter.errors = 0;

  // Makro pembacaan register untuk menyederhanakan kode
  #define RD(var, reg) { \
    var = readFloat(reg); \
    if (isnan(var)) { var = 0.0f; meter.errors++; } \
    delay(15); /* Berikan jeda waktu antar-request agar IC RS485 tidak panas */ \
  }

  // Lakukan pembacaan fasa tegangan, arus, daya, dan frekuensi
  RD(meter.Ua,  REG_UA)
  RD(meter.Ub,  REG_UB)
  RD(meter.Uc,  REG_UC)
  RD(meter.Uab, REG_UAB)
  RD(meter.Ubc, REG_UBC)
  RD(meter.Uca, REG_UCA)
  RD(meter.Ia,   REG_IA)
  RD(meter.Ib,   REG_IB)
  RD(meter.Ic,   REG_IC)
  RD(meter.Iavg, REG_IAVG)
  RD(meter.Ps,  REG_PS)
  RD(meter.Qs,  REG_QS)
  RD(meter.Ss,  REG_SS)
  RD(meter.PFs, REG_PFS)
  RD(meter.Hz,  REG_HZ)
  RD(meter.Pa,  REG_PA)
  RD(meter.Pb,  REG_PB)
  RD(meter.Pc,  REG_PC)
  RD(meter.PFa, REG_PFA)
  RD(meter.PFb, REG_PFB)
  RD(meter.PFc, REG_PFC)
  RD(meter.T1,  REG_T1)
  RD(meter.T2,  REG_T2)

  #undef RD

  // Dianggap valid jika total eror query register di bawah limit maksimal (23 parameter)
  meter.valid = (meter.errors < 23);
}

// ============================================================
//  FUNGSI MENGIRIM DATA CSV KE AURORA TRANSMITTER (SERIAL1)
// ============================================================
void sendToLora() {
  if (!meter.valid) {
    Serial.println("[LORA] Modbus offline / Data tidak valid. Menggagalkan pengiriman paket.");
    return;
  }

  char pkt[220];
  snprintf(pkt, sizeof(pkt),
    "METER,"
    "%.2f,%.2f,%.2f," // Tegangan: Ua, Ub, Uc
    "%.3f,%.3f,%.3f," // Arus: Ia, Ib, Ic
    "%.2f,%.2f,%.4f," // Ps, Qs, PFs
    "%.2f,"           // Frekuensi (Hz)
    "%.3f,%.3f\n",    // Total kWh: T1, T2
    meter.Ua, meter.Ub, meter.Uc,
    meter.Ia, meter.Ib, meter.Ic,
    meter.Ps, meter.Qs, meter.PFs,
    meter.Hz,
    meter.T1, meter.T2
  );

  // Kirim data serial asli ke modul transmitter Aurora V3
  Serial1.print(pkt);

  // Cetak bukti pengiriman ke terminal PC lokal untuk debug
  Serial.print("[KIRIM LORA TX] paket sukses: ");
  Serial.print(pkt);
}

// ============================================================
//  FUNGSI MENAMPILKAN DATA LOKAL DI SERIAL MONITOR LAPTOP
// ============================================================
void printData() {
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════╗");
  Serial.println("║       MULTIPLE ELECTRIC METER - 3 PHASE 4-WIRE      ║");
  Serial.printf( "║  Slave ID: %-3d   Baud: %-5d   Eror: %-2d/23            ║\n",
                 SLAVE_ID, BAUD_RATE, meter.errors);
  Serial.println("╚══════════════════════════════════════════════════════╝");

  if (!meter.valid) {
    Serial.println("  ✗ GAGAL MENYAMBUNG KE METERAN! Periksa kabel A/B RS485.");
    return;
  }

  Serial.println("  ┌─ TEGANGAN FASA ──────────────────────────────────┐");
  Serial.printf( "  │  Ua:%7.2fV  Ub:%7.2fV  Uc:%7.2fV         │\n", meter.Ua, meter.Ub, meter.Uc);
  Serial.println("  ├─ ARUS INDUKSI ───────────────────────────────────┤");
  Serial.printf( "  │  Ia:%7.3fA  Ib:%7.3fA  Ic:%7.3fA         │\n", meter.Ia, meter.Ib, meter.Ic);
  Serial.println("  ├─ BEBAN DAYA AKTIF & REAKTIF ─────────────────────┤");
  Serial.printf( "  │  Ps:%9.2fW   Qs:%9.2fVAR  PFs:%.4f     │\n", meter.Ps, meter.Qs, meter.PFs);
  Serial.println("  ├─ METRIK FREKUENSI ───────────────────────────────┤");
  Serial.printf( "  │  Hz:%7.2f    T1:%10.3fkWh  T2:%9.3fkWh  │\n", meter.Hz, meter.T1, meter.T2);
  Serial.println("  └──────────────────────────────────────────────────┘");
}

// ============================================================
//  SETUP UTAMA
// ============================================================
void setup() {
  Serial.begin(115200); // Port komunikasi serial ke monitor laptop
  delay(500);

  // Serial2 → Pin kontrol arah MAX485
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); // Set mode awal ke Terima data (LOW)
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RO_PIN, DI_PIN);

  // Serial1 → Koneksi fisik ke Pin 1 RX Aurora TX
  Serial1.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  Serial.println("╔══════════════════════════════════════════════════════╗");
  Serial.println("║       FIRMWARE REAL METER READER AKTIF (MODBUS)     ║");
  Serial.println("╚══════════════════════════════════════════════════════╝\n");

  delay(1000);
}

// ============================================================
//  LOOP UTAMA
// ============================================================
void loop() {
  unsigned long now = millis();

  // Timer pembacaan meteran secara berkala
  if (now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;
    readAll();
    printData();
  }

  // Timer pengiriman data ke modul LoRa
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    sendToLora();
  }
}