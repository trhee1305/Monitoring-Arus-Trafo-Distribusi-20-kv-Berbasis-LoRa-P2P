/**

 * ===================================================================================
 * FILE 2: LoRa AURORA V3 - TRANSMITTER SIDE (VERSI FIX FINAL)
 * Board   : LoRa Aurora V3 (ESP32 + RFM95W via SPI internal PCB)
 * Library : LoRa by Sandeep Mistry
 * ===================================================================================
 */

#include <SPI.h>
#include <LoRa.h>

// ─── PIN SPI INTERNAL Aurora V3 ──────────────────────────────
#define PIN_NSS   13
#define PIN_RST   16
#define PIN_DIO0  27
#define PIN_EN    15    // ENABLE RFM95W: Active LOW
#define LED_PIN   2     // LED onboard

// ─── FREKUENSI LORA ──────────────────────────────────────────
#define LORA_FREQ   915E6   // 915 MHz

// ─── BUFFER UART ─────────────────────────────────────────────
String rxBuffer  = "";
uint32_t packetCount = 0;

// ============================================================
//  KEDIP LED DEBUG
// ============================================================
void blinkLED(int n, int onMs = 80, int offMs = 120) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH); delay(onMs);
    digitalWrite(LED_PIN, LOW);  delay(offMs);
  }
}

// ============================================================
//  INISIALISASI LoRa
// ============================================================
void initLoRa() {
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW); // Aktifkan modul RFM95W
  delay(100);

  LoRa.setPins(PIN_NSS, PIN_RST, PIN_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    while (1) {
      blinkLED(5, 60, 60);
      delay(400);
    }
  }

  // Parameter Radio - Wajib sama presisi dengan Receiver
  LoRa.setSpreadingFactor(9);          // SF9
  LoRa.setSignalBandwidth(125E3);      // 125 kHz
  LoRa.setCodingRate4(5);              // CR 4/5
  LoRa.setTxPower(20);                 // 20 dBm (Daya Pancar Maksimal)
  LoRa.setSyncWord(0xF3);              // Sync Word Custom mencegah interferensi

  blinkLED(2, 400, 200); // Sukses Init: 2 kedip panjang
}

// ============================================================
//  KIRIM PAKET VIA LoRa
// ============================================================
bool sendPacket(const String &data) {
  if (data.length() == 0 || data.length() > 200) {
    blinkLED(3);
    return false;
  }

  LoRa.beginPacket();
  LoRa.print(data);
  bool ok = LoRa.endPacket();
  packetCount++;

  if (ok) {
    blinkLED(1);  // 1 kedip = Sukses Kirim Sinyal
  } else {
    blinkLED(3);  // 3 kedip = Gagal Kirim Sinyal
  }

  return ok;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Serial0 memakai Pin 1 (RX0) untuk komunikasi dari ESP32 Meter
  Serial.begin(9600);
  delay(500);
  while (Serial.available()) Serial.read(); // Bersihkan buffer awal

  initLoRa();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      rxBuffer.trim();

      if (rxBuffer.startsWith("METER,")) {
        sendPacket(rxBuffer);
      }
      rxBuffer = "";
    } else {
      rxBuffer += c;
      if (rxBuffer.length() > 210) {
        rxBuffer = ""; // Pengaman Buffer Overflow
      }
    }
  }
}