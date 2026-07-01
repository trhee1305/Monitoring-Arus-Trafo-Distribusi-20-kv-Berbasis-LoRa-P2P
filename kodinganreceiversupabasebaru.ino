/**
 * ===================================================================================
 * FILE 3: LoRa AURORA V3 - RECEIVER SIDE + SUPABASE (OPTIMASI INSTAN 2 DETIK)
 * Board   : LoRa Aurora V3 (ESP32 + RFM95W via SPI internal PCB)
 * Library : LoRa by Sandeep Mistry & ArduinoJson
 * ===================================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─── KONEKSI INTERNET WIFI ───────────────────────────────────
#define WIFI_SSID       "trhee"       
#define WIFI_PASSWORD   "1234567890"   

// ─── KREDENSI SUPABASE REST API ──────────────────────────────
const char* SUPABASE_URL = "https://ososnpeixtxejskfpceh.supabase.co";
const char* SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Im9zb3NucGVpeHR4ZWpza2ZwY2VoIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODEzMjQ3MTYsImV4cCI6MjA5NjkwMDcxNn0.N4MTTTAvqBwYSnNKuSUk0Dt6jzMjgMHzEQ_Okq2BN6c";

// ─── PIN SPI INTERNAL Aurora V3 ──────────────────────────────
#define PIN_NSS   13
#define PIN_RST   16
#define PIN_DIO0  27
#define PIN_EN    15    // ENABLE RFM95W: Active LOW
#define LED_PIN   2     // LED onboard

// ─── FREKUENSI LORA ──────────────────────────────────────────
#define LORA_FREQ   915E6

// ─── STRUKTUR DATA METER ─────────────────────────────────────
struct MeterData {
  float Ua, Ub, Uc;
  float Ia, Ib, Ic;
  float Ps, Qs;
  float PFs;
  float Hz;
  float T1, T2;
  int   rssi;
  bool  valid;
  unsigned long lastUpdate;
};

MeterData meter = {0};
uint32_t packetCount = 0;
uint32_t errorCount  = 0;

// ─── VARIABEL TAMBAHAN METRIK JARINGAN LoRa ──────────────────
unsigned long lastRxTime = 0;
uint32_t totalRx = 0;
uint32_t totalExpected = 0;
float throughputBps = 0.0;
float packetLossPct = 0.0;
unsigned long latencyMs = 0;

// ============================================================
//  FUNGSI KEDIP LED
// ============================================================
void blinkLED(int n, int onMs = 80, int offMs = 120) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);  
    delay(offMs);
  }
}

// ============================================================
//  KONEKSI WIFI
// ============================================================
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] Menghubungkan ke WiFi");
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    Serial.print(".");
    blinkLED(1, 50, 450);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Terhubung ONLINE!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
    blinkLED(2, 200, 100);
  } else {
    Serial.println("\n[WiFi] Gagal terhubung (Status: OFFLINE).");
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
    Serial.println("[ERROR] LoRa init GAGAL!");
    while (1) {
      blinkLED(5, 60, 60);
      delay(400);
    }
  }

  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0xF3);
  Serial.println("[LoRa] Init OK");
}

// ============================================================
//  PROSES UPLOAD DATA KE SUPABASE (HTTP POST JSON) - OPTIMIZED
// ============================================================
void uploadToSupabase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Supabase] Lewati upload, WiFi berstatus offline.");
    return;
  }

  HTTPClient http;
  String endpoint = String(SUPABASE_URL) + "/rest/v1/data_trafo";
  
  http.begin(endpoint);
  
  // ─── OPTIMASI KECEPATAN INTERNET ───
  http.setReuse(true);       // Menggunakan koneksi TCP yang sama (tidak perlu jabat tangan ulang)
  http.setTimeout(1500);     // Batas waktu tunggu internet maksimal 1.5 detik agar kodingan tidak macet

  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));

  StaticJsonDocument<300> doc;
  doc["arus_r"] = meter.Ia;
  doc["arus_s"] = meter.Ib;
  doc["arus_t"] = meter.Ic;
  doc["teg_r"]  = meter.Ua;
  doc["teg_s"]  = meter.Ub;
  doc["teg_t"]  = meter.Uc;
  doc["rssi"]   = meter.rssi;

  String jsonPayload;
  serializeJson(doc, jsonPayload);

  Serial.println("[Supabase] Mengirim data ke tabel data_trafo...");
  int httpResponseCode = http.POST(jsonPayload);
  if (httpResponseCode >= 200 && httpResponseCode < 300) {
    Serial.print("[Supabase] Berhasil mengunggah data! Response: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("[Supabase] Upload terputus / Timeout! Code: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

// ============================================================
//  PARSE CSV DATA LoRa
// ============================================================
bool parsePacket(const String &raw, int rssi) {
  if (!raw.startsWith("METER,")) return false;
  char buf[210];
  raw.toCharArray(buf, sizeof(buf));

  char *tok = strtok(buf, ",");  // "METER"
  if (!tok) return false;

  float vals[12];
  int idx = 0;
  while ((tok = strtok(NULL, ",")) != NULL && idx < 12) {
    vals[idx++] = atof(tok);
  }
  if (idx < 12) return false;

  meter.Ua  = vals[0];
  meter.Ub  = vals[1];  
  meter.Uc  = vals[2];
  meter.Ia  = vals[3]/1000.0;
  meter.Ib  = vals[4]/1000.0;  
  meter.Ic  = vals[5]/1000.0;
  meter.Ps  = vals[6];
  meter.Qs  = vals[7];
  meter.PFs = vals[8];
  meter.Hz  = vals[9];
  meter.T1  = vals[10]; 
  meter.T2  = vals[11];
  meter.rssi       = rssi;
  meter.valid      = true;
  meter.lastUpdate = millis();
  return true;
}

// ============================================================
//  PRINT LOKAL DEBUG
// ============================================================
void printData() {
  Serial.println("\n====================== PAKET BARU MASUK ======================");
  Serial.printf( "  RSSI         : %d dBm\n", meter.rssi);
  Serial.printf( "  Tegangan     : Ua = %.1fV | Ub = %.1fV | Uc = %.1fV\n", meter.Ua, meter.Ub, meter.Uc);
  Serial.printf( "  Arus         : Ia = %.3fA | Ib = %.3fA | Ic = %.3fA\n", meter.Ia, meter.Ib, meter.Ic);
  Serial.println("  ------------------------------------------------------------");
  Serial.printf( "  Throughput   : %.2f bps\n", throughputBps);
  Serial.printf( "  Latensi      : %lu ms (Waktu Eksekusi + ToA)\n", latencyMs);
  Serial.printf( "  Packet Loss  : %.2f %% (%d/%d diterima)\n", packetLossPct, totalRx, totalExpected);
  Serial.println("===============================================================");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initWiFi();
  initLoRa();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    unsigned long startRead = millis();
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    received.trim();
    unsigned long endRead = millis();

    // 1. KALKULASI LATENSI
    latencyMs = (endRead - startRead) + 380; 

    // 2. KALKULASI INTERVAL & THROUGHPUT
    unsigned long now = millis();
    unsigned long interval = now - lastRxTime;
    if (lastRxTime == 0 || interval > 10000) interval = 2000;
    lastRxTime = now;
    
    throughputBps = (packetSize * 8.0) / (interval / 1000.0);

    // 3. KALKULASI PACKET LOSS
    totalRx++;
    if (totalExpected == 0) {
      totalExpected = 1;
    } else {
       int missed = round((interval - 2000.0) / 2000.0);
       if (missed > 0) totalExpected += missed;
       totalExpected++;
    }
    packetLossPct = ((float)(totalExpected - totalRx) / totalExpected) * 100.0;

    int rssi = LoRa.packetRssi();
    blinkLED(1, 100, 0);

    if (parsePacket(received, rssi)) {
      packetCount++;
      printData();
      uploadToSupabase();
    } else {
      errorCount++;
      Serial.println("[RX] ERROR: format paket tidak dikenal.");
    }
  }
}