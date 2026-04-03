#include <SPI.h>
#include <SD.h>
#include <FastLED.h>
#include "SCH16T.h"
#include <PL_ADXL355.h>
#include <Adafruit_BNO08x.h>

// --- HARDWARE CONFIG & PINS ---
#define LED_PIN       23
#define NUM_LEDS      1

#define SCH_CS        5   
#define SCH_RESET     0   

#define ADXL_CS       10  
#define BNO_CS        15  
#define BNO_INT       16  
#define BNO_RESET     14  

#define SD_CS         BUILTIN_SDCARD

// --- OBJECTS ---
CRGB leds[NUM_LEDS];
SCH16T_K10 imu(SPI, SCH_CS, SCH_RESET);
PL::ADXL355 adxl355;
Adafruit_BNO08x bno08x(BNO_RESET);
File dataFile;

// --- SETTINGS & STATE ---
char fileName[] = "IMULOG00.CSV";
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 5000; 

// Flight State Machine
uint8_t currentPhase = 0; // Starts in 0: Standby
uint8_t previousPhase = 0;
bool phaseChanged = false; // Flag to trigger instant logging

unsigned long lastLowRateWrite = 0;
const unsigned long LOW_RATE_INTERVAL = 1000; // 1 Hz telemetry for Phases 0 and 4

// BNO08x Data Caches
float bno_ax = 0, bno_ay = 0, bno_az = 0;
float bno_gx = 0, bno_gy = 0, bno_gz = 0;
float bno_mx = 0, bno_my = 0, bno_mz = 0;

// --- COLOR-CODED ERROR HANDLER ---
void errorHalt(CRGB failColor, const char* msg) {
  Serial.println(msg);
  while(1) {
    leds[0] = failColor;
    FastLED.show();
    delay(200);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(200);
  }
}

void setup() {
  unsigned long init_start = micros();
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  leds[0] = CRGB::White;
  FastLED.show();

  // THE HARD CLAMP
  pinMode(SCH_CS, OUTPUT);  digitalWrite(SCH_CS, HIGH);
  pinMode(ADXL_CS, OUTPUT); digitalWrite(ADXL_CS, HIGH);
  pinMode(BNO_CS, OUTPUT);  digitalWrite(BNO_CS, HIGH);

  pinMode(13, OUTPUT); digitalWrite(13, LOW); 
  pinMode(11, OUTPUT); digitalWrite(11, LOW); 

  delay(1000); 

  // HARDWARE RESETS
  pinMode(SCH_RESET, OUTPUT);
  digitalWrite(SCH_RESET, LOW); delay(10); digitalWrite(SCH_RESET, HIGH); 
  
  pinMode(BNO_RESET, OUTPUT);
  digitalWrite(BNO_RESET, LOW); delay(10); digitalWrite(BNO_RESET, HIGH);

  delay(100); 

  // Initialize Serial (PC) and Serial5 (UART on Pins 20/21)
  Serial.begin(115200);
  Serial5.begin(9600); 
  while (!Serial && millis() < 3000) delay(10); 
  Serial5.println("MASA Sensor Payload Initializing...");

  leds[0] = CRGB::Blue; FastLED.show();

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  int sdRetries = 0;
  while (!SD.begin(SD_CS) && sdRetries < 5) {
    sdRetries++;
    delay(200); 
  }
  if (sdRetries >= 5) errorHalt(CRGB::Red, "SD Card failed!");
  
  for (uint8_t i = 0; i < 100; i++) {
    fileName[6] = i / 10 + '0';
    fileName[7] = i % 10 + '0';
    if (!SD.exists(fileName)) {
      dataFile = SD.open(fileName, FILE_WRITE);
      break;
    }
  }
  if (!dataFile) errorHalt(CRGB::Red, "Could not create file!"); 
  
  // Header Row
  dataFile.println("Phase,micros,SCH_Stat,SCH_GX,SCH_GY,SCH_GZ,SCH_AX,SCH_AY,SCH_AZ,SCH_Temp,ADXL_AX,ADXL_AY,ADXL_AZ,BNO_AX,BNO_AY,BNO_AZ,BNO_GX,BNO_GY,BNO_GZ,BNO_MX,BNO_MY,BNO_MZ");

  // Initialize SCH16T
  SPI.begin();
  SCH16T_filter Filter = {68, 68, 68};
  SCH16T_sensitivity Sensitivity = {200, 200, 3200, 3200, 3200};
  SCH16T_decimation Decimation = {4, 4};
  imu.begin(Filter, Sensitivity, Decimation, false);
  Serial.println("SCH16T Ready.");

  // Initialize ADXL359
  adxl355.beginSPI(ADXL_CS);
  adxl355.setRange(PL::ADXL355_Range::range2g); 
  adxl355.enableMeasurement();
  delay(50); 
  auto testRead = adxl355.getAccelerations();
  if (testRead.x == 0 && testRead.y == 0 && testRead.z == 0) errorHalt(CRGB::Yellow, "ADXL359 failed to start!"); 
  Serial.println("ADXL359 Ready.");

  // Initialize BNO08x
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT)) errorHalt(CRGB::Purple, "BNO08x failed to start!"); 
  
  bno08x.enableReport(SH2_ACCELEROMETER, 10000);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  Serial.println("BNO08x Ready.");

  Serial.print("Logging to: "); Serial.println(fileName);

  unsigned long init_time = (micros() - init_start);
  Serial5.print("Successfully Initialized in ");
  Serial5.print(init_time);
  Serial5.println("us");
}

void loop() {
  /*// --- 1. SAFE SHUTDOWN CHECK (PC Serial) ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      if (dataFile) { dataFile.flush(); dataFile.close(); }
      leds[0] = CRGB::Red; FastLED.show();
      Serial.println("FILE SAVED. SAFE TO POWER OFF.");
      while(1) delay(100); 
    }
  }*/

// --- 2. UART PHASE CONTROL (Serial5 - Pins 20/21) ---
  if (Serial5.available() > 0) {
    uint8_t incomingByte = Serial5.read();
    //Serial.println(incomingByte);
    uint8_t newPhase = currentPhase;
    
    // Accept raw bytes (0x00 to 0x04) OR ASCII ('0' to '4')
    if (incomingByte <= 4) {
      newPhase = incomingByte;
    } else if (incomingByte >= '0' && incomingByte <= '4') {
      newPhase = incomingByte - '0';
    }

    // Did the phase just change?
    if (newPhase != currentPhase) {
      previousPhase = currentPhase;
      currentPhase = newPhase;
      phaseChanged = true; // Set the flag to force an immediate SD write
      
      // Print to the native USB Serial
      //Serial.print("Transitioned to Phase: "); Serial.println(currentPhase);
      
      // NEW: Print back to the UART PuTTY terminal!
      //Serial5.print("ACK! Phase changed to: "); Serial5.println(currentPhase);
    } else {
      // NEW: Tell PuTTY if we received a character but it didn't change the phase
      // (Helpful to see if PuTTY is sending weird newline characters)
      //if (incomingByte >= 32 && incomingByte <= 126) {
        //Serial5.print("Received unused char: "); Serial5.println((char)incomingByte);
      //}
    }
  }

// --- 3. SENSOR READS (Constant polling to keep FIFOs clear) ---
  unsigned long currentMicros = micros();

  // Breadcrumb 1
  // Serial.println("1. Reading SCH16T...");
  SCH16T_raw_data raw;
  SCH16T_result sch_res;
  SCH16T_status sch_stat;
  imu.getData(&raw);
  imu.convertData(&raw, &sch_res);
  imu.getStatus(&sch_stat);

  // Breadcrumb 2
  // Serial.println("2. Reading ADXL...");
  auto adxl_res = adxl355.getAccelerations();

  // Breadcrumb 3
  // Serial.println("3. Reading BNO...");
  
  // FIX: Changed 'while' to 'if' to prevent infinite SPI lockups
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        bno_ax = sensorValue.un.accelerometer.x; bno_ay = sensorValue.un.accelerometer.y; bno_az = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        bno_gx = sensorValue.un.gyroscope.x; bno_gy = sensorValue.un.gyroscope.y; bno_gz = sensorValue.un.gyroscope.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        bno_mx = sensorValue.un.magneticField.x; bno_my = sensorValue.un.magneticField.y; bno_mz = sensorValue.un.magneticField.z;
        break;
    }
  }
  // Serial.println("4. Sensor reads finished!");

  // --- 4. RATE LIMITER LOGIC ---
  bool writeThisLoop = false;
  
  // If the phase just changed, ALWAYS force a write this loop
  if (phaseChanged) {
    writeThisLoop = true;
  } else if (currentPhase >= 1 && currentPhase <= 3) {
    writeThisLoop = true; // High Rate (Unthrottled)
  } else {
    // Low Rate (Phase 0 and 4): Write only every 1000ms
    if (millis() - lastLowRateWrite >= LOW_RATE_INTERVAL) {
      writeThisLoop = true;
      lastLowRateWrite = millis();
    }
  }

  // --- 5. SD WRITE & FLUSH ---
  if (writeThisLoop && dataFile) {
    dataFile.print(currentPhase); dataFile.print(","); 
    dataFile.print(currentMicros); dataFile.print(",");
    dataFile.print(sch_stat.Summary, HEX); dataFile.print(",");
    
    dataFile.print(sch_res.Rate1[0], 3); dataFile.print(",");
    dataFile.print(sch_res.Rate1[1], 3); dataFile.print(",");
    dataFile.print(sch_res.Rate1[2], 3); dataFile.print(",");
    dataFile.print(sch_res.Acc1[0], 3); dataFile.print(",");
    dataFile.print(sch_res.Acc1[1], 3); dataFile.print(",");
    dataFile.print(sch_res.Acc1[2], 3); dataFile.print(",");
    dataFile.print(sch_res.Temp, 2); dataFile.print(",");

    dataFile.print(adxl_res.x * 5.0, 4); dataFile.print(",");
    dataFile.print(adxl_res.y * 5.0, 4); dataFile.print(",");
    dataFile.print(adxl_res.z * 5.0, 4); dataFile.print(",");

    dataFile.print(bno_ax, 3); dataFile.print(",");
    dataFile.print(bno_ay, 3); dataFile.print(",");
    dataFile.print(bno_az, 3); dataFile.print(",");
    dataFile.print(bno_gx, 3); dataFile.print(",");
    dataFile.print(bno_gy, 3); dataFile.print(",");
    dataFile.print(bno_gz, 3); dataFile.print(",");
    dataFile.print(bno_mx, 3); dataFile.print(",");
    dataFile.print(bno_my, 3); dataFile.print(",");
    dataFile.println(bno_mz, 3); 

    // Aggressive Flush on Phase Change OR Time-based flush
    if (phaseChanged || millis() - lastFlushTime >= FLUSH_INTERVAL) {
      dataFile.flush();
      lastFlushTime = millis();
      phaseChanged = false; // Reset the flag AFTER the data is physically saved
      
      // Flash Blue to indicate physical save
      leds[0] = CRGB::Blue; 
      FastLED.show();
    }
  }

// --- 6. PHASE-AWARE VISUAL HEARTBEAT ---
  static uint8_t pulse = 50;
  static int8_t dir = 5;
  
  // NEW: Throttle the math so the Teensy doesn't run it at 600 MHz!
  EVERY_N_MILLISECONDS(20) {
    pulse += dir;
    if(pulse >= 150 || pulse <= 20) dir = -dir;
  }
  
  if (millis() - lastFlushTime > 50) { 
    // Set base color depending on Phase
    switch(currentPhase) {
      case 0: leds[0] = CRGB::Cyan; break;       // Standby
      case 1: leds[0] = CRGB::Yellow; break;     // Armed
      case 2: leds[0] = CRGB::DarkOrange; break; // Term
      case 3: leds[0] = CRGB::Red; break;        // Flight
      case 4: leds[0] = CRGB::Purple; break;     // Landed
      default: leds[0] = CRGB::White; break;     
    }
    
    // Apply heartbeat fade effect
    leds[0].fadeToBlackBy(255 - pulse); 
    FastLED.show();
  }
}