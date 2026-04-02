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

#define ADXL_CS       10  // From your ADXL script

#define BNO_CS        15  // From your SCH script's "keep quiet" pin
#define BNO_INT       16  // From your BNO script
#define BNO_RESET     14  // BNO physical reset pin

#define SD_CS         BUILTIN_SDCARD

// --- OBJECTS ---
CRGB leds[NUM_LEDS];
SCH16T_K10 imu(SPI, SCH_CS, SCH_RESET);
PL::ADXL355 adxl355;
Adafruit_BNO08x bno08x(BNO_RESET);
File dataFile;

// --- SETTINGS ---
char fileName[] = "IMULOG00.CSV";
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 5000; // Flush every 5 seconds

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
  // 1. NeoPixel Init
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  leds[0] = CRGB::White;
  FastLED.show();

  // 2. THE HARD CLAMP
  // Force all Chip Selects HIGH (Off) before doing anything else
  pinMode(SCH_CS, OUTPUT);  digitalWrite(SCH_CS, HIGH);
  pinMode(ADXL_CS, OUTPUT); digitalWrite(ADXL_CS, HIGH);
  pinMode(BNO_CS, OUTPUT);  digitalWrite(BNO_CS, HIGH);

  // Force SPI clock and data lines LOW to prevent power-up noise triggers
  pinMode(13, OUTPUT); digitalWrite(13, LOW); // SCK
  pinMode(11, OUTPUT); digitalWrite(11, LOW); // MOSI

  // Wait 1 second for the external power supply to physically stabilize
  delay(1000); 

  // 3. HARDWARE RESETS
  // SCH16T Reset
  pinMode(SCH_RESET, OUTPUT);
  digitalWrite(SCH_RESET, LOW); delay(10); digitalWrite(SCH_RESET, HIGH); 
  
  // BNO08x Reset (Crucial for cold-boot SPI mode)
  pinMode(BNO_RESET, OUTPUT);
  digitalWrite(BNO_RESET, LOW); delay(10); digitalWrite(BNO_RESET, HIGH);

  // Let them wake up from reset
  delay(100); 

  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10); 

  // Status: Blue = Initializing
  leds[0] = CRGB::Blue; FastLED.show();

  // 4. Initialize SD Card (With Retry for External Power Supplies)
  Serial.print("Initializing SD card...");
  int sdRetries = 0;
  // Try to mount the SD card up to 5 times before failing
  while (!SD.begin(SD_CS) && sdRetries < 5) {
    sdRetries++;
    delay(200); 
  }
  if (sdRetries >= 5) errorHalt(CRGB::Red, "SD Card failed!");
  
  // Create File
  for (uint8_t i = 0; i < 100; i++) {
    fileName[6] = i / 10 + '0';
    fileName[7] = i % 10 + '0';
    if (!SD.exists(fileName)) {
      dataFile = SD.open(fileName, FILE_WRITE);
      break;
    }
  }
  if (!dataFile) errorHalt(CRGB::Red, "Could not create file!"); // RED Error
  
  // Header Row
  dataFile.println("micros,SCH_Stat,SCH_GX,SCH_GY,SCH_GZ,SCH_AX,SCH_AY,SCH_AZ,SCH_Temp,ADXL_AX,ADXL_AY,ADXL_AZ,BNO_AX,BNO_AY,BNO_AZ,BNO_GX,BNO_GY,BNO_GZ,BNO_MX,BNO_MY,BNO_MZ");

  // 5. Initialize SCH16T
  SPI.begin();
  SCH16T_filter Filter = {68, 68, 68};
  SCH16T_sensitivity Sensitivity = {200, 200, 3200, 3200, 3200};
  SCH16T_decimation Decimation = {4, 4};
  imu.begin(Filter, Sensitivity, Decimation, false);
  Serial.println("SCH16T Ready.");

  // 6. Initialize ADXL (Actually 359)
  adxl355.beginSPI(ADXL_CS);
  adxl355.setRange(PL::ADXL355_Range::range2g); // This actually sets +/- 10g on the 359
  adxl355.enableMeasurement();
  
  // THE FIX: Wait 50ms for the ADXL to calculate its very first reading!
  delay(50); 
  
  auto testRead = adxl355.getAccelerations();
  if (testRead.x == 0 && testRead.y == 0 && testRead.z == 0) {
    errorHalt(CRGB::Yellow, "ADXL359 failed to start!"); 
  }
  Serial.println("ADXL359 Ready.");

  // 7. Initialize BNO08x (using SPI)
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT)) {
    errorHalt(CRGB::Purple, "BNO08x failed to start!"); // PURPLE Error
  }
  
  // Set desired BNO reports
  bno08x.enableReport(SH2_ACCELEROMETER, 10000);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  Serial.println("BNO08x Ready.");

  // Success!
  leds[0] = CRGB::Green; FastLED.show();
  Serial.print("Logging to: "); Serial.println(fileName);
}

void loop() {
  // --- 1. SAFE SHUTDOWN CHECK ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      Serial.println("Stop command received. Saving file...");
      if (dataFile) {
        dataFile.flush();
        dataFile.close();
      }
      leds[0] = CRGB::Red; 
      FastLED.show();
      Serial.println("FILE SAVED. SAFE TO POWER OFF.");
      while(1) delay(100); 
    }
  }

  unsigned long currentMicros = micros();

  // --- 2. READ SCH16T ---
  SCH16T_raw_data raw;
  SCH16T_result sch_res;
  SCH16T_status sch_stat;
  imu.getData(&raw);
  imu.convertData(&raw, &sch_res);
  imu.getStatus(&sch_stat);

  // --- 3. READ ADXL355 ---
  auto adxl_res = adxl355.getAccelerations();

  // --- 4. READ BNO08x ---
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

  // --- 5. HIGH SPEED SD WRITE ---
  if (dataFile) {
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
    dataFile.println(bno_mz, 3); // println at the end to cap the row

    // --- OPTIMIZED TIME-BASED FLUSH ---
    if (millis() - lastFlushTime >= FLUSH_INTERVAL) {
      dataFile.flush();
      lastFlushTime = millis();
      
      // Flash the LED Blue briefly to indicate a physical save occurred
      leds[0] = CRGB::Blue; 
      FastLED.show();
    }
  }

  // --- 6. VISUAL HEARTBEAT ---
  static uint8_t pulse = 50;
  static int8_t dir = 5;
  pulse += dir;
  if(pulse >= 150 || pulse <= 20) dir = -dir;
  
  // Only overwrite the LED color if we didn't just flash Blue for the save state
  if (millis() - lastFlushTime > 50) { 
    leds[0] = CRGB::Green;
    leds[0].fadeToBlackBy(255 - pulse); 
    FastLED.show();
  }
}