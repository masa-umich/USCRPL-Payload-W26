#include <SPI.h>
#include <SD.h>
#include "SCH16T.h"

// --- HARDWARE CONFIG ---
#define SCH_CS        5   
#define SCH_RESET     0   
#define BNO_CS        15  // Keep BNO quiet
#define chipSelect    BUILTIN_SDCARD // Use the Teensy 4.0 internal slot

// Sensor Settings
#define FILTER_FREQ   68
#define SENS_RATE     200
#define SENS_ACC      3200
#define DECIMATION    4

// Objects
SCH16T_K10 imu(SPI, SCH_CS, SCH_RESET);
File dataFile;
char fileName[] = "IMULOG00.CSV";

// Buffering logic
int flushCounter = 0;
const int FLUSH_THRESHOLD = 50; // Write to physical disk every 50 samples

void setup() {
  Serial.begin(115200);
  
  // 1. Setup Pins
  pinMode(BNO_CS, OUTPUT);
  digitalWrite(BNO_CS, HIGH); 
  pinMode(SCH_RESET, OUTPUT);
  
  // 2. Initial Hardware Reset
  digitalWrite(SCH_RESET, LOW);
  delay(100);
  digitalWrite(SCH_RESET, HIGH);
  delay(2000); 

  // 3. Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1); // Halt if no SD card
  }
  Serial.println("Card initialized.");

  // 4. Create a new file name (e.g., IMULOG01, IMULOG02...)
  for (uint8_t i = 0; i < 100; i++) {
    fileName[6] = i / 10 + '0';
    fileName[7] = i % 10 + '0';
    if (!SD.exists(fileName)) {
      dataFile = SD.open(fileName, FILE_WRITE);
      break;
    }
  }

  if (dataFile) {
    Serial.print("Logging to: "); Serial.println(fileName);
    // Write CSV Header
    dataFile.println("micros,status,GX,GY,GZ,AX,AY,AZ,Temp");
  } else {
    Serial.println("Could not open file for writing");
    while (1);
  }

  // 5. Initialize Sensor
  SPI.begin();
  SCH16T_filter Filter = {(uint16_t)FILTER_FREQ, (uint16_t)FILTER_FREQ, (uint16_t)FILTER_FREQ};
  SCH16T_sensitivity Sensitivity = {(uint16_t)SENS_RATE, (uint16_t)SENS_RATE, (uint16_t)SENS_ACC, (uint16_t)SENS_ACC, (uint16_t)SENS_ACC};
  SCH16T_decimation Decimation = {(uint8_t)DECIMATION, (uint8_t)DECIMATION};

  imu.begin(Filter, Sensitivity, Decimation, false);
  Serial.println("Setup Complete. Logging...");
}

void loop() {
  SCH16T_raw_data raw;
  SCH16T_result result;
  SCH16T_status status;

  // Read data
  imu.getData(&raw);
  imu.convertData(&raw, &result);
  imu.getStatus(&status);

  // Construct Data String
  // We use a single string to ensure the "block" is written together
  String dataString = "";
  dataString += String(micros()) + ",";
  dataString += "0x" + String(status.Summary, HEX) + ",";
  dataString += String(result.Rate1[0], 3) + "," + String(result.Rate1[1], 3) + "," + String(result.Rate1[2], 3) + ",";
  dataString += String(result.Acc1[0], 3) + "," + String(result.Acc1[1], 3) + "," + String(result.Acc1[2], 3) + ",";
  dataString += String(result.Temp, 2);

  // Write to the file buffer
  if (dataFile) {
    dataFile.println(dataString);
    
    // Increment flush counter
    flushCounter++;
    
    // Physical block write: minimize overhead by only flushing occasionally
    if (flushCounter >= FLUSH_THRESHOLD) {
      dataFile.flush(); 
      flushCounter = 0;
      // Optional: Blink an LED here so you know it's writing
    }
  }

  // Sample at ~100Hz (10ms)
  delay(1); 
}