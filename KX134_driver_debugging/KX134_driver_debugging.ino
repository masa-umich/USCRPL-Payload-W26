#include <SPI.h>
#include <SparkFun_KX13X.h>

// Use the SPI-specific class for the KX134
SparkFun_KX134_SPI kxAccel;

// PCB Configuration
const int chipSelect = 3; 

// SPI Settings: 10MHz, MSB First, SPI Mode 0
// If 10MHz is still wonky, try 1000000 (1MHz)
SPISettings kxSettings(1000000, MSBFIRST, SPI_MODE0);

outputData myData; // Struct for the accelerometer's data

void setup() {
  Serial.begin(115200);

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); 
  delay(1000);
  digitalWrite(3, HIGH);
  delay(1000);
  
  // Wait for Serial Monitor (Teensy 4.0 boots faster than most PCs can connect)
  while (!Serial && millis() < 3000) delay(10);
  
  Serial.println("KX134 SPI Initialization...");

  // 1. Start the hardware SPI bus
  SPI.begin();

  // 2. Start the sensor using the specific SPI port, settings, and CS pin
  // This matches the "Version 2" begin() in your header file
  if (!kxAccel.begin(SPI, kxSettings, chipSelect)) {
    Serial.println("Error: KX134 not found on Pin 3. Check wiring/power.");
    while (1); 
  }

  Serial.println("KX134 Detected!");

  // 3. Configure the sensor
  kxAccel.softwareReset();
  delay(10); // Give it a moment to wake up after reset

  kxAccel.enableAccel(false);      // Must be off to change most settings
  kxAccel.setRange(SFE_KX134_RANGE16G); 
  kxAccel.enableDataEngine();      // Turn on the "Data Ready" flag
  kxAccel.enableAccel();           // Power it back up
  
  Serial.println("Streaming data...");
}

void loop() {
  // Non-blocking check: Only read if the sensor says data is ready
  if (kxAccel.dataReady()) {
    kxAccel.getAccelData(&myData);
    
    Serial.print("X: "); Serial.print(myData.xData, 4);
    Serial.print(" \tY: "); Serial.print(myData.yData, 4);
    Serial.print(" \tZ: "); Serial.println(myData.zData, 4);
  }

  // No long delays here so the Teensy stays responsive for your LED or other tasks
}