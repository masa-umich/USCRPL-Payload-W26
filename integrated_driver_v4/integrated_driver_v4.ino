#include <SPI.h>
#include "SdFat.h"
#include "RingBuf.h"
#include <FastLED.h>
#include <Adafruit_BNO08x.h>
#include <stdint.h>
#include <EventResponder.h> // Required for Teensy Async SPI (DMA)

// --- HARDWARE CONFIG & PINS ---
#define LED_PIN       23
#define NUM_LEDS      1

#define SCH_CS        5   
#define SCH_DRY_PIN   6
#define SCH_RESET     0   

#define ADXL_CS       10  
#define ADXL_DRDY_PIN 9
#define ADXL_INT1_PIN 7
#define ADXL_INT2_PIN 8

#define BNO_CS        15  
#define BNO_INT       16  
#define BNO_RESET     14  

// --- SD FAT CONFIGURATION ---
SdFs sd;            // Supports FAT16/FAT32/exFAT
FsFile dataFile; 
const uint32_t PRE_ALLOCATE_SIZE = 250UL * 1024UL * 1024UL; // 250MB

// --- RING BUFFER CONFIGURATION ---
#define RING_BUF_CAPACITY 32000 // 32 KB RAM buffer for high-speed writes
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

// --- OBJECTS ---
CRGB leds[NUM_LEDS];
Adafruit_BNO08x bno08x(BNO_RESET);

// --- ASYNC DMA BUFFERS (ADXL359) ---
EventResponder adxlDmaEvent;
// First byte is the Read Command: (0x08 << 1) | 0x01 = 0x11
// The 9 following zeros simply clock the bus to read the 3 axes (20-bit each)
uint8_t adxlTxBuffer[10] = {0x11, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
volatile uint8_t adxlRxBuffer[10];
volatile bool adxlTransferInProgress = false;

// --- SETTINGS & STATE ---
char fileName[] = "IMULOG00.BIN";

uint8_t currentPhase = 0; // Boots into Phase 0 (Standby)
uint8_t previousPhase = 0;
bool phaseChanged = false;

// Independent Low Rate Timers
const uint64_t LOW_RATE_INTERVAL = 1000000; // 1 second logging interval
uint64_t lastSchWrite = 0;
uint64_t lastAdxlWrite = 0;
uint64_t lastBnoWrite = 0;

// --- SPI CONFIGURATIONS ---
SPISettings schSettings(4000000, MSBFIRST, SPI_MODE0);
SPISettings adxlSettings(8000000, MSBFIRST, SPI_MODE0);

// --- SCH16T Register Addresses AND COMMANDS---
const uint32_t READ_RATE_X2 = 0x02800001;
const uint32_t READ_RATE_Y2 = 0x02C00003;
const uint32_t READ_RATE_Z2 = 0x03000006;
const uint32_t READ_ACC_X2  = 0x03400004;
const uint32_t READ_ACC_Y2  = 0x03800002;
const uint32_t READ_ACC_Z2  = 0x03C00000;
const byte ADXL_REG_DATA_START = 0x08; 
const byte ADXL_REG_POWER_CTL = 0x2D;

// --- 64-BIT TIMING VARIABLES ---
volatile uint32_t schLastMicros = 0;
volatile uint32_t schDelta = 0;
volatile uint64_t schAbsoluteMicros = 0;
volatile bool schDataReady = false;

volatile uint32_t adxlLastMicros = 0;
volatile uint32_t adxlDelta = 0;
volatile uint64_t adxlAbsoluteMicros = 0;
volatile bool adxlDataReady = false;

uint32_t bnoLastMicros = 0;

// --- BINARY PACKET STRUCTURES ---
#pragma pack(push, 1) 
struct SCH_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB;
  char type = 'S'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  int16_t rateX, rateY, rateZ, accX, accY, accZ;
} schPack;

struct ADXL_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'A'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  int32_t accX, accY, accZ;
} adxlPack;

struct BNO_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB;
  char type = 'B'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  uint8_t sensorId; float x, y, z;
} bnoPack;
#pragma pack(pop)

// --- ERROR HANDLER ---
void errorHalt(CRGB failColor, const char* msg) {
  Serial5.println(msg);
  while(1) {
    leds[0] = failColor; FastLED.show(); delay(200);
    leds[0] = CRGB::Black; FastLED.show(); delay(200);
  }
}

// --- INTERRUPT SERVICE ROUTINES ---
void sch_ISR() {
  uint32_t current = micros();
  schDelta = current - schLastMicros;
  schAbsoluteMicros += schDelta;      
  schLastMicros = current; schDataReady = true;
}

void adxl_ISR() {
  if (adxlTransferInProgress) return; // Prevent double-triggering
  adxlTransferInProgress = true;

  uint32_t current = micros();
  adxlDelta = current - adxlLastMicros; 
  adxlAbsoluteMicros += adxlDelta;
  adxlLastMicros = current; 

  // Immediately lock the bus, pull CS Low, and hand off to DMA controller
  SPI.beginTransaction(adxlSettings);
  digitalWrite(ADXL_CS, LOW);
  SPI.transfer(adxlTxBuffer, (uint8_t*)adxlRxBuffer, 10, adxlDmaEvent);
  // CPU EXITS IMMEDIATELY
}

// --- DMA CALLBACK ---
// This runs automatically when the DMA hardware finishes transferring all 10 bytes
void adxlDmaFinishedCallback(EventResponderRef event) {
  digitalWrite(ADXL_CS, HIGH);
  SPI.endTransaction();
  adxlTransferInProgress = false;
  adxlDataReady = true; // Tell the main loop the parsed buffer is ready
}

// ==========================================
// POWER MANAGEMENT FUNCTIONS
// ==========================================

void shutdownSensors() {
  Serial5.println("Entering Deep Sleep Mode...");
  
  // BNO086 and SCH16T Hardware Reset
  digitalWrite(SCH_RESET, LOW);
  digitalWrite(BNO_RESET, LOW);
  
  // ADXL359 Software Standby
  SPI.beginTransaction(adxlSettings);
  digitalWrite(ADXL_CS, LOW);
  SPI.transfer(ADXL_REG_POWER_CTL << 1); 
  SPI.transfer(0x01); // 0x01 Sets the Standby Bit
  digitalWrite(ADXL_CS, HIGH);
  SPI.endTransaction();

  // Finalize the SD Card Data
  if (dataFile) {
    rb.sync();              // Push remaining RAM buffer to SD
    dataFile.truncate();    // Chop off the unused pre-allocated space
    dataFile.flush();
  }
}

void initializeSensors() {
  Serial5.println("Waking Sensors for Flight Mode...");

  //Match POR Sequence: Release -> Low -> Delay -> Release 
  digitalWrite(SCH_RESET, HIGH); digitalWrite(BNO_RESET, HIGH);
  delay(10); 
  digitalWrite(SCH_RESET, LOW); digitalWrite(BNO_RESET, LOW);
  delay(10);
  digitalWrite(SCH_RESET, HIGH); digitalWrite(BNO_RESET, HIGH);
  delay(100);

  // 2. Wake ADXL359 (+/- 40g, 1000 Hz, Active High INT)
  SPI.beginTransaction(adxlSettings);
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x2C << 1); SPI.transfer(0xC3); digitalWrite(ADXL_CS, HIGH); delay(2); // 0xC3 fixes Active High Polarity
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x28 << 1); SPI.transfer(0x02); digitalWrite(ADXL_CS, HIGH); delay(2); // 1000Hz ODR & 250Hz corner frequency
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x29 << 1); SPI.transfer(0x5A); digitalWrite(ADXL_CS, HIGH); delay(2); // Set the FIFO Watermark to 90 samples
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x2A << 1); SPI.transfer(0x02); digitalWrite(ADXL_CS, HIGH); delay(2); // Map the FIFO Watermark (FIFO_FULL bit) to the INT1 Pin (Register 0x2A)
  digitalWrite(ADXL_CS, LOW); SPI.transfer(ADXL_REG_POWER_CTL << 1); SPI.transfer(0x00); digitalWrite(ADXL_CS, HIGH); 
  SPI.endTransaction();

  // 3. Boot BARE-METAL SCH16T-K10 (DEC8, 1.475kHz)
  SPI.beginTransaction(schSettings);
  delay(50); 
  transfer32_safe(buildWriteCommand(0x36, 0x000A)); delay(40);  
  transfer32_safe(buildWriteCommand(0x28, 0x12DB));             
  transfer32_safe(buildWriteCommand(0x29, 0x12DB)); delay(5);   
  transfer32_safe(buildWriteCommand(0x33, 0x202C)); delay(10);  
  transfer32_safe(buildWriteCommand(0x35, 0x0001)); delay(250); 
  
  for (uint8_t addr = 0x14; addr <= 0x1D; addr++) { 
    transfer32_safe(buildReadCommand(addr)); delay(5);
  }
  transfer32_safe(buildWriteCommand(0x35, 0x0003)); delay(10);

  int flushCount = 0;
  while (digitalRead(SCH_DRY_PIN) == HIGH && flushCount < 500) { 
    transfer32_safe(READ_RATE_X2); flushCount++; 
  }
  transfer32_safe(READ_RATE_X2); 
  SPI.endTransaction();

  // 4. Boot BNO08x
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT)) {
    Serial5.println("WARNING: BNO08x failed to wake up!");
  } else { 
    bno08x.enableReport(SH2_ACCELEROMETER, 10000); 
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  }

  // 5. MASTER TIMELINE SYNC
  uint32_t absoluteStart = micros();
  schLastMicros = absoluteStart; adxlLastMicros = absoluteStart;
  schAbsoluteMicros = absoluteStart; adxlAbsoluteMicros = absoluteStart;
  lastSchWrite = absoluteStart; lastAdxlWrite = absoluteStart;
  lastBnoWrite = absoluteStart; bnoLastMicros = absoluteStart;
  schDataReady = false; adxlDataReady = false; adxlTransferInProgress = false;
  
  Serial5.println("Sensors Armed and Logging Started.");
}


// ==========================================
// MAIN SETUP
// ==========================================

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  leds[0] = CRGB::White; FastLED.show();

  // CS & Data Ready Config
  pinMode(SCH_CS, OUTPUT);  digitalWrite(SCH_CS, HIGH);
  pinMode(ADXL_CS, OUTPUT); digitalWrite(ADXL_CS, HIGH);
  pinMode(BNO_CS, OUTPUT);  digitalWrite(BNO_CS, HIGH);
  pinMode(SCH_DRY_PIN, INPUT_PULLDOWN);
  pinMode(ADXL_INT1_PIN, INPUT);

  // MOSI & SCLK Clamping
  pinMode(13, OUTPUT); digitalWrite(13, LOW); 
  pinMode(11, OUTPUT); digitalWrite(11, LOW); 
  delay(1000);

  // HARDWARE RESETS
  pinMode(SCH_RESET, OUTPUT); digitalWrite(SCH_RESET, LOW);
  pinMode(BNO_RESET, OUTPUT); digitalWrite(BNO_RESET, LOW); 

  Serial.begin(115200);
  Serial5.begin(9600); 
  while (!Serial && millis() < 3000); 
  Serial5.println("MASA Sensor Payload Initializing...");
  leds[0] = CRGB::Blue; FastLED.show();

  // Initialize SdFat on Built-in SDIO
  Serial5.print("Initializing SD card...");
  int sdRetries = 0;
  while (!sd.begin(SdioConfig(FIFO_SDIO)) && sdRetries < 5) { sdRetries++; delay(200); }
  if (sdRetries >= 5) errorHalt(CRGB::Red, "SD Card failed!");

  // Naming and Pre-allocating .BIN files
  for (uint8_t i = 0; i < 100; i++) {
    fileName[6] = i / 10 + '0';
    fileName[7] = i % 10 + '0';
    if (!sd.exists(fileName)) {
      dataFile = sd.open(fileName, O_RDWR | O_CREAT | O_TRUNC);
      break;
    }
  }
  if (!dataFile) errorHalt(CRGB::Red, "Could not create file!"); 

  // Pre-allocate contiguous space
  if (!dataFile.preAllocate(PRE_ALLOCATE_SIZE)) {
    Serial5.println("WARNING: Pre-allocation failed. Logging will be slower.");
  }
  
  rb.begin(&dataFile);

  SPI.begin();

  // --- DMA AND INTERRUPT BINDING ---
  adxlDmaEvent.attachInterrupt(adxlDmaFinishedCallback);

  // CRITICAL: Tells SPI library to mask these pins momentarily when 
  // SPI.beginTransaction() is called, preventing ISR/Main Loop bus collisions.
  SPI.usingInterrupt(digitalPinToInterrupt(ADXL_INT1_PIN));
  SPI.usingInterrupt(digitalPinToInterrupt(SCH_DRY_PIN));

  attachInterrupt(digitalPinToInterrupt(ADXL_INT1_PIN), adxl_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SCH_DRY_PIN), sch_ISR, RISING);

  shutdownSensors();
  leds[0] = CRGB::Cyan; FastLED.show();
}


// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
  // --- 1. UART PHASE CONTROL ---
  if (Serial5.available() > 0) {
    uint8_t incomingByte = Serial5.read();
    uint8_t newPhase = currentPhase;
    
    if (incomingByte <= 4) newPhase = incomingByte;
    else if (incomingByte >= '0' && incomingByte <= '4') newPhase = incomingByte - '0';
    if (newPhase != currentPhase) {
      previousPhase = currentPhase; currentPhase = newPhase; phaseChanged = true;
    } 
  }

  bool isLowPower = (currentPhase == 0 || currentPhase == 4);

  // --- 2. PHASE TRANSITION EVENT TRIGGER ---
  if (phaseChanged) {
    bool wasLowPower = (previousPhase == 0 || previousPhase == 4);
    if (wasLowPower && !isLowPower) { initializeSensors(); } 
    else if (!wasLowPower && isLowPower) { shutdownSensors(); }
    phaseChanged = false;
  }

  // --- 3. THE LOW-POWER HALT ---
  if (isLowPower) {
    static unsigned long lastBlinkTime = 0;
    static bool isLedOn = false;
    const unsigned long STANDBY_BLINK_INTERVAL = 5000;
    const unsigned long BLINK_DURATION = 10;
    
    if (!isLedOn && (millis() - lastBlinkTime >= STANDBY_BLINK_INTERVAL)) {
      leds[0] = (currentPhase == 0) ? CRGB::Cyan : CRGB::Purple;
      FastLED.show(); lastBlinkTime = millis(); isLedOn = true;
    } 
    else if (isLedOn && (millis() - lastBlinkTime >= BLINK_DURATION)) {
      leds[0] = CRGB::Black; FastLED.show(); isLedOn = false;
    }
    asm volatile("wfi");
  } 
  else {
    // ==========================================
    // HIGH-SPEED FLIGHT MODE (Phases 1, 2, 3)
    // ==========================================

    // --- HARDWARE FAILSAFES ---
    if (digitalRead(SCH_DRY_PIN) == HIGH && !schDataReady) { sch_ISR(); }
    if (digitalRead(ADXL_INT1_PIN) == HIGH && !adxlDataReady && !adxlTransferInProgress) { adxl_ISR(); }

    // --- SCH16T PROCESS ---
    if (schDataReady) {
      noInterrupts();
      schPack.timestamp = schAbsoluteMicros;
      schDataReady = false;
      interrupts();

      SPI.beginTransaction(schSettings);
      uint32_t respRx = transfer32_safe(READ_RATE_Y2); uint32_t respRy = transfer32_safe(READ_RATE_Z2); 
      uint32_t respRz = transfer32_safe(READ_ACC_X2);  uint32_t respAx = transfer32_safe(READ_ACC_Y2);  
      uint32_t respAy = transfer32_safe(READ_ACC_Z2);  uint32_t respAz = transfer32_safe(READ_RATE_X2); 
      SPI.endTransaction();

      schPack.phase = currentPhase;
      schPack.delta_t = (uint32_t)(schAbsoluteMicros - lastSchWrite); 
      schPack.rateX = (int16_t)((respRx >> 3) & 0xFFFF); schPack.rateY = (int16_t)((respRy >> 3) & 0xFFFF);
      schPack.rateZ = (int16_t)((respRz >> 3) & 0xFFFF); schPack.accX  = (int16_t)((respAx >> 3) & 0xFFFF);
      schPack.accY  = (int16_t)((respAy >> 3) & 0xFFFF); schPack.accZ  = (int16_t)((respAz >> 3) & 0xFFFF);
      
      rb.write((uint8_t*)&schPack, sizeof(schPack));
      lastSchWrite = schAbsoluteMicros;
    }

    // --- ADXL359 PROCESS ---
    if (adxlDataReady) {
      noInterrupts();
      adxlPack.timestamp = adxlAbsoluteMicros; 
      adxlPack.delta_t = adxlDelta;
      adxlDataReady = false;
      interrupts();

      adxlPack.phase = currentPhase;
      
      // Parse data out of the volatile DMA Rx buffer. 
      // Byte 0 is the dummy response to the 0x11 command byte.
      long aX = ((long)adxlRxBuffer[1] << 12) | ((long)adxlRxBuffer[2] << 4) | (adxlRxBuffer[3] >> 4);
      if (aX & 0x80000) aX |= 0xFFF00000; 

      long aY = ((long)adxlRxBuffer[4] << 12) | ((long)adxlRxBuffer[5] << 4) | (adxlRxBuffer[6] >> 4);
      if (aY & 0x80000) aY |= 0xFFF00000; 

      long aZ = ((long)adxlRxBuffer[7] << 12) | ((long)adxlRxBuffer[8] << 4) | (adxlRxBuffer[9] >> 4);
      if (aZ & 0x80000) aZ |= 0xFFF00000; 

      adxlPack.accX = aX; adxlPack.accY = aY; adxlPack.accZ = aZ;
      
      rb.write((uint8_t*)&adxlPack, sizeof(adxlPack));
      lastAdxlWrite = adxlPack.timestamp;
    }

    // --- BNO086 PROCESS ---
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue)) {
      bnoPack.phase = currentPhase;
      bnoPack.timestamp = schAbsoluteMicros; 
      bnoPack.delta_t = (uint32_t)(schAbsoluteMicros - lastBnoWrite);
      bnoPack.sensorId = sensorValue.sensorId;

      bool wroteData = false;
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        bnoPack.x = sensorValue.un.accelerometer.x; bnoPack.y = sensorValue.un.accelerometer.y; bnoPack.z = sensorValue.un.accelerometer.z;
        wroteData = true;
      } 
      else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        bnoPack.x = sensorValue.un.gyroscope.x; bnoPack.y = sensorValue.un.gyroscope.y; bnoPack.z = sensorValue.un.gyroscope.z;
        wroteData = true;
      } 
      else if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
        bnoPack.x = sensorValue.un.magneticField.x; bnoPack.y = sensorValue.un.magneticField.y; bnoPack.z = sensorValue.un.magneticField.z;
        wroteData = true;
      }
      
      if (wroteData) {
        rb.write((uint8_t*)&bnoPack, sizeof(bnoPack));
        lastBnoWrite = schAbsoluteMicros;
      }
    }

    // ==========================================
    // NON-BLOCKING SD BACKGROUND WRITE
    // ==========================================
    size_t bytesToWrite = rb.bytesUsed();
    if (bytesToWrite >= 512 && !dataFile.isBusy()) {
      rb.writeOut(512); 
    }

    // --- ACTIVE FLIGHT HEARTBEAT (Low Overhead) ---
    static unsigned long lastFlightBlinkTime = 0;
    static bool isFlightLedOn = false;
    const unsigned long FLIGHT_BLINK_INTERVAL = 5000;
    const unsigned long FLIGHT_BLINK_DURATION = 10;
    
    if (!isFlightLedOn && (millis() - lastFlightBlinkTime >= FLIGHT_BLINK_INTERVAL)) {
      switch(currentPhase) {     
        case 1: leds[0] = CRGB::Yellow; break;     
        case 2: leds[0] = CRGB::DarkOrange; break; 
        case 3: leds[0] = CRGB::Red; break;
      }
      FastLED.show();
      lastFlightBlinkTime = millis();
      isFlightLedOn = true;
    } 
    else if (isFlightLedOn && (millis() - lastFlightBlinkTime >= FLIGHT_BLINK_DURATION)) {
      leds[0] = CRGB::Black;
      FastLED.show();
      isFlightLedOn = false;
    }
  }
}

// --- HELPER FUNCTIONS ---
uint32_t transfer32_safe(uint32_t data) {
  uint32_t result = 0; digitalWrite(SCH_CS, LOW); delayMicroseconds(1);
  result |= ((uint32_t)SPI.transfer((data >> 24) & 0xFF)) << 24; result |= ((uint32_t)SPI.transfer((data >> 16) & 0xFF)) << 16;
  result |= ((uint32_t)SPI.transfer((data >> 8)  & 0xFF)) << 8;  result |= ((uint32_t)SPI.transfer(data & 0xFF));
  delayMicroseconds(1); digitalWrite(SCH_CS, HIGH); return result;
}

uint32_t buildWriteCommand(uint8_t address, uint16_t data) {
  uint32_t frame = 0; frame |= ((uint32_t)address & 0xFF) << 22;
  frame |= (1UL << 21); frame |= (0UL << 19); frame |= (uint32_t)data << 3;              
  return frame | calcCRC3(frame >> 3);
}

uint32_t buildReadCommand(uint8_t address) {
  uint32_t frame = 0; frame |= ((uint32_t)address & 0xFF) << 22;
  frame |= (0UL << 21); return frame | calcCRC3(frame >> 3);
}

uint8_t calcCRC3(uint32_t frame29) {
  uint8_t crc = 0x05;
  for (int i = 28; i >= 0; i--) { bool bit = (frame29 >> i) & 0x01;
    bool msb = (crc >> 2) & 0x01; crc = ((crc << 1) & 0x07) | bit;
    if (msb) crc ^= 0x03; }
  for (int i = 0; i < 3; i++) { bool msb = (crc >> 2) & 0x01;
    crc = (crc << 1) & 0x07; if (msb) crc ^= 0x03; }
  return crc;
}