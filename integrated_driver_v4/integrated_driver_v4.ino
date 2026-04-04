#include <SPI.h>
#include <SD.h>
#include <FastLED.h>
#include <Adafruit_BNO08x.h>
#include <stdint.h>

// --- HARDWARE CONFIG & PINS ---
#define LED_PIN       23
#define NUM_LEDS      1

#define SCH_CS        5   
#define SCH_DRY_PIN   6
#define SCH_RESET     0   

#define ADXL_CS       10  
#define ADXL_DRDY_PIN 9

#define BNO_CS        15  
#define BNO_INT       16  
#define BNO_RESET     14  

#define SD_CS         BUILTIN_SDCARD

// --- OBJECTS ---
CRGB leds[NUM_LEDS];
Adafruit_BNO08x bno08x(BNO_RESET);
File dataFile;

// --- SETTINGS & STATE ---
char fileName[] = "IMULOG00.BIN"; // Changed to Binary
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 5000; 

uint8_t currentPhase = 0; 
uint8_t previousPhase = 0;
bool phaseChanged = false; 

// Independent Low Rate Timers for Asynchronous Logging
const uint64_t LOW_RATE_INTERVAL = 1000000;
uint64_t lastSchWrite = 0;
uint64_t lastAdxlWrite = 0;
uint64_t lastBnoWrite = 0;

// --- SPI CONFIGURATIONS ---
SPISettings schSettings(4000000, MSBFIRST, SPI_MODE0);
SPISettings adxlSettings(8000000, MSBFIRST, SPI_MODE0);

// --- BARE-METAL COMMANDS ---
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
// SCH16T Packet (28 Bytes)
struct SCH_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'S'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  int16_t rateX, rateY, rateZ, accX, accY, accZ;
} schPack;

// ADXL359 Packet (28 Bytes)
struct ADXL_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'A'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  int32_t accX, accY, accZ;
} adxlPack;

// BNO086 Packet (29 Bytes)
struct BNO_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'B'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  uint8_t sensorId; float x, y, z;
} bnoPack;
#pragma pack(pop)

// --- COLOR-CODED ERROR HANDLER ---
void errorHalt(CRGB failColor, const char* msg) {
  Serial.println(msg);
  while(1) {
    leds[0] = failColor; FastLED.show(); delay(200);
    leds[0] = CRGB::Black; FastLED.show(); delay(200);
  }
}

// --- INTERRUPT SERVICE ROUTINES ---
void sch_ISR() {
  uint32_t current = micros();
  schDelta = current - schLastMicros; schAbsoluteMicros += schDelta;      
  schLastMicros = current; schDataReady = true;
}

void adxl_ISR() {
  uint32_t current = micros();
  adxlDelta = current - adxlLastMicros; adxlAbsoluteMicros += adxlDelta;
  adxlLastMicros = current; adxlDataReady = true;
}

void setup() {
  unsigned long init_start = micros();
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  leds[0] = CRGB::White; FastLED.show();

  //CS & Data Ready Config
  pinMode(SCH_CS, OUTPUT);  digitalWrite(SCH_CS, HIGH);
  pinMode(ADXL_CS, OUTPUT); digitalWrite(ADXL_CS, HIGH);
  pinMode(BNO_CS, OUTPUT);  digitalWrite(BNO_CS, HIGH);
  pinMode(SCH_DRY_PIN, INPUT_PULLDOWN);
  pinMode(ADXL_DRDY_PIN, INPUT);

  //MOSI & SCLK Config
  pinMode(13, OUTPUT); digitalWrite(13, LOW); 
  pinMode(11, OUTPUT); digitalWrite(11, LOW); 
  delay(1000); 

  // HARDWARE RESETS
  pinMode(SCH_RESET, OUTPUT); digitalWrite(SCH_RESET, LOW); delay(10); digitalWrite(SCH_RESET, HIGH); 
  pinMode(BNO_RESET, OUTPUT); digitalWrite(BNO_RESET, LOW); delay(10); digitalWrite(BNO_RESET, HIGH);
  delay(100); 

  //Serial Config
  Serial.begin(115200);
  Serial5.begin(9600); 
  while (!Serial && millis() < 3000); 
  Serial5.println("MASA Sensor Payload Initializing...");

  leds[0] = CRGB::Blue; FastLED.show();

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  int sdRetries = 0;
  while (!SD.begin(SD_CS) && sdRetries < 5) { sdRetries++; delay(200); }
  if (sdRetries >= 5) errorHalt(CRGB::Red, "SD Card failed!");
  
  //Naming of microSD .bin files
  for (uint8_t i = 0; i < 100; i++) {
    fileName[6] = i / 10 + '0';
    fileName[7] = i % 10 + '0';
    if (!SD.exists(fileName)) {
      dataFile = SD.open(fileName, FILE_WRITE);
      break;
    }
  }
  if (!dataFile) errorHalt(CRGB::Red, "Could not create file!"); 

  SPI.begin();

  // BARE-METAL ADXL359 (+/- 40g, 1000 Hz)
  SPI.beginTransaction(adxlSettings);
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x2C << 1); SPI.transfer(0x83); digitalWrite(ADXL_CS, HIGH); delay(2);
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x28 << 1); SPI.transfer(0x02); digitalWrite(ADXL_CS, HIGH); delay(2);
  digitalWrite(ADXL_CS, LOW); SPI.transfer(ADXL_REG_POWER_CTL << 1); SPI.transfer(0x00); digitalWrite(ADXL_CS, HIGH);
  SPI.endTransaction();
  Serial.println("ADXL359 Ready.");

  // BARE-METAL SCH16T-K10 (DEC8, 1.475kHz)
  SPI.beginTransaction(schSettings);
  delay(50); 
  transfer32_safe(buildWriteCommand(0x36, 0x000A)); delay(40);
  transfer32_safe(buildWriteCommand(0x28, 0x12DB)); 
  transfer32_safe(buildWriteCommand(0x29, 0x12DB)); delay(5);
  transfer32_safe(buildWriteCommand(0x33, 0x202C)); delay(10);
  transfer32_safe(buildWriteCommand(0x35, 0x0001)); delay(250); 
  for (uint8_t addr = 0x14; addr <= 0x1D; addr++) { transfer32_safe(buildReadCommand(addr)); delay(5); }
  transfer32_safe(buildWriteCommand(0x35, 0x0003)); delay(10);

  int flushCount = 0;
  while (digitalRead(SCH_DRY_PIN) == HIGH && flushCount < 500) { transfer32_safe(READ_RATE_X2); flushCount++; }
  transfer32_safe(READ_RATE_X2); 
  SPI.endTransaction();
  Serial.println("SCH16T Ready.");

  // Initialize BNO08x
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT)) errorHalt(CRGB::Purple, "BNO08x failed to start!"); 
  bno08x.enableReport(SH2_ACCELEROMETER, 10000);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  Serial.println("BNO08x Ready.");

  // ARM HARDWARE INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(ADXL_DRDY_PIN), adxl_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SCH_DRY_PIN), sch_ISR, RISING);

  Serial.print("Logging Binary to: "); Serial.println(fileName);
  unsigned long init_time = (micros() - init_start);
  Serial5.print("Successfully Initialized in "); Serial5.print(init_time); Serial5.println("us");
  
  leds[0] = CRGB::Cyan; FastLED.show();
  
  // Timestamp Sync
  uint32_t absoluteStart = micros();
  schLastMicros = absoluteStart;
  adxlLastMicros = absoluteStart;
  
  schAbsoluteMicros = absoluteStart;
  adxlAbsoluteMicros = absoluteStart;
  
  lastSchWrite = absoluteStart;
  lastAdxlWrite = absoluteStart;
  lastBnoWrite = absoluteStart;
  bnoLastMicros = absoluteStart;
}

void loop() {
  // --- 1. UART PHASE CONTROL ---
  if (Serial5.available() > 0) {
    uint8_t incomingByte = Serial5.read();
    uint8_t newPhase = currentPhase;
    
    if (incomingByte <= 4) newPhase = incomingByte;
    else if (incomingByte >= '0' && incomingByte <= '4') newPhase = incomingByte - '0';

    if (newPhase != currentPhase) {
      previousPhase = currentPhase;
      currentPhase = newPhase;
      phaseChanged = true; 
    } 
  }

  // --- 2. FAILSAFE & RATE DETERMINATION ---
  if (digitalRead(SCH_DRY_PIN) == HIGH && !schDataReady) { schLastMicros = micros(); schDataReady = true; }
  
  bool highRateMode = (currentPhase >= 1 && currentPhase <= 3);

  // --- 3. SCH16T PROCESS ---
  if (schDataReady) {
    noInterrupts();
    schPack.timestamp = schAbsoluteMicros; schPack.delta_t = schDelta;
    schDataReady = false;
    interrupts();

    SPI.beginTransaction(schSettings);
    uint32_t respRx = transfer32_safe(READ_RATE_Y2); 
    uint32_t respRy = transfer32_safe(READ_RATE_Z2); 
    uint32_t respRz = transfer32_safe(READ_ACC_X2);  
    uint32_t respAx = transfer32_safe(READ_ACC_Y2);  
    uint32_t respAy = transfer32_safe(READ_ACC_Z2);  
    uint32_t respAz = transfer32_safe(READ_RATE_X2); 
    SPI.endTransaction();

    if (highRateMode || phaseChanged || (schAbsoluteMicros - lastSchWrite >= LOW_RATE_INTERVAL)) {
      schPack.phase = currentPhase;
      schPack.delta_t = (uint32_t)(schAbsoluteMicros - lastSchWrite); // Accurate Log Delta
      schPack.rateX = (int16_t)((respRx >> 3) & 0xFFFF); schPack.rateY = (int16_t)((respRy >> 3) & 0xFFFF); schPack.rateZ = (int16_t)((respRz >> 3) & 0xFFFF);
      schPack.accX  = (int16_t)((respAx >> 3) & 0xFFFF); schPack.accY  = (int16_t)((respAy >> 3) & 0xFFFF); schPack.accZ  = (int16_t)((respAz >> 3) & 0xFFFF);
      dataFile.write((uint8_t*)&schPack, sizeof(schPack));
      lastSchWrite = schAbsoluteMicros;
    }
  }

  // --- 4. ADXL359 PROCESS ---
  if (adxlDataReady) {
    noInterrupts();
    adxlPack.timestamp = adxlAbsoluteMicros; adxlPack.delta_t = adxlDelta;
    adxlDataReady = false;
    interrupts();

    SPI.beginTransaction(adxlSettings);
    digitalWrite(ADXL_CS, LOW); SPI.transfer((ADXL_REG_DATA_START << 1) | 0x01);
    int32_t aX = readAdxl20Bit(); int32_t aY = readAdxl20Bit(); int32_t aZ = readAdxl20Bit();
    digitalWrite(ADXL_CS, HIGH);
    SPI.endTransaction();

    if (highRateMode || phaseChanged || (adxlAbsoluteMicros - lastAdxlWrite >= LOW_RATE_INTERVAL)) {
      adxlPack.phase = currentPhase;
      adxlPack.delta_t = (uint32_t)(adxlAbsoluteMicros - lastAdxlWrite); // Accurate Log Delta
      adxlPack.accX = aX; adxlPack.accY = aY; adxlPack.accZ = aZ;
      dataFile.write((uint8_t*)&adxlPack, sizeof(adxlPack));
      lastAdxlWrite = adxlAbsoluteMicros;
    }
  }

// --- 5. BNO086 PROCESS ---
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    // We use schAbsoluteMicros as the global Master Clock for the BNO
    if (highRateMode || phaseChanged || (schAbsoluteMicros - lastBnoWrite >= LOW_RATE_INTERVAL)) {
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
        dataFile.write((uint8_t*)&bnoPack, sizeof(bnoPack));
        lastBnoWrite = schAbsoluteMicros;
      }
    }
  }

  // --- 6. FLUSH LOGIC ---
  if (phaseChanged || millis() - lastFlushTime >= FLUSH_INTERVAL) {
    if (dataFile) {
      dataFile.flush();
      lastFlushTime = millis();
      phaseChanged = false; 
      leds[0] = CRGB::Blue; FastLED.show();
    }
  }

// --- 7. PHASE-AWARE POWER-SAVING HEARTBEAT ---
  static unsigned long lastBlinkTime = 0;
  static bool isLedOn = false;
  const unsigned long STANDBY_BLINK_INTERVAL = 5000; // 5 seconds between flashes
  const unsigned long BLINK_DURATION = 10;           // 10ms flash duration
  
  if (currentPhase == 0 || currentPhase == 4) {
    // --- LOW POWER MODE (Standby & Landed) ---
    // Only flash for 10ms every 5 seconds. Otherwise, stay completely off.
    
    if (!isLedOn && (millis() - lastBlinkTime >= STANDBY_BLINK_INTERVAL)) {
      leds[0] = (currentPhase == 0) ? CRGB::Cyan : CRGB::Purple;
      FastLED.show();
      lastBlinkTime = millis();
      isLedOn = true;
    } 
    else if (isLedOn && (millis() - lastBlinkTime >= BLINK_DURATION)) {
      leds[0] = CRGB::Black; // Cut power to the WS2812B PWM chip
      FastLED.show();
      isLedOn = false;
    }
  } 
  else {
    // --- HIGH VISIBILITY MODE (Armed, Term, Flight) ---
    // Restore the smooth pulsing heartbeat when power is not a concern
    
    static uint8_t pulse = 50; 
    static int8_t dir = 5;
    EVERY_N_MILLISECONDS(20) { 
      pulse += dir; 
      if(pulse >= 150 || pulse <= 20) dir = -dir; 
    }
    
    // Don't overwrite the Blue SD Card flush indicator
    if (millis() - lastFlushTime > 50) { 
      switch(currentPhase) {     
        case 1: leds[0] = CRGB::Yellow; break;     
        case 2: leds[0] = CRGB::DarkOrange; break; 
        case 3: leds[0] = CRGB::Red; break;            
      }
      leds[0].fadeToBlackBy(255 - pulse); 
      FastLED.show();
    }
  }

  if (currentPhase == 0 || currentPhase == 4) {
    asm volatile("wfi");
  }
}

// --- HELPER FUNCTIONS ---
long readAdxl20Bit() {
  byte b1 = SPI.transfer(0x00); byte b2 = SPI.transfer(0x00); byte b3 = SPI.transfer(0x00);
  long val = ((long)b1 << 12) | ((long)b2 << 4) | (b3 >> 4);
  if (val & 0x80000) val |= 0xFFF00000; 
  return val;
}

uint32_t transfer32_safe(uint32_t data) {
  uint32_t result = 0; digitalWrite(SCH_CS, LOW); delayMicroseconds(1); 
  result |= ((uint32_t)SPI.transfer((data >> 24) & 0xFF)) << 24; result |= ((uint32_t)SPI.transfer((data >> 16) & 0xFF)) << 16;
  result |= ((uint32_t)SPI.transfer((data >> 8)  & 0xFF)) << 8;  result |= ((uint32_t)SPI.transfer(data & 0xFF));
  delayMicroseconds(1); digitalWrite(SCH_CS, HIGH); return result;
}

uint32_t buildWriteCommand(uint8_t address, uint16_t data) {
  uint32_t frame = 0; frame |= ((uint32_t)address & 0xFF) << 22; frame |= (1UL << 21); frame |= (0UL << 19); frame |= (uint32_t)data << 3;              
  return frame | calcCRC3(frame >> 3);
}

uint32_t buildReadCommand(uint8_t address) {
  uint32_t frame = 0; frame |= ((uint32_t)address & 0xFF) << 22; frame |= (0UL << 21); return frame | calcCRC3(frame >> 3);
}

uint8_t calcCRC3(uint32_t frame29) {
  uint8_t crc = 0x05; 
  for (int i = 28; i >= 0; i--) { bool bit = (frame29 >> i) & 0x01; bool msb = (crc >> 2) & 0x01; crc = ((crc << 1) & 0x07) | bit; if (msb) crc ^= 0x03; }
  for (int i = 0; i < 3; i++) { bool msb = (crc >> 2) & 0x01; crc = (crc << 1) & 0x07; if (msb) crc ^= 0x03; }
  return crc;
}