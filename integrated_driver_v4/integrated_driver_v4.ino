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
char fileName[] = "IMULOG00.BIN";
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 5000; 

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
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'S'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  int16_t rateX, rateY, rateZ, accX, accY, accZ;
} schPack;

struct ADXL_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'A'; uint8_t phase;
  uint64_t timestamp; uint32_t delta_t;
  int32_t accX, accY, accZ;
} adxlPack;

struct BNO_Packet {
  uint8_t sync1 = 0xAA, sync2 = 0xBB; char type = 'B'; uint8_t phase;
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
  schDelta = current - schLastMicros; schAbsoluteMicros += schDelta;      
  schLastMicros = current; schDataReady = true;
}

void adxl_ISR() {
  uint32_t current = micros();
  adxlDelta = current - adxlLastMicros; adxlAbsoluteMicros += adxlDelta;
  adxlLastMicros = current; adxlDataReady = true;
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

  // Flush the SD Card
  if (dataFile) {
    dataFile.flush();
  }
}

void initializeSensors() {
  Serial5.println("Waking Sensors for Flight Mode...");

  //Match POR Sequence: Release -> Low -> Delay -> Release 
  
  digitalWrite(SCH_RESET, HIGH);
  digitalWrite(BNO_RESET, HIGH);
  delay(10); 
  digitalWrite(SCH_RESET, LOW);
  digitalWrite(BNO_RESET, LOW);
  delay(10);
  digitalWrite(SCH_RESET, HIGH);
  digitalWrite(BNO_RESET, HIGH);
  
  // Booting delay
  delay(100); 

  // 2. Wake ADXL359 (+/- 40g, 1000 Hz)
  SPI.beginTransaction(adxlSettings);
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x2C << 1); SPI.transfer(0xC3); digitalWrite(ADXL_CS, HIGH); delay(2); //Writing 0x83 to the range register, sets it to +/-40g, and DRY to active high
  digitalWrite(ADXL_CS, LOW); SPI.transfer(0x28 << 1); SPI.transfer(0x02); digitalWrite(ADXL_CS, HIGH); delay(2); //1000Hz ODR & 250Hz corner frequency
  digitalWrite(ADXL_CS, LOW); SPI.transfer(ADXL_REG_POWER_CTL << 1); SPI.transfer(0x00); digitalWrite(ADXL_CS, HIGH); //Enables Measurement Mode
  SPI.endTransaction();

  // 3. Boot BARE-METAL SCH16T-K10 (DEC8, 1.475kHz)
  //0x36: SPI Soft Reset
  //0x28: Settings for Gyro post-processing decimation ratio and dynamic range
  //0x29: Settings for ACC_X12, ACC_Y12, ACC_Z12 post-processing decimation ratio and dynamic range
  //0x33: User controls for SYNC, Data Ready, Strength of SPI PD/PU, slew rate ctrl, hi-speed
  //0x35: Test mode, EOI, EN_SENSOR

  SPI.beginTransaction(schSettings);
  delay(50); 
  transfer32_safe(buildWriteCommand(0x36, 0x000A)); delay(40);  //Reset SPI
  transfer32_safe(buildWriteCommand(0x28, 0x12DB));             //2000dps range for output registers 1 & 2, 8x decimation on output register 2 (1.475khz output rate)
  transfer32_safe(buildWriteCommand(0x29, 0x12DB)); delay(5);   //+-16g range for output registers 1 & 2, 8x decimation on output register 2 (1.475khz output rate)
  transfer32_safe(buildWriteCommand(0x33, 0x202C)); delay(10);  //3V3, Data register clear delay, Sync High Active, Sync Timeout 1.4ms, Data Freezing for decimated and interpolated outputs, DRY High Active, DRY Enabled, SPI Strong Pulldown, MISO and DRY Slew Rates Control Enabled, 10MHz Mode
  transfer32_safe(buildWriteCommand(0x35, 0x0001)); delay(250); //Enable measurement
  for (uint8_t addr = 0x14; addr <= 0x1D; addr++) { 
    transfer32_safe(buildReadCommand(addr)); delay(5); 
  }
  transfer32_safe(buildWriteCommand(0x35, 0x0003)); delay(10);

  int flushCount = 0;
  while (digitalRead(SCH_DRY_PIN) == HIGH && flushCount < 500) { 
    transfer32_safe(READ_RATE_X2); 
    flushCount++; 
  }

  transfer32_safe(READ_RATE_X2); 
  SPI.endTransaction();

  // 4. Boot BNO08x
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT)) {
    Serial5.println("WARNING: BNO08x failed to wake up!"); 
  } else { //Checks the state of all accel, gyro, and mag
    bno08x.enableReport(SH2_ACCELEROMETER, 10000); 
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  }

  // 5. MASTER TIMELINE SYNC
  uint32_t absoluteStart = micros();
  schLastMicros = absoluteStart;
  adxlLastMicros = absoluteStart;
  schAbsoluteMicros = absoluteStart;
  adxlAbsoluteMicros = absoluteStart;
  lastSchWrite = absoluteStart;
  lastAdxlWrite = absoluteStart;
  lastBnoWrite = absoluteStart;
  bnoLastMicros = absoluteStart;

  schDataReady = false;
  adxlDataReady = false;
  
  Serial5.println("Sensors Armed and Logging Started.");
}


// ==========================================
// MAIN SETUP
// ==========================================

void setup() {
  unsigned long init_start = micros();
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  leds[0] = CRGB::White; FastLED.show();

  // CS & Data Ready Config
  pinMode(SCH_CS, OUTPUT);  digitalWrite(SCH_CS, HIGH);
  pinMode(ADXL_CS, OUTPUT); digitalWrite(ADXL_CS, HIGH);
  pinMode(BNO_CS, OUTPUT);  digitalWrite(BNO_CS, HIGH);
  pinMode(SCH_DRY_PIN, INPUT_PULLDOWN);
  pinMode(ADXL_DRDY_PIN, INPUT);

  // MOSI & SCLK Clamping
  pinMode(13, OUTPUT); digitalWrite(13, LOW); 
  pinMode(11, OUTPUT); digitalWrite(11, LOW); 
  delay(1000);

  // HARDWARE RESETS (Start them explicitly LOW so they don't boot)
  pinMode(SCH_RESET, OUTPUT); digitalWrite(SCH_RESET, LOW); 
  pinMode(BNO_RESET, OUTPUT); digitalWrite(BNO_RESET, LOW); 

  // Serial Config
  Serial.begin(115200);
  Serial5.begin(9600); 
  while (!Serial && millis() < 3000); 
  Serial5.println("MASA Sensor Payload Initializing...");

  leds[0] = CRGB::Blue; FastLED.show();

  // Initialize SD Card
  Serial5.print("Initializing SD card...");
  int sdRetries = 0;
  while (!SD.begin(SD_CS) && sdRetries < 5) { sdRetries++; delay(200); }
  if (sdRetries >= 5) errorHalt(CRGB::Red, "SD Card failed!");

  // Naming of microSD .bin files
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

  // ARM HARDWARE INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(ADXL_DRDY_PIN), adxl_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SCH_DRY_PIN), sch_ISR, RISING);

  // IMMEDIATELY SHUT DOWN THE SENSORS
  // The system will now enter Phase 0 with all sensors asleep
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
      previousPhase = currentPhase;
      currentPhase = newPhase;
      phaseChanged = true; 
    } 
  }

  bool isLowPower = (currentPhase == 0 || currentPhase == 4);

  // --- 2. PHASE TRANSITION EVENT TRIGGER ---
  if (phaseChanged) {
    bool wasLowPower = (previousPhase == 0 || previousPhase == 4);
    
    if (wasLowPower && !isLowPower) {
      // Waking up from pad to flight
      initializeSensors();
    } 
    else if (!wasLowPower && isLowPower) {
      // Landing and shutting down
      shutdownSensors();
    }
    phaseChanged = false;
  }

  // --- 3. THE LOW-POWER HALT ---
  if (isLowPower) {
    // We handle the 5-second LED blink, and then put the CPU to sleep.
    // The Teensy's internal SysTick (which increments millis) will safely wake 
    // it up every 1 millisecond, allowing it to check the LED and UART cleanly.
    
    static unsigned long lastBlinkTime = 0;
    static bool isLedOn = false;
    const unsigned long STANDBY_BLINK_INTERVAL = 5000; 
    const unsigned long BLINK_DURATION = 10;
    
    if (!isLedOn && (millis() - lastBlinkTime >= STANDBY_BLINK_INTERVAL)) {
      leds[0] = (currentPhase == 0) ? CRGB::Cyan : CRGB::Purple;
      FastLED.show();
      lastBlinkTime = millis();
      isLedOn = true;
    } 
    else if (isLedOn && (millis() - lastBlinkTime >= BLINK_DURATION)) {
      leds[0] = CRGB::Black;
      FastLED.show();
      isLedOn = false;
    }

    // Halt the 600MHz CPU to save power until the next SysTick or UART interrupt
    asm volatile("wfi");
  } 
  else {
    // ==========================================
    // HIGH-SPEED FLIGHT MODE (Phases 1, 2, 3)
    // ==========================================

    // Failsafe
    if (digitalRead(SCH_DRY_PIN) == HIGH && !schDataReady) { schLastMicros = micros(); schDataReady = true; }

    // --- SCH16T PROCESS ---
    if (schDataReady) {
      noInterrupts();
      schPack.timestamp = schAbsoluteMicros;
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

      schPack.phase = currentPhase;
      schPack.delta_t = (uint32_t)(schAbsoluteMicros - lastSchWrite); 
      schPack.rateX = (int16_t)((respRx >> 3) & 0xFFFF); schPack.rateY = (int16_t)((respRy >> 3) & 0xFFFF); schPack.rateZ = (int16_t)((respRz >> 3) & 0xFFFF);
      schPack.accX  = (int16_t)((respAx >> 3) & 0xFFFF); schPack.accY  = (int16_t)((respAy >> 3) & 0xFFFF); schPack.accZ  = (int16_t)((respAz >> 3) & 0xFFFF);
      dataFile.write((uint8_t*)&schPack, sizeof(schPack));
      lastSchWrite = schAbsoluteMicros;
    }

    // --- ADXL359 PROCESS ---
    if (adxlDataReady) {
      noInterrupts();
      adxlPack.timestamp = adxlAbsoluteMicros; 
      adxlDataReady = false;
      interrupts();

      SPI.beginTransaction(adxlSettings);
      digitalWrite(ADXL_CS, LOW); SPI.transfer((ADXL_REG_DATA_START << 1) | 0x01);
      int32_t aX = readAdxl20Bit(); int32_t aY = readAdxl20Bit(); int32_t aZ = readAdxl20Bit();
      digitalWrite(ADXL_CS, HIGH);
      SPI.endTransaction();

      adxlPack.phase = currentPhase;
      adxlPack.delta_t = (uint32_t)(adxlAbsoluteMicros - lastAdxlWrite); 
      adxlPack.accX = aX; adxlPack.accY = aY; adxlPack.accZ = aZ;
      dataFile.write((uint8_t*)&adxlPack, sizeof(adxlPack));
      lastAdxlWrite = adxlAbsoluteMicros;
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
        dataFile.write((uint8_t*)&bnoPack, sizeof(bnoPack));
        lastBnoWrite = schAbsoluteMicros;
      }
    }

    // --- FLUSH LOGIC ---
    if (millis() - lastFlushTime >= FLUSH_INTERVAL) {
      if (dataFile) {
        dataFile.flush();
        lastFlushTime = millis();
        leds[0] = CRGB::Blue; FastLED.show();
      }
    }

    // --- ACTIVE FLIGHT HEARTBEAT ---
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