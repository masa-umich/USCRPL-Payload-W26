#include <Arduino.h>
#include "Config.h"
#include "Packets.h"
#include "SpiBus.h"
#include "SdLogger.h"
#include "PhaseManager.h"
#include "sensors/Sch16tDriver.h"
#include "sensors/Adxl359Driver.h"
#include "sensors/Kx134Driver.h"
#include "sensors/Bno086Driver.h"

// ==============================================================================
// SYSTEM INSTANCES
// ==============================================================================
SdLogger       sdLogger;
PhaseManager   phaseManager;
Sch16tDriver   schDriver(PIN_SCH_CS, PIN_SCH_DRY, PIN_SCH_RESET);
Adxl359Driver  adxlDriver(PIN_ADXL_CS, PIN_ADXL_INT1);
Kx134Driver    kxDriver(PIN_KX_CS);
Bno086Driver   bnoDriver(PIN_BNO_CS, PIN_BNO_INT, PIN_BNO_RESET);

// ==============================================================================
// TELEMETRY PACKET INSTANCES
// ==============================================================================
SCH_Packet        schPacket;
ADXL_Batch_Packet adxlBatchPacket;
KX_Packet         kxPacket;
BNO_Packet        bnoPacket;

// ==============================================================================
// INTERRUPT FLAGS & TIMING VARIABLES
// ==============================================================================
volatile bool     schDataReady = false;
volatile uint64_t schTimestamp = 0;
uint64_t          lastSchTimestamp = 0;

volatile bool     adxlDataReady = false;
volatile uint64_t adxlTimestamp = 0;

uint64_t          lastKxTimestamp = 0;
uint64_t          lastBnoTimestamp = 0;

// Low-Rate Timers (Standby & Landed Phases)
uint64_t          lastLowRateSch = 0;
uint64_t          lastLowRateAdxl = 0;
uint64_t          lastLowRateKx = 0;
uint64_t          lastLowRateBno = 0;

// ==============================================================================
// INTERRUPT SERVICE ROUTINES
// ==============================================================================
void sch_ISR() {
  schTimestamp = micros();
  schDataReady = true;
}

void adxl_ISR() {
  adxlTimestamp = micros();
  adxlDataReady = true;
}

// ==============================================================================
// SENSOR POWER MANAGEMENT
// ==============================================================================
void shutdownSensors() {
  Serial5.println("[PWR] Entering Low-Power Standby Mode...");
  schDriver.sleep();
  adxlDriver.sleep();
  kxDriver.sleep();
  bnoDriver.sleep();
  sdLogger.syncAndTruncate();
}

void initializeSensors() {
  Serial5.println("[PWR] Waking Sensors for Active Flight Logging...");
  schDriver.wake();
  adxlDriver.wake();
  kxDriver.wake();
  bnoDriver.wake();

  uint64_t startMicros = micros();
  lastSchTimestamp = startMicros;
  lastKxTimestamp = startMicros;
  lastBnoTimestamp = startMicros;
  schDataReady = false;
  adxlDataReady = false;
}

void onPhaseChange(FlightPhase oldPhase, FlightPhase newPhase) {
  bool wasLowPower = (oldPhase == PHASE_STANDBY || oldPhase == PHASE_LANDED);
  bool isLowPower  = (newPhase == PHASE_STANDBY || newPhase == PHASE_LANDED);

  if (wasLowPower && !isLowPower) {
    initializeSensors();
  } else if (!wasLowPower && isLowPower) {
    shutdownSensors();
  }
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  // SCLK and MOSI Clamping for power-on stability
  pinMode(PIN_SPI_SCLK, OUTPUT); digitalWrite(PIN_SPI_SCLK, LOW);
  pinMode(PIN_SPI_MOSI, OUTPUT); digitalWrite(PIN_SPI_MOSI, LOW);
  delay(500);

  Serial.begin(115200);
  phaseManager.begin(onPhaseChange);

  Serial5.println("==========================================");
  Serial5.println("   MASA Daybreak High-Speed IMU DAQ       ");
  Serial5.println("==========================================");

  // Initialize Safe SPI Bus
  SpiBus::begin();

  // Initialize SD Card Logger
  Serial5.print("Initializing SD Storage... ");
  if (!sdLogger.begin()) {
    Serial5.println("FAILED!");
  } else {
    Serial5.print("OK. Logging to: ");
    Serial5.println(sdLogger.getFileName());
  }

  // Initialize Sensor Hardware
  Serial5.print("Initializing Murata SCH16T... ");
  schDriver.begin();
  Serial5.println("OK.");

  Serial5.print("Initializing ADXL359... ");
  adxlDriver.begin();
  Serial5.println("OK.");

  Serial5.print("Initializing KX134-1211... ");
  kxDriver.begin();
  Serial5.println("OK.");

  Serial5.print("Initializing BNO086... ");
  bnoDriver.begin();
  Serial5.println("OK.");

  // Register Hardware Interrupts
  SPI.usingInterrupt(digitalPinToInterrupt(PIN_SCH_DRY));
  SPI.usingInterrupt(digitalPinToInterrupt(PIN_ADXL_INT1));

  attachInterrupt(digitalPinToInterrupt(PIN_SCH_DRY), sch_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ADXL_INT1), adxl_ISR, RISING);

  // Start in Standby Mode
  shutdownSensors();
  Serial5.println("System Ready in Standby Mode (Phase 0).");
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  phaseManager.update();
  uint8_t currentPhase = (uint8_t)phaseManager.getPhase();
  uint64_t now = micros();

  if (phaseManager.isLowPower()) {
    // --------------------------------------------------------------------------
    // LOW-POWER STANDBY / LANDED LOGGING (1 Hz Interval)
    // --------------------------------------------------------------------------
    if (now - lastLowRateSch >= LOW_RATE_INTERVAL_US) {
      if (schDriver.readSample(schPacket, currentPhase, now, (uint32_t)(now - lastLowRateSch))) {
        sdLogger.write(&schPacket, sizeof(schPacket));
      }
      lastLowRateSch = now;
    }

    if (now - lastLowRateAdxl >= LOW_RATE_INTERVAL_US) {
      if (adxlDriver.readBatch(adxlBatchPacket, currentPhase, now)) {
        sdLogger.write(&adxlBatchPacket, sizeof(adxlBatchPacket));
      }
      lastLowRateAdxl = now;
    }

    if (now - lastLowRateKx >= LOW_RATE_INTERVAL_US) {
      if (kxDriver.readSample(kxPacket, currentPhase, now, (uint32_t)(now - lastLowRateKx))) {
        sdLogger.write(&kxPacket, sizeof(kxPacket));
      }
      lastLowRateKx = now;
    }

    if (now - lastLowRateBno >= LOW_RATE_INTERVAL_US) {
      if (bnoDriver.checkAndRead(bnoPacket, currentPhase, now, (uint32_t)(now - lastLowRateBno))) {
        sdLogger.write(&bnoPacket, sizeof(bnoPacket));
      }
      lastLowRateBno = now;
    }

    sdLogger.processWrites();
    asm volatile("wfi"); // Low-power sleep until next hardware interrupt or UART byte
  } else {
    // --------------------------------------------------------------------------
    // HIGH-SPEED FLIGHT LOGGING (Phases 1, 2, 3)
    // --------------------------------------------------------------------------

    // 1. SCH16T-K10 (1.475 kHz)
    if (schDataReady || digitalRead(PIN_SCH_DRY) == HIGH) {
      noInterrupts();
      uint64_t tSnap = schTimestamp;
      schDataReady = false;
      interrupts();

      if (tSnap == 0) tSnap = now;
      uint32_t dt = (uint32_t)(tSnap - lastSchTimestamp);
      lastSchTimestamp = tSnap;

      schDriver.readSample(schPacket, currentPhase, tSnap, dt);
      sdLogger.write(&schPacket, sizeof(schPacket));
    }

    // 2. ADXL359 30-Sample Batch Dump (1.0 kHz ODR, ~33.3 Hz Batch Rate)
    if (adxlDataReady || digitalRead(PIN_ADXL_INT1) == HIGH) {
      noInterrupts();
      uint64_t tBatchEnd = adxlTimestamp;
      adxlDataReady = false;
      interrupts();

      if (tBatchEnd == 0) tBatchEnd = now;

      adxlDriver.readBatch(adxlBatchPacket, currentPhase, tBatchEnd);
      sdLogger.write(&adxlBatchPacket, sizeof(adxlBatchPacket));
    }

    // 3. KX134-1211 High-G Accelerometer (1.6 kHz)
    if (kxDriver.dataReady()) {
      uint32_t dt = (uint32_t)(now - lastKxTimestamp);
      lastKxTimestamp = now;

      kxDriver.readSample(kxPacket, currentPhase, now, dt);
      sdLogger.write(&kxPacket, sizeof(kxPacket));
    }

    // 4. BNO086 AHRS Reports (100 Hz, Hardware-Gated)
    if (digitalRead(PIN_BNO_INT) == LOW) {
      uint32_t dt = (uint32_t)(now - lastBnoTimestamp);
      if (bnoDriver.checkAndRead(bnoPacket, currentPhase, now, dt)) {
        sdLogger.write(&bnoPacket, sizeof(bnoPacket));
        lastBnoTimestamp = now;
      }
    }

    // 5. Non-Blocking Background SDIO Multi-Sector Draining
    sdLogger.processWrites();
  }
}
