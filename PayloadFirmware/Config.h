#pragma once
#include <Arduino.h>
#include <SPI.h>

// ==============================================================================
// HARDWARE PIN DEFINITIONS
// ==============================================================================

// WS2812B RGB Status LED
#define PIN_LED              23
#define NUM_LEDS             1

// Murata SCH16T-K10 (IC300)
#define PIN_SCH_CS           5
#define PIN_SCH_DRY          6
#define PIN_SCH_RESET        0

// Analog Devices ADXL359 (AC300)
#define PIN_ADXL_CS          10
#define PIN_ADXL_DRDY        9
#define PIN_ADXL_INT1        7
#define PIN_ADXL_INT2        8

// CEVA / Hillcrest BNO086 (U300)
#define PIN_BNO_CS           15
#define PIN_BNO_INT          16
#define PIN_BNO_RESET        14

// Kionix KX134-1211 (AC301)
#define PIN_KX_CS            3

// SPI Bus Clamping Pins
#define PIN_SPI_SCLK         13
#define PIN_SPI_MOSI         11

// ==============================================================================
// SPI BUS SETTINGS
// ==============================================================================
#define SCH16T_SPI_FREQ      10000000UL  // 10 MHz (Murata spec max)
#define ADXL359_SPI_FREQ     10000000UL  // 10 MHz (AD spec max)
#define KX134_SPI_FREQ       10000000UL  // 10 MHz (Kionix spec max)
#define BNO086_SPI_FREQ       3000000UL  // 3 MHz (Adafruit BNO SPI spec)

// ==============================================================================
// SENSOR CONFIGURATIONS
// ==============================================================================

// ADXL359 FIFO Configuration
// The FIFO holds 96 single-axis words (32 tri-axial samples = 288 bytes).
// We set watermark to 90 words (30 samples = 270 bytes data + 1 dummy byte = 271 bytes read).
#define ADXL_WATERMARK_WORDS 90
#define ADXL_BATCH_SAMPLES   30
#define ADXL_BURST_READ_LEN  (1 + (ADXL_BATCH_SAMPLES * 9)) // 271 bytes

// ==============================================================================
// SD STORAGE & RING BUFFER CONFIGURATION
// ==============================================================================
#define PRE_ALLOCATE_SIZE    (250UL * 1024UL * 1024UL) // 250 MB pre-allocated space
#define RING_BUF_CAPACITY    131072                    // 128 KB high-speed RAM buffer

// ==============================================================================
// TIMING & FLIGHT PHASES
// ==============================================================================
#define LOW_RATE_INTERVAL_US 1000000ULL                // 1 Hz interval for Standby/Landed
#define STANDBY_BLINK_MS     5000                      // 5 second blink in Standby
#define FLIGHT_BLINK_MS      5000                      // 5 second blink in Flight
#define BLINK_DURATION_MS    10                        // 10 ms flash
