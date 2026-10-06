#pragma once
#include <stdint.h>

#define PACKET_SYNC_1 0xAA
#define PACKET_SYNC_2 0xBB

#define PACKET_TYPE_SCH  'S'
#define PACKET_TYPE_ADXL 'A'
#define PACKET_TYPE_KX   'K'
#define PACKET_TYPE_BNO  'B'

#pragma pack(push, 1)

// --- Murata SCH16T-K10 Packet (28 Bytes) ---
struct SCH_Packet {
  uint8_t  sync1 = PACKET_SYNC_1;
  uint8_t  sync2 = PACKET_SYNC_2;
  char     type  = PACKET_TYPE_SCH;
  uint8_t  phase = 0;
  uint64_t timestamp = 0; // Microseconds
  uint32_t delta_t = 0;   // Microseconds since last sample
  int16_t  rateX = 0, rateY = 0, rateZ = 0; // Gyro (raw counts)
  int16_t  accX  = 0, accY  = 0, accZ  = 0; // Accel (raw counts)
};

// --- ADXL359 Tri-Axial Sample (12 Bytes) ---
struct ADXL_Sample {
  int32_t accX; // 20-bit sign-extended raw value
  int32_t accY;
  int32_t accZ;
};

// --- ADXL359 Compressed Batch Packet (377 Bytes) ---
// Emits 30 samples under a single header, reducing SD logging overhead by ~50%
struct ADXL_Batch_Packet {
  uint8_t     sync1 = PACKET_SYNC_1;
  uint8_t     sync2 = PACKET_SYNC_2;
  char        type  = PACKET_TYPE_ADXL;
  uint8_t     phase = 0;
  uint64_t    timestamp = 0; // Timestamp of latest sample in batch (sample index 29)
  uint32_t    delta_t   = 1000; // Expected delta per sample (1000 us for 1kHz ODR)
  uint8_t     count     = 30;
  ADXL_Sample samples[30];
};

// --- Kionix KX134-1211 High-G Accelerometer Packet (22 Bytes) ---
struct KX_Packet {
  uint8_t  sync1 = PACKET_SYNC_1;
  uint8_t  sync2 = PACKET_SYNC_2;
  char     type  = PACKET_TYPE_KX;
  uint8_t  phase = 0;
  uint64_t timestamp = 0;
  uint32_t delta_t   = 0;
  int16_t  accX = 0, accY = 0, accZ = 0; // 16-bit raw counts
};

// --- BNO086 AHRS Packet (29 Bytes) ---
struct BNO_Packet {
  uint8_t  sync1 = PACKET_SYNC_1;
  uint8_t  sync2 = PACKET_SYNC_2;
  char     type  = PACKET_TYPE_BNO;
  uint8_t  phase = 0;
  uint64_t timestamp = 0;
  uint32_t delta_t   = 0;
  uint8_t  sensorId  = 0; // 0x01 = Accel, 0x02 = Gyro, 0x03 = Mag
  float    x = 0.0f, y = 0.0f, z = 0.0f;
};

#pragma pack(pop)
