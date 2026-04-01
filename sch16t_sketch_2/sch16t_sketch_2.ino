#include <SPI.h>
#include "SCH16T.h"

// --- HARDWARE CONFIG ---
#define SCH_CS        5   
#define SCH_RESET     0   
#define BNO_CS        15  // Set this to your actual BNO CS pin to keep it quiet
#define SPI_SPEED     1000000 

// Sensor Settings
#define FILTER_FREQ   68
#define SENS_RATE     200
#define SENS_ACC      3200
#define DECIMATION    4

SCH16T_K10 imu(SPI, SCH_CS, SCH_RESET);
SCH16T_raw_data raw;
SCH16T_result result;
SCH16T_status status;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 3000);

  // 1. Silence the BNO086
  pinMode(BNO_CS, OUTPUT);
  digitalWrite(BNO_CS, HIGH); 

  // 2. Hardware Reset the SCH16T
  pinMode(SCH_RESET, OUTPUT);
  digitalWrite(SCH_RESET, LOW);
  delay(100);
  digitalWrite(SCH_RESET, HIGH);
  delay(2000); 

  SPI.begin();

  SCH16T_filter Filter = {(uint16_t)FILTER_FREQ, (uint16_t)FILTER_FREQ, (uint16_t)FILTER_FREQ};
  SCH16T_sensitivity Sensitivity = {(uint16_t)SENS_RATE, (uint16_t)SENS_RATE, (uint16_t)SENS_ACC, (uint16_t)SENS_ACC, (uint16_t)SENS_ACC};
  SCH16T_decimation Decimation = {(uint8_t)DECIMATION, (uint8_t)DECIMATION};

  Serial.println("Initializing SCH16T-K10...");
  
  // We call begin, but we don't 'while' loop on the error anymore.
  int init_status = imu.begin(Filter, Sensitivity, Decimation, false);
  
  Serial.print("Init Status: "); Serial.println(init_status);
  Serial.print("Serial Number: "); Serial.println(imu.getSnbr());
  Serial.println("Timestamp(us), Status, GX, GY, GZ, AX, AY, AZ, Temp(C)");
}

void loop() {
  unsigned long currentTime = micros();
  
  // 1. Grab everything from the chip
  imu.getData(&raw);
  imu.convertData(&raw, &result);
  imu.getStatus(&status);

  // 2. CSV Style Output (Easy to copy into Excel/Matlab)
  Serial.print(currentTime); Serial.print(", ");
  
  // Print Summary Status in HEX
  Serial.print("0x"); Serial.print(status.Summary, HEX); Serial.print(", ");

  // Gyro Rates (dps)
  Serial.print(result.Rate1[0], 3); Serial.print(", ");
  Serial.print(result.Rate1[1], 3); Serial.print(", ");
  Serial.print(result.Rate1[2], 3); Serial.print(", ");

  // Accel (m/s^2)
  Serial.print(result.Acc1[0], 3); Serial.print(", ");
  Serial.print(result.Acc1[1], 3); Serial.print(", ");
  Serial.print(result.Acc1[2], 3); Serial.print(", ");

  // Internal Temp
  Serial.println(result.Temp, 2);

  delay(20); // ~50Hz output for the serial monitor
}