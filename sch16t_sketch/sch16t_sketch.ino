#include "SCH16T.h"

#define FILTER_RATE         68.0f      // Hz, LPF0 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12        68.0f
#define FILTER_ACC3         68.0f
#define SENSITIVITY_RATE1   200.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_RATE2   200.0f
#define SENSITIVITY_ACC1    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_ACC2    3200.0f
#define SENSITIVITY_ACC3    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define DECIMATION_RATE     4          // DEC2, Output sample rate decimation. Nominal output rate of 5.9kHz.
#define DECIMATION_ACC      4

#define SPI_OBJECT          SPI         // Some platforms have additional SPI intefaces under different names (e.g. SPI1)
#define CS_PIN              5
#define RESET_PIN           -1          //leave as -1 if not used

SCH16T_K10 imu(SPI_OBJECT, CS_PIN, RESET_PIN);

SCH16T_raw_data raw;    // Struct to hold the bits coming off the sensor
SCH16T_result result;    // Struct to hold the converted floats (dps, m/s^2)

char serial_num[15];
int  init_status = SCH16T_ERR_OTHER;
SCH16T_filter         Filter;
SCH16T_sensitivity    Sensitivity;
SCH16T_decimation     Decimation;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Up...");
    SPI_OBJECT.begin();    // Initialize SPI hardware before initializing sensor

    delay(5000);

    Filter.Rate12 = FILTER_RATE;
    Filter.Acc12  = FILTER_ACC12;
    Filter.Acc3   = FILTER_ACC3;

    Sensitivity.Rate1 = SENSITIVITY_RATE1;
    Sensitivity.Rate2 = SENSITIVITY_RATE2;
    Sensitivity.Acc1  = SENSITIVITY_ACC1;
    Sensitivity.Acc2  = SENSITIVITY_ACC2;
    Sensitivity.Acc3  = SENSITIVITY_ACC3;

    Decimation.Rate2 = DECIMATION_RATE;
    Decimation.Acc2  = DECIMATION_ACC;
    
    // Manual SPI Test
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Force 1MHz
    digitalWrite(CS_PIN, LOW);
    uint16_t response = SPI.transfer16(0xFFFF); // Use 0xFFFF as a 'dummy' to keep MOSI high
    Serial.println(response);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    while (init_status != SCH16T_OK)
    {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        init_status = imu.begin(Filter, Sensitivity, Decimation, false);
        if (init_status != SCH16T_OK) {
            // Add this inside your setup loop when it fails
            SCH16T_status statusData;
            imu.getStatus(&statusData);

            Serial.print("Status Summary: "); Serial.println(statusData.Summary, HEX);
            Serial.print("Common Status: ");  Serial.println(statusData.Common, HEX);
            Serial.print("Rate Status: ");    Serial.println(statusData.Rate_X, HEX);
            Serial.print("Acc Status: ");     Serial.println(statusData.Acc_X, HEX);
            
            delay(2000);

            
        }
    }

    // Read serial number from the sensor.
    strcpy(serial_num, imu.getSnbr());
    Serial.print("Serial Number: ");
    Serial.println(serial_num); 
}

/*void loop() {
    imu.getData(&raw);
    imu.convertData(&raw, &result);

    Serial.print("Gyro X: ");
    Serial.println(result.Rate1[SCH16T_AXIS_X]);
    Serial.print("Gyro Y: ");
    Serial.println(result.Rate1[SCH16T_AXIS_Y]);
    Serial.print("Gyro Z: ");
    Serial.println(result.Rate1[SCH16T_AXIS_Z]);
    Serial.println();

    Serial.print("Accel X: ");
    Serial.println(result.Acc1[SCH16T_AXIS_X]);
    Serial.print("Accel Y: ");
    Serial.println(result.Acc1[SCH16T_AXIS_Y]);
    Serial.print("Accel Z: ");
    Serial.println(result.Acc1[SCH16T_AXIS_Z]);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.println(result.Temp);
    Serial.println();
    Serial.println();

    delay(100);
} */

void loop() {
  pinMode(CS_PIN, OUTPUT); // Ensure it's output
  
  // Try MODE3 if MODE0 fails
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); 
  
  // Frame 1: Send Command
  digitalWrite(CS_PIN, LOW);
  SPI.transfer16(0xFC00); 
  digitalWrite(CS_PIN, HIGH);
  
  delayMicroseconds(10); // Small gap between frames

  // Frame 2: Retrieve Response
  digitalWrite(CS_PIN, LOW);
  uint16_t resp = SPI.transfer16(0x0000); // Dummy to clock out data
  digitalWrite(CS_PIN, HIGH);
  
  SPI.endTransaction();

  Serial.print("Sensor Response: 0x");
  Serial.println(resp, HEX);
  delay(1000);
}