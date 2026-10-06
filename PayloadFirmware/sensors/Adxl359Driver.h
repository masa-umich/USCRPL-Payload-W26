#pragma once
#include <Arduino.h>
#include "../Config.h"
#include "../Packets.h"
#include "../SpiBus.h"

class Adxl359Driver {
public:
  Adxl359Driver(uint8_t csPin = PIN_ADXL_CS, uint8_t int1Pin = PIN_ADXL_INT1);

  bool begin();
  void sleep();
  void wake();

  // Reads a 30-sample batch from FIFO (271 bytes burst read)
  bool readBatch(ADXL_Batch_Packet& batchPacket, uint8_t phase, uint64_t batchEndTimestamp);

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);

private:
  uint8_t _csPin;
  uint8_t _int1Pin;
  SPISettings _spiSettings;
  uint8_t _rxBuffer[ADXL_BURST_READ_LEN];
};
