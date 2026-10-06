#pragma once
#include <Arduino.h>
#include "../Config.h"
#include "../Packets.h"
#include "../SpiBus.h"

class Sch16tDriver {
public:
  Sch16tDriver(uint8_t csPin = PIN_SCH_CS, uint8_t dryPin = PIN_SCH_DRY, uint8_t resetPin = PIN_SCH_RESET);

  bool begin();
  void sleep();
  void wake();

  // Reads the full 6-axis dataset using pipelined 32-bit SPI frames
  bool readSample(SCH_Packet& packet, uint8_t phase, uint64_t timestamp, uint32_t dt);

  // Command builders & CRC
  static uint32_t buildWriteCommand(uint8_t address, uint16_t data);
  static uint32_t buildReadCommand(uint8_t address);
  static uint8_t  calcCRC3(uint32_t frame29);

private:
  uint8_t _csPin;
  uint8_t _dryPin;
  uint8_t _resetPin;
  SPISettings _spiSettings;

  uint32_t transfer32(uint32_t data);
};
