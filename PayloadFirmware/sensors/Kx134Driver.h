#pragma once
#include <Arduino.h>
#include "../Config.h"
#include "../Packets.h"
#include "../SpiBus.h"

class Kx134Driver {
public:
  Kx134Driver(uint8_t csPin = PIN_KX_CS);

  bool begin();
  void sleep();
  void wake();

  bool dataReady();
  bool readSample(KX_Packet& packet, uint8_t phase, uint64_t timestamp, uint32_t dt);

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t startReg, uint8_t* buffer, size_t len);

private:
  uint8_t _csPin;
  SPISettings _spiSettings;
};
