#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "../Config.h"
#include "../Packets.h"
#include "../SpiBus.h"

class Bno086Driver {
public:
  Bno086Driver(uint8_t csPin = PIN_BNO_CS, uint8_t intPin = PIN_BNO_INT, uint8_t resetPin = PIN_BNO_RESET);

  bool begin();
  void sleep();
  void wake();

  // Non-blocking hardware INT check and report reader
  bool checkAndRead(BNO_Packet& packet, uint8_t phase, uint64_t timestamp, uint32_t dt);

private:
  uint8_t _csPin;
  uint8_t _intPin;
  uint8_t _resetPin;
  Adafruit_BNO08x _bno;
  bool _initialized;
};
