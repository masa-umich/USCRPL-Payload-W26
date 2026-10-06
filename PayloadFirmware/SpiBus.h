#pragma once
#include <Arduino.h>
#include <SPI.h>

class SpiBus {
public:
  static void begin();
  static void beginTransaction(SPISettings settings);
  static void endTransaction();

  // DMA / Asynchronous transfer state tracking
  static void setDmaBusy(bool busy);
  static bool isDmaBusy();

  // Helper transfer functions
  static uint8_t transfer(uint8_t data);
  static uint16_t transfer16(uint16_t data);
  static uint32_t transfer32(uint32_t data);
  static void transfer(const void* txBuffer, void* rxBuffer, size_t count);

private:
  static volatile bool _dmaBusy;
};
