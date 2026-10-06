#include "SpiBus.h"

volatile bool SpiBus::_dmaBusy = false;

void SpiBus::begin() {
  SPI.begin();
}

void SpiBus::beginTransaction(SPISettings settings) {
  // Wait if an asynchronous DMA transfer is actively utilizing the SPI bus
  while (_dmaBusy) {
    yield();
  }
  SPI.beginTransaction(settings);
}

void SpiBus::endTransaction() {
  SPI.endTransaction();
}

void SpiBus::setDmaBusy(bool busy) {
  _dmaBusy = busy;
}

bool SpiBus::isDmaBusy() {
  return _dmaBusy;
}

uint8_t SpiBus::transfer(uint8_t data) {
  return SPI.transfer(data);
}

uint16_t SpiBus::transfer16(uint16_t data) {
  return SPI.transfer16(data);
}

uint32_t SpiBus::transfer32(uint32_t data) {
  return SPI.transfer32(data);
}

void SpiBus::transfer(const void* txBuffer, void* rxBuffer, size_t count) {
  SPI.transfer(txBuffer, rxBuffer, count);
}
