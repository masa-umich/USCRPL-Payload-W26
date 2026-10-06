#include "Kx134Driver.h"

// KX134-1211 Register Addresses
static const uint8_t KX134_REG_XOUT_L    = 0x08;
static const uint8_t KX134_REG_WHO_AM_I  = 0x13;
static const uint8_t KX134_REG_INS2      = 0x17;
static const uint8_t KX134_REG_CNTL1     = 0x1B;
static const uint8_t KX134_REG_ODCNTL    = 0x1F;

Kx134Driver::Kx134Driver(uint8_t csPin)
  : _csPin(csPin), _spiSettings(KX134_SPI_FREQ, MSBFIRST, SPI_MODE0) {}

bool Kx134Driver::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  delay(10);

  // Check WHO_AM_I (KX134 returns 0x46 or 0x47 / 0x48)
  uint8_t whoami = readRegister(KX134_REG_WHO_AM_I);
  if (whoami == 0x00 || whoami == 0xFF) {
    // Sensor not responding, retry once
    delay(20);
    whoami = readRegister(KX134_REG_WHO_AM_I);
  }

  wake();
  return true;
}

void Kx134Driver::sleep() {
  // Put KX134 in Standby mode (PC1 = 0 in CNTL1)
  writeRegister(KX134_REG_CNTL1, 0x00);
}

void Kx134Driver::wake() {
  // 1. Put in standby to allow configuration writes
  writeRegister(KX134_REG_CNTL1, 0x00);
  delay(5);

  // 2. Set ODR to 1600 Hz (0x0B)
  writeRegister(KX134_REG_ODCNTL, 0x0B);
  delay(2);

  // 3. Set Range to +/- 64g, High Resolution mode, Enable Data Ready, and Power ON
  // CNTL1: PC1=1 (0x80), RES=1 (0x40), DRDYE=1 (0x20), GSEL=64g (0x18) -> 0xF8
  writeRegister(KX134_REG_CNTL1, 0xF8);
  delay(10);
}

bool Kx134Driver::dataReady() {
  // Check DRDY bit (bit 4) in INS2 register
  uint8_t ins2 = readRegister(KX134_REG_INS2);
  return (ins2 & 0x10) != 0;
}

bool Kx134Driver::readSample(KX_Packet& packet, uint8_t phase, uint64_t timestamp, uint32_t dt) {
  uint8_t rawBuf[6];
  readRegisters(KX134_REG_XOUT_L, rawBuf, 6);

  packet.phase = phase;
  packet.timestamp = timestamp;
  packet.delta_t = dt;

  packet.accX = (int16_t)((uint16_t)rawBuf[0] | ((uint16_t)rawBuf[1] << 8));
  packet.accY = (int16_t)((uint16_t)rawBuf[2] | ((uint16_t)rawBuf[3] << 8));
  packet.accZ = (int16_t)((uint16_t)rawBuf[4] | ((uint16_t)rawBuf[5] << 8));

  return true;
}

void Kx134Driver::writeRegister(uint8_t reg, uint8_t val) {
  SpiBus::beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SpiBus::transfer(reg & 0x7F); // Write: MSB = 0
  SpiBus::transfer(val);
  digitalWrite(_csPin, HIGH);
  SpiBus::endTransaction();
}

uint8_t Kx134Driver::readRegister(uint8_t reg) {
  SpiBus::beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SpiBus::transfer(reg | 0x80); // Read: MSB = 1
  uint8_t val = SpiBus::transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SpiBus::endTransaction();
  return val;
}

void Kx134Driver::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
  SpiBus::beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SpiBus::transfer(startReg | 0x80); // Read: MSB = 1
  for (size_t i = 0; i < len; i++) {
    buffer[i] = SpiBus::transfer(0x00);
  }
  digitalWrite(_csPin, HIGH);
  SpiBus::endTransaction();
}
