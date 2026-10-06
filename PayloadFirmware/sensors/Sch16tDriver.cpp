#include "Sch16tDriver.h"

// Murata SCH16T 32-bit Read Commands (DEC8, Channel 2)
static const uint32_t CMD_READ_RATE_X2 = 0x02800001;
static const uint32_t CMD_READ_RATE_Y2 = 0x02C00003;
static const uint32_t CMD_READ_RATE_Z2 = 0x03000006;
static const uint32_t CMD_READ_ACC_X2  = 0x03400004;
static const uint32_t CMD_READ_ACC_Y2  = 0x03800002;
static const uint32_t CMD_READ_ACC_Z2  = 0x03C00000;

Sch16tDriver::Sch16tDriver(uint8_t csPin, uint8_t dryPin, uint8_t resetPin)
  : _csPin(csPin), _dryPin(dryPin), _resetPin(resetPin),
    _spiSettings(SCH16T_SPI_FREQ, MSBFIRST, SPI_MODE0) {}

bool Sch16tDriver::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  pinMode(_dryPin, INPUT_PULLDOWN);
  pinMode(_resetPin, OUTPUT);

  // Hardware Reset Sequence
  digitalWrite(_resetPin, HIGH);
  delay(10);
  digitalWrite(_resetPin, LOW);
  delay(10);
  digitalWrite(_resetPin, HIGH);
  delay(100);

  // Initialization sequence for DEC8 (1.475 kHz ODR)
  SpiBus::beginTransaction(_spiSettings);
  delay(50);
  transfer32(buildWriteCommand(0x36, 0x000A)); delay(40);
  transfer32(buildWriteCommand(0x28, 0x12DB));
  transfer32(buildWriteCommand(0x29, 0x12DB)); delay(5);
  transfer32(buildWriteCommand(0x33, 0x202C)); delay(10);
  transfer32(buildWriteCommand(0x35, 0x0001)); delay(250);

  for (uint8_t addr = 0x14; addr <= 0x1D; addr++) {
    transfer32(buildReadCommand(addr)); delay(5);
  }
  transfer32(buildWriteCommand(0x35, 0x0003)); delay(10);

  // Flush stale frames
  int flushCount = 0;
  while (digitalRead(_dryPin) == HIGH && flushCount < 500) {
    transfer32(CMD_READ_RATE_X2);
    flushCount++;
  }
  transfer32(CMD_READ_RATE_X2); // Prime pipeline with initial command
  SpiBus::endTransaction();

  return true;
}

void Sch16tDriver::sleep() {
  digitalWrite(_resetPin, LOW);
}

void Sch16tDriver::wake() {
  begin();
}

bool Sch16tDriver::readSample(SCH_Packet& packet, uint8_t phase, uint64_t timestamp, uint32_t dt) {
  SpiBus::beginTransaction(_spiSettings);
  uint32_t respRx = transfer32(CMD_READ_RATE_Y2);
  uint32_t respRy = transfer32(CMD_READ_RATE_Z2);
  uint32_t respRz = transfer32(CMD_READ_ACC_X2);
  uint32_t respAx = transfer32(CMD_READ_ACC_Y2);
  uint32_t respAy = transfer32(CMD_READ_ACC_Z2);
  uint32_t respAz = transfer32(CMD_READ_RATE_X2); // Pre-load next loop
  SpiBus::endTransaction();

  packet.phase = phase;
  packet.timestamp = timestamp;
  packet.delta_t = dt;

  // Unpack 16-bit signed data from 32-bit frames (bits [23:8])
  packet.rateX = (int16_t)((respRx >> 8) & 0xFFFF);
  packet.rateY = (int16_t)((respRy >> 8) & 0xFFFF);
  packet.rateZ = (int16_t)((respRz >> 8) & 0xFFFF);
  packet.accX  = (int16_t)((respAx >> 8) & 0xFFFF);
  packet.accY  = (int16_t)((respAy >> 8) & 0xFFFF);
  packet.accZ  = (int16_t)((respAz >> 8) & 0xFFFF);

  return true;
}

uint32_t Sch16tDriver::transfer32(uint32_t data) {
  digitalWrite(_csPin, LOW);
  uint32_t result = 0;
  result |= ((uint32_t)SpiBus::transfer((data >> 24) & 0xFF)) << 24;
  result |= ((uint32_t)SpiBus::transfer((data >> 16) & 0xFF)) << 16;
  result |= ((uint32_t)SpiBus::transfer((data >> 8)  & 0xFF)) << 8;
  result |= ((uint32_t)SpiBus::transfer(data & 0xFF));
  digitalWrite(_csPin, HIGH);
  return result;
}

uint32_t Sch16tDriver::buildWriteCommand(uint8_t address, uint16_t data) {
  uint32_t frame = 0;
  frame |= ((uint32_t)address & 0xFF) << 22;
  frame |= (1UL << 21); // Write bit
  frame |= (uint32_t)data << 3;
  return frame | calcCRC3(frame >> 3);
}

uint32_t Sch16tDriver::buildReadCommand(uint8_t address) {
  uint32_t frame = 0;
  frame |= ((uint32_t)address & 0xFF) << 22;
  frame |= (0UL << 21); // Read bit
  return frame | calcCRC3(frame >> 3);
}

uint8_t Sch16tDriver::calcCRC3(uint32_t frame29) {
  uint8_t crc = 0x05;
  for (int i = 28; i >= 0; i--) {
    bool bit = (frame29 >> i) & 0x01;
    bool msb = (crc >> 2) & 0x01;
    crc = ((crc << 1) & 0x07) | bit;
    if (msb) crc ^= 0x03;
  }
  for (int i = 0; i < 3; i++) {
    bool msb = (crc >> 2) & 0x01;
    crc = (crc << 1) & 0x07;
    if (msb) crc ^= 0x03;
  }
  return crc;
}
