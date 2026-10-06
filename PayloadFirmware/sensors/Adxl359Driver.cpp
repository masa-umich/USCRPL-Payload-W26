#include "Adxl359Driver.h"

// ADXL359 Register Addresses
static const uint8_t ADXL_REG_FIFO_DATA    = 0x08;
static const uint8_t ADXL_REG_FILTER       = 0x28;
static const uint8_t ADXL_REG_FIFO_SAMPLES = 0x29;
static const uint8_t ADXL_REG_INT_MAP      = 0x2A;
static const uint8_t ADXL_REG_SYNC         = 0x2B;
static const uint8_t ADXL_REG_RANGE        = 0x2C;
static const uint8_t ADXL_REG_POWER_CTL    = 0x2D;

Adxl359Driver::Adxl359Driver(uint8_t csPin, uint8_t int1Pin)
  : _csPin(csPin), _int1Pin(int1Pin),
    _spiSettings(ADXL359_SPI_FREQ, MSBFIRST, SPI_MODE0) {}

bool Adxl359Driver::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  pinMode(_int1Pin, INPUT);

  wake();
  return true;
}

void Adxl359Driver::sleep() {
  writeRegister(ADXL_REG_POWER_CTL, 0x01); // 0x01 Sets Standby bit
}

void Adxl359Driver::wake() {
  // 1. Configure Sync & Interrupt Polarity (0x00 = Active High INT1)
  writeRegister(ADXL_REG_SYNC, 0x00);
  delay(2);

  // 2. Set Measurement Range to +/- 40g (0x03)
  writeRegister(ADXL_REG_RANGE, 0x03);
  delay(2);

  // 3. Set Filter to 1000 Hz ODR / 250 Hz Corner (0x02)
  writeRegister(ADXL_REG_FILTER, 0x02);
  delay(2);

  // 4. Set FIFO Watermark to 90 samples (0x5A = 30 tri-axial sets)
  writeRegister(ADXL_REG_FIFO_SAMPLES, ADXL_WATERMARK_WORDS);
  delay(2);

  // 5. Route FIFO_FULL watermark interrupt to INT1 pin (0x02)
  writeRegister(ADXL_REG_INT_MAP, 0x02);
  delay(2);

  // 6. Enter Measurement Mode (0x00)
  writeRegister(ADXL_REG_POWER_CTL, 0x00);
  delay(5);
}

bool Adxl359Driver::readBatch(ADXL_Batch_Packet& batchPacket, uint8_t phase, uint64_t batchEndTimestamp) {
  SpiBus::beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);

  // Send FIFO read command byte: (0x08 << 1) | 0x01 = 0x11
  _rxBuffer[0] = SpiBus::transfer((ADXL_REG_FIFO_DATA << 1) | 0x01);

  // Read the 270 FIFO data bytes (30 samples * 9 bytes/sample)
  for (int i = 1; i < ADXL_BURST_READ_LEN; i++) {
    _rxBuffer[i] = SpiBus::transfer(0x00);
  }

  digitalWrite(_csPin, HIGH);
  SpiBus::endTransaction();

  batchPacket.phase = phase;
  batchPacket.timestamp = batchEndTimestamp;
  batchPacket.delta_t = 1000; // 1000 us per sample at 1000 Hz
  batchPacket.count = ADXL_BATCH_SAMPLES;

  // Unpack 30 tri-axial samples (20-bit two's complement sign-extended)
  for (int i = 0; i < ADXL_BATCH_SAMPLES; i++) {
    int offset = 1 + (i * 9);

    long aX = ((long)_rxBuffer[offset]     << 12) | ((long)_rxBuffer[offset + 1] << 4) | (_rxBuffer[offset + 2] >> 4);
    if (aX & 0x80000) aX |= 0xFFF00000;

    long aY = ((long)_rxBuffer[offset + 3] << 12) | ((long)_rxBuffer[offset + 4] << 4) | (_rxBuffer[offset + 5] >> 4);
    if (aY & 0x80000) aY |= 0xFFF00000;

    long aZ = ((long)_rxBuffer[offset + 6] << 12) | ((long)_rxBuffer[offset + 7] << 4) | (_rxBuffer[offset + 8] >> 4);
    if (aZ & 0x80000) aZ |= 0xFFF00000;

    batchPacket.samples[i].accX = aX;
    batchPacket.samples[i].accY = aY;
    batchPacket.samples[i].accZ = aZ;
  }

  return true;
}

void Adxl359Driver::writeRegister(uint8_t reg, uint8_t val) {
  SpiBus::beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SpiBus::transfer(reg << 1);
  SpiBus::transfer(val);
  digitalWrite(_csPin, HIGH);
  SpiBus::endTransaction();
}

uint8_t Adxl359Driver::readRegister(uint8_t reg) {
  SpiBus::beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SpiBus::transfer((reg << 1) | 0x01);
  uint8_t val = SpiBus::transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SpiBus::endTransaction();
  return val;
}
