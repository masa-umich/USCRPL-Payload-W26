#include "SdLogger.h"

SdLogger::SdLogger()
  : _fileName("IMULOG00.BIN"), _ready(false) {}

bool SdLogger::begin() {
  // Initialize SDIO FIFO interface
  int sdRetries = 0;
  while (!_sd.begin(SdioConfig(FIFO_SDIO)) && sdRetries < 5) {
    sdRetries++;
    delay(200);
  }
  if (sdRetries >= 5) {
    return false;
  }

  // Find next available file name (IMULOG00.BIN to IMULOG99.BIN)
  for (uint8_t i = 0; i < 100; i++) {
    _fileName[6] = i / 10 + '0';
    _fileName[7] = i % 10 + '0';
    if (!_sd.exists(_fileName)) {
      _dataFile = _sd.open(_fileName, O_RDWR | O_CREAT | O_TRUNC);
      break;
    }
  }

  if (!_dataFile) {
    return false;
  }

  // Pre-allocate contiguous space to prevent runtime FAT allocation stalls
  if (!_dataFile.preAllocate(PRE_ALLOCATE_SIZE)) {
    Serial.println("WARNING: SD card pre-allocation failed. Write speeds may be impacted.");
  }

  _rb.begin(&_dataFile);
  _ready = true;
  return true;
}

void SdLogger::write(const void* data, size_t len) {
  if (!_ready) return;
  _rb.write((const uint8_t*)data, len);
}

void SdLogger::processWrites() {
  if (!_ready || _dataFile.isBusy()) return;

  size_t used = _rb.bytesUsed();

  // Multi-sector burst draining: drains aggressively when buffer builds up
  if (used >= 4096) {
    _rb.writeOut(4096);
  } else if (used >= 512) {
    _rb.writeOut(512);
  }
}

void SdLogger::syncAndTruncate() {
  if (!_ready || !_dataFile) return;

  _rb.sync();           // Push remaining RAM buffer to physical SD card
  _dataFile.truncate(); // Remove unused pre-allocated space
  _dataFile.flush();
}

size_t SdLogger::bytesUsed() const {
  return _rb.bytesUsed();
}

const char* SdLogger::getFileName() const {
  return _fileName;
}
