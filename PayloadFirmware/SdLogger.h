#pragma once
#include <Arduino.h>
#include "SdFat.h"
#include "RingBuf.h"
#include "Config.h"

class SdLogger {
public:
  SdLogger();

  bool begin();
  void write(const void* data, size_t len);
  void processWrites(); // Non-blocking background multi-sector SD drain
  void syncAndTruncate();

  size_t bytesUsed() const;
  const char* getFileName() const;

private:
  SdFs _sd;
  FsFile _dataFile;
  RingBuf<FsFile, RING_BUF_CAPACITY> _rb;
  char _fileName[13];
  bool _ready;
};
