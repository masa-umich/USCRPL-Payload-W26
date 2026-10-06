#include "Bno086Driver.h"

Bno086Driver::Bno086Driver(uint8_t csPin, uint8_t intPin, uint8_t resetPin)
  : _csPin(csPin), _intPin(intPin), _resetPin(resetPin),
    _bno(resetPin), _initialized(false) {}

bool Bno086Driver::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  pinMode(_intPin, INPUT_PULLUP);
  pinMode(_resetPin, OUTPUT);

  // Hardware Reset Sequence
  digitalWrite(_resetPin, HIGH);
  delay(10);
  digitalWrite(_resetPin, LOW);
  delay(10);
  digitalWrite(_resetPin, HIGH);
  delay(100);

  wake();
  return _initialized;
}

void Bno086Driver::sleep() {
  digitalWrite(_resetPin, LOW);
  _initialized = false;
}

void Bno086Driver::wake() {
  // Start Adafruit BNO08x SPI
  if (_bno.begin_SPI(_csPin, _intPin)) {
    _bno.enableReport(SH2_ACCELEROMETER, 10000);              // 100 Hz (10,000 us)
    _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);       // 100 Hz
    _bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);  // 100 Hz
    _initialized = true;
  } else {
    _initialized = false;
  }
}

bool Bno086Driver::checkAndRead(BNO_Packet& packet, uint8_t phase, uint64_t timestamp, uint32_t dt) {
  if (!_initialized) return false;

  // HARDWARE GATING: BNO08x asserts the INT pin LOW when it has a report ready
  if (digitalRead(_intPin) == HIGH) {
    return false; // No data pending, avoid SPI bus overhead
  }

  // Ensure SPI bus is free before executing SHTP transaction
  if (SpiBus::isDmaBusy()) return false;

  sh2_SensorValue_t sensorValue;
  if (_bno.getSensorEvent(&sensorValue)) {
    packet.phase = phase;
    packet.timestamp = timestamp;
    packet.delta_t = dt;
    packet.sensorId = sensorValue.sensorId;

    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      packet.x = sensorValue.un.accelerometer.x;
      packet.y = sensorValue.un.accelerometer.y;
      packet.z = sensorValue.un.accelerometer.z;
      return true;
    } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      packet.x = sensorValue.un.gyroscope.x;
      packet.y = sensorValue.un.gyroscope.y;
      packet.z = sensorValue.un.gyroscope.z;
      return true;
    } else if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
      packet.x = sensorValue.un.magneticField.x;
      packet.y = sensorValue.un.magneticField.y;
      packet.z = sensorValue.un.magneticField.z;
      return true;
    }
  }

  return false;
}
