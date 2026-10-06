#include "PhaseManager.h"

PhaseManager::PhaseManager()
  : _currentPhase(PHASE_STANDBY), _previousPhase(PHASE_STANDBY),
    _onPhaseChange(nullptr), _lastBlinkTime(0), _ledOn(false) {}

void PhaseManager::begin(PhaseChangeCallback cb) {
  _onPhaseChange = cb;

  FastLED.addLeds<WS2812B, PIN_LED, GRB>(_leds, NUM_LEDS);
  FastLED.setBrightness(5);
  _leds[0] = CRGB::Cyan;
  FastLED.show();

  Serial5.begin(9600);
}

void PhaseManager::update() {
  // 1. Process incoming UART commands on Serial5
  if (Serial5.available() > 0) {
    uint8_t byteIn = Serial5.read();
    uint8_t targetPhase = _currentPhase;

    if (byteIn <= 4) {
      targetPhase = byteIn;
    } else if (byteIn >= '0' && byteIn <= '4') {
      targetPhase = byteIn - '0';
    }

    if (targetPhase != _currentPhase) {
      setPhase((FlightPhase)targetPhase);
    }
  }

  // 2. Update visual LED heartbeat
  updateLedHeartbeat();
}

void PhaseManager::setPhase(FlightPhase phase) {
  if (phase == _currentPhase) return;

  _previousPhase = _currentPhase;
  _currentPhase = phase;

  if (_onPhaseChange) {
    _onPhaseChange(_previousPhase, _currentPhase);
  }
}

void PhaseManager::updateLedHeartbeat() {
  unsigned long now = millis();
  unsigned long interval = isLowPower() ? STANDBY_BLINK_MS : FLIGHT_BLINK_MS;

  if (!_ledOn && (now - _lastBlinkTime >= interval)) {
    switch (_currentPhase) {
      case PHASE_STANDBY: _leds[0] = CRGB::Cyan;       break;
      case PHASE_ARMED:   _leds[0] = CRGB::Yellow;     break;
      case PHASE_TERM:    _leds[0] = CRGB::DarkOrange; break;
      case PHASE_FLIGHT:  _leds[0] = CRGB::Red;        break;
      case PHASE_LANDED:  _leds[0] = CRGB::Purple;     break;
      default:            _leds[0] = CRGB::White;      break;
    }
    FastLED.show();
    _lastBlinkTime = now;
    _ledOn = true;
  } else if (_ledOn && (now - _lastBlinkTime >= BLINK_DURATION_MS)) {
    _leds[0] = CRGB::Black;
    FastLED.show();
    _ledOn = false;
  }
}
