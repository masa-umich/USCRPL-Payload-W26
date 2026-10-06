#pragma once
#include <Arduino.h>
#include <FastLED.h>
#include "Config.h"

enum FlightPhase : uint8_t {
  PHASE_STANDBY = 0,
  PHASE_ARMED   = 1,
  PHASE_TERM    = 2,
  PHASE_FLIGHT  = 3,
  PHASE_LANDED  = 4
};

typedef void (*PhaseChangeCallback)(FlightPhase oldPhase, FlightPhase newPhase);

class PhaseManager {
public:
  PhaseManager();

  void begin(PhaseChangeCallback cb = nullptr);
  void update(); // Check Serial5 UART and update LED heartbeats

  FlightPhase getPhase() const { return _currentPhase; }
  bool isLowPower() const { return _currentPhase == PHASE_STANDBY || _currentPhase == PHASE_LANDED; }

  void setPhase(FlightPhase phase);

private:
  FlightPhase _currentPhase;
  FlightPhase _previousPhase;
  PhaseChangeCallback _onPhaseChange;
  CRGB _leds[NUM_LEDS];

  unsigned long _lastBlinkTime;
  bool _ledOn;

  void updateLedHeartbeat();
};
