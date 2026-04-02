#include <FastLED.h>

#define NUM_LEDS 1
#define DATA_PIN 23

CRGB leds[NUM_LEDS];

void setup() {
  // Using WS2812B on Teensy 4 usually defaults to a high-performance 
  // hardware-assisted driver if available.
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(10); 
}

void loop() {
  // Cycle Red, Green, Blue
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(500);

  leds[0] = CRGB::Green;
  FastLED.show();
  delay(500);

  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(500);
}