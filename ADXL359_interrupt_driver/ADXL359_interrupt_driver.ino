#include <SPI.h>

// Pins
const int CS_PIN = 10;
const int DRDY_PIN = 9;

// Registers
const byte REG_DATA_START = 0x08; 
const byte REG_POWER_CTL = 0x2D;

// Interrupt logic
volatile bool dataReady = false;
volatile unsigned long lastInterruptMicros = 0;
volatile unsigned long currentInterruptMicros = 0;
float actualHz = 0.0;

// ISR: Short and fast
void adxl_ISR() {
  lastInterruptMicros = currentInterruptMicros;
  currentInterruptMicros = micros();
  dataReady = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(DRDY_PIN, INPUT); 

  SPI.begin();
  
  // Initialize ADXL359: Measurement mode
  writeRegister(REG_POWER_CTL, 0x00); 
  
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), adxl_ISR, RISING);
  
  Serial.println("ADXL359 Interrupt Test Starting...");
  Serial.println("X_Raw\tY_Raw\tZ_Raw\tRate(Hz)");
}

void loop() {
  if (dataReady) {
    // Calculate sample rate safely
    noInterrupts(); // Pause interrupts to read volatile timing vars
    unsigned long duration = currentInterruptMicros - lastInterruptMicros;
    dataReady = false;
    interrupts();

    if (duration > 0) {
      actualHz = 1000000.0 / duration; // Convert microseconds to Hz
    }

    readAndPrintData();
  }
}

void readAndPrintData() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer((REG_DATA_START << 1) | 0x01);
  
  long xRaw = read20BitValue();
  long yRaw = read20BitValue();
  long zRaw = read20BitValue();
  
  digitalWrite(CS_PIN, HIGH);

  // Tab-separated output for easy copy-pasting to Excel
  Serial.print(xRaw); Serial.print("\t");
  Serial.print(yRaw); Serial.print("\t");
  Serial.print(zRaw); Serial.print("\t");
  Serial.println(actualHz, 2); 
}

long read20BitValue() {
  byte b1 = SPI.transfer(0x00); // MSB
  byte b2 = SPI.transfer(0x00);
  byte b3 = SPI.transfer(0x00); // LSB
  
  // Shift into 20-bit format
  long val = ((long)b1 << 12) | ((long)b2 << 4) | (b3 >> 4);
  
  // Sign extension for 20-bit two's complement
  if (val & 0x80000) val |= 0xFFF00000; 
  return val;
}

void writeRegister(byte reg, byte val) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg << 1); 
  SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
}