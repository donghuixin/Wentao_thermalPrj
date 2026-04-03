// MLX90642_standalone.ino
// Author: D. Dubins
// Date: 27-Feb-26
// Last updated: 28-Feb-26
// Notes: the MLX90642 operating voltage is 3-3.6V (typical: 3.3V).
// Use a logic shifter, or connect to an MCU that operates at 3.3V (e.g. NodeMCU).
// After the device powers up and sends data, a thermal stabilization time is required
// before the device can reach the specified accuracy (up to 3 min) - 3.2.3
// Wiring: ("/\" is notch in device case, pins facing you)
//
//       _____/\______
//     /              \
//    /  4:SCL  1:SDA  \
//   |                  |
//   |                  |
//    \  3:GND  2:3.3V /
//     \______________/
//
// ESP32 - MLX90642:
// --------------------------------------
// SDA - D21 (GPIO21) - SDA
// SCL - D22 (GPIO22) - SCL
// GND -  GND
// 3.3V - VDD
//
// Note: All memories are organized in words and can only be addressed on even addresses.

#include <Wire.h>

// MLX90642 Settings:
#define MLX90642_ADDR 0x66  // I2C bit address of the MLX90642
#define NUM_PIXELS 768      // number of pixels
#define FRAME_ADDR 0x342C             // Starting address for pixel data in RAM
#define FRAME_BUFFER_SIZE 6000        // for reading the temperatures and a faster serial print
#define REFRESH_RATE 4      // Refresh rates (0x11F0 bits 0..2): 2:2Hz, 3:4Hz, 4:8Hz, 5:16Hz
#define POR_DELAY 5000      // 5 second warmup

//MCU Settings:
// According to Datasheet 4.4: I2C speed of 100 KHz: up to 4Hz refresh rate, 400KHz: up to 16Hz
#define I2C_SPEED 400000  // 400 kHz I2C bus ok up to 16Hz
#define SDA 21 // GPIO21 is SDA for ESP32
#define SCL 22 // GPIO22 is SCL for ESP32

float T_o[NUM_PIXELS];  // to hold pixel temperature information
char frameBuffer[FRAME_BUFFER_SIZE];  // frame buffer

void setup() {
  Serial.begin(921600);                // Start the Serial Monitor at 921600 bps
  Wire.begin(SDA, SCL);                // SDA, SCL for the ESP32
  Wire.setClock(I2C_SPEED);            // set I2C clock speed (slower=more stable)
  delay(POR_DELAY);                    // Power on reset delay (POR)
  if (setRefreshRate(REFRESH_RATE)) {  // set the page refresh rate (sampling frequency)
    Serial.println("Refresh rate adjusted.");
  } else {
    Serial.println("Error on adjusting refresh rate.");
  }
  float Ta = readTa();                 // Read sensor temperature
  Serial.print("Sensor temperature on start: ");
  Serial.println(Ta, 1);               // Read the sensor temperature (note: this can be 8-10°C above ambient temperature)
  //To print the full pixel map:
  //printFullPixelMap();
}

void loop() {
  while (!isNewDataAvailable()) {      // Wait for new data
    delay(1);                          // Small yield to avoid watchdog reset
  }

  float Ta = readTa();  // Read the sensor temperature (note: this can be 8-10°C above ambient temperature)
  Serial.print(Ta, 1);
  readTempC(T_o);       // Read one frame of the temperature
  printFrame(T_o);      // Print out temperature frame to Serial Monitor
}

void readTempC(float *Tram) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    Tram[i] = readAddr_signed(pix_addr(i)) / 50.0f;
  }
}

uint16_t pix_addr(uint16_t pxl) {
  if (pxl >= NUM_PIXELS) return 0;  // Error case
  return FRAME_ADDR + pxl * 2;
}

// Check if new data is available
bool isNewDataAvailable() {
  static uint16_t lastProgress = 0;
  uint16_t progress = readAddr_unsigned(0x3C10);
  bool newFrame = (progress < lastProgress);  // wrapped around
  lastProgress = progress;
  return newFrame;
}

// Read a 16-bit unsigned integer from RAM or EEPROM at the address readByte:
uint16_t readAddr_unsigned(const uint16_t readByte) {
  Wire.beginTransmission(MLX90642_ADDR);
  Wire.write(readByte >> 8);    // MSB of VDD_ADDR
  Wire.write(readByte & 0xFF);  // LSB of VDD_ADDR
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom(MLX90642_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)((hi << 8)) | lo);  // return combined unsigned 16 bit integer
}

// Read a 16-bit signed integer from RAM or EEPROM at the address readByte:
int16_t readAddr_signed(const uint16_t readByte) {
  Wire.beginTransmission(MLX90642_ADDR);
  Wire.write(readByte >> 8);    // MSB of VDD_ADDR
  Wire.write(readByte & 0xFF);  // LSB of VDD_ADDR
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom(MLX90642_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);  // return combined signed 16 bit integer
}

// Sensor temperature at address 0x3A2C (at normal operation in open air typically 8-10°C above 
// ambient temperature) not to be confused with environment temperature (Datasheet 3.1.4.1.4)
// This is "Tsensor" in the datasheet.
float readTa() {  // Read sensor temperature, 3.1.5.2
  float Ta_calc = readAddr_signed(0x3A2C) / 100.0f;
  return Ta_calc;  // return calculated ambient temperature
}

// Writes a new value to a specific EEPROM address using the MLX90642 configuration command
// The MLX90642 expects:
// 1 byte: opcode 0x3A
// 1 byte: sub‑opcode 0x2E
// 2 bytes: EEPROM address (MSB, then LSB)
// 2 bytes: data (MSB, then LSB).
bool writeEEPROMParameter(uint16_t eepromAddr, uint16_t newValue) {
  Wire.beginTransmission(MLX90642_ADDR);
  Wire.write(0x3A);               // Opcode
  Wire.write(0x2E);               // Sub‑opcode
  Wire.write(eepromAddr >> 8);    // Address MSB
  Wire.write(eepromAddr & 0xFF);  // Address LSB
  Wire.write(newValue >> 8);      // Data MSB
  Wire.write(newValue & 0xFF);    // Data LSB
  if (Wire.endTransmission() != 0) return false;
  delay(10);  // wait for internal EEPROM write
  return true;
}

// To set the refresh rate
bool setRefreshRate(uint8_t rate_hz) {
  // MLX90642 refresh rates (0x11F0 bits 0:2): 2:2Hz, 3:4Hz, 4:8Hz, 5:16Hz, default: 8Hz
  if (rate_hz < 2 || rate_hz > 5) return false;       // an invalid option has been selected
  uint16_t ctrl = readAddr_unsigned(0x11F0);  // read current refresh rate control bit
  if (ctrl == 0xFFFF) return false;           // Read failed
  Serial.print("Current EEPROM word: 0b");
  Serial.println(ctrl, BIN);
  ctrl &= ~0b00000111;          // Clear bits 0:2
  ctrl |= ((uint16_t)rate_hz);  // Set new value
  if (!writeEEPROMParameter(0x11F0, ctrl)) return false;
  delay(20); // small delay for EEPROM to settle
  uint16_t ctrl2 = readAddr_unsigned(0x11F0);  // read current refresh rate control bit
  if (ctrl2 == 0xFFFF) return false;           // Read failed
  //For debugging:
  //Serial.print("New EEPROM word: 0x");
  //Serial.println(ctrl2, HEX);
  //Serial.print("New EEPROM word: 0b");
  //Serial.println(ctrl2, BIN);
  return ((ctrl2 & 0x07) == rate_hz);  // Helper for safe write
}

void printFullPixelMap() {  // For debugging. Memory should run from 0x342C to 0x3A2A
  Serial.println("Full pixel address map:");
  for (int i = 0; i < NUM_PIXELS; i++) {
    Serial.print(i);
    Serial.print(", 0x");
    Serial.println(pix_addr(i), HEX);
  }

  // Check last pixel
  Serial.print("Last pixel address: 0x");
  Serial.println(pix_addr(NUM_PIXELS - 1), HEX);
}

// Function to print to serial monitor faster.
void printFrame(float *T_o) {
  int index = 0;
  for (int i = 0; i < NUM_PIXELS; i++) {
    // Write ",xx.x" into buffer
    int written = snprintf(&frameBuffer[index], FRAME_BUFFER_SIZE - index, ",%.1f", T_o[i]);
    if (written <= 0) break;  // error
    index += written;
    if (index >= FRAME_BUFFER_SIZE - 10)  // safety margin
      break;
  }
  frameBuffer[index++] = '\n';  // add new line character
  frameBuffer[index] = '\0';    // add null pointer
  Serial.write((uint8_t *)frameBuffer, index);
}
