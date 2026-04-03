// MLX90642.cpp, for MLX90642 Library v1.0.3
// Author: D. Dubins
// Date: 25-Feb-26
// Last Updated: 18-Mar-26
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
#include "MLX90642.h"

MLX90642::MLX90642()
{
}  

// Check if new data is available
bool MLX90642::isNewDataAvailable() {
  static uint16_t lastProgress = 0;
  uint16_t progress = readAddr_unsigned(0x3C10);
  bool newFrame = (progress < lastProgress);  // wrapped around
  lastProgress = progress;
  return newFrame;
}

// Read a 16-bit unsigned integer from RAM or EEPROM at the address readByte:
uint16_t MLX90642::readAddr_unsigned(const uint16_t readByte) {
  Wire.beginTransmission(MLX90642_ADDR);
  Wire.write(readByte >> 8);    // MSB of VDD_ADDR
  Wire.write(readByte & 0xFF);  // LSB of VDD_ADDR
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom(MLX90642_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)((hi << 8)) | lo); // return combined unsigned 16 bit integer
}

// Read a 16-bit signed integer from RAM or EEPROM at the address readByte:
int16_t MLX90642::readAddr_signed(const uint16_t readByte) {
  Wire.beginTransmission(MLX90642_ADDR);
  Wire.write(readByte >> 8);    // MSB of VDD_ADDR
  Wire.write(readByte & 0xFF);  // LSB of VDD_ADDR
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom(MLX90642_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo); // return combined signed 16 bit integer
}

// Sensor temperature at address 0x3A2C (at normal operation in open air typically 8-10°C above 
// ambient temperature) not to be confused with environment temperature (Datasheet 3.1.4.1.4)
// This is "Tsensor" in the datasheet.
float MLX90642::readTa() {  // Read sensor temperature, 3.1.5.2
  float Ta_calc = readAddr_signed(0x3A2C) / 100.0f;
#ifdef DEBUG
  Serial.print("readTa() Ta: ");
  Serial.print(Ta_calc, 2);
  Serial.println(", example value: 33.57");  // second example in 3.1.5.3
  Serial.println("Finished: read Ta, sensor temperature.");
#endif
  return Ta_calc;  // return calculated sensor temperature
}

// Read temperature data from RAM.
void MLX90642::readTempC(float *Tram){
  for (int i = 0; i < NUM_PIXELS; i++) {
    Tram[i] = readAddr_signed(pix_addr(i)) / 50.0f;
  }
}

// Writes a new value to a specific EEPROM address using the MLX90642 configuration command
// The MLX90642 expects:
// 1 byte: opcode 0x3A
// 1 byte: sub‑opcode 0x2E
// 2 bytes: EEPROM address (MSB, then LSB)
// 2 bytes: data (MSB, then LSB).
bool MLX90642::writeEEPROM(uint16_t eepromAddr, uint16_t newValue) {
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
bool MLX90642::setRefreshRate(uint8_t rate_hz) {
  // MLX90642 refresh rates (0x11F0 bits 0:2): 2:2Hz,3:4Hz,4:8Hz,5:16Hz, default: 8Hz
  if (rate_hz < 2 || rate_hz > 5) return false;       // an invalid option has been selected
  uint16_t ctrl = readAddr_unsigned(0x11F0);  // read current refresh rate control bit
  if (ctrl == 0xFFFF) return false;           // Read failed
  ctrl &= ~0b00000111;          // Clear bits 0:2
  ctrl |= ((uint16_t)rate_hz);  // Set new value
  if (!MLX90642::writeEEPROM(0x11F0, ctrl)) return false;
  delay(20); // small delay for EEPROM to settle
  uint16_t ctrl2 = readAddr_unsigned(0x11F0);  // read current refresh rate control bit
  if (ctrl2 == 0xFFFF) return false;           // Read failed
  return ((ctrl2 & 0x07) == rate_hz);  // Helper for safe write
}

void MLX90642::printFullPixelMap() { // memory should run from 0x342C to 0x3A2A
  Serial.println("Full pixel address map:");
  for (int i = 0; i < NUM_PIXELS; i++) {
    Serial.print(i);
    Serial.print(", 0x");
    Serial.println(MLX90642::pix_addr(i), HEX);
  }
  // Check last pixel
  Serial.print("Last pixel address: 0x");
  Serial.println(MLX90642::pix_addr(NUM_PIXELS-1), HEX);
}

// Print pixels to serial monitor
void MLX90642::printFrame(float *Tdat) {
  int index = 0;
  for (int i = 0; i < NUM_PIXELS; i++) {
    // Write ",xx.x" into buffer
    int written = snprintf(&frameBuffer[index], FRAME_BUFFER_SIZE - index, ",%.1f", Tdat[i]);
    if (written <= 0) break;              // error
    index += written;
    if (index >= FRAME_BUFFER_SIZE - 10)  // safety margin
      break;
  }
  frameBuffer[index++] = '\n'; // add new line character
  frameBuffer[index] = '\0';   // add null pointer
  Serial.write((uint8_t*)frameBuffer, index);
}

// Lookup specific pixel address
uint16_t MLX90642::pix_addr(uint16_t pxl) { 
  if (pxl >= NUM_PIXELS) return 0;  // Error case
  return FRAME_ADDR + pxl*2;
}







