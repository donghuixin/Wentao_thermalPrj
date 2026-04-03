// MLX90642.h, for MLX90642 Library v1.0.3
// Author: D. Dubins
// Date: 25-Feb-26
// Last Updated: 18-Mar-26

#ifndef MLX90642_h
#define MLX90642_h

#include <Arduino.h>

// USER CONFIGURATION - Override these in your .ino AFTER #include "MLX90642.h"
#define MLX90642_ADDR 0x66  // I2C bit address of the MLX90642
#define NUM_PIXELS 768      // number of pixels
#define FRAME_ADDR 0x342C   // Starting address for pixel data in RAM
#define REFRESH_RATE 4      // Refresh rates (0x11F0 bits 0:2): 2:2Hz,3:4Hz,4:8Hz,5:16Hz, default: 8Hz
#define POR_DELAY 5000      // 5 second warmup
#define FRAME_BUFFER_SIZE 6000   // for reading the temperatures and a faster serial print

// MCU Settings
#define SDA 21 // SDA is GPIO21 for ESP32
#define SCL 22 // SCL is GPIO22 for ESP32
#define I2C_SPEED 400000    // fast speed is 400 kHz

class MLX90642 {
  public:
    MLX90642();
    // Non-Static Variables: (not common across all sensors)
    // (none declared here)
    // Functions:
    bool isNewDataAvailable(); // Check if new data is available
    uint16_t readAddr_unsigned(const uint16_t readByte); // Read a 16-bit unsigned integer from RAM or EEPROM at the address readByte	
    int16_t readAddr_signed(const uint16_t readByte);	// Read a 16-bit signed integer from RAM or EEPROM at the address readByte:
    float readTa();  // Read sensor temperature, 3.1.5.2 (note: this can be 8-10°C above ambient temperature)
    void readTempC(float *Tram); // Read temperature data from RAM
    bool writeEEPROM(uint16_t eepromAddr, uint16_t newValue); // Writes new value to a EEPROM address using the MLX90642 configuration command
    bool setRefreshRate(uint8_t rate_hz); // To set the refresh rate
    void printFullPixelMap(); // For debugging. Memory should run from 0x342C to 0x3A2A
    void printFrame(float *Tdat); // Print pixels to serial monitor	
    uint16_t pix_addr(uint16_t pxl);  // Lookup specific pixel address
  
  private:
    char frameBuffer[FRAME_BUFFER_SIZE];  // member variable
};


#endif





