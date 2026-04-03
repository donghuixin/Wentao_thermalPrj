// MLX90642_basicRead.ino file for the MLX90642.h library, v1.0.3
// Author: D. Dubins
// Date: 27-Feb-26
// Last updated: 18-Mar-26
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

//MLX90642 Settings:
#define REFRESH_RATE 4    // Refresh rates (0x11F0 bits 0..2): 2:2Hz, 3:4Hz, 4:8Hz, 5:16Hz
#define POR_DELAY 5000    // 5 second warmup

//MCU Settings:
#define SDA 21 // SDA is GPIO21 for ESP32
#define SCL 22 // SCL is GPIO22 for ESP32
#define I2C_SPEED 400000  // Set I2C clock speed (safe speed is 100 kHz, up to 400 kHz possible)

float T_o[NUM_PIXELS];    // To hold pixel temperature data

MLX90642 myIRcam;         // declare an instance of class MLX90642

void setup() {
  Serial.begin(921600);                // Start the Serial Monitor at 921600 bps
  Wire.begin(SDA, SCL);                // SDA, SCL for the ESP32 (SDA: GPIO 21, SDL: GPIO22). Change these to your I2C pins if using a different bus.
  Wire.setClock(I2C_SPEED);            // Set I2C clock speed (slower=more stable)
  delay(POR_DELAY);                    // Power on reset delay (POR)
  if (myIRcam.setRefreshRate(REFRESH_RATE)) {  // set the page refresh rate (sampling frequency)
    Serial.println("Refresh rate adjusted.");
  } else {
    Serial.println("Error on adjusting refresh rate.");
  }
  float Ta = myIRcam.readTa();  // Read the sensor temperature (note: this can be 8-10°C above ambient temperature)
  Serial.print("Sensor temperature on start: ");
  Serial.println(Ta, 1);        // Print sensor temeperature
}

void loop() {
  while (!myIRcam.isNewDataAvailable()) {  // wait for new data
    delay(1);                   // Small yield to avoid watchdog reset
  }
  float Ta = myIRcam.readTa();  // Read the sensor temperature
  Serial.print(Ta, 1);          // Print the sensor temperature
  myIRcam.readTempC(T_o);       // Read one frame of the temperature
  myIRcam.printFrame(T_o);      // Print out temperature frame to Serial Monitor
}
