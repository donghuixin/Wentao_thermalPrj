<h1>Readme file for MLX90642 32x24 IR Array</h1>

MLX90642 is a library that can control the MLX90642 32x24 IR Array using an ESP32 MCU.<p>

There is much better code here: https://github.com/melexis/mlx90642-library

* The example sketch "MLX90642_basicRead.ino" illustrates a simple reading with basic temperature output to the Serial Monitor.
* The example sketch "MLX90642_processing.ino" formats the output for a processing sketch, to draw a heat map.
* The example sketch "extras/MLX90642_Heatmap.pde" is a [Processing](https://processing.org/) sketch to make a colour heat map, with a simple control panel.
* The file "extras/FAB-MLX90641-001-A.1.zip" is a Gerber and Drill file if you would like to print a PCB for the sensor, which will fit the MLX90642. The same footprint and layout for the MLX90641 works for the MLX90642.
  
**Datasheet:** Melexis. "MLX90642 32x24 InfraRed (IR) Array Datasheet", Doc server Rev 001, Datasheet Rev 003 - 19-Mar-25; DOC#3901090642  

* For those of you who don't like libraries, I included a library-free version in a self-contained sketch, "MLX90642_standalone.ino".

The functions available in the library include:
```
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
```
To use the library, copy the download to the Library directory.<p>

Technical notes:
- This MLX90642.h library was designed only for the ESP32. Feel free to adapt it to other MCUs.

Acknowledgements: 
- Thank you to Karel Vanroye at Melexis for introducting me to this sensor.
- A big thank-you to Howard Qiu for getting me interested in IR sensor arrays!





