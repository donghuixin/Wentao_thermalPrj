// Last Update Time: 2026-04-03 01:21
#include <LSM6DS3.h>
#include <PDM.h>
#include <Wire.h>
#include <bluefruit.h>

#define DEVICE_NAME "wentaoCollector"

BLEUart bleuart;
LSM6DS3 myIMU(I2C_MODE, 0x6A);

bool imuOnline = false;
bool micOnline = false;
bool mlxOnline = false;

bool isRecording = false;
bool recMic = false;
bool recAcc = false;
bool recIr = false;

bool pendingStatusReport = false;

short sampleBuffer[256];
volatile int samplesRead;

unsigned long lastImuTime = 0;
unsigned long lastMlxRowTime = 0;
int currentMlxRow = 0;

#include <Adafruit_MLX90640.h>

Adafruit_MLX90640 mlx;
float mlxFrameBuffer[768]; // 24 * 32
bool frameAvailable = false;
unsigned long lastMlxReadTime = 0;

// MLX90640 helper to check status register without blocking
bool checkMLXDataReady() {
  Wire.beginTransmission(0x33); // 0x33 is default MLX90640 address, but we use 0x66 ??
  // Wait, the MLX90640 default I2C address is 0x33.
  // Wait, earlier we were using 0x66. The user's address is 0x66!
  Wire.beginTransmission(0x66);
  Wire.write(0x80); 
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  
  Wire.requestFrom((uint8_t)0x66, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t status = (Wire.read() << 8) | Wire.read();
    return (status & 0x0008) != 0;
  }
  return false;
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void prph_connect_callback(uint16_t conn_handle) { pendingStatusReport = true; }

void sendSystemStatus() {
  bleuart.println("\n=== System Status ===");
  bleuart.printf("IMU: %s | Addr: 0x6A | Acc: 2g | Gyro: 2000dps\n",
                 imuOnline ? "OK" : "ERR");
  bleuart.printf("MIC: %s | Internal | Rate: 16kHz\n",
                 micOnline ? "OK" : "ERR");
  bleuart.printf("MLX: %s | Adafruit Calibrated | 8Hz\n", mlxOnline ? "OK" : "ERR");
  bleuart.println("=====================");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("START_REC:")) {
    if (cmd.length() >= 15) {
      recMic = (cmd.charAt(10) == '1');
      recAcc = (cmd.charAt(12) == '1');
      recIr = (cmd.charAt(14) == '1');
      isRecording = true;
      bleuart.println("SYS: Recording STARTED.");
    }
  } else if (cmd == "STOP_REC" || cmd == "S" || cmd == "s") {
    isRecording = false;
    bleuart.println("SYS: Recording STOPPED.");
  } else if (cmd == "test" || cmd == "TEST") {
    sendSystemStatus();
  }
}

String bleBuffer = "";
void checkIncomingData() {
  while (bleuart.available()) {
    char c = (char)bleuart.read();
    if (c == '\n' || c == '\r') {
      if (bleBuffer.length() > 0) {
        processCommand(bleBuffer);
        bleBuffer = "";
      }
    } else {
      bleBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);

  bleuart.begin();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  Wire.begin();
  Wire.setClock(400000); // Adafruit Library works best at 400kHz Fast Mode for MLX90640 math
  
  if (mlx.begin(0x66, &Wire)) {
      mlxOnline = true;
      mlx.setMode(MLX90640_CHESS);
      mlx.setResolution(MLX90640_ADC_18BIT);
      mlx.setRefreshRate(MLX90640_8_HZ);
  } else if (mlx.begin(0x33, &Wire)) {
      mlxOnline = true; // Fallback to standard 0x33
      mlx.setMode(MLX90640_CHESS);
      mlx.setResolution(MLX90640_ADC_18BIT);
      mlx.setRefreshRate(MLX90640_8_HZ);
  }

  myIMU.settings.gyroRange = 2000;
  myIMU.settings.accelRange = 2;
  if (myIMU.begin() == 0)
    imuOnline = true;

  PDM.onReceive(onPDMdata);
  if (PDM.begin(1, 16000))
    micOnline = true;

  Serial.println("System Ready.");
}

void loop() {
  if (pendingStatusReport && bleuart.notifyEnabled()) {
    sendSystemStatus();
    pendingStatusReport = false;
  }

  checkIncomingData();

  if (isRecording) {
    unsigned long now = millis();

    // 1. Time Division Multiplexing - MLX Data 
    if (recIr && mlxOnline) {
      if (!frameAvailable && checkMLXDataReady()) {
          // If a frame is ready, pull the whole frame (Takes ~30ms)
          if (mlx.getFrame(mlxFrameBuffer) == 0) {
              frameAvailable = true;
              currentMlxRow = 0;
          }
      }

      // If we have a buffered frame, transmit it row by row!
      // This spreads the 24 rows across 24 loop increments, preserving 5ms gaps.
      if (frameAvailable && (now - lastMlxRowTime >= 5)) {
        lastMlxRowTime = now;

        // Compress the float data exactly matching the old Web/Python parser logic: temp * 50
        char txBuf[140];
        sprintf(txBuf, "T%02X:", currentMlxRow);
        int idx = 4;
        
        for (int c = 0; c < 32; c++) {
          float temp = mlxFrameBuffer[currentMlxRow * 32 + c];
          
          int16_t mappedRaw = (int16_t)(temp * 50.0f);
          
          // Re-encode into the legacy big-endian 4-char hex
          // So no HTML or python code changes at all!
          sprintf(txBuf + idx, "%04X", (uint16_t)mappedRaw);
          idx += 4;
        }
        txBuf[idx++] = '\n';
        txBuf[idx] = '\0';

        bleuart.print(txBuf);

        currentMlxRow++;
        if (currentMlxRow >= 24) {
          frameAvailable = false; // Done transmitting 1 full frame
        }
      }
    }

    // 2. Time Division Multiplexing - IMU Data (1 sample per 10ms -> 100Hz)
    if (recAcc && imuOnline && (now - lastImuTime >= 10)) {
      lastImuTime = now;
      float ax = myIMU.readFloatAccelX();
      float ay = myIMU.readFloatAccelY();
      float az = myIMU.readFloatAccelZ();
      float gx = myIMU.readFloatGyroX();
      float gy = myIMU.readFloatGyroY();
      float gz = myIMU.readFloatGyroZ();

      bleuart.printf("I:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", ax, ay, az, gx, gy,
                     gz);
    }

    // 3. MIC Data
    if (recMic && micOnline && samplesRead > 0) {
      bleuart.printf("MIC:%d,%d\n", sampleBuffer[0], sampleBuffer[1]);
      samplesRead = 0;
    }
  }
}