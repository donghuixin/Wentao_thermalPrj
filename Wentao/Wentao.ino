// Last Update Time: 2026-04-03 16:45
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

// ===================== Ring Buffer Template =====================
template <typename T, uint16_t N>
class RingBuf {
public:
  T buf[N];
  volatile uint16_t head = 0;
  volatile uint16_t tail = 0;

  bool push(T val) {
    uint16_t next = (head + 1) % N;
    if (next == tail) return false;
    buf[head] = val;
    head = next;
    return true;
  }
  
  bool pop(T &val) {
    if (head == tail) return false;
    val = buf[tail];
    tail = (tail + 1) % N;
    return true;
  }
  
  uint16_t available() {
    uint16_t h = head;
    uint16_t t = tail;
    if (h >= t) return h - t;
    return N - t + h;
  }
};

RingBuf<uint8_t, 4096> micBuf;
RingBuf<uint8_t, 2048> thermalBuf;
RingBuf<uint8_t, 256> imuBuf;

short sampleBuffer[512]; // Buffer for PDM library

unsigned long lastPackTime = 0;
unsigned long lastMlxReadTime = 0;
unsigned long lastImuTime = 0;
int currentMlxRow = 0;
uint8_t packetSeq = 0;

// ===================== MLX90642 =====================
#define MLX90642_I2C_ADDR 0x66
#define MLX90642_STATUS_REG 0x3C14
#define MLX90642_CONFIG_ADDR 0x11F0
#define MLX90642_RAM_START 0x342C
#define MLX90642_OP_CONFIG 0x3A2E

class ThermalDataCollector {
public:
  bool begin() {
    Wire.begin();
    Wire.setClock(100000);
    uint16_t config = readRegister(MLX90642_CONFIG_ADDR);
    if (config == 0 || config == 0xFFFF) return false;
    config &= ~0x0007;
    config |= 0x0004;
    writeRegister(MLX90642_CONFIG_ADDR, config);
    return true;
  }

  bool readRowToBuf(int r) {
    for (int c = 0; c < 32; c++) {
      uint16_t pxl_idx = r * 32 + c;
      uint16_t startAddress = MLX90642_RAM_START + pxl_idx * 2; 

      Wire.beginTransmission(MLX90642_I2C_ADDR);
      Wire.write(startAddress >> 8);
      Wire.write(startAddress & 0xFF);
      
      if (Wire.endTransmission(false) != 0) return false;

      int req = Wire.requestFrom((uint8_t)MLX90642_I2C_ADDR, (uint8_t)2);
      if (req != 2 || Wire.available() < 2) return false;

      // 推入环形缓冲区 (MSB先推, LSB后推)
      thermalBuf.push(Wire.read());
      thermalBuf.push(Wire.read());
    }
    return true;
  }

private:
  uint16_t readRegister(uint16_t regAddress) {
    Wire.beginTransmission(MLX90642_I2C_ADDR);
    Wire.write(regAddress >> 8);
    Wire.write(regAddress & 0xFF);
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((uint8_t)MLX90642_I2C_ADDR, (uint8_t)2);
    if (Wire.available() == 2) {
      uint16_t data = Wire.read() << 8;
      data |= Wire.read();
      return data;
    }
    return 0;
  }

  void writeRegister(uint16_t regAddress, uint16_t data) {
    Wire.beginTransmission(MLX90642_I2C_ADDR);
    Wire.write(regAddress >> 8);
    Wire.write(regAddress & 0xFF);
    Wire.write(data >> 8);
    Wire.write(data & 0xFF);
    Wire.endTransmission();
  }
};
ThermalDataCollector thermalCollector;

// ===================== Interrupts & Callbacks =====================
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  if (!isRecording || !recMic) return;
  // 直接以 Byte 的形式喂进 micBuf
  int samples = bytesAvailable / 2;
  for (int i = 0; i < samples; i++) {
    int16_t s = sampleBuffer[i];
    // 转为小端序存放，前端读取 Int16Array 方便
    micBuf.push((uint8_t)(s & 0xFF));
    micBuf.push((uint8_t)(s >> 8));
  }
}

void prph_connect_callback(uint16_t conn_handle) { 
  pendingStatusReport = true; 
  // 协商全速 2M PHY 和 247 MTU
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  if (conn) {
    conn->requestPHY();
    conn->requestDataLengthUpdate();
    conn->requestMtuExchange(247);
  }
}

void sendSystemStatus() {
  // 我们保留 ASCII 发送给系统状态
  bleuart.println("\n=== System Status ===");
  bleuart.printf("IMU: %s | Addr: 0x6A | Acc: 2g | Gyro: 2000dps\n", imuOnline ? "OK" : "ERR");
  bleuart.printf("MIC: %s | Internal | Rate: 16kHz\n", micOnline ? "OK" : "ERR");
  bleuart.printf("MLX: %s | Addr: 0x66 | 8Hz\n", mlxOnline ? "OK" : "ERR");
  bleuart.println("=====================");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("START_REC:")) {
    if (cmd.length() >= 15) {
      recMic = (cmd.charAt(10) == '1');
      recAcc = (cmd.charAt(12) == '1');
      recIr  = (cmd.charAt(14) == '1');
      isRecording = true;
      bleuart.println("SYS: Recording STARTED (TLV).");
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

  mlxOnline = thermalCollector.begin();

  myIMU.settings.gyroRange = 2000;
  myIMU.settings.accelRange = 2;
  if (myIMU.begin() == 0) imuOnline = true;

  PDM.onReceive(onPDMdata);
  if (PDM.begin(1, 16000)) micOnline = true;

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

    // =============== 生产端 (Producers) ===============
    // 1. 每 5ms 抓一行 MLX 并入队 (耗时 ~2ms)
    if (recIr && mlxOnline && (now - lastMlxReadTime >= 5)) {
      lastMlxReadTime = now;
      thermalCollector.readRowToBuf(currentMlxRow);
      currentMlxRow++;
      if (currentMlxRow >= 24) currentMlxRow = 0;
    }

    // 2. 每 10ms 抓一帧 IMU 并入队 (耗时极低)
    if (recAcc && imuOnline && (now - lastImuTime >= 10)) {
      lastImuTime = now;
      int16_t ax = myIMU.readRawAccelX();
      int16_t ay = myIMU.readRawAccelY();
      int16_t az = myIMU.readRawAccelZ();
      int16_t gx = myIMU.readRawGyroX();
      int16_t gy = myIMU.readRawGyroY();
      int16_t gz = myIMU.readRawGyroZ();
      
      int16_t imuData[6] = {ax, ay, az, gx, gy, gz};
      uint8_t* bytePtr = (uint8_t*)imuData;
      for(int i=0; i<12; i++) imuBuf.push(bytePtr[i]);
    }

    // Mic 数据由底层 DMA 硬件在 onPDMdata 内自动生产...

    // =============== 消费端 (Consumer) ===============
    // 3. 严格 5ms 打包发送 TLV
    if (now - lastPackTime >= 5) {
      // 用 244 byte 做 payload buffer
      uint8_t txBuf[247]; 
      int len = 0;
      bool isFullSec = (now % 1000 < 5) && (now - lastPackTime > 500); 
      lastPackTime = now;

      txBuf[len++] = packetSeq++; // Seq byte
      txBuf[len++] = isFullSec ? 0x01 : 0x00; // Time Flag
      if (isFullSec) {
        txBuf[len++] = (now >> 24) & 0xFF;
        txBuf[len++] = (now >> 16) & 0xFF;
        txBuf[len++] = (now >> 8) & 0xFF;
        txBuf[len++] = now & 0xFF;
      }

      // 提取 M: Mic 数据 (最高抽 160 Bytes)
      uint16_t mAvail = micBuf.available();
      if (mAvail > 160) mAvail = 160;
      // 保证只能抽取出偶数个字节(完整int16)
      mAvail = mAvail & ~1;
      if (mAvail > 0) {
        txBuf[len++] = 'M';
        txBuf[len++] = (uint8_t)mAvail;
        for(uint16_t i=0; i<mAvail; i++) {
          micBuf.pop(txBuf[len++]);
        }
      }

      // 提取 F: 热成像数据 (最高抽 64 Bytes, 即1行)
      uint16_t fAvail = thermalBuf.available();
      if (fAvail > 64) fAvail = 64;
      // 保证整组双字节出入
      fAvail = fAvail & ~1;
      if (fAvail > 0) {
        txBuf[len++] = 'F';
        txBuf[len++] = (uint8_t)fAvail;
        for(uint16_t i=0; i<fAvail; i++) {
          thermalBuf.pop(txBuf[len++]);
        }
      }

      // 提取 I: IMU 数据 (最高抽 12 Bytes)
      uint16_t iAvail = imuBuf.available();
      if (iAvail >= 12) iAvail = 12; // 强行对齐12
      else iAvail = 0;
      if (iAvail > 0) {
        txBuf[len++] = 'I';
        txBuf[len++] = (uint8_t)iAvail;
        for(uint16_t i=0; i<iAvail; i++) {
          imuBuf.pop(txBuf[len++]);
        }
      }

      // 只在有数据时发送
      if (len > 2) {
        bleuart.write(txBuf, len);
      }
    }
  }
}