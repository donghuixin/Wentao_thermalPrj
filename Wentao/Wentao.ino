// Last Update Time: 2026-04-03 16:50
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

// 缓冲池扩容与评估论证
RingBuf<uint8_t, 4096> micBuf; // 16kHz*2B=32KB/s。5ms需160B。4096B足以支撑连续 125ms 的蓝牙阻塞不丢数。
RingBuf<uint8_t, 8192> thermalBuf; // 提升到 8192B，可缓存至少 5 帧完整的红外数据图。
RingBuf<uint8_t, 256> imuBuf; // 每10ms生产12B(1.2KB/s)。256B可支撑 200ms 的阻塞积压。

short sampleBuffer[512]; // PDM library DMA Cache

unsigned long lastPackTime = 0;
unsigned long lastMlxReadTime = 0;
unsigned long lastImuTime = 0;
int currentMlxRow = 0;
int currentMlxRowOut = 0;
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
    Wire.setClock(1000000); // 提升到 1MHz Fast Mode Plus 降低阻塞
    uint16_t config = readRegister(MLX90642_CONFIG_ADDR);
    if (config == 0 || config == 0xFFFF) return false;
    config &= ~0x0007;
    config |= 0x0004;
    writeRegister(MLX90642_CONFIG_ADDR, config);
    return true;
  }

  bool readRowToBuf(int r) {
    uint16_t startPxlIdx = r * 32;
    // 將 32 像素 (64 bytes) 分為兩次 16 像素 (32 bytes) 的連續讀取，以適應 Arduino 一般 32 字節的 Wire 緩衝區限制
    for (int chunk = 0; chunk < 2; chunk++) {
      uint16_t startAddress = MLX90642_RAM_START + (startPxlIdx + chunk * 16) * 2;
      
      Wire.beginTransmission(MLX90642_I2C_ADDR);
      Wire.write(startAddress >> 8);
      Wire.write(startAddress & 0xFF);
      if (Wire.endTransmission(false) != 0) return false;

      int req = Wire.requestFrom((uint8_t)MLX90642_I2C_ADDR, (uint8_t)32);
      if (req != 32 || Wire.available() < 32) return false;

      // 連續推入環形緩衝區
      for (int i = 0; i < 16; i++) {
        thermalBuf.push(Wire.read());
        thermalBuf.push(Wire.read());
      }
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
  int samples = bytesAvailable / 2;
  for (int i = 0; i < samples; i++) {
    int16_t s = sampleBuffer[i];
    micBuf.push((uint8_t)(s & 0xFF));
    micBuf.push((uint8_t)(s >> 8));
  }
}

void prph_connect_callback(uint16_t conn_handle) { 
  pendingStatusReport = true; 
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  if (conn) {
    conn->requestPHY();
    conn->requestDataLengthUpdate();
    conn->requestMtuExchange(247);
    // Request an aggressive connection interval (7.5ms ~ 15ms)
    // Helps Android / Windows / iOS drain the Nordic TX FIFO faster
    conn->requestConnectionParameter(6); // interval 6*1.25=7.5ms, defaults used for latency/timeout
  }
}

void sendSystemStatus() {
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
      bleuart.println("SYS: Recording STARTED (Fixed Struct TLV).");
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

  // 確保 1MHz 設定不被 myIMU 等其他庫覆寫
  Wire.setClock(1000000);

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

    // 只要距离上次读取大于 2ms 且缓冲池有一行 (64B) 以上空余量，就提早并发读取下一行
    // 避免 5ms 的死板卡点导致来不急填满缓冲区
    if (recIr && mlxOnline && (now - lastMlxReadTime >= 2)) {
      if (thermalBuf.available() < (8192 - 64)) {
        lastMlxReadTime = now;
        thermalCollector.readRowToBuf(currentMlxRow);
        currentMlxRow++;
        if (currentMlxRow >= 24) currentMlxRow = 0;
      }
    }

    if (recAcc && imuOnline && (now - lastImuTime >= 10)) {
      lastImuTime = now;
      // Burst read IMU Data (12 bytes starting from 0x22 OUTX_L_G)
      Wire.beginTransmission(0x6A);
      Wire.write(0x22);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x6A, (uint8_t)12) == 12) {
        int16_t gx = Wire.read() | (Wire.read() << 8);
        int16_t gy = Wire.read() | (Wire.read() << 8);
        int16_t gz = Wire.read() | (Wire.read() << 8);
        int16_t ax = Wire.read() | (Wire.read() << 8);
        int16_t ay = Wire.read() | (Wire.read() << 8);
        int16_t az = Wire.read() | (Wire.read() << 8);
        
        int16_t imuData[6] = {ax, ay, az, gx, gy, gz};
        uint8_t* bytePtr = (uint8_t*)imuData;
        for(int i=0; i<12; i++) imuBuf.push(bytePtr[i]);
      }
    }

    // 严苛定长定偏移打包 (Fixed Padding Struct)
    if (now - lastPackTime >= 5) {
      lastPackTime = now;

      // 固定创建 244 字节的缓冲区并默认全部初始化为 0 (满足默认为 0 的需求)
      uint8_t txBuf[244] = {0}; 
      
      // ---------- Header & Time ----------
      // [Byte 0] Seq
      txBuf[0] = packetSeq++; 
      // [Byte 1-4] Timestamp (Big Endian 保护时序)
      txBuf[1] = (now >> 24) & 0xFF;
      txBuf[2] = (now >> 16) & 0xFF;
      txBuf[3] = (now >> 8) & 0xFF;
      txBuf[4] = now & 0xFF;

      // ---------- Mic Data (Byte 5~165) ----------
      uint16_t mAvail = micBuf.available();
      if (mAvail > 160) mAvail = 160;
      mAvail = mAvail & ~1; // 仅限偶数
      
      txBuf[5] = (uint8_t)mAvail; // Valid Length
      for(uint16_t i=0; i<mAvail; i++) {
        micBuf.pop(txBuf[6 + i]);
      }
      // 如果 mAvail < 160，剩下的因为数组初始化过了自然全是 0 

      // ---------- Thermal Data (Byte 166~230) ----------
      uint16_t fAvail = thermalBuf.available();
      if (fAvail > 64) fAvail = 64;
      fAvail = fAvail & ~1; // 仅限偶数
      
      if (fAvail == 64) {
        txBuf[166] = 0x80 | (currentMlxRowOut & 0x1F);
        currentMlxRowOut++;
        if (currentMlxRowOut >= 24) currentMlxRowOut = 0;
      } else {
        txBuf[166] = (uint8_t)fAvail; // Valid Length
      }

      for(uint16_t i=0; i<fAvail; i++) {
        thermalBuf.pop(txBuf[167 + i]);
      }

      // ---------- IMU Data (Byte 231~243) ----------
      uint16_t iAvail = imuBuf.available();
      if (iAvail >= 12) iAvail = 12;
      else iAvail = 0;
      
      txBuf[231] = (uint8_t)iAvail; // Valid Length
      for(uint16_t i=0; i<iAvail; i++) {
        imuBuf.pop(txBuf[232 + i]);
      }

      // 一次性推送最高额 244 字节（如果接收端支持DLE）
      bleuart.write(txBuf, 244);
    }
  }
}