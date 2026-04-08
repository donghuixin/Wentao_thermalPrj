// Last Update Time: 2026-04-03 13:20
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
bool pendingLinkDiagReport = false;
uint8_t pendingLinkDiagCount = 0;
unsigned long lastLinkDiagMs = 0;

bool bleNegotiationDone = false;
unsigned long bleConnectTime = 0;
const unsigned long BLE_NEGO_TIMEOUT_MS = 8000; // 8秒超時容錯

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
RingBuf<uint8_t, 8192> micBuf; // 约250ms窗口（与2帧红外时间尺度一致）：160B*50包≈8000B。
RingBuf<uint8_t, 3072> thermalBuf; // 约可缓存 2 帧红外图，平衡 RAM 占用与抗抖动能力。
RingBuf<uint8_t, 512> imuBuf; // 100Hz时每10ms产12B，512B可覆盖约420ms抖动。

short sampleBuffer[512]; // PDM library DMA Cache

unsigned long lastPackTime = 0;
unsigned long lastMlxReadTime = 0;
unsigned long lastImuTime = 0;
uint32_t nextPackUs = 0;
uint32_t nextImuUs = 0;
uint32_t nextMlxUs = 0;
const uint32_t PACK_PERIOD_US = 5000; // 200Hz
const uint32_t IMU_PERIOD_US = 10000; // 100Hz
const uint32_t MLX_PERIOD_US = 2000;  // up to 500Hz row prefetch

uint8_t lastImuPacket[12] = {0};
bool hasLastImuPacket = false;
bool imuSampleFresh = false;
int currentMlxRow = 0;
int currentMlxRowOut = 0;
uint8_t packetSeq = 0;
uint32_t nextTxUs = 0;
uint32_t txBackoffUs = 0;

uint32_t txSentCount = 0;
uint32_t txSkipNoCreditCount = 0;
uint32_t txWriteFailCount = 0;
uint32_t txWriteCount = 0;
uint32_t txWriteTotalUs = 0;
uint32_t txWriteMaxUs = 0;

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
    Wire.setClock(1000000);
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

static const char* phyToStr(uint8_t phy) {
  if (phy == BLE_GAP_PHY_2MBPS) return "2M";
  if (phy == BLE_GAP_PHY_1MBPS) return "1M";
  if (phy == BLE_GAP_PHY_CODED) return "CODED";
  return "AUTO/UNK";
}

void sendLinkDiagnostics(void) {
  uint16_t conn_hdl = Bluefruit.connHandle();
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  if (!conn || !bleuart.notifyEnabled(conn_hdl)) return;

  float ci_ms = conn->getConnectionInterval() * 1.25f;
  bleuart.printf("LINK: PHY=%s | CI=%.2fms | MTU=%u | DLE=%u\n",
                 phyToStr(conn->getPHY()),
                 ci_ms,
                 conn->getMtu(),
                 conn->getDataLength());
  bleuart.printf("TX: sent=%lu skipNoCredit=%lu fail=%lu\n",
                 txSentCount,
                 txSkipNoCreditCount,
                 txWriteFailCount);
  uint32_t avgWriteUs = txWriteCount ? (txWriteTotalUs / txWriteCount) : 0;
  bleuart.printf("TX_LAT: avg=%luus max=%luus backoff=%luus\n",
                 avgWriteUs,
                 txWriteMaxUs,
                 txBackoffUs);
}

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
  pendingLinkDiagReport = true;
  pendingLinkDiagCount = 8; // 连接后连续报告几次，观察参数协商是否成功
  lastLinkDiagMs = 0;
  bleNegotiationDone = false;
  bleConnectTime = millis();
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

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  isRecording = false;
  bleNegotiationDone = false;
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
      if (!bleNegotiationDone) {
        bleuart.println("SYS: WAIT - BLE negotiation in progress...");
        return;
      }
      recMic = (cmd.charAt(10) == '1');
      recAcc = (cmd.charAt(12) == '1');
      recIr  = (cmd.charAt(14) == '1');
      isRecording = true;
      uint32_t nowUs = micros();
      nextPackUs = nowUs + PACK_PERIOD_US;
      nextImuUs = nowUs + IMU_PERIOD_US;
      nextMlxUs = nowUs + MLX_PERIOD_US;
      nextTxUs = nowUs;
      txBackoffUs = 0;
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
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 6); // 优先请求最短 7.5ms

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

  // BLE 參數協商完成檢測
  if (!bleNegotiationDone && Bluefruit.connected()) {
    uint16_t conn_hdl = Bluefruit.connHandle();
    BLEConnection* conn = Bluefruit.Connection(conn_hdl);
    if (conn && bleuart.notifyEnabled(conn_hdl)) {
      uint16_t mtu = conn->getMtu();
      unsigned long elapsed = millis() - bleConnectTime;
      if (mtu >= 200) {
        bleNegotiationDone = true;
        bleuart.printf("SYS: BLE_READY (MTU=%u, %.1fs)\n", mtu, elapsed / 1000.0);
      } else if (elapsed > BLE_NEGO_TIMEOUT_MS) {
        bleNegotiationDone = true;
        bleuart.printf("SYS: BLE_READY (timeout, MTU=%u)\n", mtu);
      }
    }
  }

  if (isRecording) {
    uint32_t nowUs = micros();
    unsigned long now = millis();

    if (pendingLinkDiagReport && bleuart.notifyEnabled() && (now - lastLinkDiagMs >= 500)) {
      lastLinkDiagMs = now;
      sendLinkDiagnostics();
      if (pendingLinkDiagCount > 0) pendingLinkDiagCount--;
      if (pendingLinkDiagCount == 0) pendingLinkDiagReport = false;
    }

    // 只要距离上次读取大于 2ms 且缓冲池有一行 (64B) 以上空余量，就提早并发读取下一行
    // 避免 5ms 的死板卡点导致来不急填满缓冲区
    if (recIr && mlxOnline && (int32_t)(nowUs - nextMlxUs) >= 0) {
      nextMlxUs += MLX_PERIOD_US;
      if ((int32_t)(nowUs - nextMlxUs) > (int32_t)(MLX_PERIOD_US * 2)) nextMlxUs = nowUs + MLX_PERIOD_US;
      if (thermalBuf.available() < (3072 - 64)) {
        lastMlxReadTime = now;
        thermalCollector.readRowToBuf(currentMlxRow);
        currentMlxRow++;
        if (currentMlxRow >= 24) currentMlxRow = 0;
      }
    }

    // 保持 IMU 200Hz；优先使用库函数确保稳定出数，避免低层 burst 在不同核心版本下偶发失败
    if (recAcc && imuOnline && (int32_t)(nowUs - nextImuUs) >= 0) {
      nextImuUs += IMU_PERIOD_US;
      if ((int32_t)(nowUs - nextImuUs) > (int32_t)(IMU_PERIOD_US * 2)) nextImuUs = nowUs + IMU_PERIOD_US;
      lastImuTime = now;
      int16_t ax = myIMU.readRawAccelX();
      int16_t ay = myIMU.readRawAccelY();
      int16_t az = myIMU.readRawAccelZ();
      int16_t gx = myIMU.readRawGyroX();
      int16_t gy = myIMU.readRawGyroY();
      int16_t gz = myIMU.readRawGyroZ();

      int16_t imuData[6] = {ax, ay, az, gx, gy, gz};
      uint8_t* bytePtr = (uint8_t*)imuData;
      for(int i=0; i<12; i++) {
        imuBuf.push(bytePtr[i]);
        lastImuPacket[i] = bytePtr[i];
      }
      hasLastImuPacket = true;
      imuSampleFresh = true;
    }

    // 严苛定长定偏移打包 (Fixed Padding Struct)
    if ((int32_t)(nowUs - nextPackUs) >= 0) {
      nextPackUs += PACK_PERIOD_US;
      if ((int32_t)(nowUs - nextPackUs) > (int32_t)(PACK_PERIOD_US * 2)) nextPackUs = nowUs + PACK_PERIOD_US;
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
      // 仅在有新样本时发送一次，避免 100Hz 采样在 200Hz 包率下重复发送同一帧。
      uint16_t iAvail = imuBuf.available();
      if (iAvail >= 12) {
        while (imuBuf.available() >= 12) {
          for (uint16_t i = 0; i < 12; i++) {
            imuBuf.pop(lastImuPacket[i]);
          }
          hasLastImuPacket = true;
          imuSampleFresh = true;
        }
      }

      if (hasLastImuPacket && imuSampleFresh) {
        txBuf[231] = 12;
        for (uint16_t i = 0; i < 12; i++) {
          txBuf[232 + i] = lastImuPacket[i];
        }
        imuSampleFresh = false;
      } else {
        txBuf[231] = 0;
      }

      // 自适应节流：在当前 core 无可用 TX-credit API 的情况下，用写入耗时反馈避免持续阻塞
      uint16_t conn_hdl = Bluefruit.connHandle();
      if (conn_hdl == BLE_CONN_HANDLE_INVALID || !bleuart.notifyEnabled(conn_hdl)) {
        txSkipNoCreditCount++;
      } else if ((int32_t)(nowUs - nextTxUs) < 0) {
        txSkipNoCreditCount++;
      } else {
        uint32_t t0 = micros();
        size_t written = bleuart.write(conn_hdl, txBuf, 244);
        uint32_t dt = micros() - t0;
        txWriteCount++;
        txWriteTotalUs += dt;
        if (dt > txWriteMaxUs) txWriteMaxUs = dt;

        if (written == 244) txSentCount++;
        else txWriteFailCount++;

        // 写入越慢，说明中央/链路越接近饱和，增加退避减少主循环被阻塞概率
        if (dt > 1800) {
          if (txBackoffUs < 3000) txBackoffUs += 200;
        } else if (dt < 600) {
          if (txBackoffUs >= 100) txBackoffUs -= 100;
          else txBackoffUs = 0;
        }

        nextTxUs = nowUs + txBackoffUs;
      }
    }
  }
}