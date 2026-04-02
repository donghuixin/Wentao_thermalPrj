// Last Update Time: 2026-04-03 00:57
#include <LSM6DS3.h>
#include <Wire.h>
#include <bluefruit.h>
#include <PDM.h> 

#define DEVICE_NAME "wentaoCollector"

BLEUart bleuart;
LSM6DS3 myIMU(I2C_MODE, 0x6A);

bool imuOnline = false;
bool micOnline = false;
bool mlxOnline = false;

bool isRecording = false;
bool recMic = false;
bool recAcc = false;
bool recIr  = false;

bool pendingStatusReport = false;

short sampleBuffer[256];
volatile int samplesRead;

unsigned long lastImuTime = 0;
unsigned long lastMlxRowTime = 0;
int currentMlxRow = 0;

TwoWire WireMLX(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, D4, D5);

// ===================== MLX90642 正确寄存器定义 =====================
#define MLX90642_I2C_ADDR    0x66   // [cite: 108, 188]
#define MLX90642_STATUS_REG  0x3C14  // 包含 READY 标志位 
#define MLX90642_CONFIG_REG  0x11F0  // 刷新率配置地址 
#define MLX90642_RAM_START   0x342C  // 温度数据 (To) 起始地址 [cite: 214, 456, 463]

class ThermalDataCollector {
public:
    unsigned long lastFrameTime = 0;

    bool begin() {
        WireMLX.begin();
        // 降低时钟速率到标准的 100kHz。实验连线时 400kHz 极易因为分布电容导致通讯截断报错！
        WireMLX.setClock(100000); 
        
        uint16_t config = readRegister(MLX90642_CONFIG_REG);
        if (config == 0 || config == 0xFFFF) return false; 

        // 设置刷新率为 8Hz (Bit 0-2 设为 100)
        config &= ~0x0007; 
        config |= 0x0004; 
        writeRegister(MLX90642_CONFIG_REG, config);
        return true;
    }

    bool readRow(int r, uint8_t* rowData) {
        return readBlock(MLX90642_RAM_START + r * 32, rowData, 32); 
    }

private:
    uint16_t readRegister(uint16_t regAddress) {
        WireMLX.beginTransmission(MLX90642_I2C_ADDR);
        WireMLX.write(regAddress >> 8);   
        WireMLX.write(regAddress & 0xFF); 
        if(WireMLX.endTransmission(false) != 0) return 0; 
        WireMLX.requestFrom((uint8_t)MLX90642_I2C_ADDR, (uint8_t)2);
        uint16_t data = 0;
        if (WireMLX.available() == 2) {
            data = WireMLX.read() << 8;
            data |= WireMLX.read();
        }
        return data;
    }

    bool readBlock(uint16_t startAddress, uint8_t* buffer, int words) {
        int bytesToRead = words * 2;
        
        WireMLX.beginTransmission(MLX90642_I2C_ADDR);
        WireMLX.write(startAddress >> 8);   
        WireMLX.write(startAddress & 0xFF); 
        if(WireMLX.endTransmission(false) != 0) return false; 
        
        int bytesRead = 0;
        WireMLX.requestFrom((uint8_t)MLX90642_I2C_ADDR, (uint8_t)bytesToRead);
        while (WireMLX.available() && bytesRead < bytesToRead) {
            buffer[bytesRead++] = WireMLX.read();
        }
        
        return (bytesRead == bytesToRead);
    }

    void writeRegister(uint16_t regAddress, uint16_t data) {
        WireMLX.beginTransmission(MLX90642_I2C_ADDR);
        WireMLX.write(regAddress >> 8);   
        WireMLX.write(regAddress & 0xFF); 
        WireMLX.write(data >> 8);         
        WireMLX.write(data & 0xFF);       
        WireMLX.endTransmission();
    }
};

ThermalDataCollector thermalCollector;

void onPDMdata() {
    int bytesAvailable = PDM.available();
    PDM.read(sampleBuffer, bytesAvailable);
    samplesRead = bytesAvailable / 2; 
}

void prph_connect_callback(uint16_t conn_handle) {
    pendingStatusReport = true;
}

void sendSystemStatus() {
    bleuart.println("\n=== System Status ===");
    bleuart.printf("IMU: %s | Addr: 0x6A\n", imuOnline ? "OK" : "ERR");
    bleuart.printf("MIC: %s | Internal\n", micOnline ? "OK" : "ERR");
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
            bleuart.println("SYS: Recording STARTED.");
        }
    }
    else if (cmd == "STOP_REC" || cmd == "S" || cmd == "s") {
        isRecording = false;
        bleuart.println("SYS: Recording STOPPED.");
    }
    else if (cmd == "test" || cmd == "TEST") {
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

        // 1. Time Division Multiplexing - MLX Data (5ms per row, finishes 24 rows in 120ms -> covers 8Hz)
        if (recIr && mlxOnline && (now - lastMlxRowTime >= 5)) {
            lastMlxRowTime = now;
            
            uint8_t rowData[64];
            if (thermalCollector.readRow(currentMlxRow, rowData)) {
                // T + rowHex(2) + : + data(128) + \n = 133 chars.
                char txBuf[140];
                sprintf(txBuf, "T%02X:", currentMlxRow);
                int idx = 4;
                for (int i = 0; i < 64; i++) {
                    sprintf(txBuf + idx, "%02X", rowData[i]);
                    idx += 2;
                }
                txBuf[idx++] = '\n';
                txBuf[idx] = '\0';
                
                bleuart.print(txBuf);
            } else {
                static unsigned long lastMlxErrTime = 0;
                if (now - lastMlxErrTime > 1500) {
                    bleuart.printf("SYS: [ERR] MLX I2C read ROW %d failed.\n", currentMlxRow);
                    lastMlxErrTime = now;
                }
            }
            
            currentMlxRow++;
            if (currentMlxRow >= 24) currentMlxRow = 0;
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
            
            bleuart.printf("I:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", ax, ay, az, gx, gy, gz);
        }

        // 3. MIC Data
        if (recMic && micOnline && samplesRead > 0) {
            bleuart.printf("MIC:%d,%d\n", sampleBuffer[0], sampleBuffer[1]);
            samplesRead = 0; 
        }
    }
}