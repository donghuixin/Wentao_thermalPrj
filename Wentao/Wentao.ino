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
char imuBatchBuffer[250]; 
int imuBatchLen = 0;
int imuSampleCount = 0;

TwoWire WireMLX(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, D4, D5);

// ===================== MLX90642 正确寄存器定义 =====================
#define MLX90642_I2C_ADDR    0x66   // [cite: 108, 188]
#define MLX90642_STATUS_REG  0x3C14  // 包含 READY 标志位 
#define MLX90642_CONFIG_REG  0x11F0  // 刷新率配置地址 
#define MLX90642_RAM_START   0x342C  // 温度数据 (To) 起始地址 [cite: 214, 456, 463]

class ThermalDataCollector {
public:
    bool begin() {
        WireMLX.begin();
        WireMLX.setClock(400000); // 建议 400kHz 以上速率 [cite: 1130]
        
        uint16_t config = readRegister(MLX90642_CONFIG_REG);
        if (config == 0 || config == 0xFFFF) return false; 

        // 设置刷新率为 8Hz (Bit 0-2 设为 100) [cite: 501, 521, 533]
        config &= ~0x0007; 
        config |= 0x0004; 
        writeRegister(MLX90642_CONFIG_REG, config);
        return true;
    }

    void update(bool transmit) {
        uint16_t status = readRegister(MLX90642_STATUS_REG);
        // 检查 READY 标志位 (Bit 3) [cite: 966, 968]
        if (!(status & 0x0008)) return; 

        if (transmit) {
            for (int r = 0; r < 24; r++) { // 24 行
                uint8_t rowData[64]; // 32 像素点 * 2 字节
                bool ok = readBlock(MLX90642_RAM_START + r * 32, rowData, 32); 
                
                if (ok) {
                    uint8_t packet[66];
                    packet[0] = 0xFE; // Binary Chunk Magic Byte
                    packet[1] = (uint8_t)r; // Row Index (0-23)
                    memcpy(packet + 2, rowData, 64);
                    bleuart.write(packet, 66); // 发送二进制行数据
                }
            }
        }
        // 注意：READY 标志在读取起始地址 0x342C 后会自动清除（或者手动读取一次起址清除）
        if (!transmit) {
            readRegister(MLX90642_RAM_START); // 未发送热成像时，也需读取一次清空READY
        }
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
        // 由于绝大部分 Wire 库单次读取限制为 32 字节或 256 字节，分每 16 字 (32 字节) 一块读取确保最高兼容性
        int offset = 0;
        while (words > 0) {
            int chunkWords = (words > 16) ? 16 : words;
            int chunkBytes = chunkWords * 2;
            
            WireMLX.beginTransmission(MLX90642_I2C_ADDR);
            WireMLX.write((startAddress + offset) >> 8);   
            WireMLX.write((startAddress + offset) & 0xFF); 
            if(WireMLX.endTransmission(false) != 0) return false; 
            
            int bytesRead = 0;
            WireMLX.requestFrom((uint8_t)MLX90642_I2C_ADDR, (uint8_t)chunkBytes);
            while (WireMLX.available() && bytesRead < chunkBytes) {
                buffer[offset * 2 + bytesRead++] = WireMLX.read();
            }
            if (bytesRead != chunkBytes) return false;
            
            words -= chunkWords;
            offset += chunkWords;
        }
        return true;
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
            imuBatchLen = 0; 
            imuSampleCount = 0;
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
    
    // 定时检查热成像数据，若 READY 则输出全矩阵
    if (mlxOnline) {
        thermalCollector.update(isRecording && recIr);
    }

    if (isRecording) {
        if (recAcc && imuOnline) {
            if (millis() - lastImuTime >= 10) {
                lastImuTime = millis();
                float ax = myIMU.readFloatAccelX();
                float ay = myIMU.readFloatAccelY();
                float az = myIMU.readFloatAccelZ();
                float gx = myIMU.readFloatGyroX();
                float gy = myIMU.readFloatGyroY();
                float gz = myIMU.readFloatGyroZ();
                
                imuBatchLen += sprintf(imuBatchBuffer + imuBatchLen, "I:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", ax, ay, az, gx, gy, gz);
                imuSampleCount++;

                if (imuSampleCount >= 5) {
                    bleuart.print(imuBatchBuffer);
                    imuBatchLen = 0;
                    imuSampleCount = 0;
                }
            }
        }

        if (recMic && micOnline && samplesRead > 0) {
            bleuart.printf("MIC:%d,%d\n", sampleBuffer[0], sampleBuffer[1]);
            samplesRead = 0; 
        }
    }
}