# Wentao Thermal Sensor Data Collector

A BLE-based multi-sensor data collection system built around the **Seeed XIAO nRF52840 (Sense) Plus** board. The firmware collects data from an IMU, microphone, and thermal IR sensor, and streams it over Bluetooth to a browser-based dashboard for real-time visualization, analysis, and CSV export.

## Latest Updates (2026-04-03)

1. **High-rate link diagnostics added**
   - Firmware now reports negotiated BLE link parameters after connection:
   - `PHY`, `Connection Interval`, `MTU`, `Data Length`
   - TX runtime stats: sent / skipped / failed packets and write-latency metrics.

2. **IMU transmission de-duplication for 100Hz mode**
   - IMU sampling is set to **100Hz** while packet transport remains high-rate.
   - Device sends IMU payload only when a **new IMU sample** is available.
   - If no new sample is available in that packet slot, `IMU Valid Len` is set to `0` (no zero-filling).
   - Web dashboard keeps orientation continuity by holding the last valid IMU frame (`HOLD` state in log).

3. **Buffer policy aligned to thermal 2-frame window**
   - Thermal buffer kept at about 2 full frames.
   - Mic and IMU buffering adjusted to match practical anti-jitter windows, avoiding unnecessary long queue delay.

4. **Dashboard usability updates**
   - IMU and Mic charts now use adaptive y-axis range.
   - CSV export remains raw-value oriented for offline analysis.

## Project Structure

```
Wentao_Project/
├── Wentao/
│   └── Wentao.ino          # Arduino firmware for the sensor board
├── TestConsole.html         # Web Bluetooth dashboard (open in Chrome)
└── README.md
```

## Hardware

| Component | Model | Interface | Address / Details |
|---|---|---|---|
| MCU | Seeed XIAO nRF52840 (Sense) | — | BLE 5.0, ARM Cortex-M4F |
| IMU | LSM6DS3 (6-axis Accel + Gyro) | I²C | `0x6A` · Accel ±2 g · Gyro ±2000 dps |
| Microphone | Built-in PDM Mic | PDM | Mono, 16 kHz sample rate |
| Thermal IR | MLX90642 (24 × 32 array) | I²C (Wire1, pins D4/D5) | `0x66` · 8 Hz refresh rate |

## Firmware (`Wentao/Wentao.ino`)

### Overview

The firmware runs on the Arduino platform using the **Adafruit nRF52 BSP**. On boot it:

1. Initializes BLE with the Nordic UART Service (NUS).
2. Probes each sensor (IMU, Microphone, MLX90642) and records online/offline status.
3. Begins BLE advertising as **`wentaoCollector`**.
4. On client connection, automatically sends a system status report.

### BLE Communication

The firmware exposes the **Nordic UART Service (NUS)**:

| UUID | Role |
|---|---|
| `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | Service |
| `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | TX Characteristic (write from client → device) |
| `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | RX Characteristic (notify, device → client) |

### Commands (Client → Device)

| Command | Description |
|---|---|
| `START_REC:<mic>,<acc>,<ir>` | Start recording. Each flag is `1` (on) or `0` (off). Example: `START_REC:1,1,0` |
| `STOP_REC` / `S` / `s` | Stop recording |
| `test` / `TEST` | Request system status report |

### Data Output & Storage Format (Device → Client)

The firmware completely abandoned ASCII string transmission format, leveraging a highly optimized, high-bandwidth padding-aligned **Binary `C-Struct`**. Each transmission pushes exactly 244 bytes down the BLE pipe. 

This strict layout allows any external parser (e.g. C#, Python, MATLAB) to unpack the buffers directly based on strict memory offsets:

| Byte Offset | Length (B) | Field Name | Type / Endianness | Parsing Logic & Padding Rules |
|:---|:---|:---|:---|:---|
| `[0]` | 1 | **Seq** | `uint8_t` | A running counter `0-255` rolling continuously. Used on the Web UI to explicitly detect and flag packet loss jumps in physical space. |
| `[1-4]` | 4 | **Timestamp** | `uint32_t` (Big-Endian) | Device `millis()` heartbeat marker to correlate logs externally. |
| `[5]` | 1 | **Mic Valid Len** | `uint8_t` | Identifies how many *Valid* Microphone bytes are in the next block. Max is 160. Must be strictly an even number. |
| `[6-165]` | 160 | **Mic Payload** | `int16_t[80]` (Little-Endian) | The raw PCM Audio array. If purely valid data occupies `V` bytes (`V <= 160`), the remainder `160 - V` bytes are explicitly zero-padded `0x00`. |
| `[166]` | 1 | **Thermal Valid Len** | `uint8_t` | Identifies valid Thermal frame length. Standard size is 64 Bytes (single row of 32 pixels). If 0, no data is available. |
| `[167-230]` | 64 | **Thermal Payload** | `int16_t[32]` (**Big-Endian**) | Heat signature block representing exactly 1 row (32 pixels). Bypasses memory layout conversion and injects directly from I2C reading (hence Big Endian). Trailing layout space explicitly zeros if `Len == 0`. |
| `[231]` | 1 | **IMU Valid Len** | `uint8_t` | IMU de-dup mode: `12` = new IMU sample in this packet, `0` = no new IMU sample in this packet slot. |
| `[232-243]` | 12 | **IMU Payload** | `int16_t[6]` (Little-Endian) | Struct layout: `[Ax, Ay, Az, Gx, Gy, Gz]` when `Len == 12`. When `Len == 0`, receiver should keep the last valid IMU state for display continuity. |

### Architectural Features

1. **Wait-free TDM (Time-Division Multiplexing) Bluetooth Streaming**:
   By spacing IR row queries every 5ms and IMU polls every 10ms, the system "drip-feeds" data into the nRF52 BLE TX buffer without triggering fatal `GATTC_OUT_OF_RESOURCES` errors.
2. **Unified Hardware I2C (TWIM0)**:
   A single, standardized `Wire` (D4 & D5) object controls both internal and external sensors to completely avert Nordic DMA multiplexer conflicts during concurrent reads.

## Web Dashboard (`TestConsole.html`)

### Overview

A single-page HTML dashboard that connects to the sensor board via **Web Bluetooth API**. Open it directly in **Google Chrome** (Web Bluetooth is required).

### Features

| Panel | Description |
|---|---|
| **IMU 6-Axis Realtime** | ECharts line chart showing accelerometer (X/Y/Z) and gyroscope (X/Y/Z) data in real time |
| **Microphone Waveform** | ECharts bar chart displaying live audio amplitude |
| **Thermal Heatmap (32×24)** | Canvas-rendered color heatmap (blue → red, 15 °C – 40 °C range) |
| **3D Board Posture** | Three.js scene showing a PCB model whose orientation tracks IMU data via Mahony AHRS filter |
| **Live Data Stream** | Scrolling terminal log of all TX/RX messages with timestamps |

### Controls

- **Connect Bluetooth** — pair with the `wentaoCollector` device
- **Sensor toggles** — enable / disable Mic, IMU, Thermal IR individually before recording
- **Test** — send `test` command to retrieve sensor status
- **Start / Stop Log** — begin / end data recording on the device
- **Pause Tx** — send pause command `P`
- **Save CSV** — export all logged data to a timestamped `.csv` file

### Dependencies (loaded via CDN)

- [ECharts 5.6.0](https://cdn.jsdelivr.net/npm/echarts@5.6.0/dist/echarts.min.js)
- [Three.js r160](https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js)

## Getting Started

### 1. Flash the Firmware

1. Install the **Arduino IDE** with the [Adafruit nRF52 Board Support Package](https://github.com/adafruit/Adafruit_nRF52_Arduino).
2. Install the following Arduino libraries:
   - `LSM6DS3` (SparkFun)
   - `Adafruit nRF52` core (includes `bluefruit.h` and `PDM.h`)
3. Open `Wentao/Wentao.ino` in Arduino IDE.
4. Select board **Seeed XIAO nRF52840 (Sense)** and the correct COM port.
5. Click **Upload**.

### 2. Open the Dashboard

1. Open `TestConsole.html` in **Google Chrome** (desktop, version 56+).
2. Click **🔗 Connect Bluetooth** and select the **wentaoCollector** device.
3. The system status will appear automatically in the terminal.
4. Toggle desired sensors, then click **▶ Start Log** to begin streaming.

### 3. Export Data

Click **💾 Save CSV** at any time to download all logged data as a CSV file.

## Requirements

- **Browser**: Google Chrome 56+ (Web Bluetooth support required)
- **Hardware**: Seeed XIAO nRF52840 Sense + MLX90642 thermal sensor module
- **Arduino BSP**: Adafruit nRF52 Arduino Core

## System Architecture Analysis & XIAO nRF52840 Guidelines

Based on the [BLE-develop-skills-nRF_Connect](https://github.com/donghuixin/BLE-develop-skills-nRF_Connect) knowledge base and hardware deep dives, the current architecture was analyzed for enterprise-level RTOS scalability:

1. **BLE Protocol (Unified UART vs Custom GATT)**:
   Currently, the project utilizes firmware-level Time Division Multiplexing (TDM) to multiplex HEX MLX streams and String IMU streams over a single BLE UART Characteristic. While efficient for prototyping, a professional Zephyr (NCS) stack mandates strictly defining distinct GATT characteristic structured payloads for maximum decoupling.

2. **Deep Sleep & Peripherals**:
   The XIAO nRF52840 Sense has critical hardware paradigms for low power:
   - **Battery Monitoring**: `P0.14` acts as a logic switch to the resistor divider to prevent continuous leakage.
   - **QSPI Flash**: The 2MB external Flash draws 50-100µA at idle unless a dedicated `0xB9` SPI Deep Power-Down command is sent.
   - **Superloop vs RTOS**: `loop()` handles polling gracefully but spins the CPU. Fully utilizing Zephyr's `k_work` or hardware GPIO interrupts (e.g., IMU `INT1`) allows the CPU to enter System ON idle sleep.

3. **Recommended Migration**:
   Moving forward into production or integration with Mesh/Matter networks warrants migrating this project from the Arduino Adafruit BSP to the **nRF Connect SDK (NCS)**. Doing so shifts hardware definitions explicitly out of C structs and into Devicetree (`.overlay`) files, preventing bricking and maintaining rigorous hardware truth.

## License

This project is provided as-is for educational and prototyping purposes.
