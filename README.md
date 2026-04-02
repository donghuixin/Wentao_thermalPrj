# Wentao Thermal Sensor Data Collector

A BLE-based multi-sensor data collection system built around the **Seeed XIAO nRF52840 (Sense) Plus** board. The firmware collects data from an IMU, microphone, and thermal IR sensor, and streams it over Bluetooth to a browser-based dashboard for real-time visualization, analysis, and CSV export.

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

### Data Output Format (Device → Client)

| Prefix | Meaning | Format |
|---|---|---|
| `I:` | IMU sample | `I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>` (g / dps) |
| `MIC:` | Microphone sample | `MIC:<sample0>,<sample1>` (16-bit PCM) |
| `MLX:S` … `MLX:E` | Thermal matrix frame | 24 rows of 32 comma-separated temperature values (°C) |
| `IMU: OK\|ERR` | Sensor status line | Part of system status report |
| `MIC: OK\|ERR` | Sensor status line | Part of system status report |
| `MLX: OK\|ERR` | Sensor status line | Part of system status report |

### IMU Batching

To maximize BLE throughput, IMU samples are batched — 5 samples are collected at 100 Hz (every 10 ms) and sent as a single BLE packet.

## Web Dashboard (`TestConsole.html`)

### Overview

A single-page HTML dashboard that connects to the sensor board via **Web Bluetooth API**. Open it directly in **Google Chrome** (Web Bluetooth is required).

### Features

| Panel | Description |
|---|---|
| **IMU 6-Axis Realtime** | ECharts line chart showing accelerometer (X/Y/Z) and gyroscope (X/Y/Z) data in real time |
| **Microphone Waveform** | ECharts bar chart displaying live audio amplitude |
| **Thermal Heatmap (8×8)** | Canvas-rendered color heatmap (blue → red, 15 °C – 40 °C range) |
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

## License

This project is provided as-is for educational and prototyping purposes.
