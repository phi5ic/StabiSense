---
publishDate: 2026-05-25T00:00:00Z
title: StabiSense – A Self-Calibrating IMU-Based Micro-Scale Stabilization System with Real-Time Adaptive Control
excerpt: A real-time seismic monitoring system using the ESP32 MYOSA kit that classifies motion events into four severity levels using an on-device TinyML model, with a browser-based telemetry dashboard for live analysis, event replay, and data export.
image: full-setup.jpeg
tags:
  - ESP32
  - TinyML
  - Seismic
  - MYOSA
  - IMU
---

> An embedded AI system that detects tremors, landslides, and seismic shocks in real time — displayed locally on an OLED and remotely through a browser telemetry dashboard.

---

## Acknowledgements

Built using the MYOSA sensor kit platform with the AccelAndGyro, BarometricPressure, LightProximityAndGesture, and OLED libraries. TinyML inference powered by `eloquent_tinyml` and `tflm_esp32`.

---

## Overview

StabiSense turns the MYOSA ESP32 kit into a standalone seismic and motion monitor. The device reads 3-axis acceleration and gyroscope data from the MPU6050, fuses them into roll and pitch angles using a Mahony AHRS filter, and feeds a rolling 200-sample window into a 1D-CNN TensorFlow Lite Micro model running entirely on the ESP32.

The model classifies incoming motion into one of four states in real time. A local SSD1306 OLED display provides standalone field feedback, while a browser-based dashboard connects over Web Serial or BLE for live oscilloscope plots, event recording, timeline replay, and CSV/JSON export.

**Key features:**

* On-device TinyML seismic classification with four severity classes
* Mahony filter orientation — roll, pitch, and calibrated gyro bias
* Multi-page SSD1306 OLED interface with gesture-swipe navigation
* Browser telemetry dashboard with live graphs, 3D orientation cube, and event replay
* FreeRTOS mutex-protected I2C bus for reliable multi-sensor operation
* Hybrid detection combining AI model output with rule-based motion score fallback

---

## Demo / Examples

### **Images**

<p align="center">
  <img src="full-setup.jpeg" width="800"><br/>
  <i>Complete system in operation — MYOSA sensor kit connected over USB, browser dashboard streaming live accelerometer waveforms from the ESP32 TinyML inference pipeline</i>
</p>

<p align="center">
  <img src="hardware-closeup.jpeg" width="800"><br/>
  <i>MYOSA ESP32 board with MPU6050 (AccelAndGyro), BMP180 (BarometricPressure), APDS9960 (gesture), and SSD1306 OLED mounted in the sensor kit enclosure</i>
</p>

<p align="center">
  <img src="dashboard-live-seismic.jpeg" width="800"><br/>
  <i>Browser telemetry dashboard showing a live seismic event — three-channel oscilloscope traces (ax, ay, az) with AI classification panel and real-time sensor values</i>
</p>

### **Videos**

<p align="center">
  <a href="https://github.com/phi5ic/StabiSense/raw/main/demo_YP5JgmB8.mp4">
    <img src="dashboard-live-seismic.jpeg" width="800"><br/>
    <i>▶ Click to watch demo video</i>
  </a>
</p>

---

## Features (Detailed)

### **1. Four-Class Seismic Classification**

The on-device TinyML model classifies motion into four states:

| Class | Label       | Description                                   |
|-------|-------------|-----------------------------------------------|
| 0     | `IDLE`      | No significant motion detected                |
| 1     | `TREMOR`    | Small rapid vibrations or shaking             |
| 2     | `LANDSLIDE` | Stronger movement, tilt, or instability       |
| 3     | `SEISMIC`   | High-intensity shock or earthquake-like event |

The model is a 1D-CNN stored as a C byte array in `seismic_model.h` and runs entirely on-chip using TensorFlow Lite Micro. It accepts a rolling window of 200 samples × 5 features (`roll, pitch, accX, accY, accZ`) = 1000 float inputs, updated every 10 ms.

### **2. Sensor Fusion with Mahony Filter**

Raw MPU6050 data is fused at approximately 100 Hz. The Mahony filter produces stable roll and pitch estimates robust against gyro drift. Gyro bias is calibrated at boot from 500 still samples — the device must remain stationary for the first 5 seconds.

```cpp
float ax_g = raw_ax / 16384.0f;           // ±2g range
float gx_deg = (raw_gx - gyroX_bias) / 131.0f;  // ±250°/s range
```

### **3. Multi-Page OLED Interface**

The SSD1306 display cycles through four pages, navigated by APDS9960 swipe gestures:

| Swipe        | Action                            |
|--------------|-----------------------------------|
| LEFT / RIGHT | Switch OLED page                  |
| UP           | Tare IMU to current orientation   |
| DOWN         | Pause / resume serial telemetry   |

Pages include a boot splash, live AI dashboard (roll, pitch, class, confidence), roll/pitch oscilloscope, and a bubble-level calibration view. An emergency warning screen overrides all pages when class is `LANDSLIDE` or `SEISMIC` and confidence exceeds 85%.

### **4. Browser Telemetry Dashboard**

A single-file HTML dashboard (`index.html`) requires no installation and runs in Chrome or Edge. It accepts data in three modes:

- **Simulation** — generates realistic idle and seismic event sequences for UI testing
- **Web Serial** — connects directly to the ESP32 at 115200 baud
- **BLE** — connects via the WASTP BLE service

Dashboard capabilities include live oscilloscope traces, FFT spectrum, 3D orientation cube, AI classification panel with per-class confidence bars, event recording, timeline replay with event markers, and export to CSV or JSON.

### **5. FreeRTOS Multi-Task Architecture**

The IMU pipeline runs in a dedicated FreeRTOS task pinned to core 1, sampling at 100 Hz independently of the main loop. All four I2C devices (MPU6050, BMP180, APDS9960, SSD1306) share the same bus, protected by a binary semaphore mutex to prevent bus collisions and ensure stable readings.

```cpp
extern SemaphoreHandle_t i2cMutex;
```

---

## Usage Instructions

**Startup:**

1. Wire all MYOSA sensors to the ESP32 I2C bus.
2. Upload the firmware using Arduino IDE.
3. Open Serial Monitor at `115200` baud.
4. Keep the device completely still for gyro calibration (approximately 5 seconds).
5. Wait for the confirmation:

```plaintext
>>> Gyro calibration complete.
>>> IMU task started.
```

6. The OLED will display the boot splash, then switch to the AI dashboard.

**Dashboard:**

1. Open `index.html` in Chrome or Edge.
2. Select the data source (Simulation, Serial Port, or BLE).
3. Click **Connect**, then **Record** to begin capturing events.
4. Use **Mark Event** to manually tag notable moments.
5. Open the **Replay** tab to review recordings.
6. Export data using **CSV** or **JSON**.

---

## Tech Stack

* **ESP32** — main microcontroller, FreeRTOS, I2C master
* **MPU6050** — 3-axis accelerometer and gyroscope via MYOSA AccelAndGyro library
* **BMP180** — temperature and pressure via MYOSA BarometricPressure library
* **APDS9960** — gesture input via MYOSA LightProximityAndGesture library
* **SSD1306** — 128×64 OLED display via MYOSA OLED library
* **TensorFlow Lite Micro** — on-device 1D-CNN inference (`tflm_esp32`, `eloquent_tinyml`)
* **Mahony AHRS** — complementary sensor fusion filter
* **HTML / CSS / JavaScript** — browser telemetry dashboard (no framework, single file)

---

## Requirements / Installation

Install the following libraries via the Arduino Library Manager:

```bash
Wire
Adafruit_GFX
Adafruit_SSD1306
Adafruit_BMP085
Adafruit_APDS9960
MahonyAHRS
tflm_esp32
eloquent_tinyml
```

Install the MYOSA sensor drivers from the official repository:

```bash
https://github.com/myosa-sensors/arduino-libraries
```

**Board settings (Arduino IDE):**

```plaintext
Board:            ESP32 Dev Module
CPU Frequency:    240 MHz
Flash Mode:       DIO
Partition Scheme: Huge APP (for TinyML model)
Upload Speed:     921600
Baud Rate:        115200
```

**Wiring:**

| Component | ESP32 Pin |
|-----------|-----------|
| I2C SDA   | GPIO 21   |
| I2C SCL   | GPIO 22   |
| VCC       | 3.3V      |
| GND       | GND       |

---

## File Structure

```
/StabiSense
 ├── stabl.ino
 ├── Globals.h
 ├── IMU_Module.h
 ├── IMU_Module.cpp
 ├── Display_Module.h
 ├── Display_Module.cpp
 ├── BMP_Module.h
 ├── BMP_Module.cpp
 ├── seismic_model.h
 ├── index.html
 ├── full-setup.jpeg
 ├── hardware-closeup.jpeg
 ├── dashboard-live-seismic.jpeg
 └── demo_YP5JgmB8.mp4
```

---

## License

MIT License.

---

## Contribution Notes

To improve model accuracy, retrain using data collected from this hardware at 100 Hz with features in the order `roll, pitch, accX, accY, accZ`, normalized using training-set mean and standard deviation. Export the normalization constants to firmware so inference input matches training input precisely.

Issues and pull requests are welcome via the project repository.
