# 🚁 Gesture-Controlled Dual-ESP32 Drone

A custom-built, scratch-engineered micro quadcopter controlled entirely by hand gestures. This project utilizes a dual-microcontroller architecture, leveraging the high-speed ESP-NOW protocol to translate physical hand movements and acceleration spikes into real-time flight dynamics.

## 🧠 System Architecture

This project is divided into two distinct, isolated environments that communicate wirelessly with near-zero latency.

### 1. The Transmitter (The Glove)
A wearable, self-contained unit that tracks the user's hand orientation and broadcasts the target setpoints.
* **Microcontroller:** Seeed Studio XIAO ESP32-C3
* **Sensor:** GY-521 (MPU6050) 6-Axis Accelerometer & Gyroscope
* **Power:** 3.7V 150mAh Micro LiPo Battery
* **Logic:** Runs a Madgwick Sensor Fusion filter to continuously calculate hyper-accurate Pitch, Roll, and Yaw angles, while monitoring the Z-axis for sudden acceleration spikes (throttle gestures). Packages this telemetry and transmits it at high frequency.

### 2. The Receiver (The Drone)
A lightweight micro quadcopter that receives telemetry, monitors its own physical orientation, and drives the brushed motors to maintain stability.
* **Frame:** Q30 Micro Quadcopter Frame
* **Microcontroller:** ESP32-CAM
* **Onboard IMU:** GY-521 (MPU6050) 6-Axis Accelerometer & Gyroscope (Required for Closed-Loop Auto-Correction)
* **Motors:** 4x 8520 Coreless Brushed Motors
* **Power System:** TATTU R-Line 1S LiPo (500mAh, 95C discharge rate)
* **Motor Drivers:** Custom power distribution circuit using SI2302 N-Channel MOSFETs (SOT-23), SOT23-to-DIP routing adapters, and 1N4148 flyback diodes to protect the ESP32 from voltage spikes.
* **Logic:** Receives the pilot's "target" angles via ESP-NOW. Reads its own "current" angles using an onboard Madgwick filter. Feeds both sets of data into a high-speed PID Controller loop to calculate the exact PWM voltage signals needed to auto-level the drone and execute maneuvers.

## 📡 Communication Protocol (ESP-NOW)
To achieve the lowest possible control latency without relying on an external Wi-Fi router, this project uses Espressif's **ESP-NOW** protocol. It operates as a direct, peer-to-peer 2.4GHz radio link between the XIAO ESP32-C3 and the ESP32-CAM.

## 🕹️ Flight Dynamics Mapping
The drone's directional movement is mapped directly to the physical tilt and acceleration of the glove's MPU6050 sensor:

* **Pitch (Forward/Backward):** Tilt hand fingers downward / pull wrist upward.
* **Roll (Bank Left/Right):** Bank hand left / right.
* **Yaw (Turn Left/Right):** Twist wrist left / right (Z-axis rotation).
* **Throttle (Altitude):** Controlled via Z-axis acceleration "bumps" (Discrete Step Function).
  * **Upward Bump:** Adds a fixed step of base power (drone climbs).
  * **Downward Bump:** Subtracts the exact fixed step of base power (returns to hover, or descends if bumped again).

## 📂 Repository Structure
This repository is managed via PlatformIO and contains two separate microcontroller environments to prevent cross-compilation errors:
* `/glove-transmitter` - C++ source code configured specifically for the XIAO ESP32-C3.
* `/flight-controller` - C++ source code configured specifically for the ESP32-CAM.