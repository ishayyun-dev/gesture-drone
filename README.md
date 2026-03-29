# Gesture-Controlled Drone (ESP32)

An ESP32-based drone system controlled by hand gestures, using an IMU sensor worn on the wrist to translate motion into flight commands.

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────┐
│                  CONTROLLER UNIT                     │
│  ┌─────────┐   ┌──────────────┐   ┌───────────────┐ │
│  │  IMU    │──▶│  ESP32 (TX)  │──▶│  NRF24L01 /   │ │
│  │ MPU6050 │   │  Gesture     │   │  ESP-NOW      │ │
│  │ (wrist) │   │  Interpreter │   │  Radio TX     │ │
│  └─────────┘   └──────────────┘   └───────────────┘ │
└──────────────────────────────────────────────────────┘
                          │ wireless
                          ▼
┌──────────────────────────────────────────────────────┐
│                    DRONE UNIT                        │
│  ┌───────────────┐   ┌──────────────┐   ┌─────────┐ │
│  │  NRF24L01 /   │──▶│  ESP32 (RX)  │──▶│  ESCs   │ │
│  │  ESP-NOW      │   │  Flight      │   │  + BLDC │ │
│  │  Radio RX     │   │  Controller  │   │  Motors │ │
│  └───────────────┘   └──────┬───────┘   └─────────┘ │
│                             │                        │
│                      ┌──────▼───────┐                │
│                      │  IMU (drone) │                │
│                      │  MPU6050 /   │                │
│                      │  BMP280      │                │
│                      └──────────────┘                │
└──────────────────────────────────────────────────────┘
```

---

## Gesture Mapping

| Gesture              | Drone Action        |
|----------------------|---------------------|
| Tilt wrist forward   | Pitch forward       |
| Tilt wrist back      | Pitch backward      |
| Roll wrist left      | Roll left           |
| Roll wrist right     | Roll right          |
| Rotate hand CW       | Yaw right           |
| Rotate hand CCW      | Yaw left            |
| Fist (closed hand)   | Throttle up (hold)  |
| Open palm            | Throttle hold/hover |
| Sharp drop           | Emergency land      |

---

## Project Structure

```
gesture-drone/
├── controller/              # Wrist controller firmware
│   ├── src/
│   │   └── main.cpp
│   ├── include/
│   │   └── gesture.h
│   └── platformio.ini
├── drone/                   # Drone flight controller firmware
│   ├── src/
│   │   └── main.cpp
│   ├── include/
│   │   ├── flight_controller.h
│   │   └── pid.h
│   └── platformio.ini
└── README.md
```

---

## Hardware

### Controller Unit
- ESP32 DevKit v1
- MPU-6050 (6-axis IMU)
- NRF24L01+ (or ESP-NOW over built-in WiFi)
- LiPo battery (500mAh)

### Drone Unit
- ESP32 DevKit v1
- MPU-6050 or MPU-9250 (flight IMU)
- BMP280 (barometric altimeter, optional)
- 4x ESCs (30A)
- 4x BLDC brushless motors
- NRF24L01+ (or ESP-NOW)
- LiPo battery (1300–2200mAh, 3S)
- F450 / custom frame

---

## Communication Protocol

ESP-NOW (peer-to-peer, ~1ms latency) or NRF24L01+ radio.

Packet structure (8 bytes):
```
[ throttle | pitch | roll | yaw | flags | crc ]
  uint8      int8    int8   int8   uint8   uint8
```

---

## Flight Controller Design

- **PID loops** for roll, pitch, and yaw stabilization
- **Complementary filter** or Madgwick filter for IMU fusion
- **Arming sequence** required before motors spin
- **Failsafe**: motors cut on packet loss > 500ms

---

## Getting Started

1. Install [PlatformIO](https://platformio.org/) (VS Code extension recommended)
2. Flash `controller/` firmware to the wrist ESP32
3. Flash `drone/` firmware to the drone ESP32
4. Pair the two boards (MAC address pairing for ESP-NOW)
5. Arm the drone with the arming gesture, then fly

---

## Safety

- Always test indoors with propeller guards
- Never arm the drone while holding it
- Keep a killswitch gesture accessible at all times
