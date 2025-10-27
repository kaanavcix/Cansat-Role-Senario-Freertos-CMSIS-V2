# Cansat-Role-Senario-Freertos-CMSIS-V2

This repository showcases **real embedded engineering** — not just code, but **architecture, design hierarchy, and process discipline**.  
It’s about how to structure systems, handle concurrency, and manage communication between independent processes.  

We’re living in the **AI age**, and writing code is no longer the challenge — building **robust, maintainable, and scalable systems** is.  
This project represents that philosophy through the development of a **FreeRTOS-based telemetry and control system**, 
inspired by **CANSAT role scenarios** — where real-time sensor acquisition, communication, and control come together in a unified embedded platform.

All shared code is for **educational and reference purposes** only.  
Please do not copy or reuse without understanding the context.

Best Wishes


# 🛰️ RTOS-Based Telemetry and Control System

A **FreeRTOS-based embedded system** designed for real-time telemetry, sensor fusion, and command execution using **STM32 microcontrollers**.  
It integrates multiple sensors and a **LoRa E22 communication module** for reliable wireless telemetry and control.

---

## 🧩 Overview

This project demonstrates a **modular real-time architecture**, using **tasks, queues, timers, and mutexes** for concurrent sensor data collection, telemetry transmission, and telecommand handling.

It ensures:
- Deterministic **1 Hz LoRa transmission**
- Thread-safe sensor access
- Stable synchronization across tasks and interrupts
- Structured communication between software layers

---

## ⚙️ Key Features
- LoRa E22 wireless telemetry uplink  
- GPS (NEO-6M) position and velocity tracking  
- BMP280 pressure & temperature sensing  
- MPU6050 IMU for orientation (pitch, roll, yaw)  
- Servo-based mechanical separation  
- ADC-based battery voltage monitoring  
- Mutex-protected data sharing  
- Event flag synchronization (ISR ↔ Tasks)  
- Layered, modular software structure  

---

## 🧱 Software Architecture

| Layer | Description |
|:--|:--|
| **Hardware Abstraction** | STM32 HAL Drivers (ADC, I²C, UART, SPI, TIM) |
| **Sensor Layer** | BMP280, MPU6050, GPS modules |
| **Control Layer** | Servo & button event management |
| **Communication Layer** | LoRa E22 telemetry and telecommand parser |
| **RTOS Middleware** | Tasks, Queues, Mutexes, Timers, Event Flags |
| **Application Layer** | High-level telemetry logic & state handling |

---

## 🧠 RTOS Resources

| Resource | Count | Description |
|:--|:--:|:--|
| **Tasks** | 8 | LoRa, GPS, Sensor, Telecommand, and Control threads |
| **Queues** | 4 | UART RX/TX, GPS, Telecommand |
| **Mutexes** | 2 | Shared resource protection |
| **Timers** | 3 | LoRa Tx, Velocity Calc, Separation |
| **Event Flags** | 10+ | Synchronization signals between ISR and tasks |

---

## ⚡ Technical Details

| Module | Function |
|:--|:--|
| `LoraWriteProcess()` | Builds binary + ASCII telemetry packets |
| `LoraWriteTimerCallBack()` | Sends queued telemetry every 1 second |
| `LoraReadProcess()` | Parses incoming LoRa packets & telecommands |
| `TelecommandTask()` | Executes received commands asynchronously |
| `GPSProcessCallback()` | Updates GPS position & velocity |
| `ServoTask()` | Controls servo motors for separation |
| `CalculateVelocityCallback()` | Computes vertical velocity from altitude changes |
| `BMP280Process()` | Reads and filters pressure/temperature data |
| `MPU6050Process()` | Updates orientation and acceleration data |

---

## 🧰 Hardware Setup

| Component | Model | Interface |
|:--|:--|:--|
| MCU | STM32F446RE | ARM Cortex-M4 @ 180 MHz |
| LoRa Module | Ebyte E22 | UART |
| GPS | L86-M33 | UART |
| IMU | MPU6050 | I²C |
| Pressure Sensor | BMP280 | I²C |
| Servo Motors | SG90 / MG90 | PWM (TIM2, TIM3) |
| Power Source | Li-ion Battery Pack | ADC voltage monitoring |
| Debug Interface | ST-Link V2 | SWD |

---

## 🖼️ Hardware & PCB Design

### 🧩 PCB Layout  
```markdown
![PCB Layout](Docs/pcb_layout.png)
