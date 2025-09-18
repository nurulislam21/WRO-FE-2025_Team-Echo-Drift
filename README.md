# Team Echo Drift — WRO Future Engineers 2025

![WRO Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-orange?style=for-the-badge)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)
![C Language](https://img.shields.io/badge/Language-C,Python-blue?style=for-the-badge)
![Platform](https://img.shields.io/badge/Platform-Raspberry_Pi-red?style=for-the-badge)

---

## Our Team

<p align="center">
  <img src="t-photos/image.webp" alt="Team Echo Drift" width="500"/>
</p>

### Core Team Members & Roles
- **Nurul Islam** – Mechanical Engineering
- **Tanim SK** – Programming & Software Architecture
- **Majedul Islam Nayem** – Electronics & Circuit Design

---

## Navigation Menu

- [Project Overview](#project-overview)  
- [Hardware Design](#hardware-design--gear)  
- [Software Architecture](#software-architecture)  
- [Setup & Dependencies](#setup--dependencies)  
- [Build, Deployment & Usage](#build-deployment--usage)  
- [Project Structure](#project-structure)
- [Resources & Media](#resources--media)  
- [Team & Contributors](#team--contributors)  
---

## Project Overview

Team **Echo Drift** is our entry for **WRO Future Engineers 2025**.  
We are developing an **autonomous drift-capable vehicle** that can:

- Navigate a predefined track with precision  
- Detect and avoid obstacles dynamically  
- Perform controlled drift maneuvers  
- Ensure robustness, safety, and repeatability  

Our robot is built on a **fully 3D-printed chassis** designed in **SolidWorks**, ensuring lightweight strength and complete design flexibility.  

<p align="center">
  <img src="v-photos/IMG_20250618_170008.webp" alt="Vehicle Front View" width="500"/>
</p>

---

## Hardware Design

Our vehicle combines **mechanical precision** with **robust electronics**.  

Chassis: Fully 3D-printed in STL format using SolidWorks design, providing a lightweight, modular, and strong structure.

Drive System: BLDC motors powered via a VESC controller, enabling smooth acceleration, deceleration, and torque.

Differential Gear: 3D-printed differential with a herringbone outer gear, ensuring efficient power distribution and smooth cornering.

Steering: Ackermann steering configuration controlled by a servo motor, allowing controlled sliding and accurate path following.

<p align="center"> <img src="v-photos/IMG_20250618_162815.jpg" alt="Vehicle Front View" width="500"/> </p>
<p align="center">
  <img src="v-photos/Screenshot 2025-03-10 004102.png" alt="Designing in Solidworks" width="500"/>
</p>

| Component                    | Model / Specification                                               | Purpose / Role                                                                 |
| ---------------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| **Main Controller**          | Raspberry Pi 5 Model B ( 8 GB RAM)                                  | High-level navigation & vision processing                                      |
| **Motor Controller**         | TB6612FNG                                                           | Smooth motor control, regenerative braking                                     |
| **Drive Motors**             | N20 Motor, 12 V                                                     | Propulsion & drift torque                                                      |
| **Steering Servo**           | MG995 / similar                                                     | Ackermann steering for drift control                                           |
| **Differential Gear System** | Custom 3D-printed differential gear with **herringbone outer gear** | Power distribution between left & right wheels, improved torque and smoothness |
| **Sensors**                  | Ultrasonic HC-SR04                                                  | Obstacle detection & orientation                                               |
| **Battery**                  | 3S LiPo, 11.1 V 2200 mAh                                            | Power for motors & controller                                                  |
| **Chassis**                  | Fully 3D-printed (SolidWorks STL)                                   | Lightweight, strong, modular                                                   |
| **Gear System**              | Spur gears (15T : 45T ratio)                                        | Provides torque boost for drifting                                             |
| **Wheels & Tires**           | Low-grip drift tires                                                | Enables controlled sliding                                                     |
| **Other**                    | Wiring harness, fuses, connectors                                   | Reliability & safety                                                           |


### Design Files & Diagrams

<p align="center">
  <img src="schemes/Schematic_WRO-echo-drift_2025-09-19.png" alt="Wiring Diagram" width="500"/>
</p>

<p align="center">
  <img src="models/chassis_model.png" alt="3D Chassis Model" width="500"/>
</p>

<p align="center">
  <img src="v-photos/IMG_20250831_032659688_HDR-removebg-preview.png" alt="Gear Mechanism" width="500"/>
</p>

---

## Software Architecture

- **Sensor Module** — collects and filters sensor data  
- **Control Module** — motor control, PID/ESC tuning, steering  
- **Navigation Module** — path planning & drift logic  
- **Vision Module** — optional, OpenCV/AI-based detection  
- **Main Loop** — integrates all modules into real-time operation  

<p align="center">
  <img src="other/software_flowchart.png" alt="Software Flowchart" width="600"/>
</p>

---
## Video 

[![Video 1](https://img.youtube.com/vi/efOUVDhcxk8/maxresdefault.jpg)](https://www.youtube.com/watch?v=efOUVDhcxk8)

[![Video 2](https://img.youtube.com/vi/GM8HPATsVBk/maxresdefault.jpg)](https://www.youtube.com/watch?v=GM8HPATsVBk)

---
## Setup & Dependencies

- **OS:** Raspberry Pi OS / Ubuntu 20.04  
- **Language:** C, C++, Python
- **Compiler:** GCC
- **Libraries:** WiringPi, bcm2835, Standard C Library
- **Tools:** Make, GCC toolchain

Install dependencies on Raspberry Pi:

```bash
# Update system
sudo apt update
sudo apt upgrade

# Install build tools
sudo apt install build-essential git make

# Install WiringPi for GPIO access
git clone https://github.com/WiringPi/WiringPi
cd WiringPi
./build
cd ..

# Install bcm2835 library for hardware access
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
tar zxvf bcm2835-1.71.tar.gz
cd bcm2835-1.71
./configure
make
sudo make check
sudo make install
cd ..
