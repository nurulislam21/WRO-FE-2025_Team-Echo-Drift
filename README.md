# Team Echo Drift — WRO Future Engineers 2025

![WRO Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-orange?style=for-the-badge)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)
![Language: C, Python](https://img.shields.io/badge/Language-C,Python-blue?style=for-the-badge)
![Platform: Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry_Pi-red?style=for-the-badge)

---

## Our Team

<p align="center">
  <img src="t-photos/image.webp" alt="Team Echo Drift" width="500"/>
</p>

### Core Team Members & Roles
- **Nurul Islam Noman** – Mechanical Engineering  
- **Tanim SK** – Programming & Software Architecture  
- **Majedul Islam Nayem** – Electronics & Circuit Design  

---

## Navigation Menu
- [Project Overview](#project-overview)
- [Hardware Design & Gear System](#hardware-design--gear-system)
- [Differential Gear Mechanism](#differential-gear-mechanism)
- [Differential Gear Conversion](#differential-gear-conversion)
- [Software Architecture](#software-architecture)
- [Setup & Dependencies](#setup--dependencies)
- [Video Demonstration](#video)
- [Project Structure](#project-structure)
- [Resources & Media](#resources--media)

---

## Project Overview

Team **Echo Drift** is participating in **WRO Future Engineers 2025** with a **state-of-the-art autonomous drift-capable vehicle**.

Our vehicle is engineered to:
- Navigate complex tracks with high precision  
- Detect and avoid dynamic obstacles in real time  
- Execute controlled drift maneuvers for competitive advantage  
- Maintain robust, repeatable, and safe performance  

The entire chassis and drivetrain are **fully 3D-printed** in **STL format**, designed in **SolidWorks**, enabling a lightweight, modular, and customizable structure.

<p align="center">
  <img src="v-photos/IMG_20250618_170008.webp" alt="Vehicle Front View" width="500"/>
</p>

---

## Hardware Design & Gear System

The hardware design balances **mechanical precision** with **reliable electronics**.

**Key features:**
- **Chassis:** Fully 3D-printed (SolidWorks STL); lightweight yet strong  
- **Drive System:** N20/BLDC motors + TB6612FNG driver for smooth torque/drift  
- **Differential Gear:** Custom 3D-printed with **herringbone outer gear**  
- **Steering:** Ackermann system with MG995 servo for precise control  

<p align="left"> <img src="v-photos/IMG_20250618_162815.jpg" alt="Vehicle Side View" width="500" style="display:inline-block; vertical-align: top; margin-right:10px;"/> <img src="v-photos/Screenshot 2025-03-10 004102.png" alt="SolidWorks Design" width="500" style="display:inline-block; vertical-align: top;"/> </p>

| Component                    | Model / Specification                                          | Purpose |
| ---------------------------- | -------------------------------------------------------------- | ------- |
| **Main Controller**          | Raspberry Pi 5 (8 GB)                                          | High-level navigation & vision |
| **Motor Controller**         | TB6612FNG                                                      | Motor control & braking |
| **Drive Motors**             | N20 Motor, 12 V                                                | Propulsion & torque |
| **Steering Servo**           | MG995                                                          | Steering control |
| **Differential Gear System** | 3D-printed + **herringbone outer gear**                        | Torque distribution |
| **Sensors**                  | Ultrasonic HC-SR04                                             | Obstacle detection |
| **Battery**                  | 3S LiPo, 11.1 V 2200 mAh                                       | Power |
| **Chassis**                  | 3D-printed STL (SolidWorks)                                    | Lightweight & modular |
| **Gear System**              | Spur gears (15T : 45T)                                         | Torque amplification |
| **Wheels & Tires**           | Low-grip drift tires                                           | Controlled sliding |
| **Other**                    | Wiring, fuses, connectors                                      | Reliability |

---

## Differential Gear Mechanism

The **differential gear** is the core of our vehicle’s drift performance:
- **Herringbone Outer Gear:** Reduces axial thrust, improves torque distribution  
- **3D-Printed Precision:** Designed in SolidWorks, printed with high-resolution filament  
- **Smooth Drift Control:** Balances wheel speed during cornering  
- **Customizable:** Adjustable gear ratios for drift tuning  

<p align="center">
  <img src="other/dfaf80437e9cb2198109392ebfdeb3cd.jpg" alt="3D Differential Assembly" width="500"/>
</p>

> **Key Advantage:** Efficient power transmission with minimal slippage during drift maneuvers.

### Why a Differential Gear?
- Improves stability in high-speed drift  
- Reduces tire wear & drivetrain stress  
- Ensures smooth handling for navigation  

[![Video: Why Differential Gear](https://img.youtube.com/vi/F40ZBDAG8-o/hqdefault.jpg)](https://www.youtube.com/watch?v=F40ZBDAG8-o)

---

## Differential Gear Conversion

We began with a ready-made RC differential:  

**Model:** Differential with Bearing 284010-2252  
**Application:** WLtoys 284010 / 284161 / K989 / 284131 (1/28 RC Car)  

The stock gear lacked torque efficiency for drifting.  
✅ Solution: Converted it by designing a **3D-printed herringbone outer gear** in SolidWorks.  

<p align="center">
  <img src="other/Saa7894b3510d4cbb8e30c03ba71673cb7.jpg_960x960q75.jpg" alt="Ready-made Differential Gear" width="400"/>
  <img src="other/IMG_20250826_220431800_HDR_1.jpg" alt="Converted Differential with Herringbone Gear" width="400"/>
</p>

<p align="center"><em>Left: Ready-made RC Differential → Right: Converted with 3D-Printed Herringbone Outer Gear</em></p>

---

## Software Architecture

Our software is modular, optimized for **real-time autonomous control**.

- **Sensor Module:** Collects & processes sensor data  
- **Control Module:** PID motor & steering control  
- **Navigation Module:** Path planning, drift logic, obstacle avoidance  
- **Vision Module:** (optional) AI with OpenCV  
- **Main Loop:** Integrates all modules in real time  

<p align="center">
  <img src="other/software_flowchart.png" alt="Software Flowchart" width="600"/>
</p>

---

## Setup & Dependencies

**Environment:**
- OS: Raspberry Pi OS / Ubuntu 20.04  
- Languages: C, C++, Python  
- Compiler: GCC  
- Libraries: WiringPi, bcm2835  
- Tools: Make, GCC toolchain  

**Install Dependencies:**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install build-essential git make -y

# WiringPi
git clone https://github.com/WiringPi/WiringPi
cd WiringPi && ./build && cd ..

# bcm2835
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
tar zxvf bcm2835-1.71.tar.gz
cd bcm2835-1.71
./configure && make
sudo make check
sudo make install
cd ..
