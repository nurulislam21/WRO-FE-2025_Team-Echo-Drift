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

* **Nurul Islam** – Mechanical Engineering
* **Tanim SK** – Programming & Software Architecture
* **Majedul Islam Nayem** – Electronics & Circuit Design

---

## Navigation Menu

* [Project Overview](#project-overview)
* [Hardware Design & Gear System](#hardware-design--gear)
* [Differential Gear Mechanism](#differential-gear-mechanism)
* [Software Architecture](#software-architecture)
* [Video Demonstration](#video)
* [Setup & Dependencies](#setup--dependencies)
* [Build, Deployment & Usage](#build-deployment--usage)
* [Project Structure](#project-structure)
* [Resources & Media](#resources--media)

---

## Project Overview

Team **Echo Drift** is participating in **WRO Future Engineers 2025** with a **state-of-the-art autonomous drift-capable vehicle**.

Our vehicle is engineered to:

* Navigate complex tracks with high precision
* Detect and avoid dynamic obstacles in real time
* Execute controlled drift maneuvers for competitive advantage
* Maintain robust, repeatable, and safe performance

The entire chassis and drivetrain are **fully 3D-printed** in **STL format**, designed in **SolidWorks**, enabling a lightweight, modular, and customizable structure.

<p align="center">
  <img src="v-photos/IMG_20250618_170008.webp" alt="Vehicle Front View" width="500"/>
</p>

---

## Hardware Design & Gear System

The hardware design balances **mechanical precision** with **reliable electronics**. Key features include:

* **Chassis:** Fully 3D-printed using STL files from SolidWorks; lightweight yet structurally strong
* **Drive System:** N20/BLDC motors controlled via TB6612FNG motor driver for smooth torque and drift capability
* **Differential Gear System:** Custom 3D-printed differential with **herringbone outer gear**, ensuring even power distribution to both wheels
* **Steering:** Ackermann steering configuration controlled via MG995 servo motor for precise drift control

<p align="center">
  <img src="v-photos/IMG_20250618_162815.jpg" alt="Vehicle Side View" width="500"/>
</p>

<p align="center">
  <img src="v-photos/Screenshot 2025-03-10 004102.png" alt="SolidWorks Design" width="500"/>
</p>

| Component                    | Model / Specification                                          | Purpose / Role                                                         |
| ---------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------------- |
| **Main Controller**          | Raspberry Pi 5 Model B (8 GB RAM)                              | High-level navigation & vision processing                              |
| **Motor Controller**         | TB6612FNG                                                      | Smooth motor control, torque modulation, regenerative braking          |
| **Drive Motors**             | N20 Motor, 12 V                                                | Propulsion & drift torque                                              |
| **Steering Servo**           | MG995 / similar                                                | Ackermann steering for precise maneuvering                             |
| **Differential Gear System** | Custom 3D-printed differential with **herringbone outer gear** | Distributes torque evenly, improves cornering efficiency and stability |
| **Sensors**                  | Ultrasonic HC-SR04                                             | Obstacle detection & orientation                                       |
| **Battery**                  | 3S LiPo, 11.1 V 2200 mAh                                       | Powers motors & controller                                             |
| **Chassis**                  | Fully 3D-printed (SolidWorks STL)                              | Lightweight, modular, robust                                           |
| **Gear System**              | Spur gears (15T : 45T ratio)                                   | Torque amplification for controlled drifting                           |
| **Wheels & Tires**           | Low-grip drift tires                                           | Enables controlled sliding for drift maneuvers                         |
| **Other**                    | Wiring harness, fuses, connectors                              | Safety and reliability                                                 |

---

## Differential Gear Mechanism

The **differential gear** is the core of our vehicle’s drift performance:

* **Herringbone Outer Gear:** Reduces axial thrust, improves torque distribution, and increases durability
* **3D-Printed Precision:** All gears are designed in SolidWorks and printed with high-resolution filament for accuracy
* **Smooth Drift Control:** Balances wheel speed during cornering for precise drift angles
* **Customizable:** Gear ratios can be adjusted to tune drift response

<p align="center">
  <img src="other/dfaf80437e9cb2198109392ebfdeb3cd.jpg" alt="3D Differential Assembly" width="500"/>
</p>

> The combination of **herringbone gears** and **precise alignment** ensures efficient power transmission and minimal slippage during high-speed drift maneuvers.

### Why We Are Using a Differential Gear

The differential gear allows each wheel to rotate at different speeds during cornering, which:

* Improves stability and control during high-speed drift
* Reduces tire wear and mechanical stress on the drivetrain
* Ensures smooth and predictable handling for precision navigation

[![Video: Why Differential Gear](https://img.youtube.com/vi/F40ZBDAG8-o/hqdefault.jpg)](https://www.youtube.com/watch?v=F40ZBDAG8-o)

---


## Software Architecture

The software is modular, maintainable, and optimized for **real-time autonomous control**:

* **Sensor Module:** Collects, filters, and processes data from ultrasonic and other sensors
* **Control Module:** PID-based motor control and servo steering management
* **Navigation Module:** Implements path planning, drift logic, and obstacle avoidance
* **Vision Module:** Optional AI/Computer Vision using OpenCV
* **Main Loop:** Integrates all modules to operate in real-time for autonomous performance

<p align="center">
  <img src="other/software_flowchart.png" alt="Software Flowchart" width="600"/>
</p>

---

## Video

* **Qualifying Round:**
  [![Video 1](https://img.youtube.com/vi/efOUVDhcxk8/hqdefault.jpg)](https://www.youtube.com/watch?v=efOUVDhcxk8)

* **Obstacle Round:**
  [![Video 2](https://img.youtube.com/vi/GM8HPATsVBk/maxresdefault.jpg)](https://www.youtube.com/watch?v=GM8HPATsVBk)

---

## Setup & Dependencies

* **OS:** Raspberry Pi OS / Ubuntu 20.04
* **Languages:** C, C++, Python
* **Compiler:** GCC
* **Libraries:** WiringPi, bcm2835, Standard C Library
* **Tools:** Make, GCC toolchain

### Install Dependencies on Raspberry Pi

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install build tools
sudo apt install build-essential git make -y

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
```

---
