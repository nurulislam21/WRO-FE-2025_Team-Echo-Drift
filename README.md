# Team Echo Drift — WRO Future Engineers 2025

## Navigation Menu

- [Project Overview](#project-overview)  
- [Hardware Design & Gear](#hardware-design--gear)  
- [Software Architecture](#software-architecture)  
- [Setup & Dependencies](#setup--dependencies)  
- [Build, Deployment & Usage](#build-deployment--usage)  
- [Resources & Media](#resources--media)  
- [Team & Contributors](#team--contributors)  
- [License](#license)  

---

## Project Overview

Team Echo Drift is our entry for **WRO Future Engineers 2025**. The goal is to build an autonomous drift-capable vehicle that can:

- Navigate a predefined track with precision  
- Handle obstacles and adjust its path dynamically  
- Perform drift maneuvers safely and reproducibly  
- Be robust and reliable under competition conditions  

This repository contains everything: software, hardware design files, schematics, models, photos, video, etc.

---

## Hardware Design & Gear

Here are the key components and design details of our vehicle:

| Component | Model / Specification | Purpose / Role |
|-----------|------------------------|-----------------|
| **Microcontroller / Main Processing Unit** | *[e.g. Raspberry Pi 4 Model B]* — Quad-core, 4 GB RAM | High-level path planning, image processing, communication |
| **Motor Controller / ESC** | *[e.g. VESC 6-Plus]* — Supports regenerative braking, configurable limits | Control BLDC / DC motors safely and responsively |
| **Drive Motors** | *[e.g. HobbyWing 56123, 12 kV]* or (DC/Stepper depending) | Provide driving force, torque for drift |
| **Drift / Steering Mechanism** | *[e.g. servo motor + Ackermann steering setup]* | For steering control and stability during drifts |
| **Sensors** | *Ultrasonic sensors (HC-SR04), IMU (MPU-6050), LiDAR / ToF as needed* | Obstacle detection, orientation, speed feedback |
| **Power Supply / Battery** | *[e.g. 3S LiPo, 11.1 V 5000 mAh]* + DC-DC converters | Provide the needed voltages for motors + electronics |
| **Chassis & Gear** | – Chassis material: *aluminum / acrylic / carbon fiber* <br> – Gears: *spur / bevel / timing belts / pulleys* depending on drive setup <br> – Wheels / Tires: *drift wheels / low-grip for controlled slide* | Mechanical structure and transmission of power; drift-friendly setup |
| **Other Accessories** | Wiring, connectors, safety features (fuses / thermal sensors), structural supports | Reliability and safety during operation |

### Design Files & Diagrams

- Schematics & wiring diagrams are in `schemes/`  
- 3D / CAD / model files for chassis, gear mounts etc. are in `models/`  
- Gear ratio calculations / transmission layouts should be documented here (you may include spec sheets or spreadsheets)  

---

## Software Architecture

We structure the software as modular components:

- **Control Module** — interfaces with motors, ESC / motor driver; handles low-level control, safety limits  
- **Sensor Module** — reads data from IMU, ultrasonic / LiDAR sensors; filters and pre-processes data  
- **Navigation Module** — path planning, drift maneuvers, obstacle avoidance logic  
- **Perception Module** (if applicable) — image processing or vision-based inputs  
- **Communication Module** — logging, command input, telemetry (if remote or for debugging)  
- **Main Loop / Integration** — ties all modules together into real-time operation  

---

## Setup & Dependencies

Before building/running the project, ensure you have:

- OS: *Ubuntu 20.04 / Raspberry Pi OS / whichever is used*  
- Programming language: *Python ≥ 3.x* (if using Python), or specify C++ version etc.  
- Libraries / Packages: e.g., `numpy`, `opencv`, `RPi.GPIO`, `scipy`, etc.  
- Tools: e.g. build tools for microcontrollers (gcc, make, etc.), firmware upload tools  
- CAD software to view / modify models (SolidWorks / Fusion 360 / Onshape etc.)  

---

## Build, Deployment & Usage

1. **Cloning the Repository**  
   ```bash
   git clone https://github.com/nurulislam21/WRO-FE-2025_Team-Echo-Drift.git
   cd WRO-FE-2025_Team-Echo-Drift
