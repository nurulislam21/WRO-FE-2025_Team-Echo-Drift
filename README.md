# Team Echo Drift — WRO Future Engineers 2025
<img src="https://github.com/majednaeem/WRO/blob/main/Assets/wro2.gif" alt="About Me GIF" width="1021" height="300">


![WRO Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-orange?style=for-the-badge)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)
![Language: C, Python](https://img.shields.io/badge/Language-C,Python-blue?style=for-the-badge)
![Platform: Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry_Pi-red?style=for-the-badge)

---


Welcome to the official GitHub repository of **Team Echo Drift**. This repository documents the journey of **Team Echo Drift** at the **WRO Bangladesh National 2025 – Future Engineers category**.Here you’ll find the **complete documentation of our robot** — an innovation imagined, designed, and brought to life by three passionate students pushing the limits of creativity and engineering.  

# Table of Contents  

---

### Ⅰ. [About the Team](#about-the-team)  
### Ⅱ. [Competition Overview](#competition-overview)  
### Ⅲ. [Our Robot](#our-robot)  
### Ⅳ. [System Architecture](#system-architecture)  
### Ⅴ. [Hardware Components](#hardware-components)  
### Ⅵ. [Software & Algorithms](#software--algorithms)  
### Ⅶ. [CAD & Mechanical Design](#cad--mechanical-design)  
### Ⅷ. [Electronics & Wiring](#electronics--wiring)  
### Ⅸ. [Testing & Validation](#testing--validation)  
### Ⅹ. [Results & Performance](#results--performance)  
### Ⅺ. [Future Improvements](#future-improvements)  
### Ⅻ. [How to Run the Code](#how-to-run-the-code)  
### XIII. [Contributors](#contributors)  
### XIV. [License](#license)  

---
## About the Team  

We are a team of three young innovators, coming from different corners of Bangladesh. At first, each of us participated individually in many competitions across the country. Through those journeys, we eventually met one another, shared our dreams, and realised that together we could achieve something even greater. That was the beginning of our team.

From the very start, we have built a strong bond based on trust, hard work, and a shared passion for robotics. Step by step, we worked side by side and successfully won several national robotics competitions, which made us more confident about our vision.

A few months ago, we set ourselves a bigger goal—to represent Bangladesh in the World Robot Olympiad. Since then, we have been working with full dedication and determination. Our dream is simple but powerful: to raise the flag of Bangladesh high on the international stage and prove that with passion, teamwork, and belief, anything is possible.
 
<p align="center">
  <img src="t-photos/GP_WRO.jpg" alt="Team Echo Drift" width="700"/>
</p>

---

## Meet the Team  

### *Nurul Islam Noman – Mechanical Design*

Specializing in Mechanical Design with strong experience in Robotics and Embedded Systems. Extensive work in research and development has built expertise where precision, innovation, and reliability are key.

Skills span Robotics, Embedded Systems, and Mechanical Design, enabling a holistic approach to complex technical challenges. With a focus on Mechanical Design, the goal is to create solutions that seamlessly integrate hardware, software, and system reliability.
<p align="center">
  <img src="t-photos/Noman.jpg" alt="Nurul Islam Noman" width="700" height="900"/>
</p>

### *Tanim SK – Programming & Software Architecture* 

A versatile programmer with expertise in Python, JavaScript, HTML, CSS, and C++, and over 5 years of experience in software development. Skilled in building web applications using frameworks like Django, Flask, and FastAPI for the backend, and ReactJS for the frontend, with PostgreSQL for databases and Redis for caching. Experienced in deploying web applications on AWS and similar platforms with CI/CD integration.

Also proficient in working with Raspberry Pi and microcontrollers, with experience in home automation, WebSocket programming, RF communication, and integrating TinyML for lightweight AI projects.
<p align="center">
  <img src="t-photos/Tanim.jpeg" alt="Tanim SK" width="700" height="900"/>
</p>


### *Majedul Islam Naeem – Electronics & Circuit Design* 

Majedul Islam Naeem is a passionate Robotics and Embedded Systems innovator with strong expertise in Electronics & Circuit Design. He excels at transforming ideas into practical, intelligent solutions, designing efficient circuits, and integrating electronics seamlessly with robotics projects. 

Driven by curiosity and creativity, Naeem thrives in collaborative environments and is committed to pushing the boundaries of technology while inspiring others to innovate.
<p align="center">
  <img src="t-photos/Naeem.jpg" alt="Majedul Islam Naeem" width="700" height="900"/>
</p>
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

## Competition Overview  

WRO **Future Engineers** challenges teams to design autonomous electric vehicles (EVs) that can:  

- Navigate complex tracks  
- Handle lane following, obstacle avoidance, parking, and overtaking  
- Optimize performance for efficiency and reliability  

>  **For 2025, the challenge emphasizes real-world autonomous driving concepts aligned with SDG 11 — Sustainable Cities and Communities.**


---

##  Competition Rounds 

The WRO Future Engineers 2025 competition is divided into **two progressive rounds**. Each round adds new layers of complexity and pushes teams to demonstrate engineering excellence.  


| Round | Description | Key Tasks | Track Layout |
|:------|:-----------:|:----------|:-------------|
| **Round 1 – Endurance Lap** | Focuses on **reliability and lap consistency**. The robot must complete **3 laps** without error while maintaining smooth navigation. Judges emphasize **stability, accuracy, and precise lane following**. | - 🎯 Focus on **stability, accuracy, and smooth lane following**  <br> - 🏁 Complete **3 full laps** without error  <br> ⏱️ Time-based scoring adds pressure   | <img src="https://github.com/majednaeem/WRO/blob/main/Assets/Round%201.png" width="400" align="top" align="right">|
| **Round 2 – Smart Navigation + Parking** | Introduces **real-time intelligence**. After completing laps, the robot must avoid **color-coded obstacles** and then park in a **designated zone**.| - 🟩 **Green obstacle → move left**  <br> - 🟥 **Red obstacle → move right**  <br> - 🏎️ Maintain speed while **avoiding collisions**   <br> -- 🅿️ Finish with **precision parking** inside a marked box  |<img src="https://github.com/majednaeem/WRO/blob/main/Assets/Round%202.png" width="400" align="top" align="left">   |


> **Both rounds are aligned with the [official WRO 2025 Future Engineers Rules (PDF)](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).**



---

## Our Robot
### Robot Overview  
**Echo Drift** brings a **next-generation autonomous EV** to WRO 2025, built for speed, accuracy, and adaptability.
  
- 🚗 Precise track navigation  
- ⚡ Real-time obstacle avoidance  
- 🎯 Controlled drift maneuvers  
- 🔒 Safe & reliable performance  

🛠️ 3D-printed chassis | 🔌 Smart sensors & drivers | 🧠 Vision & PID control

---

## 📊 Robot Specifications  

| Parameter | Value |
|-----------|-------|
| **Dimensions** | 280 mm (L) × 190 mm (W) × 160 mm (H) |
| **Weight** | ~ 1.4 kg |
| **Chassis** | Modular 3D-printed PLA with reinforced mounts |
| **Motors** | N20  |
| **Motor Driver** | TB6612FNG dual-channel |
| **Steering** | MG995 Servo |
| **Sensors** | HC-SR04 Ultrasonic × 2, IMU, Optional Camera |
| **Controller** | Raspberry Pi 5 (8GB) |
| **Battery** | 3S 11.1V Li-Po, 2200mAh |
| **Special Feature** | Custom herringbone gear differential for drift |
---

## 📸 Vehicle Photos  

| Front | Right | Back |
|-------|-------|------|
| <img src="v-photos/front.jpg" width="250"> | <img src="v-photos/right.jpg" width="250"> | <img src="v-photos/back.jpg" width="250"> |

| Left | Top | Bottom |
|------|-----|--------|
| <img src="v-photos/leftside.jpg" width="250"> | <img src="v-photos/top.jpg" width="250"> | <img src="v-photos/bottom.jpg" width="250"> |

---

## 🎥 Videos of Our Robot  

[![Test Run 1 – Track Navigation](https://img.youtube.com/vi/efOUVDhcxk8/0.jpg)](https://www.youtube.com/watch?v=efOUVDhcxk8)  
[![Test Run 2 – Obstacle Avoidance](https://img.youtube.com/vi/GM8HPATsVBk/0.jpg)](https://www.youtube.com/watch?v=GM8HPATsVBk)  
[![Test Run 3 – Precision Parking](https://img.youtube.com/vi/ZZZZZZZZ/0.jpg)](https://youtu.be/ZZZZZZZZ)  
 

👉 *(Videos will be linked here once uploaded to YouTube)*  

---

## System Architecture  

The **Echo Drift Autonomous EV** is designed with a **layered and modular architecture** to ensure clarity, testability, and compliance with WRO 2025 Future Engineers standards.  

## System Flow Diagram

<img src="https://github.com/majednaeem/WRO/blob/main/Assets/System_Architecture_Flow_Infographic.png">  
---
## Layered Architecture Overview 

| **Layer**                | **Key Components**                                                                                                                                                    | **Role & Engineering Considerations**                                                                                                                                                                                                                                                           |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Hardware**             | - PLA + Aluminum Hybrid Chassis  <br> - 2× DC Geared Motors (12V, 300RPM, 1.2Nm)  <br> - High-Torque Servo (15kg·cm)  <br> - 65mm Rubberized Wheels                   | - Rigid but lightweight frame ensures stability and durability. <br> - Motor torque chosen with ~30% safety margin for acceleration under load. <br> - Servo provides precise steering with quick response. <br> - Wheel diameter selected for balance between speed and traction.             |
| **Power**                | - 3S LiPo (11.1V, 2200mAh)  <br> - Power Distribution Board (12V, 5V, 3.3V outputs)  <br> - Fuse + XT60 Connectors                                                    | - LiPo selected for high discharge rate, lightweight, and compact size. <br> - Separate regulated lines prevent voltage drop issues. <br> - Fuse + XT60 provide short-circuit and overload safety.                                                                                              |
| **Perception**           | - Camera (USB/PiCam)  <br> - 2× Ultrasonic Sensors (front-left & front-right)  <br> - IMU (MPU6050)  <br> - Wheel Encoders                                            | - Camera handles **lane detection and vision-based markers**. <br> - Ultrasonic ensures reliable short-range obstacle sensing. <br> - IMU improves orientation and stability on turns. <br> - Encoders provide real-time speed & distance for closed-loop control.                              |
| **Control & Processing** | - Raspberry Pi 4 (Python + OpenCV)  <br> - Arduino Mega (C++)  <br> - UART Serial Link                                                                                | - Pi processes camera input & makes high-level decisions. <br> - Arduino handles **PWM signals, interrupts, and motor control** with real-time precision. <br> - UART ensures fast, low-latency communication between subsystems.                                                               |
| **Decision**             | - OpenCV Line Detection  <br> - Sensor Fusion (Camera + Ultrasonic)  <br> - PID Steering Control  <br> - Encoder-based Speed Feedback  <br> - Emergency Stop Failsafe | - Lane tracking optimized with **real-time vision algorithms**. <br> - Sensor fusion improves obstacle avoidance accuracy. <br> - PID ensures smooth steering corrections. <br> - Encoders maintain consistent velocity. <br> - Safety protocol: robot halts when conflicting data is detected. |
| **Actuation**            | - H-Bridge Motor Driver (e.g., BTS7960)  <br> - PWM Servo Driver                                                                                                      | - H-Bridge supplies bidirectional control for drive motors. <br> - Servo driver ensures precise angle control. <br> - Final output: **smooth differential drive with adaptive steering**.                                                                                                       |
---
## Threads & Rates
| Task         |       Rate | Notes                              |
| ------------ | ---------: | ---------------------------------- |
| Sensor read  |  50–100 Hz | median filter, timeouts            |
| Vision       |  20–30 FPS | HSV masks for lane & pillar colors |
| Fusion       |      50 Hz | complementary/Kalman (optional)    |
| Navigation   |   20–50 Hz | lane-keeping, color rule, parking  |
| Control      | 100–200 Hz | PID for speed + steering           |
| Actuator I/O | 100–200 Hz | PWM/UART                           |
| Logger       |      10 Hz | CSV + HUD overlay                  |
| Watchdog     |       5 Hz | trips SAFE\_STOP                   |

---


## Hardware & Mechanical Part
Our robot is designed with a balanced mix of mechanical, electrical, and electronic components, ensuring rule compliance (size, weight, drive system) and performance (precision, stability, obstacle avoidance).

### List of components

| Component | Image | Qty | Notes |
|-----------|-------|-----|-------|
| **Arduino Nano A000005** | <img src="images/arduino-nano.jpg" width="120"> | x1 | Microcontroller for low-level control |
| **Raspberry Pi 5** | <img src="images/rpi5.jpg" width="120"> | x1 | Main controller for vision & navigation |
| **TB6612FNG Motor Driver** | <img src="images/tb6612fng.jpg" width="120"> | x1 | Controls 2 DC motors |
| **L298N Motor Driver (Backup)** | <img src="images/l298n.jpg" width="120"> | x1 | Legacy driver for testing |
| **DC Gear Motors (12V)** | <img src="images/dc-motor.jpg" width="120"> | x2 | Linked to rear axle |
| **MG995 Servo** | <img src="images/mg995.jpg" width="120"> | x1 | Ackermann steering |
| **HC-SR04 Ultrasonic Sensor** | <img src="images/hcsr04.jpg" width="120"> | x3 | Obstacle detection (front + side) |
| **IMU MPU6050 Gyroscope** | <img src="images/mpu6050.jpg" width="120"> | x1 | Orientation & stability |
| **Camera Module (CSI/USB)** | <img src="images/camera.jpg" width="120"> | x1 | Lane & obstacle detection |
| **Neo-6M GPS Module** | <img src="images/gps.jpg" width="120"> | x1 | Outdoor navigation (optional) |
| **Li-Po Battery (3S, 11.1V, 2200mAh)** | <img src="images/lipo.jpg" width="120"> | x1 | Main power source |
| **LM2596 Step-down Converter** | <img src="images/lm2596.jpg" width="120"> | x2 | Voltage regulation (5V & 3.3V) |
| **Prototype PCB Board** | <img src="images/pcb-proto.jpg" width="120"> | x2 | Custom wiring layout |
| **Breadboard (Mini)** | <img src="images/breadboard.jpg" width="120"> | x1 | Quick prototyping |
| **Wheels with Rubber Tires** | <img src="images/wheel.jpg" width="120"> | x4 | High grip on track |
| **3D Printed Chassis Parts** | <img src="images/chassis.jpg" width="120"> | - | Lightweight & modular |
| **Power Switch** | <img src="images/switch.jpg" width="120"> | x1 | Main power control |
| **Start Button** | <img src="images/button.jpg" width="120"> | x1 | Competition start |
| **Emergency Stop (E-Stop) Switch** | <img src="images/estop.jpg" width="120"> | x1 | Safety cutoff |
| **Jumper Wires (Male-Female)** | <img src="images/jumpers.jpg" width="120"> | 40+ | Connections |
| **Heat Shrink Tubes** | <img src="images/heatshrink.jpg" width="120"> | Pack | Wire insulation |
| **Screws & Standoffs** | <img src="images/screws.jpg" width="120"> | - | Mounting hardware |
| **Cooling Fan (5V)** | <img src="images/fan.jpg" width="120"> | x1 | Keeps Pi cool |
| **USB Power Bank (Backup)** | <img src="images/powerbank.jpg" width="120"> | x1 | Debugging power |
| **LED Indicators (Red/Green)** | <img src="images/led.jpg" width="120"> | x2 | Status display |


### Mobility 

## Hardware Design & Gear System

The hardware design balances **mechanical precision** with **reliable electronics**.

**Key features:**
- **Chassis:** Fully 3D-printed (SolidWorks STL); lightweight yet strong  
- **Drive System:** N20/BLDC motors + TB6612FNG driver for smooth torque/drift  
- **Differential Gear:** Custom 3D-printed with **herringbone outer gear**  
- **Steering:** Ackermann system with MG995 servo for precise control  

<p align="left"> <img src="v-photos/IMG_20250618_162815.jpg" alt="Vehicle Side View" width="500" style="display:inline-block; vertical-align: top; margin-right:10px;"/> <img src="v-photos/Screenshot 2025-03-10 004102.png" alt="SolidWorks Design" width="500" style="display:inline-block; vertical-align: top;"/> </p>

| Component                    | Model / Specification                                          | Purpose |
| ---------------------------- | -------------------------------------------------------------- | ------- |
| **Main Controller**          | Raspberry Pi 5 (8 GB)                                          | High-level navigation & vision |
| **Motor Controller**         | TB6612FNG                                                      | Motor control & braking |
| **Drive Motors**             | N20 Motor, 12 V                                                | Propulsion & torque |
| **Steering Servo**           | MG995                                                          | Steering control |
| **Differential Gear System** | 3D-printed + **herringbone outer gear**                        | Torque distribution |
| **Sensors**                  | Ultrasonic HC-SR04                                             | Obstacle detection |
| **Battery**                  | 3S LiPo, 11.1 V 2200 mAh                                       | Power |
| **Chassis**                  | 3D-printed STL (SolidWorks)                                    | Lightweight & modular |
| **Gear System**              | Spur gears (15T : 45T)                                         | Torque amplification |
| **Wheels & Tires**           | Low-grip drift tires                                           | Controlled sliding |
| **Other**                    | Wiring, fuses, connectors                                      | Reliability |

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
