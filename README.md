# Team Echo Drift — WRO Future Engineers 2025
<img src="https://github.com/majednaeem/WRO/blob/main/Assets/github%20cover.gif" alt="About Me GIF" width="1021" height="300">


![WRO Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-orange?style=for-the-badge)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)
![Language: C, Python](https://img.shields.io/badge/Language-C,Python-blue?style=for-the-badge)
![Platform: Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry_Pi-red?style=for-the-badge)

---


**Welcome** to the official GitHub repository of **Team Echo Drift**. This repository documents the journey of **Team Echo Drift** at the **World Robot Olympiad 2025 – Future Engineers category**.Here you’ll find the **complete documentation of our robot** — an innovation imagined, designed, and brought to life by three passionate students pushing the limits of creativity and engineering.  

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
  <img src="t-photos/WROGP.png" alt="Team Echo Drift" width="700"/>
</p>

---

## Meet the Team  

<!-- Nurul Islam Noman -->
<table>
  <tr>
    <td>
<h2> Nurul Islam Noman – Mechanical Engineering </h2>

Specializing in Mechanical Design with strong experience in Robotics and Embedded Systems. Extensive work in research and development has built expertise where precision, innovation, and reliability are key.

Skills span Robotics, Embedded Systems, and Mechanical Design, enabling a holistic approach to complex technical challenges. With a focus on Mechanical Design, the goal is to create solutions that seamlessly integrate hardware, software, and system reliability.
    </td>
    <td>
      <img src="t-photos/noman.jpg" alt="Nurul Islam Noman" width="400" style="object-fit: cover;"/>
    </td>
  </tr>
</table>

<!-- Tanim SK -->
<table>
  <tr>
    <td>
<h2> Tanim – Programming & Software Architecture </h2>

A versatile programmer with expertise in Python, C++, and JavaScript, specializing in robotics and embedded systems. Skilled in integrating Raspberry Pi and microcontrollers for automation and control. Proficient in image processing and computer vision with OpenCV, enabling advanced robotic perception. Experienced in real-time communication, lightweight AI models, and deploying reliable robotics applications.
    </td>
    <td>
      <img src="t-photos/Tanim.jpeg" alt="Tanim SK" width="700" style="object-fit: cover;"/>
    </td>
  </tr>
</table>

<!-- Majedul Islam Naeem -->
<table>
  <tr>
    <td>
<h2> Majedul Islam Naeem – Electronics & Circuit Design </h2>

Majedul Islam Naeem is a passionate Robotics and Embedded Systems innovator with strong expertise in Electronics & Circuit Design. He excels at transforming ideas into practical, intelligent solutions, designing efficient circuits, and integrating electronics seamlessly with robotics projects.

Driven by curiosity and creativity, Naeem thrives in collaborative environments and is committed to pushing the boundaries of technology while inspiring others to innovate.
    </td>
    <td>
      <img src="t-photos/Naeem.jpg" alt="Majedul Islam Naeem" width="400" style="object-fit: cover;"/>
    </td>
  </tr>
</table>

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

<h2>  Competition Rounds </h2>

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
| *Dimensions* | 17 cm (L) × 11 cm (W) × 18 cm (H) |
| *Weight* | ~ 0.7 kg |
| *Chassis* | Modular 3D-printed PLA with reinforced mounts |
| *Motors* | N20 with encoder  |
| *Motor Driver* | TB6612FNG dual-channel |
| *Steering* | MG995 Servo |
| *Sensors* | HC-SR04 Ultrasonic × 2, IMU, Camera |
| *Controller* | Raspberry Pi 5 (8GB) |
| *Battery* | 3S 11.1V Li-Po, 2200mAh |
| *Special Feature* | Custom herringbone gear differential for drift |

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
[![Test Run 3 – Precision Parking](https://img.youtube.com/vi/KPs8n2rFoqplYith/0.jpg)](https://youtu.be/XEHfCzGVoek?si=KPs8n2rFoqplYith)
 

👉 *(Videos will be linked here once uploaded to YouTube)*  

---

## System Architecture  

The **Echo Drift Autonomous EV** is designed with a **layered and modular architecture** to ensure clarity, testability, and compliance with WRO 2025 Future Engineers standards.  

## System Flow Diagram

<img src="https://github.com/majednaeem/WRO/blob/main/Assets/System_Architecture_Flow_Infographic.png">  
---
## Layered Architecture Overview 

| *Layer*                | *Key Components*                                                                                                                                                    | *Role & Engineering Considerations*                                                                                                                                                                                                                                                           |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| *Hardware*             | - PLA + Aluminum Hybrid Chassis  <br> - 1× DC Geared Motors (12V, 600RPM, 1.2Nm)  <br> - High-Torque Servo (15kg·cm)  <br> - 65mm Rubberized Wheels                   | - Rigid but lightweight frame ensures stability and durability. <br> - Motor torque chosen with ~30% safety margin for acceleration under load. <br> - Servo provides precise steering with quick response. <br> - Wheel diameter selected for a balance between speed and traction.             |
| *Power*                | - 3S LiPo (11.1V, 2200mAh)  <br> - Power Distribution Board (12V, 5V outputs)  <br> - XT60 Connectors                                                    | - LiPo selected for high discharge rate, lightweight, and compact size. <br> - Separate regulated lines prevent voltage drop issues. <br> - XT60 provides short-circuit and overload safety.                                                                                              |
| *Perception*           | - Camera   <br> - 2× Ultrasonic Sensors (front)   <br> - Wheel                                             | - Camera handles *lane detection and vision-based markers*. <br> - Ultrasonic ensures reliable short-range obstacle sensing. <br> - IMU improves orientation and stability on turns. <br> - Encoders provide real-time speed & distance for closed-loop control.                              |
| *Control & Processing* | - Raspberry Pi 5 (Python + OpenCV)  <br> - Arduino Nano (C++)  <br> - UART Serial Link                                                                                | - Pi processes camera input & makes high-level decisions. <br> - Arduino handles *PWM signals, interrupts, and motor control* with real-time precision. <br> - UART ensures fast, low-latency communication between subsystems.                                                               |
| *Decision*             | - OpenCV Line Detection  <br> - Sensor Fusion (Camera + Ultrasonic)  <br> - PID Steering Control  <br> - Encoder-based Speed Feedback  <br> - Emergency Stop Failsafe | - Lane tracking optimized with *real-time vision algorithms*. <br> - Sensor fusion improves obstacle avoidance accuracy. <br> - PID ensures smooth steering corrections. <br> - Encoders maintain consistent velocity. <br> - Safety protocol: robot halts when conflicting data is detected. |
| *Actuation*            | -  Motor Driver   <br> - PWM Servo Driver                                                                                                      | - H-Bridge supplies bidirectional control for drive motors. <br> - Servo driver ensures precise angle control. <br> - Final output: *smooth differential drive with adaptive steering*.                                                                                                       |
---

## Threads & Rates
| Task         |       Rate | Notes                              |
| ------------ | ---------: | ---------------------------------- |
| Sensor read  |  50–100 Hz | median filter, timeouts            |
| Vision       |  20–30 FPS | HSV masks for lane & pillar colors |
| Fusion       |      50 Hz | complementary/Kalman (optional)    |
| Navigation   |   20–50 Hz | lane-keeping, color rule, parking  |
| Control      | 100–200 Hz | PID for speed + steering           |

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


## Mobility Management
Mobility management defines how the robot moves, steers, and maintains stability. For Echo Drift, we adopted a rear-wheel drive + front servo steering configuration. This gives better maneuverability, efficient obstacle avoidance, and realistic car-like motion.

### Motor Selection 

#### Drive Motors
- **Type:** N20 DC Geared Motors with Encoders  
- **Specs:** 12V, ~600 RPM, ~1.2 Nm torque  
- **Reason:** Higher speed compared to 300 RPM version. Encoders provide feedback for closed-loop control (speed & distance). Torque margin (~30% extra) ensures reliable acceleration.

#### Steering Servo
- **Type:** High-torque Servo (~15 kg·cm)  
- **Reason:** Quick, precise steering even under load.

#### Engineering Principle
Torque Required = Wheel Radius × Force (Load + Friction)

-Our calculation showed motor Required torque ≈ **0.9 Nm**  
- Selected N20 motors rated ≈ **1.2 Nm** → safe margin  
- Speed doubles from ~1.0 m/s (300 RPM) to ~2.0 m/s (600 RPM) on 65 mm wheels.

📂 Encoders connected to Arduino for speed control and stability.

| Drive Motors (N20 with Encoders) | Steering Servo Assembly |
|----------------------------------|--------------------------|
| [N20 Motors] <img src="https://github.com/majednaeem/WRO/blob/main/Assets/n20.jpg"> | [Steering Servo] <img src="https://github.com/majednaeem/WRO/blob/main/Assets/n20.jpg"> |


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

## Software Architecture

Our software is modular, optimized for **real-time autonomous control**.

- **Sensor Module:** Collects & processes sensor data  
- **Control Module:** PID motor & steering control  
- **Navigation Module:** Path planning, drift logic, obstacle avoidance  
- **Vision Module:** (optional) AI with OpenCV  
- **Main Loop:** Integrates all modules in real time  

<!-- <p align="center">
  <img src="other/software_flowchart.png" alt="Software Flowchart" width="600"/>
</p> -->

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

# clone our repository
git clone https://github.com/nurulislam21/WRO-FE-2025_Team-Echo-Drift/
cd WRO-FE-2025_Team-Echo-Drift/src
# install the dependencies
pip install -r requirements.txt
# run the program
python raspberrypi/main.py --debug

# for starting the program on startup, register it as a service or a crontab

```

We have divided the whole into 8 segments, each segment runs a seperate image processing thread.

<img width="500" height="736" alt="image" src="https://github.com/user-attachments/assets/ffdb6ecb-f719-429e-8884-5655da6b346c" />


The frame regions are assigned here:


```py
# Region of Interest coordinates
LEFT_REGION = [20, 220, 270, 280]
RIGHT_REGION = [370, 220, 620, 280]
LAP_REGION = [225, 295, 415, 350]
OBS_REGION = [85, 140, 555, 320]
REVERSE_REGION = [233, 300, 407, 320]
FRONT_WALL_REGION = [300, 195, 340, 215]
PARKING_LOT_REGION = [0, 185, CAM_WIDTH, 400]
DANGER_ZONE_POINTS = [
    {
        "x1": 302,
        "y1": OBS_REGION[1],
        "x2": 205,
        "y2": OBS_REGION[3],
    },
    {
        "x1": 350,
        "y1": OBS_REGION[1],
        "x2": 435,
        "y2": OBS_REGION[3],
    },
]
```

<br>


We are using LAB colors for detecting a object based on a certain color range, the color ranges are assigned as numpy arrays in [main.py](https://github.com/nurulislam21/WRO-FE-2025_Team-Echo-Drift/blob/main/src/raspberrypi/main.py):

```py
# Color ranges
LOWER_BLACK = np.array([0, 108, 90])
UPPER_BLACK = np.array([89, 148, 163])

LOWER_ORANGE = np.array([135, 125, 83])
UPPER_ORANGE = np.array([195, 165, 123])

LOWER_BLUE = np.array([93, 144, 164])
UPPER_BLUE = np.array([153, 184, 204])

# obstacle color ranges HSV
LOWER_RED = np.array([160, 100, 200])
UPPER_RED = np.array([180, 255, 255])

# HSV
LOWER_GREEN = np.array([35, 100, 50])
UPPER_GREEN = np.array([85, 255, 255])

# parking color ranges
LOWER_MAGENTA = np.array([100, 81, 105])
UPPER_MAGENTA = np.array([170, 121, 145])
```

<br>

Then we made a class called `ContourWorkers.py` in [contour_workers.py](https://github.com/nurulislam21/WRO-FE-2025_Team-Echo-Drift/blob/main/src/raspberrypi/contour_workers.py), this class is used to get contours based on the passed color ranges. ContourWorker starts 8 seperate thread for 8 regions, and processes them individually. Thus we are able to keep a decent FPS (around 25) while our bot is on the go. We are using `Queue` with size of 2 to make sure our frames are queued and passed to the thread and processed accordingly:

```py
# queues
self.frame_queue_left = Queue(maxsize=2)
self.frame_queue_right = Queue(maxsize=2)
self.frame_queue_orange = Queue(maxsize=2)
self.frame_queue_blue = Queue(maxsize=2)
self.frame_queue_green = Queue(maxsize=2)
...
```

<br>

After that, we were facing an issue for motion blur due to speed. We've resolved this issue by setting explicit `Exposure rate` and `Analogue gain` in our camera configuration:

<img width="560" alt="image" src="https://github.com/user-attachments/assets/d37004e0-7c14-49ed-90fa-8d76a48db845" />

<br>

```py
picam2.set_controls(
    {
        "ExposureTime": 16000,
        "AnalogueGain": 42.0,
        "AeEnable": False,
        "AwbEnable": False,
        "FrameDurationLimits": (40000, 40000),
    }
)
```

<br>

After the program is executed, we start 8 threads, and inside while loop, we are collecting the contour result in each iteration:

```py
# Retrieve all results from queues (non-blocking)
(
    left_result,
    right_result,
    orange_result,
    blue_result,
    green_result,
    red_result,
    reverse_result,
    front_wall_result,
    parking_result,
) = contour_workers.collect_results()
```

<br>

We are using PID steering, and for the sake of simplicity, we've used [python simple-pid](https://pypi.org/project/simple-pid/). We've made a tool [adjust_pid_values.py](https://github.com/nurulislam21/WRO-FE-2025_Team-Echo-Drift/blob/main/src/raspberrypi/adjust_pid_values.py) for adjusting the Kp, Ki, and Kd values.

<img width="450" alt="image" src="https://github.com/user-attachments/assets/dc247885-04d4-4326-8de0-ae7abf3497f0" />

<br>

We are comparing the area of left region and right region, and normalizing the error to range `[-1, 1]`.  If the left area is greater than right area the bot tends to turn right and vice versa


<img width="500" height="87" alt="image" src="https://github.com/user-attachments/assets/dc494ae2-a1a2-488d-ab57-26dad2af9408" />


```py
left_buf.append(left_area)
right_buf.append(right_area)
left_s = sum(left_buf) / len(left_buf)
right_s = sum(right_buf) / len(right_buf)
aDiff = right_s - left_s
aSum = left_s + right_s
error = aDiff / (aSum + 1e-6)  # normalized between roughly [-1,1]
normalized_angle_offset = pid(error)
```

<br>


For obstacle avoidance, we use a pivot point as the target.
- For a red object, the pivot is the midpoint between the object’s x-coordinate and the right wall’s centroid x-coordinate.
- For a green object, the pivot is the midpoint between the object’s x-coordinate and the left wall’s centroid x-coordinate.

The pivot is calculated only when the objects enter the danger zone; otherwise, it is ignored.


<img width="500" alt="image" src="https://github.com/user-attachments/assets/dd9c0363-e1f3-450a-bc4d-29f72356bce5" />



```py
r_wall_x, r_wall_y = get_overall_centroid(right_result.contours)

if r_wall_x is None:
    print("No wall detected!")
    # set default wall position if none detected
    r_wall_x = CAM_WIDTH
else:
    # transform to global coordinates
    r_wall_x += RIGHT_REGION[0]

# transform to global coordinates
red_obj_x += OBS_REGION[0]
red_obj_y += OBS_REGION[1]

# compute how far is the bot from the object and walls middle point
offset_x = (red_obj_x + ((r_wall_x - red_obj_x) // 2)) - (
    CAM_WIDTH // 2
)

# show a circle dot on the middle of the object and wall
obstacle_wall_pivot = (
    red_obj_x + ((r_wall_x - red_obj_x) // 2),
    red_obj_y,
)

obj_error = offset_x / (CAM_WIDTH // 2)  # normalized [-1, 1]
```



