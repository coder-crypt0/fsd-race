# Welcome to the Driverless Transition Program 🏎️🤖

Great to have you here! I've worked on ADAS systems across multiple OEMs and Tier-1s, and I've seen what it takes to build a competent autonomous stack — and trust me, going from EV to Driverless in Formula Student is one of the most exciting and challenging transitions you'll make.

Before I lay out any architecture, write or review any code, or set up safety protocols, **I need to understand exactly what we're working with.** In the industry, we call this the **Requirements Gathering & System Definition Phase** — you don't build anything without knowing your constraints.

So let's treat this like a real project kickoff. I need you to answer the following as thoroughly as possible:

---

## 📋 SECTION 1: Vehicle & Hardware Baseline

**1.1 — Current EV Platform**
- What is your current EV architecture? (Motor type, count, drive layout — FWD/RWD/AWD)
- What is your battery system? (Voltage, capacity, cells, BMS type)
- What is your vehicle weight, wheelbase, track width?
- What tires are you running? (Compound, size)
- What is your current top speed and acceleration profile?

**1.2 — Steering System**
- Is the steering currently mechanical (rack & pinion) or do you already have any electronic assistance?
- What is the steering ratio? (degrees of steering wheel → degrees of tire)
- Is there any existing steering actuator, or will we need to design one from scratch?
- What is the max steering angle (left & right)?

**1.3 — Braking System**
- Is the brake system hydraulic? Vacuum assist? Electromechanical?
- What brake balance is currently set up?
- Is there any existing electronic brake actuator?

**1.4 — Propulsion / Drive**
- What motor controller / inverter are you using? (e.g., Sevcon, custom, etc.)
- What communication protocol does the motor controller speak? (CAN, analog, etc.)
- Is torque control available via the existing motor controller?

---

## 📋 SECTION 2: Compute & Sensor Resources

**2.1 — Compute Hardware**
- What onboard compute do you have or plan to have? (e.g., NVIDIA Jetson Orin/Xavier, Raspberry Pi, x86 mini PC, custom board?)
- How much RAM, GPU compute, CPU cores?
- Do you have any FPGA or microcontrollers onboard? (e.g., STM32 for low-level control)
- What is your power budget for compute? (Watts available from the LV system)

**2.2 — Sensors (What do you have or plan to acquire?)**
- **LIDAR**: Do you have one? Model? (e.g., Ouster OS0, Velodyne, RoboSense RS-LiDAR-16, Innoviz, etc.)
- **Stereo/mono cameras**: What cameras? (e.g., Intel RealSense D435i/455, ZED, FLIR, raw Global Shutter cameras?)
- **IMU**: Do you have a dedicated IMU? (e.g., VN200, BNO085, MPU9250, SBG Ellipse?)
- **GPS/RTK**: Do you have any positioning system? (Formula Student Driverless doesn't strictly require GPS — most tracks are cone-based)
- **Ultrasonic / Radar**: Any short-range obstacle detection planned?

**2.3 — Communication Bus**
- What CAN bus topology are you running? (CAN FD? CAN 2.0B?)
- How many CAN buses? (Separate buses for powertrain vs driveline vs autonomy?)
- What ECU(s) are you using for vehicle control?

---

## 📋 SECTION 3: Software Stack & Team

**3.1 — Current Software**
- What framework are you using or planning to use? (ROS2, custom bare-metal, AUTOSAR-style?)
- What programming languages is your team comfortable with? (C++, Python, MATLAB/Simulink?)
- Do you have any existing codebase yet? (Even partial — perception nodes, control loops, etc.)
- What OS are you planning to run? (Ubuntu + RT patch? QNX? Custom RTOS?)

**3.2 — Team Composition**
- How many people are on the driverless sub-team?
- What are their skill levels? (Embedded, perception, control, planning, systems engineering?)
- Do you have any industry mentors or alumni with autonomy experience?
- What is your timeline? Which competition(s) are you targeting, and when?

**3.3 — Budget & Resources**
- What is your approximate budget for the driverless transition? (This affects sensor choices, compute, actuators)
- Do you have access to a test track or skidpad area?
- Do you have a vehicle dynamics simulation model? (e.g., in MATLAB/Simulink, IPG CarMaker, or custom?)

---

## 📋 SECTION 4: Competition & Regulations

**4.1 — Rulebook Familiarity**
- Which competition are you entering? (Formula Student Germany, Formula Student East, Formula SAE, etc.)
- Have you reviewed the specific Driverless rules? (e.g., FSG rules section on autonomous systems, AS (Autonomous System) emergency stop, brake system requirements for driverless, etc.)
- Are you aware of the **ASMS (Autonomous System Mission Status)** requirements and the **E-Stop (Emergency Stop)** system rules?
- Do you understand the requirements for the **RES (Ready-to-Drive Sound)** and brake light indicators for driverless vehicles?

---

## 📋 SECTION 5: Current Plan of Action

**5.1 — What do you have so far?**
- Have you started any development? (Even conceptual diagrams, BOM lists, CAD for actuator mounts?)
- Have you identified the cone-detection approach? (Classical CV, deep learning, LIDAR clustering?)
- Do you have any trajectory planning approach in mind?
- Have you thought about the safety architecture? (Watchdog, emergency braking, fail-safe states?)

---

## How to Answer

You don't need to answer every single question perfectly — if you don't know something, just say **"TBD"** or **"Not sure yet"**. That's actually important information for me too. It tells me where we need to allocate design effort.

**Be as detailed as possible.** The more I know, the better I can tailor the architecture...

*[Note: The source text is cut off at the bottom of the image and continues beyond what was visible.]*
