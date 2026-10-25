# Hardware Components & Technical Specifications — Blind Assistant Robot

This document provides an exhaustive technical catalog of all 15 electronic, sensor, drive, power, and haptic components used in the **NAVI Blind Assistant Robot** circuit as modeled in Cirkit Designer project [`d6c1d5d1-9d43-4f9d-b31e-ef1cfa1fba4a`](https://app.cirkitdesigner.com/project/d6c1d5d1-9d43-4f9d-b31e-ef1cfa1fba4a).

---

## 📋 Comprehensive Bill of Materials (BOM) & Specifications Table

| # | Component Label | Part / Model | Operating Voltage | Current Rating | Primary Interface | Key Function |
| :-: | :--- | :--- | :--- | :--- | :--- | :--- |
| **1** | **Arduino UNO Q** | Arduino UNO Q Dual-Brain | 5.0V DC (Input), 3.3V Logic | ~500mA–1.2A | Dual-Core IPC / GPIO / I2C / UART | Main robot controller (Linux MPU + Real-Time MCU) |
| **2** | **12V 3S Li-ion Battery** | 18650 3S2P Li-ion Pack | 11.1V Nom / 12.6V Peak | 5200 mAh (20A Disch.) | XT60 / Screw Terminal | Main system energy source |
| **3** | **Master Kill Switch** | Heavy-Duty Toggle Switch | Up to 250V / 12VDC | 15A – 20A DC | In-line SPST | Emergency power disconnect (E-Stop) |
| **4** | **12V to 5V Buck Converter** | LM2596 / MP1584 Step-Down | Input: 7–28V, Output: 5.0V | 3.0A continuous (4A pk) | 2-Wire Input / 2-Wire Output | High-efficiency 5V logic & haptic power |
| **5** | **Left BTS7960 Driver** | BTS7960 High-Power H-Bridge | Motor: 6–28V, Logic: 5.0V | 43A Peak (15A Cont.) | PWM (RPWM/LPWM), Dir (REN/LEN) | High-current left drive motor control |
| **6** | **Right BTS7960 Driver** | BTS7960 High-Power H-Bridge | Motor: 6–28V, Logic: 5.0V | 43A Peak (15A Cont.) | PWM (RPWM/LPWM), Dir (REN/LEN) | High-current right drive motor control |
| **7** | **Left Drive Motor + Encoder** | JGA25-371 Geared DC Motor | Motor: 12V DC, Encoder: 3.3V | Stall: ~3.2A, No-load: 0.2A | Differential H-Bridge + Quadrature AB | Left wheel propulsion and wheel odometry |
| **8** | **Right Drive Motor + Encoder** | JGA25-371 Geared DC Motor | Motor: 12V DC, Encoder: 3.3V | Stall: ~3.2A, No-load: 0.2A | Differential H-Bridge + Quadrature AB | Right wheel propulsion and wheel odometry |
| **9** | **MPU-6050 IMU** | 6-DoF Gyro + Accelerometer | 3.3V – 5.0V (Runs at 3.3V) | ~3.8mA | Fast I2C (Address `0x68`) | Heading stabilization & rotational slip rejection |
| **10** | **ToF Sensor #1 (Front-Left)** | VL53L0X Time-of-Flight | 2.8V – 3.3V DC | 19mA Active | I2C (Default `0x29` -> `0x30`) + XSHUT | Low obstacle & step drop-off detection |
| **11** | **ToF Sensor #2 (Front-Right)** | VL53L0X Time-of-Flight | 2.8V – 3.3V DC | 19mA Active | I2C (Default `0x29` -> `0x31`) + XSHUT | Low obstacle & curb detection |
| **12** | **NEO-6M GPS Module** | U-blox 6 Positioning Engine | 3.3V – 5.0V DC | 45mA Active | UART Serial (9600 baud) | Outdoor waypoint navigation & campus localization |
| **13** | **TB6612FNG Haptic Driver** | Toshiba Dual MOSFET Bridge | Logic: 3.3V, Motor: 5.0V | 1.2A Cont. (3.2A Peak) | 2-Ch PWM + Direction + STBY | Inductive driver for dual vibration feedback |
| **14** | **Coin Vibration Motor** | 1030 Flat ERM Motor | 3.0V – 5.0V DC | ~75mA – 90mA | DC PWM via TB6612FNG Ch-A | Subtle directional navigation haptic cue |
| **15** | **Vibration Motor (ROB-08449)** | Encased Cylindrical ERM | 3.0V – 5.0V DC | ~90mA – 120mA | DC PWM via TB6612FNG Ch-B | High-intensity emergency / obstacle warning cue |

---

## 🔍 Detailed Component Profiles & Engineering Rationale

### 1. Arduino UNO Q (Dual-Brain MPU + MCU)
* **Architecture**: Combines a high-performance **Qualcomm QRB2210 Quad-Core ARM Cortex-A53** running Debian Linux with an ultra-low-latency **STM32U585 ARM Cortex-M33** microcontroller in an Arduino UNO form-factor.
* **Technical Specifications**:
  * Microprocessor: 4x ARM Cortex-A53 @ 2.0 GHz, 2GB LPDDR4 RAM, 16GB eMMC storage.
  * Microcontroller: ARM Cortex-M33 @ 160 MHz with TrustZone, 2MB Flash, 786KB SRAM.
  * Inter-Processor Communication: High-speed hardware serial/RPC bridge.
  * Operating Voltage: 5.0V power input on `+5V` pin, 3.3V regulated output on `+3V3`.
* **Why It Is Used**:
  Autonomous assistive robotics requires two fundamentally incompatible computing paradigms:
  1. High-level operating systems (Linux, ROS 2, OpenCV, SLAM Toolbox, speech synthesis).
  2. Hard real-time control (sub-microsecond hardware interrupts for optical wheel encoders, deterministic PWM wave generation, and a 500ms emergency safety watchdog).
  Traditional single-board computers (Raspberry Pi) jitter under Linux kernel scheduler latency, causing lost encoder ticks and erratic odometry. Traditional microcontrollers (Arduino Uno/Mega) lack the RAM and compute power to run SLAM and computer vision. The **Arduino UNO Q** unites both on a single PCB with shared power rails and zero ribbon-cable failure points.

---

### 2. 12V 3S Li-ion Battery Pack (3S2P 18650)
* **Technical Specifications**:
  * Cell Configuration: 3-Series, 2-Parallel (6x 18650 Lithium-Ion cells).
  * Voltage: $11.1\text{V}$ nominal, $12.6\text{V}$ fully charged, $9.0\text{V}$ low-voltage cutoff.
  * Capacity: $5200\text{ mAh}$ ($57.7\text{ Wh}$).
  * Continuous Discharge Rating: $20\text{A}$ ($4C$).
  * Built-in Battery Management System (BMS): Over-charge, over-discharge, short-circuit, and cell-balancing protection.
* **Why It Is Used**:
  The drive motors and high-current BTS7960 bridges require a nominal 12V supply to achieve the necessary speed and torque. Standard lead-acid batteries are prohibitively heavy ($>2.5\text{ kg}$), while standard USB power banks cannot supply the $6\text{A}+$ stall currents of dual DC motors. A 3S2P Li-ion pack offers the optimal power-to-weight ratio for an assistive robot carrying payloads.

---

### 3. Master Kill Switch (High-Current SPST Toggle)
* **Technical Specifications**:
  * Contact Rating: $15\text{A}$ @ 250VAC, $20\text{A}$ @ 12VDC.
  * Contact Resistance: $<15\text{ m}\Omega$.
  * Actuation: Heavy-duty mechanical toggle with positive snap action.
* **Why It Is Used**:
  In physical assistive robotics carrying heavy gearmotors, a hard mechanical kill switch placed directly in-line on the battery positive terminal is an indispensable safety barrier. If software crashes or a MOSFET fails closed, the operator or visually impaired user can instantly cut all physical power, preventing runaway conditions.

---

### 4. 12V to 5V DC-DC Buck Converter (3A+ Continuous)
* **Technical Specifications**:
  * Topology: Synchronous step-down switching regulator (LM2596 / MP1584).
  * Input Range: $7.0\text{V} - 28.0\text{V}$ DC; Output: $5.00\text{V} \pm 1\%$ regulated.
  * Efficiency: $90\% - 94\%$.
  * Continuous Current: $3.0\text{A}$ (peak $4.0\text{A}$).
  * Switching Frequency: $150\text{ kHz} - 1.5\text{ MHz}$ with low ripple ($<30\text{mV}$).
* **Why It Is Used**:
  Dropping $12\text{V}$ down to $5\text{V}$ using a standard linear regulator (e.g. 7805 or onboard Arduino LDO) would dissipate:
  $$P_{loss} = (V_{in} - V_{out}) \cdot I = (12\text{V} - 5\text{V}) \cdot 2\text{A} = 14\text{ Watts}$$
  This would instantly incinerate a linear regulator. The switching buck converter safely and efficiently powers the Arduino UNO Q, the BTS7960 logic buffers, and the haptic driver without heat dissipation or thermal throttling.

---

### 5 & 6. Left & Right BTS7960 Motor Drivers (43A High-Power H-Bridges)
* **Technical Specifications**:
  * Chipset: Dual Infineon BTN7960 half-bridges + 74HC244 Schmitt-trigger buffer per module.
  * Voltage Rating: Motor supply $6.0\text{V} - 28.0\text{V}$, Logic supply $5.0\text{V}$.
  * Output Current: $43\text{A}$ instantaneous peak, $15\text{A}$ continuous without auxiliary fan.
  * On-Resistance: $R_{DS(on)} = 16\text{ m}\Omega$ per switch (ultra-low conduction losses).
  * PWM Capability: Up to $25\text{ kHz}$ with internal dead-time shoot-through prevention.
  * Diagnostic Features: Current sensing output (`IS`), over-temperature, under-voltage, over-current latch.
* **Why They Are Used**:
  Common hobbyist L298N drivers rely on obsolete bipolar junction transistors that drop $2.0\text{V} - 2.5\text{V}$ across the internal switches, robbing the motors of $20\%$ of battery voltage and overheating at just $1.5\text{A}$. The BTS7960 uses low-resistance MOSFETs that run cool, deliver full battery voltage, and provide independent left and right current paths for agile differential steering.

---

### 7 & 8. Left & Right JGA25-371 Geared DC Motors with Encoders
* **Technical Specifications**:
  * Voltage: $12\text{V}$ DC nominal.
  * Gearbox: All-metal precision planetary/spur gearbox (e.g. 30:1 / 45:1 ratio).
  * Rated Speed: $\approx 180\text{ RPM}$ under load; Stall Torque: $>7.0\text{ kg}\cdot\text{cm}$.
  * Encoder Type: Integrated dual-channel Hall effect / optical quadrature encoder.
  * Resolution: 11 pulses per motor revolution (PPR) $\times$ gear ratio $\approx 360 - 495$ counts per wheel turn.
  * Encoder Logic Supply: $3.3\text{V}$ (powered from Arduino `+3V3` pin).
* **Why They Are Used**:
  Provides the low-speed, high-torque propulsion required to smoothly transport the robot across carpets, tile, and door thresholds. The quadrature Phase A/B signals provide bi-directional wheel displacement, enabling real-time dead-reckoning odometry computation at 20 Hz.

---

### 9. MPU-6050 6-DoF Inertial Measurement Unit (IMU)
* **Technical Specifications**:
  * Sensors: 3-axis MEMS accelerometer ($\pm 2g, \pm 4g, \pm 8g, \pm 16g$) + 3-axis MEMS gyroscope ($\pm 250, \pm 500, \pm 1000, \pm 2000^\circ/\text{s}$).
  * ADC Resolution: 16-bit analog-to-digital converters on each channel.
  * Digital Interface: Fast I2C up to $400\text{ kHz}$ at address `0x68`.
  * Voltage: $3.3\text{V}$ logic compatible (onboard regulator accepts 3.3V–5V).
* **Why It Is Used**:
  Differential drive mobile robots inevitably experience wheel slippage when pivoting on smooth competition floors. Wheel odometry alone accumulates severe heading error over time. By fusing the MPU-6050's Gyro Z angular rate ($\omega_z$) with the wheel kinematics ($0.90 \cdot \theta_{gyro} + 0.10 \cdot \theta_{wheels}$), rotational heading drift is eliminated, ensuring SLAM maps remain square and accurate.

---

### 10 & 11. ToF Sensors #1 & #2 (VL53L0X Time-of-Flight Distance Sensors)
* **Technical Specifications**:
  * Technology: STMicroelectronics FlightSense™ Time-of-Flight.
  * Emitter: $940\text{nm}$ invisible Class 1 eye-safe VCSEL (Vertical-Cavity Surface-Emitting Laser).
  * Range: $30\text{ mm}$ to $2000\text{ mm}$ ($2.0\text{m}$).
  * Ranging Accuracy: $\pm 3\%$ under indoor lighting conditions.
  * Control Pins: Hardware reset / shutdown (`XSHUT`), I2C (`SDA`, `SCL`).
* **Why They Are Used**:
  The 2D planar LiDAR is mounted at a fixed elevation above the chassis and only scans a horizontal slice. It is physically blind to:
  1. Low-lying floor obstacles (shoes, toys, power cords, door stoppers).
  2. Drop-offs, staircases, and open elevator shafts.
  Mounting ToF #1 (Front-Left) and ToF #2 (Front-Right) angled downward toward the floor provides an active laser safety curtain that detects drop-offs and obstacles before the robot rolls over them. Dynamic I2C re-addressing via `XSHUT` allows both sensors to operate on the same I2C bus without address collisions.

---

### 12. NEO-6M GPS Module
* **Technical Specifications**:
  * Engine: U-blox 6 positioning engine, 50 tracking channels.
  * Time-To-First-Fix (TTFF): Cold start $27\text{s}$, Hot start $1\text{s}$.
  * Sensitivity: $-161\text{ dBm}$ tracking.
  * Interface: 3.3V UART Serial (default 9600 baud, 1 Hz update rate).
  * Antenna: Detachable high-gain active ceramic patch antenna.
* **Why It Is Used**:
  Enables global geographic coordinates (Latitude, Longitude) for outdoor campus navigation, waypoint tracking between buildings, and geo-fenced safety boundaries.

---

### 13. TB6612FNG Dual H-Bridge Haptic Driver
* **Technical Specifications**:
  * IC: Toshiba TB6612FNG dual DMOS H-bridge.
  * Logic Voltage: $3.3\text{V}$ (`VCC`); Motor Voltage: $5.0\text{V}$ (`VM1, VM2, VM3`).
  * Output Current: $1.2\text{A}$ continuous per channel ($3.2\text{A}$ peak).
  * Flyback Protection: Internal parasitic clamping diodes suppress inductive kickback.
  * Standby Mode: Power-down logic pin (`STBY`).
* **Why It Is Used**:
  Directly driving DC vibration motors from microcontroller GPIO pins is hazardous because GPIO pins can only supply $\approx 20\text{mA}$, while vibration motors draw $80 - 150\text{mA}$ with inductive back-EMF spikes that destroy silicon gates. The TB6612FNG acts as an isolated, PWM-modulated power driver that allows precise control over haptic buzz intensity and rhythm.

---

### 14 & 15. Dual Haptic Vibration Actuators (Coin + Cylindrical ERM)
* **Component 14 — Coin Vibration Motor (1030 ERM)**:
  * Dimensions: $\varnothing 10\text{ mm} \times 3.0\text{ mm}$ flat disc.
  * Operating Voltage: $3.0\text{V} - 5.0\text{V}$ DC, $\approx 80\text{mA}$.
  * Vibration Style: Gentle, high-frequency tactile buzz.
  * Purpose: Subtle directional navigation guidance (e.g. gentle pulsing on the left side to prompt the user to pivot left).
* **Component 15 — High-Amplitude Vibration Motor (ROB-08449)**:
  * Dimensions: Cylindrical weighted eccentric rotating mass.
  * Operating Voltage: $3.0\text{V} - 5.0\text{V}$ DC, $\approx 110\text{mA}$.
  * Vibration Style: Vigorous, high G-force tactile rumble.
  * Purpose: Emergency obstacle proximity alert and urgent stop signal (felt distinctly even through thick gloves).
