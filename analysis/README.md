# Engineering Analysis & Kinematics Calculations

This document details the mathematical models, kinematics, power budget, and sensor coverage calculations for the NAVI assistive mobility robot.

---

## 1. Differential Drive Kinematics

The robot utilizes a differential drive configuration with two independent drive wheels and passive dual casters.

### Parameters
* **Wheel Radius ($r$)**: $33.0\text{ mm} = 0.033\text{ m}$ (66mm diameter rubber tires)
* **Track Width / Wheel Separation ($L$)**: $176.0\text{ mm} = 0.176\text{ m}$
* **Encoder Resolution**: $11\text{ PPR}$ base $\times 31:1\text{ gearbox} = 341.2\text{ Pulses Per Revolution (PPR)}$
* **Sampling Period ($\Delta t$)**: $0.01\text{ s}$ ($100\text{ Hz}$ control loop)

### Forward Kinematics
Given left and right wheel angular velocities $\omega_L$ and $\omega_R$ (rad/s):

$$\begin{aligned}
v &= \frac{r}{2} (\omega_R + \omega_L) \quad \text{[Linear Velocity in m/s]} \\
\omega &= \frac{r}{L} (\omega_R - \omega_L) \quad \text{[Angular Velocity in rad/s]}
\end{aligned}$$

### Odometry Integration (Runge-Kutta 2nd Order)
At time step $k$ with linear displacement $\Delta s = v \Delta t$ and angular displacement $\Delta \theta = \omega \Delta t$:

$$\begin{aligned}
\theta_{k} &= \theta_{k-1} + \Delta \theta \\
x_{k} &= x_{k-1} + \Delta s \cos\left(\theta_{k-1} + \frac{\Delta \theta}{2}\right) \\
y_{k} &= y_{k-1} + \Delta s \sin\left(\theta_{k-1} + \frac{\Delta \theta}{2}\right)
\end{aligned}$$

---

## 2. Encoder Pulse-to-Distance Conversion

Given tick counts $\Delta N_L$ and $\Delta N_R$ over time $\Delta t$:

$$\begin{aligned}
\text{Distance per tick} &= \frac{2 \pi r}{341.2} = \frac{2 \pi \times 0.033}{341.2} \approx 0.0006077\text{ m/tick} \quad (0.608\text{ mm/tick}) \\
v_L &= \frac{\Delta N_L \times 0.0006077}{\Delta t} \\
v_R &= \frac{\Delta N_R \times 0.0006077}{\Delta t}
\end{aligned}$$

---

## 3. Electrical Power Budget & Battery Life Estimation

* **Battery Pack**: 3S2P 18650 Li-ion ($11.1\text{V}$ nominal, $12.6\text{V}$ peak, $5200\text{ mAh} = 57.72\text{ Wh}$)

| Subsystem / Component | Nominal Voltage | Idle Current | Peak/Active Current | Average Power |
| :--- | :--- | :--- | :--- | :--- |
| Arduino UNO Q (Linux MPU + STM32 MCU) | 5.0V (via Buck) | 450 mA | 1200 mA | 4.5 W |
| YDLidar X4 (Laser + Motor) | 5.0V (via USB) | 350 mA | 500 mA | 2.5 W |
| 2× JGA25-371 DC Motors (Cruising 0.4 m/s) | 12.0V (Direct) | 200 mA | 1400 mA | 9.6 W |
| 2× BTS7960 Driver Quiescent Loss | 12.0V (Direct) | 30 mA | 50 mA | 0.5 W |
| 2× VL53L0X ToF Sensors | 3.3V (LDO) | 20 mA | 40 mA | 0.1 W |
| MPU-6050 6-DoF IMU | 3.3V (LDO) | 5 mA | 5 mA | 0.02 W |
| TB6612FNG Haptic Actuator Driver | 5.0V (via Buck) | 2 mA | 250 mA (pulsed) | 0.4 W |
| NEO-6M GPS Receiver | 3.3V (LDO) | 45 mA | 60 mA | 0.2 W |
| **Total System** | — | **~1.1 A @ 12V** | **~3.5 A @ 12V** | **~17.8 W (Avg)** |

### Calculated Continuous Mission Runtime

$$\text{Effective Capacity (85\% DoD)} = 5200\text{ mAh} \times 0.85 = 4420\text{ mAh} = 49.06\text{ Wh}$$

$$\text{Runtime} = \frac{49.06\text{ Wh}}{17.8\text{ W}} \approx 2.75\text{ hours continuous autonomous escort}$$

---

## 4. Optical Ranging Geometry & ToF Blind Spot Analysis

* **VL53L0X Cone of View (FoV)**: $25^\circ$ conical aperture
* **Sensor Mounting Height**: $120\text{ mm}$ above floor level
* **Pitch Angle**: $5^\circ$ downward tilt
* **Floor Intersection Distance**:

$$D_{\text{floor}} = \frac{0.120\text{ m}}{\tan(5^\circ + 12.5^\circ)} = \frac{0.120}{\tan(17.5^\circ)} = \frac{0.120}{0.315} \approx 0.38\text{ m}$$

* **Result**: Any step-down, drop-off, or floor obstacle higher than $25\text{ mm}$ is reliably captured within the $0.15\text{ m}$ to $1.2\text{ m}$ safety envelope before the robot chassis reaches the hazard.
