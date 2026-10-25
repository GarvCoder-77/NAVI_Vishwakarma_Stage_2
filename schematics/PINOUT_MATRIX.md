# Arduino UNO Q Pinout & Peripheral Assignment Matrix

This matrix summarizes the pin multiplexing, timer allocations, and logic domains on the Arduino UNO Q board for the NAVI Stage 2 assistive robot.

---

## 📌 Microcontroller Pin Allocation

| Arduino Pin | Physical Interface | Subsystem Connected | Signal Function | Electrical Level |
| :--- | :--- | :--- | :--- | :--- |
| **`D0 / RX`** | UART RX | MCU Programming / Debug | Serial Console | 3.3V Logic |
| **`D1 / TX`** | UART TX | MCU Programming / Debug | Serial Console | 3.3V Logic |
| **`D2 / INT0`** | External Interrupt | Left JGA25-371 Encoder | **Channel A Pulse (Hardware ISR)** | 3.3V Pulled-Up |
| **`D3 / INT1`** | External Interrupt | Right JGA25-371 Encoder | **Channel A Pulse (Hardware ISR)** | 3.3V Pulled-Up |
| **`D4`** | GPIO Output | Left BTS7960 Driver | Left Bridge Forward/Reverse Enable (`LEN`/`REN`) | 3.3V Logic |
| **`~D5`** | Timer PWM | Left BTS7960 Driver | Left Motor Forward Speed (`RPWM`) | 3.3V PWM (980 Hz) |
| **`~D6`** | Timer PWM | Left BTS7960 Driver | Left Motor Reverse Speed (`LPWM`) | 3.3V PWM (980 Hz) |
| **`D7`** | GPIO Output | Right BTS7960 Driver | Right Bridge Forward/Reverse Enable (`LEN`/`REN`) | 3.3V Logic |
| **`D8`** | GPIO Input | Left JGA25-371 Encoder | Channel B Quadrature Direction | 3.3V Logic |
| **`~D9`** | Timer PWM | Right BTS7960 Driver | Right Motor Forward Speed (`RPWM`) | 3.3V PWM (490 Hz) |
| **`~D10`** | Timer PWM | Right BTS7960 Driver | Right Motor Reverse Speed (`LPWM`) | 3.3V PWM (490 Hz) |
| **`~D11`** | Timer PWM | TB6612FNG Haptic Driver | Vibration Intensity Modulation (`PWMA`/`PWMB`) | 3.3V PWM (490 Hz) |
| **`D12`** | GPIO Input | Right JGA25-371 Encoder | Channel B Quadrature Direction | 3.3V Logic |
| **`D13`** | GPIO Output | TB6612FNG Haptic Driver | Motor Direction Logic (`AIN1`/`BIN1`) | 3.3V Logic |
| **`A0`** | GPIO Output | Front-Left VL53L0X ToF | Sensor Reset / Shutdown (`XSHUT_1`) | 3.3V Logic |
| **`A1`** | GPIO Output | Front-Right VL53L0X ToF | Sensor Reset / Shutdown (`XSHUT_2`) | 3.3V Logic |
| **`A2`** | Software / Hardware UART | u-blox NEO-6M GPS | GPS Data Receive (`TXD -> A2`) | 3.3V Logic |
| **`A3`** | Software / Hardware UART | u-blox NEO-6M GPS | GPS Configuration Transmit (`RXD <- A3`) | 3.3V Logic |
| **`A4`** | Hardware I2C (SDA) | MPU-6050 + 2× VL53L0X | Shared I2C Data Line (4.7k Pull-up) | 3.3V Logic |
| **`A5`** | Hardware I2C (SCL) | MPU-6050 + 2× VL53L0X | Shared I2C Clock Line (400 kHz Fast-Mode) | 3.3V Logic |

---

## ⚡ Power Bus Domain Rules

1. **12V High-Current Rail**:
   * Feeds directly to BTS7960 motor drivers (`B+`) through the Master Kill Switch.
   * Never routed to any microcontroller header pin.
2. **5V Logic Rail**:
   * Derived from high-efficiency DC-DC buck converter.
   * Feeds Arduino UNO Q `5V` input pin, YDLidar X4 laser scanner motor, and TB6612FNG motor supply (`VM`).
3. **3.3V Clean Reference Rail**:
   * Regulated by onboard Arduino UNO Q LDO.
   * Feeds encoder Hall ICs, MPU-6050 IMU, VL53L0X ToF sensors, and NEO-6M GPS module.
