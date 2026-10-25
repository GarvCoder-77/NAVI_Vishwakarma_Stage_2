# Hardware Bench Commissioning & Bringup Runbook

This guide details the procedure for safely powering up, verifying voltages, testing actuators, and calibrating sensors on the physical NAVI robot platform.

---

## ⚠️ Pre-Power Electrical Inspection

1. **Multimeter Continuity Check**:
   * Verify continuity between **Battery Negative (-)** and all ground pins:
     * Arduino UNO Q `GND`
     * Buck converter `GND_OUT`
     * Left & Right BTS7960 `GND`
     * TB6612FNG `GND`
   * Confirm **NO short circuit** between Battery Positive (+) and Common Ground before turning on the Kill Switch (Resistance $> 10\text{ k}\Omega$).
2. **Voltage Verification (No Logic Connected)**:
   * Turn ON the Master Kill Switch.
   * Measure Buck Converter output: must read between **`5.05V` and `5.20V`**.
   * Measure Battery bus: must read between **`11.1V` (nominal) and `12.6V` (fully charged)**.

---

## 🔌 Sensor Bus Calibration & Address Remapping

### 1. I2C Bus Verification
Connect Arduino UNO Q via USB to bench PC and run an I2C bus scanner sketch:
* **MPU-6050 IMU**: Found at default address `0x68`.
* **VL53L0X Initialization Sequence**:
  1. Hold both `XSHUT_1` (`A0`) and `XSHUT_2` (`A1`) LOW to reset both sensors into shutdown.
  2. Set `XSHUT_1` HIGH, initialize sensor at `0x29`, then execute `setAddress(0x30)`.
  3. Set `XSHUT_2` HIGH, initialize sensor at `0x29`, then execute `setAddress(0x31)`.
  4. Both ToF sensors now operate simultaneously on the shared I2C bus without bus collisions.

### 2. MPU-6050 Zero-Point Gyro Calibration
* Keep robot completely motionless on a level surface.
* Collect 1,000 gyro yaw readings over 5 seconds.
* Compute mean offset:
  $$\text{Offset}_Z = \frac{1}{N} \sum_{i=1}^N \omega_{z,i}$$
* Subtract $\text{Offset}_Z$ from live yaw rates in STM32 firmware to eliminate angular drift during straight-line travel.

---

## 🏎️ Motor Polarity & Encoder Direction Test

1. **Wheel Rotation Test (Elevated on Stand)**:
   * Command forward velocity:
     * Left Motor: `RPWM (~D5) = 80`, `LPWM (~D6) = 0`, `LEN/REN (D4) = HIGH`.
     * Right Motor: `RPWM (~D9) = 80`, `LPWM (~D10) = 0`, `LEN/REN (D7) = HIGH`.
   * Visually verify both wheels spin forward. If a wheel spins backward, swap its `MTOR+` and `MTOR-` leads at the BTS7960 screw terminal.
2. **Encoder Sign Verification**:
   * Rotate left wheel forward by hand for 1 complete revolution:
     * `left_ticks` should increment by $+341 \pm 3$ counts.
   * Rotate right wheel forward by hand for 1 complete revolution:
     * `right_ticks` should increment by $+341 \pm 3$ counts.
   * If ticks decrement, swap `ENC A` and `ENC B` in software or hardware.

---

## 📳 Haptic Driver Checkout (TB6612FNG)

* Set `STBY` (`+3.3V`) and `VCC` (`+3.3V`).
* Apply PWM pulse (`~D11 = 180`, `D13 = HIGH`) to Channel A: Confirm Coin Vibration Motor pulses for 200ms.
* Apply PWM pulse (`~D11 = 255`, `D13 = HIGH`) to Channel B: Confirm Cylindrical Vibration Motor generates high-amplitude buzz.
