/*
 * ==============================================================================
 * Arduino Uno Firmware for Blind Assistant Robot Base
 * - Reads wheel encoders using hardware external interrupts (Pins 2 & 3)
 * - Controls dual differential drive DC motors via H-Bridge (PWM + Direction)
 * - Communicates with ROS 2 laptop over USB Serial (115200 baud)
 * - Built-in safety watchdog: Auto-stops if no command received for 500ms
 * ==============================================================================
 */

#include <Wire.h>

// --- PIN DEFINITIONS (Standard L298N / TB6612 / L9110) ---
// Motor Left
const int ENA = 5;    // Left Motor PWM (Speed: 0 - 255)
const int IN1 = 7;    // Left Motor Dir 1
const int IN2 = 8;    // Left Motor Dir 2

// Motor Right
const int ENB = 6;    // Right Motor PWM (Speed: 0 - 255)
const int IN3 = 9;    // Right Motor Dir 1
const int IN4 = 10;   // Right Motor Dir 2

// Encoders (Arduino Uno hardware external interrupts are Pins 2 and 3)
const int ENC_LEFT_PIN  = 2; // Interrupt 0
const int ENC_RIGHT_PIN = 3; // Interrupt 1

// --- VOLATILE ENCODER TICK COUNTERS ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Direction multipliers (-1 or 1 depending on current motor direction)
volatile int dir_left = 1;
volatile int dir_right = 1;

// --- TIMING & WATCHDOG ---
unsigned long last_cmd_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500; // Emergency stop if ROS 2 disconnects

unsigned long last_pub_time = 0;
const unsigned long PUB_INTERVAL_MS = 50;      // Send encoder ticks to ROS 2 at 20 Hz

// --- OPTIONAL I2C IMU (MPU6050 / BNO055 auto-detect at 0x68) ---
bool has_imu = false;
const int MPU_ADDR = 0x68;
int16_t gyro_z_offset = 0;

void init_imu() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    // Device found at 0x68, wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission(true);

    // Calibrate gyro Z zero offset across 20 samples
    long sum_gz = 0;
    for (int i = 0; i < 20; i++) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x47); // GYRO_ZOUT_H
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 2, true);
      if (Wire.available() >= 2) {
        int16_t gz = (Wire.read() << 8) | Wire.read();
        sum_gz += gz;
      }
      delay(5);
    }
    gyro_z_offset = sum_gz / 20;
    has_imu = true;
  } else {
    has_imu = false;
  }
}

int16_t read_gyro_z() {
  if (!has_imu) return 0;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  if (Wire.available() >= 2) {
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
    return raw_gz - gyro_z_offset;
  }
  return 0;
}

// --- INTERRUPT SERVICE ROUTINES (ISRs) ---
void isr_left_encoder() {
  left_ticks += dir_left;
}

void isr_right_encoder() {
  right_ticks += dir_right;
}

// --- MOTOR CONTROL HELPERS ---
void set_motor_left(int pwm) {
  if (pwm > 0) {
    dir_left = 1;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(pwm, 0, 255));
  } else if (pwm < 0) {
    dir_left = -1;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-pwm, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void set_motor_right(int pwm) {
  if (pwm > 0) {
    dir_right = 1;
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(pwm, 0, 255));
  } else if (pwm < 0) {
    dir_right = -1;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-pwm, 0, 255));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stop_motors() {
  set_motor_left(0);
  set_motor_right(0);
}

void setup() {
  // Initialize Serial at 115200 baud
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // Motor pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pin modes (using internal pullups)
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);

  // Attach interrupts on RISING edge
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), isr_right_encoder, RISING);

  init_imu();

  stop_motors();
  last_cmd_time = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. Process incoming commands from ROS 2 Laptop
  // Format: "m <left_pwm> <right_pwm>\n" (e.g. "m 120 120\n" or "m -80 80\n")
  // Or Reset Encoders: "r\n"
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'm') {
      int pwm_l = Serial.parseInt();
      int pwm_r = Serial.parseInt();
      set_motor_left(pwm_l);
      set_motor_right(pwm_r);
      last_cmd_time = now;
    } else if (cmd == 'r') {
      noInterrupts();
      left_ticks = 0;
      right_ticks = 0;
      interrupts();
    }
  }

  // 2. Safety Watchdog: Stop if no command within timeout
  if (now - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
    stop_motors();
  }

  // 3. Periodic Encoder Publishing to ROS 2 (20 Hz)
  // Format: "e <left_ticks> <right_ticks>\n"
  if (now - last_pub_time >= PUB_INTERVAL_MS) {
    last_pub_time = now;

    long cur_left, cur_right;
    noInterrupts();
    cur_left = left_ticks;
    cur_right = right_ticks;
    interrupts();

    Serial.print("e ");
    Serial.print(cur_left);
    Serial.print(" ");
    Serial.print(cur_right);
    if (has_imu) {
      int16_t gz = read_gyro_z();
      // MPU6050 default ±250 deg/s sensitivity is 131 LSB / (deg/s)
      // Convert to milliradians/sec: (gz / 131.0) * (PI / 180.0) * 1000.0 = gz * 0.13323
      long gz_mrad_s = (long)((float)gz * 0.13323f);
      Serial.print(" ");
      Serial.print(gz_mrad_s);
    }
    Serial.println();
  }
}
