#pragma once

// Auto-generated from config/calibration.json by calibration.py

constexpr int BASE_CENTER_DEG = 90;
constexpr int BASE_HOME_DEG = 90;
constexpr int BASE_MIN_DEG = 0;
constexpr int BASE_MAX_DEG = 180;

constexpr int LOWER_GEAR_RATIO = 4;
constexpr int LOWER_SERVO_ZERO_DEG = 0;
constexpr int LOWER_SERVO_SIGN = 1;
constexpr int LOWER_SERVO_MIN_DEG = 0;
constexpr int LOWER_SERVO_MAX_DEG = 180;

constexpr int MIDDLE_GEAR_RATIO = 3;
constexpr int MIDDLE_SERVO_ZERO_DEG = 0;
constexpr int MIDDLE_SERVO_SIGN = 1;
constexpr int MIDDLE_SERVO_MIN_DEG = 0;
constexpr int MIDDLE_SERVO_MAX_DEG = 180;

constexpr int UPPER_GEAR_RATIO = 2;
constexpr int UPPER_SERVO_ZERO_DEG = 0;
constexpr int UPPER_SERVO_SIGN = 1;
constexpr int UPPER_SERVO_MIN_DEG = 0;
constexpr int UPPER_SERVO_MAX_DEG = 180;

constexpr int GRIPPER_OPEN_SERVO_DEG = 180;
constexpr int GRIPPER_CLOSED_SERVO_DEG = 0;
constexpr float STARTUP_GRIPPER_OPEN_FRACTION = 1.0000f;

constexpr unsigned long WATCHDOG_TIMEOUT_MS = 250UL;
constexpr float SERVO_SLEW_DEG_PER_S = 120.0000f;
constexpr float BASE_SLEW_DEG_PER_S = 45.0000f;
