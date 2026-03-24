#pragma once

// Auto-generated from config/calibration.json by calibration.py

constexpr int BASE_STOP_DEG = 90;
constexpr int BASE_SPEED_RANGE_DEG = 28;
constexpr int BASE_DIRECTION_SIGN = 1;
constexpr float BASE_INPUT_DEADBAND = 0.0500f;
constexpr float BASE_HOLD_TRIM_COMMAND = 0.2500f;

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
constexpr float STARTUP_GRIPPER_OPEN_FRACTION = 0.5000f;

constexpr unsigned long WATCHDOG_TIMEOUT_MS = 250UL;
constexpr float SERVO_SLEW_DEG_PER_S = 120.0000f;
constexpr float BASE_SLEW_UNITS_PER_S = 2.5000f;
