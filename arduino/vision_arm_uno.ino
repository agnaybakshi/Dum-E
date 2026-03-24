#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdlib.h>
#include <string.h>
#include "generated_calibration.h"

constexpr uint8_t BASE_CHANNEL = 0;
constexpr uint8_t LOWER_CHANNEL = 1;
constexpr uint8_t MIDDLE_CHANNEL = 2;
constexpr uint8_t UPPER_CHANNEL = 3;
constexpr uint8_t GRIPPER_CHANNEL = 4;

constexpr uint8_t PCA9685_DEVICE_ADDRESS = 0x40;
constexpr float PCA9685_SERVO_FREQ_HZ = 50.0f;
constexpr float SERVO_MIN_PULSE_US = 500.0f;
constexpr float SERVO_MAX_PULSE_US = 2500.0f;

constexpr float LOWER_JOINT_MIN_DEG = 0.0f;
constexpr float LOWER_JOINT_MAX_DEG = 45.0f;
constexpr float MIDDLE_JOINT_MIN_DEG = 0.0f;
constexpr float MIDDLE_JOINT_MAX_DEG = 60.0f;
constexpr float UPPER_JOINT_MIN_DEG = 0.0f;
constexpr float UPPER_JOINT_MAX_DEG = 90.0f;
constexpr size_t RX_BUFFER_SIZE = 96;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_DEVICE_ADDRESS);

struct CommandTarget {
  char mode = 'H';
  float baseInput = 0.0f;
  float lowerJointDeg = 0.0f;
  float middleJointDeg = 0.0f;
  float upperJointDeg = 0.0f;
  float gripperOpen = STARTUP_GRIPPER_OPEN_FRACTION;
  unsigned long sequence = 0UL;
};

CommandTarget target;
float currentBaseInput = 0.0f;
float currentLowerServoDeg = LOWER_SERVO_ZERO_DEG;
float currentMiddleServoDeg = MIDDLE_SERVO_ZERO_DEG;
float currentUpperServoDeg = UPPER_SERVO_ZERO_DEG;
float currentGripperServoDeg = GRIPPER_CLOSED_SERVO_DEG + STARTUP_GRIPPER_OPEN_FRACTION * (GRIPPER_OPEN_SERVO_DEG - GRIPPER_CLOSED_SERVO_DEG);

char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
unsigned long lastPacketMs = 0UL;
unsigned long lastUpdateMs = 0UL;

float clampf(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

float applyInputDeadband(float value) {
  float magnitude = fabs(value);
  if (magnitude <= BASE_INPUT_DEADBAND) {
    return 0.0f;
  }
  float scaled = (magnitude - BASE_INPUT_DEADBAND) / max(0.0001f, 1.0f - BASE_INPUT_DEADBAND);
  return value >= 0.0f ? scaled : -scaled;
}

float slewToward(float current, float targetValue, float maxDelta) {
  float delta = targetValue - current;
  if (delta > maxDelta) {
    delta = maxDelta;
  } else if (delta < -maxDelta) {
    delta = -maxDelta;
  }
  return current + delta;
}

float lowerJointToServo(float jointDeg) {
  float servoDeg = LOWER_SERVO_ZERO_DEG + LOWER_SERVO_SIGN * LOWER_GEAR_RATIO * jointDeg;
  return clampf(servoDeg, LOWER_SERVO_MIN_DEG, LOWER_SERVO_MAX_DEG);
}

float middleJointToServo(float jointDeg) {
  float servoDeg = MIDDLE_SERVO_ZERO_DEG + MIDDLE_SERVO_SIGN * MIDDLE_GEAR_RATIO * jointDeg;
  return clampf(servoDeg, MIDDLE_SERVO_MIN_DEG, MIDDLE_SERVO_MAX_DEG);
}

float upperJointToServo(float jointDeg) {
  float servoDeg = UPPER_SERVO_ZERO_DEG + UPPER_SERVO_SIGN * UPPER_GEAR_RATIO * jointDeg;
  return clampf(servoDeg, UPPER_SERVO_MIN_DEG, UPPER_SERVO_MAX_DEG);
}

float gripperOpenToServo(float openFraction) {
  float openClamped = clampf(openFraction, 0.0f, 1.0f);
  float servoDeg = GRIPPER_CLOSED_SERVO_DEG + openClamped * (GRIPPER_OPEN_SERVO_DEG - GRIPPER_CLOSED_SERVO_DEG);
  int lower = min(GRIPPER_CLOSED_SERVO_DEG, GRIPPER_OPEN_SERVO_DEG);
  int upper = max(GRIPPER_CLOSED_SERVO_DEG, GRIPPER_OPEN_SERVO_DEG);
  return clampf(servoDeg, lower, upper);
}

float baseInputToServo(float input) {
  float inputClamped = clampf(input, -1.0f, 1.0f);
  // Let small trim commands around zero pass through so we can cancel
  // continuous-servo drift without needing a large manual deadband break.
  float shaped = inputClamped;
  if (fabs(inputClamped) > BASE_INPUT_DEADBAND) {
    shaped = applyInputDeadband(inputClamped);
  }
  float servoDeg = BASE_STOP_DEG + BASE_DIRECTION_SIGN * shaped * BASE_SPEED_RANGE_DEG;
  return clampf(servoDeg, 0.0f, 180.0f);
}

uint16_t servoDegToTicks(float servoDeg) {
  float degClamped = clampf(servoDeg, 0.0f, 180.0f);
  float pulseUs = SERVO_MIN_PULSE_US + (degClamped / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
  float ticks = (pulseUs * PCA9685_SERVO_FREQ_HZ * 4096.0f) / 1000000.0f;
  return static_cast<uint16_t>(clampf(ticks, 0.0f, 4095.0f));
}

void writeServoChannel(uint8_t channel, float servoDeg) {
  pwm.setPWM(channel, 0, servoDegToTicks(servoDeg));
}

void holdPose() {
  target.mode = 'H';
  target.baseInput = BASE_HOLD_TRIM_COMMAND;
}

void goHome() {
  target.mode = 'M';
  target.baseInput = BASE_HOLD_TRIM_COMMAND;
  target.lowerJointDeg = 0.0f;
  target.middleJointDeg = 0.0f;
  target.upperJointDeg = 0.0f;
  target.gripperOpen = STARTUP_GRIPPER_OPEN_FRACTION;
}

bool parseFloatField(const char* text, float& value) {
  char* endPtr = nullptr;
  value = static_cast<float>(strtod(text, &endPtr));
  return endPtr != text && *endPtr == '\0';
}

bool parseUnsignedField(const char* text, unsigned long& value) {
  char* endPtr = nullptr;
  value = strtoul(text, &endPtr, 10);
  return endPtr != text && *endPtr == '\0';
}

bool parsePacket(char* line) {
  char* tokens[8];
  uint8_t count = 0;
  char* context = nullptr;
  char* token = strtok_r(line, ",", &context);
  while (token != nullptr && count < 8) {
    tokens[count++] = token;
    token = strtok_r(nullptr, ",", &context);
  }
  if (count != 8) {
    return false;
  }
  if (strcmp(tokens[0], "T") != 0) {
    return false;
  }

  unsigned long sequence = 0UL;
  float baseInput = 0.0f;
  float lowerJoint = 0.0f;
  float middleJoint = 0.0f;
  float upperJoint = 0.0f;
  float gripperOpen = 0.0f;
  if (!parseUnsignedField(tokens[1], sequence)) {
    return false;
  }
  char mode = tokens[2][0];
  if (tokens[2][1] != '\0' || (mode != 'A' && mode != 'H' && mode != 'S' && mode != 'M')) {
    return false;
  }
  if (!parseFloatField(tokens[3], baseInput) || !parseFloatField(tokens[4], lowerJoint) ||
      !parseFloatField(tokens[5], middleJoint) || !parseFloatField(tokens[6], upperJoint) ||
      !parseFloatField(tokens[7], gripperOpen)) {
    return false;
  }

  target.sequence = sequence;
  target.mode = mode;
  target.baseInput = clampf(baseInput, -1.0f, 1.0f);
  target.lowerJointDeg = clampf(lowerJoint, LOWER_JOINT_MIN_DEG, LOWER_JOINT_MAX_DEG);
  target.middleJointDeg = clampf(middleJoint, MIDDLE_JOINT_MIN_DEG, MIDDLE_JOINT_MAX_DEG);
  target.upperJointDeg = clampf(upperJoint, UPPER_JOINT_MIN_DEG, UPPER_JOINT_MAX_DEG);
  target.gripperOpen = clampf(gripperOpen, 0.0f, 1.0f);

  if (mode == 'S') {
    holdPose();
  } else if (mode == 'M') {
    goHome();
  } else if (mode == 'H') {
    target.mode = 'H';
  }
  return true;
}

void processSerial() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      rxBuffer[rxLength] = '\0';
      if (rxLength > 0 && parsePacket(rxBuffer)) {
        lastPacketMs = millis();
      }
      rxLength = 0;
      continue;
    }
    if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = ch;
    } else {
      rxLength = 0;
    }
  }
}

void updateOutputs() {
  unsigned long nowMs = millis();
  float dt = (nowMs - lastUpdateMs) * 0.001f;
  lastUpdateMs = nowMs;
  if (dt <= 0.0f) {
    dt = 0.02f;
  }

  if (nowMs - lastPacketMs > WATCHDOG_TIMEOUT_MS) {
    holdPose();
  }

  float targetBaseInput = clampf(target.baseInput, -1.0f, 1.0f);
  float lowerServoTarget = lowerJointToServo(target.lowerJointDeg);
  float middleServoTarget = middleJointToServo(target.middleJointDeg);
  float upperServoTarget = upperJointToServo(target.upperJointDeg);
  float gripperServoTarget = gripperOpenToServo(target.gripperOpen);

  currentBaseInput = slewToward(currentBaseInput, targetBaseInput, BASE_SLEW_UNITS_PER_S * dt);
  currentLowerServoDeg = slewToward(currentLowerServoDeg, lowerServoTarget, SERVO_SLEW_DEG_PER_S * dt);
  currentMiddleServoDeg = slewToward(currentMiddleServoDeg, middleServoTarget, SERVO_SLEW_DEG_PER_S * dt);
  currentUpperServoDeg = slewToward(currentUpperServoDeg, upperServoTarget, SERVO_SLEW_DEG_PER_S * dt);
  currentGripperServoDeg = slewToward(currentGripperServoDeg, gripperServoTarget, SERVO_SLEW_DEG_PER_S * dt);

  writeServoChannel(BASE_CHANNEL, baseInputToServo(currentBaseInput));
  writeServoChannel(LOWER_CHANNEL, currentLowerServoDeg);
  writeServoChannel(MIDDLE_CHANNEL, currentMiddleServoDeg);
  writeServoChannel(UPPER_CHANNEL, currentUpperServoDeg);
  writeServoChannel(GRIPPER_CHANNEL, currentGripperServoDeg);
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PCA9685_SERVO_FREQ_HZ);
  delay(10);

  goHome();
  lastPacketMs = millis();
  lastUpdateMs = millis();

  writeServoChannel(BASE_CHANNEL, baseInputToServo(BASE_HOLD_TRIM_COMMAND));
  writeServoChannel(LOWER_CHANNEL, currentLowerServoDeg);
  writeServoChannel(MIDDLE_CHANNEL, currentMiddleServoDeg);
  writeServoChannel(UPPER_CHANNEL, currentUpperServoDeg);
  writeServoChannel(GRIPPER_CHANNEL, currentGripperServoDeg);
}

void loop() {
  processSerial();
  updateOutputs();
  delay(5);
}
