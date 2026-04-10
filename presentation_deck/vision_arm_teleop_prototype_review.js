const pptxgen = require("pptxgenjs");
const {
  warnIfSlideHasOverlaps,
  warnIfSlideElementsOutOfBounds,
} = require("./pptxgenjs_helpers/layout");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "OpenAI Codex";
pptx.company = "OpenAI";
pptx.subject = "Robotic arm teleoperation prototype review";
pptx.title = "Vision-Based Robotic Arm Teleoperation Prototype Review";
pptx.lang = "en-US";
pptx.theme = {
  headFontFace: "Arial",
  bodyFontFace: "Arial",
  lang: "en-US",
};

const COLORS = {
  navy: "10243E",
  blue: "1E5AA8",
  blueLight: "DDEAF7",
  teal: "127A78",
  greenLight: "DDF4EE",
  amber: "A86700",
  amberLight: "FFF0D7",
  red: "8B1E3F",
  redLight: "FCE3EA",
  gray900: "1F2937",
  gray700: "4B5563",
  gray500: "6B7280",
  gray300: "D1D5DB",
  gray100: "F6F7F9",
  white: "FFFFFF",
};

function addChrome(slide, title, subtitle = "") {
  slide.background = { color: COLORS.white };
  slide.addShape(pptx.ShapeType.rect, {
    x: 0,
    y: 0,
    w: 13.333,
    h: 0.12,
    line: { color: COLORS.navy, transparency: 100 },
    fill: { color: COLORS.navy },
  });
  slide.addText(title, {
    x: 0.7,
    y: 0.24,
    w: 9.8,
    h: 0.36,
    fontFace: "Arial",
    fontSize: 24,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  if (subtitle) {
    slide.addText(subtitle, {
      x: 0.72,
      y: 0.72,
      w: 11.6,
      h: 0.22,
      fontFace: "Arial",
      fontSize: 10,
      color: COLORS.gray500,
      margin: 0,
      italic: true,
    });
  }
  slide.addText("Vision Arm Teleoperation Prototype", {
    x: 10.6,
    y: 0.02,
    w: 2.0,
    h: 0.08,
    fontFace: "Arial",
    fontSize: 8,
    color: COLORS.white,
    align: "right",
    margin: 0,
  });
}

function addFooter(slide, pageNum) {
  slide.addText(`Slide ${pageNum}`, {
    x: 12.2,
    y: 7.1,
    w: 0.7,
    h: 0.18,
    fontFace: "Arial",
    fontSize: 8,
    color: COLORS.gray500,
    align: "right",
    margin: 0,
  });
}

function addBulletList(slide, items, x, y, w, opts = {}) {
  const gap = opts.gap || 0.47;
  const fontSize = opts.fontSize || 18;
  const color = opts.color || COLORS.gray900;
  const bulletIndent = opts.bulletIndent || 16;
  items.forEach((item, idx) => {
    slide.addText(item, {
      x,
      y: y + idx * gap,
      w,
      h: opts.h || 0.34,
      fontFace: "Arial",
      fontSize,
      color,
      margin: 0,
      valign: "mid",
      bullet: { indent: bulletIndent },
    });
  });
}

function addSectionLabel(slide, text, x, y, w, fillColor) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h: 0.28,
    rectRadius: 0.04,
    line: { color: fillColor, transparency: 100 },
    fill: { color: fillColor },
  });
  slide.addText(text, {
    x: x + 0.08,
    y: y + 0.04,
    w: w - 0.16,
    h: 0.14,
    fontFace: "Arial",
    fontSize: 9,
    color: COLORS.white,
    bold: true,
    margin: 0,
  });
}

function addNoteBox(slide, title, text, x, y, w, h, fillColor, titleColor = COLORS.navy) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h,
    rectRadius: 0.06,
    line: { color: COLORS.gray300, pt: 1 },
    fill: { color: fillColor },
  });
  slide.addText(title, {
    x: x + 0.12,
    y: y + 0.08,
    w: w - 0.24,
    h: 0.18,
    fontFace: "Arial",
    fontSize: 12,
    bold: true,
    color: titleColor,
    margin: 0,
  });
  slide.addText(text, {
    x: x + 0.12,
    y: y + 0.3,
    w: w - 0.24,
    h: h - 0.38,
    fontFace: "Arial",
    fontSize: 10,
    color: COLORS.gray900,
    margin: 0,
    valign: "top",
  });
}

function addProcessBox(slide, text, x, y, w, h, fill) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h,
    rectRadius: 0.04,
    line: { color: fill, pt: 1 },
    fill: { color: fill, transparency: 14 },
  });
  slide.addText(text, {
    x: x + 0.08,
    y: y + 0.1,
    w: w - 0.16,
    h: h - 0.2,
    fontFace: "Arial",
    fontSize: 12,
    bold: true,
    color: COLORS.navy,
    margin: 0,
    align: "center",
    valign: "mid",
  });
}

function addArrow(slide, x, y, w) {
  slide.addShape(pptx.ShapeType.chevron, {
    x,
    y,
    w,
    h: 0.28,
    line: { color: COLORS.blue, transparency: 100 },
    fill: { color: COLORS.blue },
  });
}

function validateSlide(slide) {
  warnIfSlideHasOverlaps(slide, pptx);
  warnIfSlideElementsOutOfBounds(slide, pptx);
}

// Slide 1
{
  const slide = pptx.addSlide();
  slide.background = { color: COLORS.white };
  slide.addShape(pptx.ShapeType.rect, {
    x: 0,
    y: 0,
    w: 13.333,
    h: 1.05,
    line: { color: COLORS.navy, transparency: 100 },
    fill: { color: COLORS.navy },
  });
  slide.addText("Vision-Based Robotic Arm Teleoperation", {
    x: 0.7,
    y: 1.1,
    w: 6.8,
    h: 0.7,
    fontFace: "Arial",
    fontSize: 28,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  slide.addText(
    "Single-camera hand tracking mapped to servo commands through deterministic Python logic, configurable transport, and PCA9685-based firmware",
    {
      x: 0.75,
      y: 1.92,
      w: 6.7,
      h: 0.7,
      fontFace: "Arial",
      fontSize: 16,
      color: COLORS.gray700,
      margin: 0,
      valign: "mid",
    }
  );
  addSectionLabel(slide, "Repo-verified prototype review", 0.76, 2.78, 2.4, COLORS.blue);
  addBulletList(
    slide,
    [
      "Host-side Python performs camera capture, hand tracking, feature extraction, filtering, and operator state logic.",
      "Two board targets exist: Arduino Uno over serial and ESP32 DevKit V1 over Wi-Fi UDP.",
      "Both firmware paths drive a PCA9685 and accept the same ASCII command packet.",
      "The checked-in runtime profile currently targets ESP32 + UDP.",
    ],
    0.92,
    3.35,
    6.15,
    { fontSize: 16, gap: 0.5 }
  );
  addNoteBox(
    slide,
    "Prototype scope",
    "This is a practical first integration prototype. It is not a closed-loop industrial manipulator, and the base is not angle-observed because it uses a continuous-rotation servo.",
    8.0,
    2.2,
    4.4,
    2.1,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Current checked-in profile",
    "Transport: UDP\nBoard: ESP32 Arduino + PlatformIO\nServo output: PCA9685 channels 0..4\nCamera profile: 1280x720 at 30 FPS",
    8.0,
    4.55,
    4.4,
    1.9,
    COLORS.greenLight,
    COLORS.teal
  );
  addFooter(slide, 1);
  validateSlide(slide);
}

// Slide 2
{
  const slide = pptx.addSlide();
  addChrome(slide, "Repo Findings and Scope Boundaries");
  addSectionLabel(slide, "Implemented now", 0.78, 1.08, 1.5, COLORS.teal);
  addSectionLabel(slide, "Computed but inactive", 6.92, 1.08, 2.0, COLORS.amber);
  addSectionLabel(slide, "Not present in source", 10.02, 1.08, 1.85, COLORS.red);
  addNoteBox(
    slide,
    "Primary implemented system",
    "Python teleop in main.py is the active host path. It uses MediaPipe landmarks, direct joint mapping, filtering, and a transport abstraction that can send the same command semantics over serial or UDP.",
    0.75,
    1.45,
    5.55,
    2.35,
    COLORS.greenLight,
    COLORS.teal
  );
  addBulletList(
    slide,
    [
      "Whole-hand finger curl actively drives the gripper.",
      "Direct joint mapping actively drives lower, middle, and upper pitch joints.",
      "Both Uno and ESP32 firmware actively parse packets and drive the PCA9685.",
      "The same ASCII TeleopCommand packet is shared across all active paths.",
    ],
    0.92,
    4.15,
    5.35,
    { fontSize: 15, gap: 0.44 }
  );
  addBulletList(
    slide,
    [
      "pinch_metric is computed and calibratable, but not used to actuate the gripper.",
      "wrist_tilt_metric is computed and displayed, but not used to drive a joint.",
      "map_workspace() exists, but the live loop does not call it.",
      "Kinematics and workspace config fields remain, but the runtime is not using IK.",
    ],
    6.92,
    1.5,
    2.95,
    { fontSize: 14, gap: 0.56 }
  );
  addBulletList(
    slide,
    [
      "No active kinematics.py source file",
      "No project send_fixed.py",
      "No project main.c",
      "No project-level Zephyr CMakeLists.txt",
    ],
    10.02,
    1.5,
    2.5,
    { fontSize: 14, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Important caveat",
    "The deck should describe the current runtime as direct hand-to-joint teleoperation, not as inverse-kinematics task-space control.",
    6.92,
    4.75,
    5.6,
    1.45,
    COLORS.amberLight,
    COLORS.amber
  );
  addFooter(slide, 2);
  validateSlide(slide);
}

// Slide 3
{
  const slide = pptx.addSlide();
  addChrome(slide, "End-to-End System Overview");
  const y = 2.1;
  addProcessBox(slide, "Camera frame\nOpenCV capture", 0.6, y, 1.7, 0.9, COLORS.blueLight);
  addArrow(slide, 2.4, y + 0.3, 0.4);
  addProcessBox(slide, "MediaPipe Hands\n21 landmarks", 2.9, y, 1.7, 0.9, COLORS.blueLight);
  addArrow(slide, 4.7, y + 0.3, 0.4);
  addProcessBox(slide, "Feature extraction\ncenter, depth, curl", 5.2, y, 1.8, 0.9, COLORS.greenLight);
  addArrow(slide, 7.15, y + 0.3, 0.4);
  addProcessBox(slide, "Filtering + state\nlogic", 7.65, y, 1.55, 0.9, COLORS.greenLight);
  addArrow(slide, 9.35, y + 0.3, 0.4);
  addProcessBox(slide, "TeleopCommand\nASCII packet", 9.85, y, 1.55, 0.9, COLORS.amberLight);
  addArrow(slide, 11.55, y + 0.3, 0.35);
  addProcessBox(slide, "Firmware + PCA9685\nservo outputs", 11.95, y, 0.9, 0.9, COLORS.redLight);
  addBulletList(
    slide,
    [
      "The host owns perception, normalized signal generation, filtering, and operating-mode decisions.",
      "The board firmware owns packet validation, watchdog behavior, calibration mapping, and final PWM output.",
      "The transport boundary is intentionally narrow: one shared line-oriented command packet.",
    ],
    0.85,
    4.0,
    11.8,
    { fontSize: 16, gap: 0.52 }
  );
  addNoteBox(
    slide,
    "Current deployment choice",
    "The architecture supports both serial and UDP. The checked-in calibration profile uses ESP32 + UDP, but the host-side control semantics remain the same across both transport paths.",
    0.85,
    5.85,
    11.8,
    0.95,
    COLORS.blueLight
  );
  addFooter(slide, 3);
  validateSlide(slide);
}

// Slide 4
{
  const slide = pptx.addSlide();
  addChrome(slide, "Hardware Architecture");
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 0.75,
    y: 1.35,
    w: 3.35,
    h: 4.75,
    rectRadius: 0.06,
    line: { color: COLORS.gray300, pt: 1 },
    fill: { color: COLORS.gray100 },
  });
  slide.addText("Host side", {
    x: 0.95,
    y: 1.55,
    w: 1.0,
    h: 0.2,
    fontSize: 16,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  addBulletList(
    slide,
    [
      "Laptop or host PC runs Python teleop stack.",
      "Single RGB webcam is the primary perception sensor.",
      "OpenCV window provides overlay and keyboard controls.",
      "Current camera profile in calibration.json is 1280x720 at 30 FPS.",
    ],
    1.0,
    1.95,
    2.8,
    { fontSize: 15, gap: 0.55 }
  );
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 4.55,
    y: 1.35,
    w: 4.1,
    h: 4.75,
    rectRadius: 0.06,
    line: { color: COLORS.gray300, pt: 1 },
    fill: { color: COLORS.gray100 },
  });
  slide.addText("Board / drive side", {
    x: 4.75,
    y: 1.55,
    w: 1.6,
    h: 0.2,
    fontSize: 16,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  addBulletList(
    slide,
    [
      "Arduino Uno over serial remains implemented.",
      "ESP32 DevKit V1 over Wi-Fi UDP is the current checked-in target.",
      "Both firmware paths drive a PCA9685 at 50 Hz.",
      "Servo channels 0..4 map to base, lower, middle, upper, gripper.",
    ],
    4.82,
    1.95,
    3.45,
    { fontSize: 15, gap: 0.55 }
  );
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 9.1,
    y: 1.35,
    w: 3.45,
    h: 4.75,
    rectRadius: 0.06,
    line: { color: COLORS.gray300, pt: 1 },
    fill: { color: COLORS.gray100 },
  });
  slide.addText("Actuation side", {
    x: 9.3,
    y: 1.55,
    w: 1.3,
    h: 0.2,
    fontSize: 16,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  addBulletList(
    slide,
    [
      "Continuous-rotation base yaw servo",
      "Three pitch joints with gear reductions",
      "One gripper servo",
      "External servo power and common ground are documented assumptions",
    ],
    9.35,
    1.95,
    2.9,
    { fontSize: 15, gap: 0.55 }
  );
  addNoteBox(
    slide,
    "Documented wiring, not code-enforced",
    "The repo documents ESP32 I2C as GPIO21/GPIO22 and Uno I2C as A4/A5. The firmware confirms the PCA9685 channel map, but physical wiring and power integrity still need hardware confirmation.",
    0.95,
    6.25,
    11.55,
    0.55,
    COLORS.amberLight,
    COLORS.amber
  );
  addFooter(slide, 4);
  validateSlide(slide);
}

// Slide 5
{
  const slide = pptx.addSlide();
  addChrome(slide, "Mechanical Command Model and Calibration Mapping");
  addBulletList(
    slide,
    [
      "Host commands lower, middle, and upper joints in output joint degrees, not raw servo degrees.",
      "Firmware converts joint angles to servo angles using zero offsets, direction signs, and fixed gear ratios.",
      "Current output limits are q1<=45°, q2<=60°, q3<=90°.",
      "The base is now a bounded positional yaw joint with explicit center, min, max, and home angles.",
      "Current checked-in gripper endpoints are 0° closed and 180° open.",
    ],
    0.92,
    1.55,
    7.0,
    { fontSize: 17, gap: 0.55 }
  );
  addNoteBox(
    slide,
    "Firmware-side mapping",
    "lower_servo = zero + sign × 4 × lower_joint\nmiddle_servo = zero + sign × 3 × middle_joint\nupper_servo = zero + sign × 2 × upper_joint",
    8.35,
    1.7,
    4.1,
    1.35,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Generated header values in current profile",
    "BASE_CENTER_DEG = 90\nBASE_HOME_DEG = 90\nBASE_MIN_DEG = 0\nBASE_MAX_DEG = 180",
    8.35,
    3.35,
    4.1,
    1.55,
    COLORS.greenLight,
    COLORS.teal
  );
  addNoteBox(
    slide,
    "Important caveat",
    "The repo still stores richer geometry and kinematics parameters, but the live runtime does not currently use them for inverse kinematics.",
    8.35,
    5.15,
    4.1,
    1.1,
    COLORS.redLight,
    COLORS.red
  );
  addFooter(slide, 5);
  validateSlide(slide);
}

// Slide 6
{
  const slide = pptx.addSlide();
  addChrome(slide, "Software Stack and Library Roles");
  addSectionLabel(slide, "Python host", 0.78, 1.08, 1.2, COLORS.blue);
  addSectionLabel(slide, "Firmware", 6.78, 1.08, 1.0, COLORS.teal);
  addBulletList(
    slide,
    [
      "OpenCV handles camera capture, image mirroring, display, and overlay drawing.",
      "MediaPipe Hands provides landmark detection and tracking for one hand.",
      "NumPy handles distances, angles, normalization, and geometric feature computation.",
      "pyserial implements the serial transport path; socket implements the UDP path.",
      "dataclasses + JSON provide structured configuration and calibration persistence.",
    ],
    0.92,
    1.55,
    5.2,
    { fontSize: 16, gap: 0.56 }
  );
  addBulletList(
    slide,
    [
      "Arduino framework keeps both board targets simple and readable.",
      "Wire provides I2C access to the PCA9685.",
      "WiFi and WiFiUDP are used only in the ESP32 path.",
      "Adafruit PWM Servo Driver Library generates servo PWM through the PCA9685.",
      "PlatformIO builds the ESP32 Arduino target.",
    ],
    6.92,
    1.55,
    5.15,
    { fontSize: 16, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Branch separation",
    "There is no active Zephyr application in this repo snapshot. The PlatformIO README files are template placeholders, not alternate firmware implementations.",
    0.92,
    5.4,
    11.2,
    0.95,
    COLORS.amberLight,
    COLORS.amber
  );
  addFooter(slide, 6);
  validateSlide(slide);
}

// Slide 7
{
  const slide = pptx.addSlide();
  addChrome(slide, "Vision Pipeline");
  addBulletList(
    slide,
    [
      "main.py opens the camera with DirectShow, applies the configured frame size/FPS, and sets CAP_PROP_BUFFERSIZE to 1.",
      "If mirror_view is true, the frame is flipped before hand tracking so control uses mirrored image coordinates.",
      "MediaPipe Hands runs with static_image_mode=false, model_complexity=1, and max_num_hands from config.",
      "Candidates are filtered by requested handedness and a minimum normalized palm width threshold.",
      "The tracker returns one HandObservation containing normalized landmarks, pixel landmarks, center_xy, and derived metrics.",
    ],
    0.92,
    1.5,
    11.4,
    { fontSize: 16, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Current profile values",
    "max_num_hands = 1\nmin_detection_confidence = 0.45\nmin_tracking_confidence = 0.35\nmin_palm_width_norm = 0.015",
    0.92,
    5.25,
    3.0,
    1.1,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Center definition",
    "The hand center is the mean of landmarks [0, 5, 9, 13, 17], which approximates a palm center rather than using a fingertip or bounding-box center.",
    4.25,
    5.25,
    4.0,
    1.1,
    COLORS.greenLight,
    COLORS.teal
  );
  addNoteBox(
    slide,
    "Accuracy caveat",
    "This is monocular hand tracking. The landmark z values are relative model outputs, not metrically calibrated camera depth.",
    8.6,
    5.25,
    3.7,
    1.1,
    COLORS.redLight,
    COLORS.red
  );
  addFooter(slide, 7);
  validateSlide(slide);
}

// Slide 8
{
  const slide = pptx.addSlide();
  addChrome(slide, "Feature Extraction: Active Signals vs Computed Signals");
  addSectionLabel(slide, "Actively used in control", 0.78, 1.08, 1.95, COLORS.teal);
  addSectionLabel(slide, "Computed or stored, but inactive", 7.0, 1.08, 2.25, COLORS.amber);
  addBulletList(
    slide,
    [
      "x_offset_norm: hand horizontal displacement from a captured neutral x position",
      "height_norm: hand height inside the active control box",
      "depth_norm: normalized reach command derived from a heuristic depth metric",
      "finger_curl_norm: normalized whole-hand closure used to command the gripper",
    ],
    0.92,
    1.55,
    5.35,
    { fontSize: 16, gap: 0.62 }
  );
  addBulletList(
    slide,
    [
      "pinch_metric is computed from thumb-tip to index-tip distance",
      "wrist_tilt_metric is computed from the palm normal",
      "wrist_tilt_reference is still captured and stored",
      "pinch_open_reference and pinch_closed_reference are still captured and stored",
      "map_workspace() exists, but the live loop does not call it",
    ],
    7.0,
    1.55,
    5.0,
    { fontSize: 15, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Depth proxy formula",
    "depth_metric = 0.65 × palm_width + 0.25 × wrist_middle + 0.10 × mean_palm_depth",
    0.92,
    5.25,
    5.35,
    0.95,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Gripper feature",
    "finger_curl_metric blends joint-angle curl with fingertip folding distance and then combines all five fingers as 0.7 × mean + 0.3 × min.",
    7.0,
    5.25,
    5.0,
    0.95,
    COLORS.greenLight,
    COLORS.teal
  );
  addFooter(slide, 8);
  validateSlide(slide);
}

// Slide 9
{
  const slide = pptx.addSlide();
  addChrome(slide, "Teleoperation Mapping Math");
  addBulletList(
    slide,
    [
      "Base command = signed-deadband(x_offset_norm), then exponent shaping, then yaw_max scaling.",
      "Lower joint target comes from depth_norm; the current profile inverts this mapping.",
      "Upper joint target comes from height_norm.",
      "Middle joint target is a weighted average of upper and lower; the current profile uses 0.5.",
      "Joint norms are multiplied by q1_max, q2_max, q3_max to produce transmitted joint angles in degrees.",
      "Gripper openness is thresholded: fully open below the open threshold, fully closed above the close threshold, proportional in between.",
    ],
    0.92,
    1.45,
    11.2,
    { fontSize: 16, gap: 0.54 }
  );
  addNoteBox(
    slide,
    "Current profile",
    "yaw_deadband = 0.12\nyaw_exponent = 1.5\ngripper_full_open_threshold = 0.03\ngripper_full_close_threshold = 0.5",
    0.92,
    5.55,
    3.25,
    1.0,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Direct joint mapping, not IK",
    "control_mode is present in config and set to direct_joint, but main.py does not branch on it. The checked-in runtime is explicitly direct hand-to-joint mapping.",
    4.45,
    5.55,
    4.0,
    1.0,
    COLORS.redLight,
    COLORS.red
  );
  addNoteBox(
    slide,
    "Unused config caveat",
    "middle_height_invert is present in config but not consumed by compute_joint_norms() in the current source.",
    8.75,
    5.55,
    3.4,
    1.0,
    COLORS.amberLight,
    COLORS.amber
  );
  addFooter(slide, 9);
  validateSlide(slide);
}

// Slide 10
{
  const slide = pptx.addSlide();
  addChrome(slide, "Filtering, Motion Conditioning, and Grasp Stabilization");
  addBulletList(
    slide,
    [
      "Low-pass filters smooth x, height, depth, and grip before state logic uses them.",
      "Slew rate limiters cap the rate of change for lower, middle, upper, and gripper targets.",
      "When the gripper is actively closing, arm-rate scaling drops to grasp_arm_slowdown_factor.",
      "Depth hold freezes the depth signal when curl exceeds a threshold, then auto-releases after 6 seconds or when the hand reopens enough.",
      "If tracking drops briefly, the host stays in hold for lost_hold_timeout_s before falling back to waiting-for-hand behavior.",
    ],
    0.92,
    1.5,
    11.3,
    { fontSize: 16, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Current filter profile",
    "x_alpha = 0.35\ny_alpha = 0.25\ndepth_alpha = 0.18\ngrip_alpha = 0.28",
    0.92,
    5.45,
    2.4,
    1.0,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Current rate profile",
    "joint_rate_deg_s = 95\ngripper_rate_per_s = 1.8\ngrasp_arm_slowdown_factor = 0.2",
    3.6,
    5.45,
    3.0,
    1.0,
    COLORS.greenLight,
    COLORS.teal
  );
  addNoteBox(
    slide,
    "Important precision note",
    "This is signal conditioning and mode logic, not closed-loop control. There is no measurement of actual joint angle or gripper force in the current implementation.",
    6.95,
    5.45,
    5.2,
    1.0,
    COLORS.redLight,
    COLORS.red
  );
  addFooter(slide, 10);
  validateSlide(slide);
}

// Slide 11
{
  const slide = pptx.addSlide();
  addChrome(slide, "Runtime Modes and Operator State Logic");
  addBulletList(
    slide,
    [
      "Current startup profile enters a 1-second home phase first, because send_home_on_start is true.",
      "After startup homing, the system remains frozen until the operator unfreezes it.",
      "A mode is used for active teleop when a tracked hand is inside the active region.",
      "H mode is used for frozen, waiting, and brief tracking-loss hold behavior.",
      "S mode is the estop-hold command and is also force-sent immediately when estop toggles on.",
      "M mode sends a home posture: zero joint angles plus the startup gripper opening fraction.",
    ],
    0.92,
    1.5,
    11.2,
    { fontSize: 16, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Operator controls",
    "q quit\nf / space freeze\nx estop\nh home\nr reload config",
    0.92,
    5.48,
    2.3,
    1.0,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Live calibration controls",
    "n capture neutral\no capture open hand\np capture closed hand\n[ ] trim base neutral",
    3.5,
    5.48,
    2.8,
    1.0,
    COLORS.greenLight,
    COLORS.teal
  );
  addNoteBox(
    slide,
    "Behavior nuance",
    "Hold and estop are posture-hold behaviors, not torque disable. The arm keeps its last commanded pose while the base is driven to a neutral-trim stop command.",
    6.65,
    5.48,
    5.55,
    1.0,
    COLORS.amberLight,
    COLORS.amber
  );
  addFooter(slide, 11);
  validateSlide(slide);
}

// Slide 12
{
  const slide = pptx.addSlide();
  addChrome(slide, "Transport and Packet Interface");
  addNoteBox(
    slide,
    "Shared packet",
    "T,seq,mode,base_deg,lower_deg,middle_deg,upper_deg,gripper_open",
    0.92,
    1.4,
    11.4,
    0.7,
    COLORS.blueLight
  );
  addBulletList(
    slide,
    [
      "TeleopCommand.encode() is the source of truth for the line format and uses ASCII with a trailing newline.",
      "SerialController and UdpController both rate-limit sends independently of camera frame rate.",
      "force=True bypasses rate limiting for home and estop packets.",
      "SerialController retries port open and provides strong diagnostics for common Windows serial conflicts.",
      "UdpController treats socket creation as connected-enough and does not require a session handshake.",
      "The host overlay surfaces transport status, but the main loop does not currently implement autonomous reconnect.",
    ],
    0.92,
    2.35,
    11.2,
    { fontSize: 16, gap: 0.54 }
  );
  addNoteBox(
    slide,
    "Current checked-in transport profile",
    "transport.kind = udp\nudp.host = 192.168.137.50\nudp.port = 4210\nudp.write_hz = 60",
    0.92,
    5.62,
    3.2,
    0.95,
    COLORS.greenLight,
    COLORS.teal
  );
  addNoteBox(
    slide,
    "Serial profile remains implemented",
    "serial.enabled = false in the checked-in profile, but the serial path still exists and uses the same command semantics at 30 Hz by default in calibration.json.",
    4.45,
    5.62,
    4.0,
    0.95,
    COLORS.amberLight,
    COLORS.amber
  );
  addNoteBox(
    slide,
    "Diagnostic helper",
    "send.py reuses TeleopCommand and can optionally wait for an ESP32 UDP reply after each test packet.",
    8.8,
    5.62,
    3.45,
    0.95,
    COLORS.blueLight
  );
  addFooter(slide, 12);
  validateSlide(slide);
}

// Slide 13
{
  const slide = pptx.addSlide();
  addChrome(slide, "Firmware Paths: Arduino Uno and ESP32");
  slide.addText("Arduino Uno + serial", {
    x: 0.92,
    y: 1.3,
    w: 2.5,
    h: 0.2,
    fontSize: 17,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  slide.addText("ESP32 DevKit V1 + Wi-Fi UDP", {
    x: 6.85,
    y: 1.3,
    w: 3.4,
    h: 0.2,
    fontSize: 17,
    bold: true,
    color: COLORS.navy,
    margin: 0,
  });
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 0.78,
    y: 1.65,
    w: 5.35,
    h: 3.8,
    rectRadius: 0.06,
    line: { color: COLORS.gray300, pt: 1 },
    fill: { color: COLORS.gray100 },
  });
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 6.72,
    y: 1.65,
    w: 5.85,
    h: 3.8,
    rectRadius: 0.06,
    line: { color: COLORS.gray300, pt: 1 },
    fill: { color: COLORS.gray100 },
  });
  addBulletList(
    slide,
    [
      "Parses the shared ASCII packet over Serial.",
      "Uses PCA9685 channels 0..4 for the five actuators.",
      "Applies watchdog hold, clamping, zero/sign/gear-ratio mapping, and slew limiting.",
      "Lets very small base trim commands pass through without forcing the deadband break.",
    ],
    1.0,
    1.95,
    4.8,
    { fontSize: 15, gap: 0.62 }
  );
  addBulletList(
    slide,
    [
      "Parses the same ASCII packet over UDP port 4210.",
      "Connects to Wi-Fi in station mode and uses a static IP from wifi_secrets.h.",
      "Sends simple OK,seq,mode or ERR replies to the sender.",
      "Applies watchdog hold, clamping, zero/sign/gear-ratio mapping, and slew limiting.",
      "Currently applies base deadband directly, which differs slightly from the Uno path.",
    ],
    6.95,
    1.95,
    5.2,
    { fontSize: 15, gap: 0.55 }
  );
  addNoteBox(
    slide,
    "Why this matters",
    "The host-side teleop semantics are intentionally preserved across both board targets. The transport changes; the meaning of the command does not.",
    1.0,
    5.75,
    11.1,
    0.55,
    COLORS.blueLight
  );
  addFooter(slide, 13);
  validateSlide(slide);
}

// Slide 14
{
  const slide = pptx.addSlide();
  addChrome(slide, "Calibration, UI, and Operator Workflow");
  addBulletList(
    slide,
    [
      "config/calibration.json is the master runtime and calibration source.",
      "n captures neutral_x, neutral_y, depth_reference, and wrist_tilt_reference.",
      "o and p capture both pinch references and grip-curl references.",
      "The overlay shows status, transport, landmarks, joint targets, grip, finger curl, wrist tilt, and depth-hold state.",
      "Host-side config changes can be reloaded live with r; firmware-side changes require header export and reflashing.",
      "calibration.py can export both the Uno and ESP32 generated calibration headers.",
    ],
    0.92,
    1.45,
    7.0,
    { fontSize: 16, gap: 0.54 }
  );
  addNoteBox(
    slide,
    "Host-side live tuning",
    "Region boundaries, neutral references, grip references, trim, and transport choice live in JSON and can be updated without rewriting host logic.",
    8.2,
    1.65,
    4.0,
    1.2,
    COLORS.greenLight,
    COLORS.teal
  );
  addNoteBox(
    slide,
    "Firmware-side reflashing",
    "Servo zeroes, signs, limits, base stop, slew, and gripper endpoints are exported into generated_calibration.h and then compiled into the board firmware.",
    8.2,
    3.15,
    4.0,
    1.35,
    COLORS.blueLight
  );
  addNoteBox(
    slide,
    "Caveat",
    "Pinch and wrist-tilt capture still exist in the workflow even though those features do not currently drive the live teleop mapping.",
    8.2,
    4.82,
    4.0,
    1.0,
    COLORS.amberLight,
    COLORS.amber
  );
  addFooter(slide, 14);
  validateSlide(slide);
}

// Slide 15
{
  const slide = pptx.addSlide();
  addChrome(slide, "Safety, Limitations, and Technical Value");
  addSectionLabel(slide, "Safety and robustness present", 0.78, 1.08, 2.2, COLORS.teal);
  addSectionLabel(slide, "Limitations to state plainly", 6.92, 1.08, 2.35, COLORS.red);
  addBulletList(
    slide,
    [
      "Startup freeze prevents immediate free motion.",
      "Active teleop requires a valid tracked hand inside the active region.",
      "Brief tracking-loss hold avoids abrupt command drops on short perception failures.",
      "Host and firmware both clamp or limit motion in different ways.",
      "Firmware watchdog returns the arm to hold if packets stop arriving.",
    ],
    0.92,
    1.55,
    5.25,
    { fontSize: 16, gap: 0.6 }
  );
  addBulletList(
    slide,
    [
      "No joint encoders and no closed-loop state feedback",
      "Base yaw is bounded position control with no encoder feedback",
      "Depth is a heuristic monocular proxy, not metric 3D",
      "The live runtime is direct joint mapping, not task-space IK",
      "Some config and computed signals are inactive or only partially wired",
    ],
    6.92,
    1.55,
    5.2,
    { fontSize: 16, gap: 0.6 }
  );
  addNoteBox(
    slide,
    "Why this still matters technically",
    "Prototype 1 already solves a hard integration problem: turning a noisy monocular hand-tracking pipeline into a stable actuator command stream with transport abstraction, firmware clamping, and an operator-visible calibration workflow.",
    0.92,
    5.55,
    11.2,
    0.95,
    COLORS.blueLight
  );
  addFooter(slide, 15);
  validateSlide(slide);
}

// Slide 16
{
  const slide = pptx.addSlide();
  addChrome(slide, "Roadmap: From Prototype 1 to a More Industry-Style System");
  addBulletList(
    slide,
    [
      "Move from hobby-servo assumptions to stiffer mechanics and higher-quality actuators.",
      "Add encoders and actual joint-state measurement so motion can be closed-loop rather than purely open-loop.",
      "Promote dormant geometry and workspace concepts into a real inverse-kinematics and trajectory-generation path.",
      "Measure end-to-end latency and jitter explicitly, especially across the Wi-Fi path.",
      "Separate timing-critical firmware work if complexity grows; RTOS partitioning becomes relevant only after tighter control requirements appear.",
      "Add hardware safety interlocks, current/temperature limits, and richer fault handling.",
      "Move toward custom electronics and sensor fusion once the manipulation objectives justify the added complexity.",
    ],
    0.92,
    1.45,
    11.25,
    { fontSize: 16, gap: 0.56 }
  );
  addNoteBox(
    slide,
    "Concrete meaning of 'closer to industry standard'",
    "Measured state, calibrated kinematics, trajectory planning, timing visibility, fault handling, and hardware-level safety. The next step is not just a better mapping function; it is a better instrumented robot system.",
    0.92,
    5.9,
    11.2,
    0.72,
    COLORS.greenLight,
    COLORS.teal
  );
  addFooter(slide, 16);
  validateSlide(slide);
}

pptx.writeFile({ fileName: "vision_arm_teleop_prototype_review.pptx" });
