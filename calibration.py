from __future__ import annotations

import argparse
from pathlib import Path
from typing import TYPE_CHECKING

from config import AppConfig, load_config, save_config

if TYPE_CHECKING:
    from vision import HandObservation


DEFAULT_FIRMWARE_HEADER_OUTPUTS = (
    Path("arduino/generated_calibration.h"),
    Path("arduino/esp32/vision_arm_esp/include/generated_calibration.h"),
)


def capture_neutral_reference(config: AppConfig, observation: HandObservation) -> None:
    config.vision.neutral_x = observation.center_xy[0]
    config.vision.neutral_y = observation.center_xy[1]
    config.vision.depth_reference = observation.depth_metric
    config.vision.wrist_tilt_reference = observation.wrist_tilt_metric


def capture_pinch_reference(config: AppConfig, observation: HandObservation, open_reference: bool) -> None:
    if open_reference:
        config.vision.pinch_open_reference = observation.pinch_metric
    else:
        config.vision.pinch_closed_reference = observation.pinch_metric


def capture_grip_reference(config: AppConfig, observation: HandObservation, open_reference: bool) -> None:
    if open_reference:
        config.vision.grip_open_reference = observation.finger_curl_metric
    else:
        config.vision.grip_closed_reference = observation.finger_curl_metric


def export_firmware_header(config: AppConfig, output_path: str | Path) -> None:
    firmware = config.firmware
    lines = [
        "#pragma once",
        "",
        "// Auto-generated from config/calibration.json by calibration.py",
        "",
        f"constexpr int BASE_STOP_DEG = {firmware.base_stop_deg};",
        f"constexpr int BASE_SPEED_RANGE_DEG = {firmware.base_speed_range_deg};",
        f"constexpr int BASE_DIRECTION_SIGN = {firmware.base_direction_sign};",
        f"constexpr float BASE_INPUT_DEADBAND = {firmware.base_input_deadband:.4f}f;",
        f"constexpr float BASE_HOLD_TRIM_COMMAND = {config.mapping.base_trim_command:.4f}f;",
        "",
        "constexpr int LOWER_GEAR_RATIO = 4;",
        f"constexpr int LOWER_SERVO_ZERO_DEG = {firmware.lower_zero_deg};",
        f"constexpr int LOWER_SERVO_SIGN = {firmware.lower_sign};",
        f"constexpr int LOWER_SERVO_MIN_DEG = {firmware.lower_servo_min_deg};",
        f"constexpr int LOWER_SERVO_MAX_DEG = {firmware.lower_servo_max_deg};",
        "",
        "constexpr int MIDDLE_GEAR_RATIO = 3;",
        f"constexpr int MIDDLE_SERVO_ZERO_DEG = {firmware.middle_zero_deg};",
        f"constexpr int MIDDLE_SERVO_SIGN = {firmware.middle_sign};",
        f"constexpr int MIDDLE_SERVO_MIN_DEG = {firmware.middle_servo_min_deg};",
        f"constexpr int MIDDLE_SERVO_MAX_DEG = {firmware.middle_servo_max_deg};",
        "",
        "constexpr int UPPER_GEAR_RATIO = 2;",
        f"constexpr int UPPER_SERVO_ZERO_DEG = {firmware.upper_zero_deg};",
        f"constexpr int UPPER_SERVO_SIGN = {firmware.upper_sign};",
        f"constexpr int UPPER_SERVO_MIN_DEG = {firmware.upper_servo_min_deg};",
        f"constexpr int UPPER_SERVO_MAX_DEG = {firmware.upper_servo_max_deg};",
        "",
        f"constexpr int GRIPPER_OPEN_SERVO_DEG = {firmware.gripper_open_servo_deg};",
        f"constexpr int GRIPPER_CLOSED_SERVO_DEG = {firmware.gripper_closed_servo_deg};",
        f"constexpr float STARTUP_GRIPPER_OPEN_FRACTION = {firmware.startup_gripper_open_fraction:.4f}f;",
        "",
        f"constexpr unsigned long WATCHDOG_TIMEOUT_MS = {firmware.watchdog_timeout_ms}UL;",
        f"constexpr float SERVO_SLEW_DEG_PER_S = {firmware.servo_slew_deg_s:.4f}f;",
        f"constexpr float BASE_SLEW_UNITS_PER_S = {firmware.base_slew_units_per_s:.4f}f;",
        "",
    ]
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("\n".join(lines), encoding="ascii")


def export_firmware_headers(config: AppConfig, output_paths: tuple[Path, ...] = DEFAULT_FIRMWARE_HEADER_OUTPUTS) -> None:
    for output_path in output_paths:
        export_firmware_header(config, output_path)


def export_arduino_header(config: AppConfig, output_path: str | Path) -> None:
    export_firmware_header(config, output_path)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibration helper utilities.")
    parser.add_argument("--config", default="config/calibration.json", help="Path to calibration JSON.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("show", help="Print the current config path.")

    neutral = subparsers.add_parser("set-neutral", help="Set neutral hand reference.")
    neutral.add_argument("--x", type=float, required=True)
    neutral.add_argument("--y", type=float, required=True)
    neutral.add_argument("--depth", type=float, required=True)

    pinch = subparsers.add_parser("set-pinch", help="Set pinch calibration values.")
    pinch.add_argument("--open", dest="pinch_open", type=float)
    pinch.add_argument("--closed", dest="pinch_closed", type=float)

    grip = subparsers.add_parser("set-grip", help="Set grip curl calibration values.")
    grip.add_argument("--open", dest="grip_open", type=float)
    grip.add_argument("--closed", dest="grip_closed", type=float)

    base_stop = subparsers.add_parser("set-base-stop", help="Set continuous-servo neutral stop degree.")
    base_stop.add_argument("--deg", type=int, required=True)

    region = subparsers.add_parser("set-region", help="Set teleop active region.")
    region.add_argument("--x-min", type=float, required=True)
    region.add_argument("--y-min", type=float, required=True)
    region.add_argument("--x-max", type=float, required=True)
    region.add_argument("--y-max", type=float, required=True)

    export = subparsers.add_parser("export-arduino", help="Generate the firmware calibration header.")
    export.add_argument(
        "--output",
        default="arduino/generated_calibration.h",
        help="Destination header path.",
    )
    subparsers.add_parser("export-firmware", help="Generate all standard firmware calibration headers.")

    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    config = load_config(args.config)

    if args.command == "show":
        print(config.config_path)
        return

    if args.command == "set-neutral":
        config.vision.neutral_x = args.x
        config.vision.neutral_y = args.y
        config.vision.depth_reference = args.depth
        save_config(config)
        return

    if args.command == "set-pinch":
        if args.pinch_open is not None:
            config.vision.pinch_open_reference = args.pinch_open
        if args.pinch_closed is not None:
            config.vision.pinch_closed_reference = args.pinch_closed
        save_config(config)
        return

    if args.command == "set-grip":
        if args.grip_open is not None:
            config.vision.grip_open_reference = args.grip_open
        if args.grip_closed is not None:
            config.vision.grip_closed_reference = args.grip_closed
        save_config(config)
        return

    if args.command == "set-base-stop":
        config.firmware.base_stop_deg = args.deg
        save_config(config)
        return

    if args.command == "set-region":
        config.vision.active_region.x_min = args.x_min
        config.vision.active_region.y_min = args.y_min
        config.vision.active_region.x_max = args.x_max
        config.vision.active_region.y_max = args.y_max
        save_config(config)
        return

    if args.command == "export-arduino":
        export_firmware_header(config, args.output)
        return

    if args.command == "export-firmware":
        export_firmware_headers(config)
        return


if __name__ == "__main__":
    main()
