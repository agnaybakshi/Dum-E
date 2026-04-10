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


def export_firmware_header(config: AppConfig, output_path: str | Path) -> Path:
    firmware = config.firmware
    base_min_deg = min(firmware.base_min_deg, firmware.base_max_deg)
    base_max_deg = max(firmware.base_min_deg, firmware.base_max_deg)
    base_center_deg = max(base_min_deg, min(firmware.base_center_deg, base_max_deg))
    base_home_deg = max(base_min_deg, min(firmware.base_home_deg, base_max_deg))
    lines = [
        "#pragma once",
        "",
        "// Auto-generated from config/calibration.json by calibration.py",
        "",
        f"constexpr int BASE_CENTER_DEG = {base_center_deg};",
        f"constexpr int BASE_HOME_DEG = {base_home_deg};",
        f"constexpr int BASE_MIN_DEG = {base_min_deg};",
        f"constexpr int BASE_MAX_DEG = {base_max_deg};",
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
        f"constexpr float BASE_SLEW_DEG_PER_S = {firmware.base_slew_deg_s:.4f}f;",
        "",
    ]
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("\n".join(lines), encoding="ascii")
    return target


def export_firmware_headers(config: AppConfig, output_paths: tuple[Path, ...] = DEFAULT_FIRMWARE_HEADER_OUTPUTS) -> tuple[Path, ...]:
    written_paths: list[Path] = []
    for output_path in output_paths:
        written_paths.append(export_firmware_header(config, output_path))
    return tuple(written_paths)


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

    base_center = subparsers.add_parser("set-base-center", help="Set positional base center angle.")
    base_center.add_argument("--deg", type=int, required=True)

    base_home = subparsers.add_parser("set-base-home", help="Set positional base home angle.")
    base_home.add_argument("--deg", type=int, required=True)

    base_range = subparsers.add_parser("set-base-range", help="Set positional base joint limits.")
    base_range.add_argument("--min-deg", type=int, required=True)
    base_range.add_argument("--max-deg", type=int, required=True)

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

    if args.command == "set-base-center":
        config.firmware.base_center_deg = max(
            config.firmware.base_min_deg,
            min(args.deg, config.firmware.base_max_deg),
        )
        save_config(config)
        return

    if args.command == "set-base-home":
        config.firmware.base_home_deg = max(
            config.firmware.base_min_deg,
            min(args.deg, config.firmware.base_max_deg),
        )
        save_config(config)
        return

    if args.command == "set-base-range":
        config.firmware.base_min_deg = min(args.min_deg, args.max_deg)
        config.firmware.base_max_deg = max(args.min_deg, args.max_deg)
        config.firmware.base_center_deg = max(
            config.firmware.base_min_deg,
            min(config.firmware.base_center_deg, config.firmware.base_max_deg),
        )
        config.firmware.base_home_deg = max(
            config.firmware.base_min_deg,
            min(config.firmware.base_home_deg, config.firmware.base_max_deg),
        )
        save_config(config)
        return

    if args.command == "export-arduino":
        written_path = export_firmware_header(config, args.output)
        print(written_path.resolve())
        return

    if args.command == "export-firmware":
        for written_path in export_firmware_headers(config):
            print(written_path.resolve())
        return


if __name__ == "__main__":
    main()
