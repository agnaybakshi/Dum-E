from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import cv2

from calibration import (
    capture_grip_reference,
    capture_neutral_reference,
    capture_pinch_reference,
    export_firmware_headers,
)
from config import AppConfig, load_config, save_config
from filters import LowPassFilter, SlewRateLimiter, clamp
from hand_mapping import compute_base_command, compute_joint_norms, extract_signals
from transport import TeleopCommand, build_transport
from ui_overlay import draw_overlay
from vision import HandTracker


@dataclass
class JointState:
    lower_deg: float = 0.0
    middle_deg: float = 0.0
    upper_deg: float = 0.0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Single-camera hand teleop for a 5-servo robotic arm.")
    parser.add_argument("--config", default="config/calibration.json", help="Calibration JSON path.")
    parser.add_argument("--transport", choices=("serial", "udp"), help="Override transport kind.")
    parser.add_argument("--serial-port", help="Override serial port from config.")
    parser.add_argument("--udp-host", help="Override UDP target host from config.")
    parser.add_argument("--udp-port", type=int, help="Override UDP target port from config.")
    parser.add_argument("--camera", type=int, help="Override camera device index.")
    parser.add_argument("--no-serial", action="store_true", help="Run without sending commands.")
    parser.add_argument(
        "--export-arduino-header",
        action="store_true",
        help="Refresh generated firmware headers before starting.",
    )
    return parser.parse_args()


def _open_camera(config: AppConfig) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(config.camera.device_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera.frame_height)
    cap.set(cv2.CAP_PROP_FPS, config.camera.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {config.camera.device_index}.")
    return cap


def _build_filters(config: AppConfig) -> dict[str, object]:
    return {
        "x": LowPassFilter(config.filters.x_alpha),
        "height": LowPassFilter(config.filters.y_alpha),
        "depth": LowPassFilter(config.filters.depth_alpha),
        "grip": LowPassFilter(config.filters.grip_alpha),
        "lower": SlewRateLimiter(config.filters.joint_rate_deg_s, 0.0),
        "middle": SlewRateLimiter(config.filters.joint_rate_deg_s, 0.0),
        "upper": SlewRateLimiter(config.filters.joint_rate_deg_s, 0.0),
        "gripper": SlewRateLimiter(
            config.filters.gripper_rate_per_s,
            config.firmware.startup_gripper_open_fraction,
        ),
    }


def _reload_config(path: str) -> AppConfig:
    config = load_config(path)
    export_firmware_headers(config)
    return config


def _home_joints() -> JointState:
    return JointState()


def _apply_runtime_overrides(config: AppConfig, args: argparse.Namespace) -> None:
    if args.transport:
        config.transport.kind = args.transport
    if args.serial_port:
        config.serial.port = args.serial_port
    if args.udp_host:
        config.udp.host = args.udp_host
    if args.udp_port is not None:
        config.udp.port = args.udp_port
    if args.camera is not None:
        config.camera.device_index = args.camera
    if args.no_serial:
        config.transport.enabled = False

    if config.transport.kind == "serial":
        config.serial.enabled = config.transport.enabled
        config.udp.enabled = False
    elif config.transport.kind == "udp":
        config.udp.enabled = config.transport.enabled
        config.serial.enabled = False


def main() -> int:
    args = _parse_args()
    config = load_config(args.config)
    _apply_runtime_overrides(config, args)
    if args.export_arduino_header:
        export_firmware_headers(config)

    filters = _build_filters(config)
    tracker = HandTracker(config.vision)
    transport_controller = build_transport(config)
    cap = None

    frozen = config.control.freeze_on_start
    estop = False
    status = "startup hold"
    hand_active = False
    tracking_ok = False
    last_valid_time = 0.0
    home_until = 0.0
    transport_status = "disabled" if not config.transport.enabled else "disconnected"

    current_base_command = 0.0
    current_gripper_open = config.firmware.startup_gripper_open_fraction
    current_gripper_target_open = config.firmware.startup_gripper_open_fraction
    current_lower_target_deg = 0.0
    current_middle_target_deg = 0.0
    current_upper_target_deg = 0.0
    current_joints = _home_joints()
    current_hand = None
    depth_hold_active = False
    depth_hold_ready = True
    depth_hold_started_at = 0.0
    held_depth_norm = 0.5

    try:
        cap = _open_camera(config)
        transport_controller.connect()
        if not config.transport.enabled:
            transport_status = "disabled"
        elif config.transport.kind == "serial":
            transport_status = f"serial {config.serial.port}"
        else:
            transport_status = f"udp {config.udp.host}:{config.udp.port}"

        if config.control.send_home_on_start:
            home_until = time.perf_counter() + 1.0

        previous_time = time.perf_counter()

        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera frame grab failed.")

            if config.camera.mirror_view:
                frame = cv2.flip(frame, 1)

            now = time.perf_counter()
            dt = max(1e-3, now - previous_time)
            previous_time = now

            current_hand = tracker.process(frame)
            tracking_ok = current_hand is not None
            base_command = 0.0

            if tracking_ok:
                signals = extract_signals(current_hand, config)
                x_norm = filters["x"].update(signals.x_offset_norm)
                height_norm = filters["height"].update(signals.height_norm)
                raw_grip_norm = signals.gripper_open

                if signals.finger_curl_norm <= config.filters.depth_hold_release_curl_threshold:
                    depth_hold_ready = True

                if depth_hold_active:
                    hold_elapsed_s = now - depth_hold_started_at
                    if signals.finger_curl_norm <= config.filters.depth_hold_release_curl_threshold:
                        depth_hold_active = False
                        depth_hold_started_at = 0.0
                    elif hold_elapsed_s >= config.filters.depth_hold_max_s:
                        depth_hold_active = False
                        depth_hold_started_at = 0.0
                    else:
                        depth_norm = held_depth_norm
                        filters["depth"].reset(held_depth_norm)

                if not depth_hold_active:
                    depth_norm = filters["depth"].update(signals.depth_norm)
                    if (
                        depth_hold_ready
                        and signals.finger_curl_norm >= config.filters.depth_hold_curl_threshold
                    ):
                        depth_hold_active = True
                        depth_hold_ready = False
                        depth_hold_started_at = now
                        held_depth_norm = depth_norm
                        filters["depth"].reset(held_depth_norm)
                else:
                    depth_norm = held_depth_norm

                grip_norm = filters["grip"].update(raw_grip_norm)
                hand_active = signals.inside_region
            else:
                x_norm = filters["x"].value if filters["x"].value is not None else 0.0
                height_norm = filters["height"].value if filters["height"].value is not None else 0.5
                depth_norm = filters["depth"].value if filters["depth"].value is not None else 0.5
                grip_norm = filters["grip"].value if filters["grip"].value is not None else current_gripper_open
                hand_active = False
                depth_hold_active = False
                depth_hold_started_at = 0.0

            mode = "H"
            if estop:
                status = "estop hold"
                mode = "S"
                base_command = config.mapping.base_trim_command
            elif now < home_until:
                status = "homing"
                mode = "M"
                base_command = config.mapping.base_trim_command
                current_lower_target_deg = 0.0
                current_middle_target_deg = 0.0
                current_upper_target_deg = 0.0
                current_joints = _home_joints()
                current_gripper_open = config.firmware.startup_gripper_open_fraction
            elif frozen:
                status = "frozen"
                mode = "H"
                base_command = config.mapping.base_trim_command
            elif tracking_ok and hand_active:
                last_valid_time = now
                base_command = clamp(
                    compute_base_command(x_norm, config) + config.mapping.base_trim_command,
                    -1.0,
                    1.0,
                )
                lower_joint_norm, middle_joint_norm, upper_joint_norm = compute_joint_norms(
                    height_norm,
                    depth_norm,
                    config,
                )
                current_lower_target_deg = lower_joint_norm * config.kinematics.q1_max_deg
                current_middle_target_deg = middle_joint_norm * config.kinematics.q2_max_deg
                current_upper_target_deg = upper_joint_norm * config.kinematics.q3_max_deg
                current_gripper_target_open = grip_norm
                is_closing_gripper = (
                    current_gripper_target_open
                    < current_gripper_open - config.filters.grasp_close_deadband
                )
                arm_rate_scale = (
                    config.filters.grasp_arm_slowdown_factor if is_closing_gripper else 1.0
                )
                current_joints = JointState(
                    lower_deg=filters["lower"].update(current_lower_target_deg, dt, arm_rate_scale),
                    middle_deg=filters["middle"].update(current_middle_target_deg, dt, arm_rate_scale),
                    upper_deg=filters["upper"].update(current_upper_target_deg, dt, arm_rate_scale),
                )
                current_gripper_open = filters["gripper"].update(grip_norm, dt)
                status = "teleop grasp-hold" if (is_closing_gripper or depth_hold_active) else "teleop active"
                mode = "A"
            elif (now - last_valid_time) <= config.workspace.lost_hold_timeout_s:
                status = "brief tracking loss hold"
                mode = "H"
                base_command = config.mapping.base_trim_command
            else:
                status = "waiting for active hand"
                mode = "H"
                base_command = config.mapping.base_trim_command
                current_gripper_target_open = current_gripper_open

            current_base_command = base_command
            command = TeleopCommand(
                mode=mode,
                base_command=current_base_command,
                lower_deg=current_joints.lower_deg,
                middle_deg=current_joints.middle_deg,
                upper_deg=current_joints.upper_deg,
                gripper_open=current_gripper_open,
            )
            sent = transport_controller.send(command, force=(mode in {"S", "M"}))
            if transport_controller.connected and config.transport.enabled:
                if config.transport.kind == "serial":
                    transport_status = f"serial {config.serial.port}"
                else:
                    transport_status = f"udp {config.udp.host}:{config.udp.port}"
            elif config.transport.enabled:
                transport_status = (
                    f"transport offline: {transport_controller.last_error}"
                    if transport_controller.last_error
                    else "transport offline"
                )
                if sent is False and mode == "A":
                    status = "teleop active, transport dropped"
            else:
                transport_status = "disabled"

            overlay = draw_overlay(
                frame,
                config,
                current_hand,
                {
                    "status": status,
                    "tracking_ok": tracking_ok,
                    "hand_active": hand_active,
                    "frozen": frozen,
                    "estop": estop,
                    "base_command": current_base_command,
                    "base_trim_command": config.mapping.base_trim_command,
                    "lower_target_deg": current_lower_target_deg,
                    "middle_target_deg": current_middle_target_deg,
                    "upper_target_deg": current_upper_target_deg,
                    "lower_deg": current_joints.lower_deg,
                    "middle_deg": current_joints.middle_deg,
                    "upper_deg": current_joints.upper_deg,
                    "gripper_open": current_gripper_open,
                    "gripper_target_open": current_gripper_target_open,
                    "depth_hold_active": depth_hold_active,
                    "transport_status": transport_status,
                    "x_norm": 0.5 * (x_norm + 1.0),
                    "height_norm": height_norm,
                    "depth_norm": depth_norm,
                    "grip_norm": grip_norm,
                    "finger_curl_norm": signals.finger_curl_norm if tracking_ok else 0.0,
                    "wrist_tilt_delta": signals.wrist_tilt_delta if tracking_ok else 0.0,
                },
            )
            cv2.imshow("Vision Arm Teleop", overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in (ord("f"), ord(" ")):
                frozen = not frozen
            elif key == ord("x"):
                estop = not estop
                if estop:
                    transport_controller.send(
                        TeleopCommand(
                            "S",
                            config.mapping.base_trim_command,
                            current_joints.lower_deg,
                            current_joints.middle_deg,
                            current_joints.upper_deg,
                            current_gripper_open,
                        ),
                        force=True,
                    )
            elif key == ord("h"):
                frozen = True
                home_until = now + 1.0
            elif key == ord("r"):
                config = _reload_config(args.config)
                _apply_runtime_overrides(config, args)
                filters = _build_filters(config)
                transport_controller.close()
                transport_controller = build_transport(config)
                transport_controller.connect()
                tracker.close()
                tracker = HandTracker(config.vision)
            elif key == ord("["):
                config.mapping.base_trim_command = clamp(
                    config.mapping.base_trim_command - 0.005, -0.25, 0.25
                )
                save_config(config)
            elif key == ord("]"):
                config.mapping.base_trim_command = clamp(
                    config.mapping.base_trim_command + 0.005, -0.25, 0.25
                )
                save_config(config)
            elif key == ord("n") and current_hand is not None:
                capture_neutral_reference(config, current_hand)
                save_config(config)
                export_firmware_headers(config)
            elif key == ord("o") and current_hand is not None:
                capture_pinch_reference(config, current_hand, open_reference=True)
                capture_grip_reference(config, current_hand, open_reference=True)
                save_config(config)
            elif key == ord("p") and current_hand is not None:
                capture_pinch_reference(config, current_hand, open_reference=False)
                capture_grip_reference(config, current_hand, open_reference=False)
                save_config(config)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Fatal error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            transport_controller.send(
                TeleopCommand(
                    mode="S",
                    base_command=config.mapping.base_trim_command,
                    lower_deg=current_joints.lower_deg,
                    middle_deg=current_joints.middle_deg,
                    upper_deg=current_joints.upper_deg,
                    gripper_open=current_gripper_open,
                ),
                force=True,
            )
        except Exception:
            pass
        transport_controller.close()
        tracker.close()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
