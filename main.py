from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import cv2

from calibration import (
    capture_neutral_reference,
    export_firmware_headers,
)
from config import AppConfig, load_config, save_config
from filters import LowPassFilter, SlewRateLimiter, clamp
from hand_mapping import compute_base_velocity_deg_per_s, compute_joint_norms, extract_signals
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
        "yaw_rate": LowPassFilter(config.filters.yaw_rate_alpha, 0.0),
        "base": SlewRateLimiter(config.firmware.base_slew_deg_s, float(config.firmware.base_home_deg)),
        "lower": SlewRateLimiter(config.filters.joint_rate_deg_s, 0.0),
        "middle": SlewRateLimiter(config.filters.joint_rate_deg_s, 0.0),
        "upper": SlewRateLimiter(config.filters.joint_rate_deg_s, 0.0),
        "gripper": SlewRateLimiter(
            config.filters.gripper_rate_per_s,
            config.firmware.startup_gripper_open_fraction,
        ),
    }

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
    startup_pose_until = 0.0
    wave_started_at = 0.0
    wave_until = 0.0
    transport_status = "disabled" if not config.transport.enabled else "disconnected"

    current_base_target_deg = float(config.firmware.base_home_deg)
    current_base_deg = current_base_target_deg
    current_yaw_rate_deg_s = 0.0
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
            startup_pose_until = home_until + max(0.0, config.control.startup_pose_hold_s)
            if config.control.startup_wave_cycles > 0:
                wave_started_at = startup_pose_until
                wave_until = wave_started_at + (
                    2.0
                    * config.control.startup_wave_half_period_s
                    * config.control.startup_wave_cycles
                )

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
                hand_active = True
            else:
                x_norm = filters["x"].value if filters["x"].value is not None else 0.0
                height_norm = filters["height"].value if filters["height"].value is not None else 0.5
                depth_norm = filters["depth"].value if filters["depth"].value is not None else 0.5
                grip_norm = filters["grip"].value if filters["grip"].value is not None else current_gripper_open
                hand_active = False
                depth_hold_active = False
                depth_hold_started_at = 0.0
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)

            mode = "H"
            if estop:
                status = "estop hold"
                mode = "S"
                current_base_target_deg = current_base_deg
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)
            elif now < home_until:
                status = "homing"
                mode = "M"
                current_base_target_deg = float(config.firmware.base_home_deg)
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)
                current_lower_target_deg = 0.0
                current_middle_target_deg = 0.0
                current_upper_target_deg = 0.0
                current_joints = _home_joints()
                current_gripper_open = config.firmware.startup_gripper_open_fraction
                current_gripper_target_open = current_gripper_open
            elif now < startup_pose_until:
                status = "startup pose"
                mode = "A"
                current_base_target_deg = float(config.firmware.base_home_deg)
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)
                current_lower_target_deg = clamp(
                    config.control.startup_wave_lower_deg,
                    0.0,
                    config.kinematics.q1_max_deg,
                )
                current_middle_target_deg = clamp(
                    config.control.startup_wave_middle_deg,
                    0.0,
                    config.kinematics.q2_max_deg,
                )
                current_upper_target_deg = 0.0
                current_joints = JointState(
                    lower_deg=filters["lower"].update(current_lower_target_deg, dt),
                    middle_deg=filters["middle"].update(current_middle_target_deg, dt),
                    upper_deg=filters["upper"].update(current_upper_target_deg, dt),
                )
                current_gripper_target_open = config.firmware.startup_gripper_open_fraction
                current_gripper_open = filters["gripper"].update(current_gripper_target_open, dt)
            elif now < wave_until:
                status = "startup hello"
                mode = "A"
                current_base_target_deg = float(config.firmware.base_home_deg)
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)
                current_lower_target_deg = clamp(
                    config.control.startup_wave_lower_deg,
                    0.0,
                    config.kinematics.q1_max_deg,
                )
                current_middle_target_deg = clamp(
                    config.control.startup_wave_middle_deg,
                    0.0,
                    config.kinematics.q2_max_deg,
                )
                wave_half_period_s = max(0.05, config.control.startup_wave_half_period_s)
                wave_phase = (now - wave_started_at) / wave_half_period_s
                wave_cycle = wave_phase % 2.0
                wave_fraction = wave_cycle if wave_cycle <= 1.0 else 2.0 - wave_cycle
                current_upper_target_deg = clamp(
                    wave_fraction * config.control.startup_wave_upper_deg,
                    0.0,
                    config.kinematics.q3_max_deg,
                )
                current_joints = JointState(
                    lower_deg=filters["lower"].update(current_lower_target_deg, dt),
                    middle_deg=filters["middle"].update(current_middle_target_deg, dt),
                    upper_deg=filters["upper"].update(current_upper_target_deg, dt),
                )
                current_gripper_target_open = clamp(1.0 - wave_fraction, 0.0, 1.0)
                current_gripper_open = filters["gripper"].update(current_gripper_target_open, dt)
            elif frozen:
                status = "frozen"
                mode = "H"
                current_base_target_deg = current_base_deg
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)
            elif tracking_ok and hand_active:
                last_valid_time = now
                raw_yaw_rate_deg_s = compute_base_velocity_deg_per_s(x_norm, config)
                current_yaw_rate_deg_s = filters["yaw_rate"].update(raw_yaw_rate_deg_s)
                if abs(current_yaw_rate_deg_s) < config.filters.yaw_stop_rate_deg_s:
                    current_yaw_rate_deg_s = 0.0
                    filters["yaw_rate"].reset(0.0)
                current_base_target_deg = clamp(
                    current_base_target_deg + current_yaw_rate_deg_s * dt,
                    float(config.firmware.base_min_deg),
                    float(config.firmware.base_max_deg),
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
                current_base_target_deg = current_base_deg
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)
            else:
                status = "waiting for tracked hand"
                mode = "H"
                current_gripper_target_open = current_gripper_open
                current_base_target_deg = current_base_deg
                current_yaw_rate_deg_s = 0.0
                filters["yaw_rate"].reset(0.0)

            teleop_live = tracking_ok and mode == "A" and now >= wave_until
            current_base_deg = filters["base"].update(current_base_target_deg, dt)
            command = TeleopCommand(
                mode=mode,
                base_deg=current_base_deg,
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
                    "hand_active": teleop_live,
                    "frozen": frozen,
                    "estop": estop,
                    "yaw_max_rate_deg_s": config.mapping.yaw_max_rate_deg_s,
                    "base_target_deg": current_base_target_deg,
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
                            current_base_deg,
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
            elif key == ord("["):
                config.mapping.yaw_max_rate_deg_s = clamp(
                    config.mapping.yaw_max_rate_deg_s - 5.0,
                    5.0,
                    180.0,
                )
                save_config(config)
            elif key == ord("]"):
                config.mapping.yaw_max_rate_deg_s = clamp(
                    config.mapping.yaw_max_rate_deg_s + 5.0,
                    5.0,
                    180.0,
                )
                save_config(config)
            elif key == ord("n") and current_hand is not None:
                capture_neutral_reference(config, current_hand)
                save_config(config)
                export_firmware_headers(config)

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
                    base_deg=current_base_deg,
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
