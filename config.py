from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Type, TypeVar, get_args, get_origin, get_type_hints


T = TypeVar("T")


@dataclass
class RegionConfig:
    x_min: float = 0.08
    y_min: float = 0.08
    x_max: float = 0.92
    y_max: float = 0.92

    @property
    def width(self) -> float:
        return max(1e-6, self.x_max - self.x_min)

    @property
    def height(self) -> float:
        return max(1e-6, self.y_max - self.y_min)


@dataclass
class CameraConfig:
    device_index: int = 0
    frame_width: int = 1280
    frame_height: int = 720
    fps: int = 60
    mirror_view: bool = True


@dataclass
class VisionConfig:
    max_num_hands: int = 1
    min_detection_confidence: float = 0.45
    min_tracking_confidence: float = 0.35
    handedness: str = "either"
    active_region: RegionConfig = field(default_factory=RegionConfig)
    neutral_x: float = 0.5
    neutral_y: float = 0.55
    depth_reference: float = 0.18
    wrist_tilt_reference: float = 0.0
    grip_open_reference: float = 0.2
    grip_closed_reference: float = 0.75
    pinch_open_reference: float = 0.95
    pinch_closed_reference: float = 0.28
    min_palm_width_norm: float = 0.015


@dataclass
class MappingConfig:
    control_mode: str = "direct_joint"
    yaw_deadband: float = 0.12
    yaw_exponent: float = 1.5
    yaw_max_command: float = 1.0
    yaw_invert: bool = False
    base_trim_command: float = 0.0
    depth_to_reach_sign: float = 1.0
    depth_gain: float = 1.3
    lower_depth_invert: bool = True
    middle_height_invert: bool = False
    upper_height_invert: bool = False
    middle_average_weight: float = 0.5
    upper_tilt_sign: float = -1.0
    upper_tilt_gain: float = 1.8
    upper_tilt_deadband: float = 0.06
    gripper_exponent: float = 1.0
    gripper_invert: bool = False
    gripper_full_open_threshold: float = 0.03
    gripper_full_close_threshold: float = 0.5


@dataclass
class WorkspaceConfig:
    target_r_min_mm: float = 70.0
    target_r_max_mm: float = 185.0
    target_z_min_mm: float = 35.0
    target_z_max_mm: float = 185.0
    default_r_mm: float = 105.0
    default_z_mm: float = 155.0
    lost_hold_timeout_s: float = 0.35


@dataclass
class FilterConfig:
    x_alpha: float = 0.45
    y_alpha: float = 0.35
    depth_alpha: float = 0.24
    grip_alpha: float = 0.34
    reach_rate_mm_s: float = 130.0
    height_rate_mm_s: float = 160.0
    joint_rate_deg_s: float = 70.0
    gripper_rate_per_s: float = 2.3
    grasp_arm_slowdown_factor: float = 0.2
    grasp_close_deadband: float = 0.02
    depth_hold_curl_threshold: float = 0.45
    depth_hold_release_curl_threshold: float = 0.28
    depth_hold_max_s: float = 6.0


@dataclass
class KinematicsConfig:
    shoulder_r_offset_mm: float = 10.2
    shoulder_z_offset_mm: float = 55.37
    l1_mm: float = 65.92
    l2_mm: float = 66.42
    l3_mm: float = 62.5
    q1_min_deg: float = 0.0
    q1_max_deg: float = 45.0
    q2_min_deg: float = 0.0
    q2_max_deg: float = 60.0
    q3_min_deg: float = 0.0
    q3_max_deg: float = 90.0
    tool_angle_min_deg: float = -90.0
    tool_angle_max_deg: float = 90.0
    tool_angle_step_deg: float = 2.0
    preferred_tool_angle_deg: float = 55.0
    orientation_preference_weight: float = 0.08
    continuity_weight: float = 0.35


@dataclass
class SerialConfig:
    enabled: bool = True
    port: str = "COM5"
    baudrate: int = 115200
    timeout_s: float = 0.02
    warmup_s: float = 2.0
    write_hz: float = 60.0
    open_retries: int = 8
    retry_delay_s: float = 0.5


@dataclass
class TransportConfig:
    kind: str = "serial"
    enabled: bool = True


@dataclass
class UdpConfig:
    enabled: bool = True
    host: str = "192.168.137.50"
    port: int = 4210
    local_port: int = 0
    timeout_s: float = 0.02
    write_hz: float = 60.0


@dataclass
class ControlConfig:
    freeze_on_start: bool = True
    send_home_on_start: bool = False
    overlay_scale: float = 1.0


@dataclass
class FirmwareCalibrationConfig:
    base_stop_deg: int = 90
    base_speed_range_deg: int = 28
    base_direction_sign: int = 1
    base_input_deadband: float = 0.05
    lower_zero_deg: int = 0
    lower_sign: int = 1
    lower_servo_min_deg: int = 0
    lower_servo_max_deg: int = 180
    middle_zero_deg: int = 0
    middle_sign: int = 1
    middle_servo_min_deg: int = 0
    middle_servo_max_deg: int = 180
    upper_zero_deg: int = 0
    upper_sign: int = 1
    upper_servo_min_deg: int = 0
    upper_servo_max_deg: int = 180
    gripper_open_servo_deg: int = 180
    gripper_closed_servo_deg: int = 0
    startup_gripper_open_fraction: float = 0.5
    watchdog_timeout_ms: int = 250
    servo_slew_deg_s: float = 90.0
    base_slew_units_per_s: float = 2.5


@dataclass
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)
    filters: FilterConfig = field(default_factory=FilterConfig)
    kinematics: KinematicsConfig = field(default_factory=KinematicsConfig)
    transport: TransportConfig = field(default_factory=TransportConfig)
    serial: SerialConfig = field(default_factory=SerialConfig)
    udp: UdpConfig = field(default_factory=UdpConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    firmware: FirmwareCalibrationConfig = field(default_factory=FirmwareCalibrationConfig)
    config_path: str = ""


def _convert_value(field_type: Any, value: Any) -> Any:
    origin = get_origin(field_type)
    if origin is None and is_dataclass(field_type):
        return _from_dict(field_type, value or {})
    if origin in (list, tuple):
        args = get_args(field_type)
        item_type = args[0] if args else Any
        return [_convert_value(item_type, item) for item in value]
    return value


def _from_dict(cls: Type[T], values: dict[str, Any]) -> T:
    kwargs: dict[str, Any] = {}
    type_hints = get_type_hints(cls)
    for current_field in fields(cls):
        if current_field.name not in values:
            continue
        field_type = type_hints.get(current_field.name, current_field.type)
        kwargs[current_field.name] = _convert_value(field_type, values[current_field.name])
    return cls(**kwargs)


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    config = _from_dict(AppConfig, raw)
    config.config_path = str(config_path)
    return config


def save_config(config: AppConfig, path: str | Path | None = None) -> None:
    target = Path(path or config.config_path)
    payload = asdict(config)
    payload.pop("config_path", None)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
