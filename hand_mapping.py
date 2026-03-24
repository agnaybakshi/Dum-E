from __future__ import annotations

from dataclasses import dataclass

from config import AppConfig
from filters import apply_signed_deadband, clamp
from vision import HandObservation


@dataclass
class TeleopSignals:
    x_offset_norm: float
    height_norm: float
    depth_norm: float
    lower_joint_norm: float
    middle_joint_norm: float
    upper_joint_norm: float
    gripper_open: float
    finger_curl_norm: float
    wrist_tilt_delta: float
    inside_region: bool
    region_xy: tuple[float, float]


def _inside_region(x: float, y: float, cfg: AppConfig) -> bool:
    region = cfg.vision.active_region
    return region.x_min <= x <= region.x_max and region.y_min <= y <= region.y_max


def _normalize_region(x: float, y: float, cfg: AppConfig) -> tuple[float, float]:
    region = cfg.vision.active_region
    region_x = clamp((x - region.x_min) / region.width, 0.0, 1.0)
    region_y = clamp((y - region.y_min) / region.height, 0.0, 1.0)
    return region_x, region_y


def extract_signals(observation: HandObservation, cfg: AppConfig) -> TeleopSignals:
    center_x, center_y = observation.center_xy
    region_x, region_y = _normalize_region(center_x, center_y, cfg)
    x_offset = (center_x - cfg.vision.neutral_x) / max(cfg.vision.active_region.width * 0.5, 1e-6)
    x_offset = clamp(x_offset, -1.0, 1.0)
    if cfg.mapping.yaw_invert:
        x_offset *= -1.0

    height_norm = 1.0 - region_y

    depth_ratio = observation.depth_metric / max(cfg.vision.depth_reference, 1e-6)
    depth_norm = clamp(
        0.5 + cfg.mapping.depth_to_reach_sign * (depth_ratio - 1.0) * cfg.mapping.depth_gain,
        0.0,
        1.0,
    )

    curl_span = max(
        cfg.vision.grip_closed_reference - cfg.vision.grip_open_reference,
        1e-6,
    )
    finger_curl_norm = clamp(
        (observation.finger_curl_metric - cfg.vision.grip_open_reference) / curl_span,
        0.0,
        1.0,
    )
    full_open_threshold = clamp(cfg.mapping.gripper_full_open_threshold, 0.0, 0.95)
    full_close_threshold = clamp(
        cfg.mapping.gripper_full_close_threshold,
        full_open_threshold + 0.01,
        1.0,
    )
    proportional_span = max(full_close_threshold - full_open_threshold, 1e-6)
    if finger_curl_norm <= full_open_threshold:
        gripper_open = 1.0
    elif finger_curl_norm >= full_close_threshold:
        gripper_open = 0.0
    else:
        gripper_open = clamp(
            1.0 - (finger_curl_norm - full_open_threshold) / proportional_span,
            0.0,
            1.0,
        )
        gripper_open = gripper_open ** max(cfg.mapping.gripper_exponent, 1e-6)
    if cfg.mapping.gripper_invert:
        gripper_open = 1.0 - gripper_open

    wrist_tilt_delta = observation.wrist_tilt_metric - cfg.vision.wrist_tilt_reference

    lower_joint_norm, middle_joint_norm, upper_joint_norm = compute_joint_norms(
        height_norm,
        depth_norm,
        cfg,
    )
    return TeleopSignals(
        x_offset_norm=x_offset,
        height_norm=height_norm,
        depth_norm=depth_norm,
        lower_joint_norm=lower_joint_norm,
        middle_joint_norm=middle_joint_norm,
        upper_joint_norm=upper_joint_norm,
        gripper_open=gripper_open,
        finger_curl_norm=finger_curl_norm,
        wrist_tilt_delta=wrist_tilt_delta,
        inside_region=_inside_region(center_x, center_y, cfg),
        region_xy=(region_x, region_y),
    )


def compute_base_command(x_offset_norm: float, cfg: AppConfig) -> float:
    value = apply_signed_deadband(x_offset_norm, cfg.mapping.yaw_deadband)
    magnitude = abs(value) ** cfg.mapping.yaw_exponent
    command = magnitude * cfg.mapping.yaw_max_command
    return command if value >= 0.0 else -command


def compute_joint_norms(height_norm: float, depth_norm: float, cfg: AppConfig) -> tuple[float, float, float]:
    lower_joint_norm = depth_norm if not cfg.mapping.lower_depth_invert else 1.0 - depth_norm
    upper_joint_norm = height_norm if not cfg.mapping.upper_height_invert else 1.0 - height_norm
    middle_weight = clamp(cfg.mapping.middle_average_weight, 0.0, 1.0)
    middle_joint_norm = clamp(
        middle_weight * upper_joint_norm + (1.0 - middle_weight) * lower_joint_norm,
        0.0,
        1.0,
    )
    return (
        clamp(lower_joint_norm, 0.0, 1.0),
        middle_joint_norm,
        clamp(upper_joint_norm, 0.0, 1.0),
    )


def map_workspace(depth_norm: float, height_norm: float, cfg: AppConfig) -> tuple[float, float]:
    workspace = cfg.workspace
    target_r = workspace.target_r_min_mm + depth_norm * (
        workspace.target_r_max_mm - workspace.target_r_min_mm
    )
    target_z = workspace.target_z_min_mm + height_norm * (
        workspace.target_z_max_mm - workspace.target_z_min_mm
    )
    return target_r, target_z
