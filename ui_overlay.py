from __future__ import annotations

import cv2
import numpy as np

from config import AppConfig
from vision import HandObservation


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def _draw_bar(frame: np.ndarray, label: str, value: float, x: int, y: int, width: int) -> None:
    cv2.putText(frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.rectangle(frame, (x, y), (x + width, y + 10), (70, 70, 70), 1)
    fill = int(max(0.0, min(1.0, value)) * width)
    cv2.rectangle(frame, (x, y), (x + fill, y + 10), (40, 200, 120), -1)


def draw_overlay(
    frame: np.ndarray,
    config: AppConfig,
    observation: HandObservation | None,
    runtime: dict[str, object],
) -> np.ndarray:
    display = frame.copy()
    height, width = display.shape[:2]
    region = config.vision.active_region

    x0 = int(region.x_min * width)
    y0 = int(region.y_min * height)
    x1 = int(region.x_max * width)
    y1 = int(region.y_max * height)
    region_color = (0, 210, 120) if runtime.get("hand_active", False) else (110, 110, 110)
    cv2.rectangle(display, (x0, y0), (x1, y1), region_color, 2)

    neutral_px = (int(config.vision.neutral_x * width), int(config.vision.neutral_y * height))
    cv2.drawMarker(display, neutral_px, (255, 180, 0), cv2.MARKER_CROSS, 18, 2)

    if observation is not None:
        pts = observation.pixel_landmarks
        for a, b in HAND_CONNECTIONS:
            cv2.line(display, tuple(pts[a]), tuple(pts[b]), (255, 170, 60), 2, cv2.LINE_AA)
        for point in pts:
            cv2.circle(display, tuple(point), 3, (60, 220, 255), -1, cv2.LINE_AA)

    lines = [
        f"Status: {runtime.get('status', 'idle')}",
        f"Tracking: {'yes' if runtime.get('tracking_ok', False) else 'no'}",
        f"Teleop: {'active' if runtime.get('hand_active', False) else 'hold'}",
        f"Freeze: {'on' if runtime.get('frozen', False) else 'off'}",
        f"E-stop: {'on' if runtime.get('estop', False) else 'off'}",
        f"Base cmd: {runtime.get('base_command', 0.0): .2f}",
        f"Base neutral trim: {runtime.get('base_trim_command', 0.0): .2f}",
        (
            "Joint targets deg: "
            f"{runtime.get('lower_target_deg', 0.0):.1f}, "
            f"{runtime.get('middle_target_deg', 0.0):.1f}, "
            f"{runtime.get('upper_target_deg', 0.0):.1f}"
        ),
        (
            "Joints deg: "
            f"{runtime.get('lower_deg', 0.0):.1f}, "
            f"{runtime.get('middle_deg', 0.0):.1f}, "
            f"{runtime.get('upper_deg', 0.0):.1f}"
        ),
        f"Gripper open: {runtime.get('gripper_open', 0.0):.2f}",
        f"Finger curl: {runtime.get('finger_curl_norm', 0.0):.2f}",
        f"Wrist tilt: {runtime.get('wrist_tilt_delta', 0.0): .2f}",
        f"Depth hold: {'on' if runtime.get('depth_hold_active', False) else 'off'}",
        f"Transport: {runtime.get('transport_status', 'unknown')}",
        "Keys: q quit | f freeze | x e-stop | h home | n neutral | o open hand | p closed hand | r reload | [ ] base trim",
    ]

    y = 26
    for line in lines:
        cv2.putText(display, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (240, 240, 240), 1, cv2.LINE_AA)
        y += 22

    _draw_bar(display, "X", float(runtime.get("x_norm", 0.5)), 14, height - 70, 180)
    _draw_bar(display, "Height", float(runtime.get("height_norm", 0.5)), 214, height - 70, 180)
    _draw_bar(display, "Depth", float(runtime.get("depth_norm", 0.5)), 414, height - 70, 180)
    _draw_bar(display, "Grip", float(runtime.get("grip_norm", 0.0)), 614, height - 70, 180)
    return display
