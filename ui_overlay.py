from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

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


CONSOLAS_REGULAR_PATH = Path("C:/Windows/Fonts/consola.ttf")
CONSOLAS_BOLD_PATH = Path("C:/Windows/Fonts/consolab.ttf")


@lru_cache(maxsize=8)
def _load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    font_path = CONSOLAS_BOLD_PATH if bold else CONSOLAS_REGULAR_PATH
    try:
        return ImageFont.truetype(str(font_path), size=size)
    except OSError:
        return ImageFont.load_default()


def _draw_text(
    draw: ImageDraw.ImageDraw,
    text: str,
    origin: tuple[int, int],
    *,
    color: tuple[int, int, int],
    size: int,
    bold: bool = False,
) -> None:
    draw.text(
        origin,
        text,
        font=_load_font(size, bold),
        fill=color,
        stroke_width=2,
        stroke_fill=(16, 18, 24),
    )


def _composite_rgba(frame: np.ndarray, overlay_rgba: np.ndarray, x: int, y: int) -> None:
    overlay_h, overlay_w = overlay_rgba.shape[:2]
    frame_h, frame_w = frame.shape[:2]
    if overlay_h <= 0 or overlay_w <= 0 or x >= frame_w or y >= frame_h:
        return

    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(frame_w, x + overlay_w)
    y1 = min(frame_h, y + overlay_h)
    if x1 <= x0 or y1 <= y0:
        return

    overlay_slice = overlay_rgba[(y0 - y):(y1 - y), (x0 - x):(x1 - x)]
    alpha = overlay_slice[:, :, 3:4].astype(np.float32) / 255.0
    if not np.any(alpha):
        return

    overlay_bgr = overlay_slice[:, :, :3][:, :, ::-1].astype(np.float32)
    frame_roi = frame[y0:y1, x0:x1].astype(np.float32)
    blended = overlay_bgr * alpha + frame_roi * (1.0 - alpha)
    frame[y0:y1, x0:x1] = blended.astype(np.uint8)


def _render_text_overlay(
    size: tuple[int, int],
    title: str,
    lines: list[str],
) -> np.ndarray:
    width, height = size
    image = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)

    _draw_text(draw, title, (0, 0), color=(121, 220, 205), size=18, bold=True)

    y = 26
    for line in lines:
        _draw_text(draw, line, (0, y), color=(232, 236, 244), size=14)
        y += 17

    return np.array(image, dtype=np.uint8)


def _render_bar_labels(width: int, labels: list[tuple[str, int]]) -> np.ndarray:
    image = Image.new("RGBA", (width, 24), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    for text, x in labels:
        _draw_text(draw, text, (x, 0), color=(230, 235, 242), size=13, bold=True)
    return np.array(image, dtype=np.uint8)


def _draw_bar(frame: np.ndarray, value: float, x: int, y: int, width: int) -> None:
    cv2.rectangle(frame, (x, y), (x + width, y + 12), (92, 102, 122), 1, cv2.LINE_AA)
    fill = int(max(0.0, min(1.0, value)) * width)
    cv2.rectangle(frame, (x, y), (x + fill, y + 12), (64, 202, 174), -1, cv2.LINE_AA)
    cv2.circle(frame, (x + fill, y + 6), 5, (240, 250, 255), -1, cv2.LINE_AA)


def draw_overlay(
    frame: np.ndarray,
    config: AppConfig,
    observation: HandObservation | None,
    runtime: dict[str, object],
) -> np.ndarray:
    display = frame.copy()
    height, width = display.shape[:2]

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
        f"Yaw rate deg/s: {runtime.get('yaw_max_rate_deg_s', 0.0): .1f}",
        (
            "Pitch targets deg: "
            f"{runtime.get('lower_target_deg', 0.0):.1f}, "
            f"{runtime.get('middle_target_deg', 0.0):.1f}, "
            f"{runtime.get('upper_target_deg', 0.0):.1f}"
        ),
        f"Base target deg: {runtime.get('base_target_deg', 0.0): .1f}",
        f"Gripper open: {runtime.get('gripper_open', 0.0):.2f}",
        f"Finger curl: {runtime.get('finger_curl_norm', 0.0):.2f}",
        f"Depth hold: {'on' if runtime.get('depth_hold_active', False) else 'off'}",
        f"Transport: {runtime.get('transport_status', 'unknown')}",
        "Keys: q quit | f freeze | x e-stop | h home | n neutral | [ ] yaw rate",
    ]

    hud_width = min(width - 28, 520)
    hud_height = 26 + len(lines) * 17
    hud_overlay = _render_text_overlay((hud_width, hud_height), "hand teleop", lines)
    _composite_rgba(display, hud_overlay, 24, 24)

    bar_y = height - 58
    bar_width = 220
    bar_specs = [
        ("X", 28, float(runtime.get("x_norm", 0.5))),
        ("Height", 286, float(runtime.get("height_norm", 0.5))),
        ("Depth", 544, float(runtime.get("depth_norm", 0.5))),
        ("Grip", 802, float(runtime.get("grip_norm", 0.0))),
    ]
    label_overlay = _render_bar_labels(
        width,
        [(label, x) for label, x, _value in bar_specs],
    )
    _composite_rgba(display, label_overlay, 0, bar_y - 24)

    for _label, x, value in bar_specs:
        _draw_bar(display, value, x, bar_y, bar_width)

    return display
