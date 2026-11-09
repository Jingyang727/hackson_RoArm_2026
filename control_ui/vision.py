"""P1 overhead vision: chessboard plane mapping + yellow ball detection.

No robot commands, serial connections or actuator access anywhere in this module.
Image coordinates use image origin top-left. Grid origin uses the upper-left
inner chessboard corner with +X right and +Y down in the captured image.
This is NOT the robot-base coordinate frame and is NOT full camera intrinsics.
"""
from __future__ import annotations

import math
import cv2
import numpy as np

PATTERN = (9, 6)  # inner corners: columns, rows
MAX_IMAGE_DIM = 4096
MIN_CIRCULARITY = 0.55


class VisionError(ValueError):
    pass


def decode_image(raw: bytes):
    if not raw or len(raw) > 12_000_000:
        raise VisionError("上传图像为空或超过 12 MB")
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise VisionError("图像格式无法识别，请使用 PNG 或 JPEG")
    height, width = image.shape[:2]
    if min(height, width) < 240 or max(height, width) > MAX_IMAGE_DIM:
        raise VisionError("图像像素尺寸不符合要求")
    return image


def find_chessboard(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, PATTERN, flags=flags)
    if not found or corners is None or len(corners) != PATTERN[0] * PATTERN[1]:
        return None, None
    # Use a reproducible board axis direction despite 180-degree orientation.
    grid = corners.reshape(PATTERN[1], PATTERN[0], 2)
    if grid[0, -1, 0] < grid[0, 0, 0]:
        grid = grid[:, ::-1, :]
    if grid[-1, 0, 1] < grid[0, 0, 1]:
        grid = grid[::-1, :, :]
    image_points = np.ascontiguousarray(grid.reshape(-1, 2), dtype=np.float32)
    target_points = np.array(
        [(column, row) for row in range(PATTERN[1]) for column in range(PATTERN[0])],
        dtype=np.float32,
    )
    h, _ = cv2.findHomography(image_points, target_points, method=0)
    if h is None or not np.all(np.isfinite(h)):
        return None, None
    return grid, h


def find_yellow_ball(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Broad enough for pastel foam balls in ambient white-room lighting.
    mask = cv2.inRange(hsv, (15, 40, 50), (45, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = image.shape[:2]
    candidates = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < max(120, width * height * 0.00010) or area > width * height * 0.08:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        ratio = w / max(h, 1)
        if not (0.6 <= ratio <= 1.5):
            continue
        moments = cv2.moments(cnt)
        if not moments["m00"]:
            continue
        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]
        _, radius = cv2.minEnclosingCircle(cnt)
        score = area * circularity
        candidates.append((score, center_x, center_y, radius, x, y, w, h, circularity))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    _, center_x, center_y, radius, x, y, w, h, circularity = candidates[0]
    return {
        "center_px": [round(center_x, 1), round(center_y, 1)],
        "radius_px": round(radius, 1),
        "bbox_px": [int(x), int(y), int(w), int(h)],
        "circularity": round(circularity, 3),
        "near_edge": bool(min(x, y, width - (x + w), height - (y + h)) < 24),
        "candidate_count": len(candidates),
    }


def analyze_image(raw: bytes, square_mm: float | None = None) -> dict:
    if square_mm is not None and (
        isinstance(square_mm, bool) or not math.isfinite(square_mm) or square_mm <= 0 or square_mm > 200
    ):
        raise VisionError("单格尺寸必须是有效的 0～200 mm 正数")
    image = decode_image(raw)
    height, width = image.shape[:2]
    grid, h = find_chessboard(image)
    ball = find_yellow_ball(image)
    warnings = []
    checker = {"detected": grid is not None, "pattern_inner": list(PATTERN)}
    if grid is not None:
        pixel_corners = grid.reshape(-1, 2)
        ideal = np.array(
            [(column, row) for row in range(PATTERN[1]) for column in range(PATTERN[0])],
            dtype=np.float32,
        )
        projected_grid = cv2.perspectiveTransform(
            pixel_corners.reshape(-1, 1, 2), h
        ).reshape(-1, 2)
        error_squares = np.linalg.norm(projected_grid - ideal, axis=1)
        corner_bounding = [
            round(float(np.min(grid[:, :, 0])), 1),
            round(float(np.min(grid[:, :, 1])), 1),
            round(float(np.max(grid[:, :, 0])), 1),
            round(float(np.max(grid[:, :, 1])), 1),
        ]
        checker.update({
            "corners_px": [[round(float(x), 1), round(float(y), 1)] for x, y in pixel_corners],
            "corners_count": int(pixel_corners.shape[0]),
            "inner_bounds_px": corner_bounding,
            "grid_origin_px": [round(float(v), 1) for v in grid[0, 0]],
            "square_size_mm": square_mm,
            "fit_error_grid_median": round(float(np.median(error_squares)), 5),
        })
    else:
        warnings.append("未检测到9×6棋盘格内部角点；检查图案、照明和完整性")
    if ball:
        if ball["near_edge"]:
            warnings.append("黄色球距离画面边界过近，请移向画面内部，避免后续抓取定位不稳定")
        if grid is not None:
            px = np.array([[ball["center_px"]]], dtype=np.float32)
            coordinates = cv2.perspectiveTransform(px, h)[0, 0]
            gx, gy = (float(coordinates[0]), float(coordinates[1]))
            inside = 0 <= gx <= PATTERN[0] - 1 and 0 <= gy <= PATTERN[1] - 1
            ball["board_grid_xy"] = [round(gx, 3), round(gy, 3)]
            ball["inside_inner_board"] = inside
            if square_mm is not None:
                ball["board_mm_xy"] = [round(gx * square_mm, 1), round(gy * square_mm, 1)]
            if not inside:
                warnings.append(
                    "黄色球位于棋盘格内部角点区域之外：坐标是外推估算，不可直接用于机械臂运动"
                )
    else:
        warnings.append("当前未检测到黄色圆形物体，请检查光照、遮挡或HSV阈值")
    warnings.append("尚未建立相机到机械臂基座的变换，检测结果仅供观察，禁止自动抓取")
    return {
        "image": {"width": width, "height": height},
        "checkerboard": checker,
        "ball": {"detected": ball is not None, **ball} if ball else {"detected": False},
        "warnings": warnings,
        "robot_motion_enabled": False,
    }
