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


def verify_visible_reference_corners(image, reference, ball=None):
    """Independently cross-check visible chessboard intersections from a saved
    54-corner reference when a ball blocks the full chessboard detector.

    This is deliberately NOT 'trust old corners': it requires many fresh,
    spatially distributed live corner features close to the saved locations.
    Unknown camera movement, occlusion or stale references return None.
    Nothing in this module can issue robot motor commands.
    """
    if not isinstance(reference, dict):
        return None
    height, width = image.shape[:2]
    if reference.get("image_size") != [width, height]:
        return None
    raw = reference.get("corners_px")
    try:
        stored = np.asarray(raw, dtype=np.float32)
    except (ValueError, TypeError):
        return None
    if stored.shape != (54, 2) or not np.isfinite(stored).all():
        return None
    if np.any(stored[:, 0] < 16) or np.any(stored[:, 0] >= width - 16):
        return None
    if np.any(stored[:, 1] < 16) or np.any(stored[:, 1] >= height - 16):
        return None
    # Match fresh, live strong corner features to expected inner-corner sites.
    # Mask away the yellow object and nearby pixels to avoid treating the ball
    # edge as a checkerboard intersection.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    roi = np.zeros((height, width), dtype=np.uint8)
    for x, y in stored:
        cv2.circle(roi, (int(round(x)), int(round(y))), 11, 255, -1)
    if ball and ball.get("center_px"):
        bx, by = ball["center_px"]
        radius = max(15, float(ball.get("radius_px", 0)) * 1.4)
        cv2.circle(roi, (int(round(bx)), int(round(by))),
                   int(math.ceil(radius)), 0, -1)
    features = cv2.goodFeaturesToTrack(
        gray, maxCorners=250, qualityLevel=0.04,
        minDistance=5, mask=roi, blockSize=5
    )
    if features is None:
        return None
    features = features.reshape(-1, 2)
    matches = []
    seen = set()
    for index, expected in enumerate(stored):
        delta = np.linalg.norm(features - expected, axis=1)
        nearby = np.flatnonzero(delta <= 9.0)
        if not len(nearby):
            continue
        nearest = min(nearby, key=lambda i: float(delta[i]))
        if int(nearest) in seen:
            continue
        seen.add(int(nearest))
        matches.append((index, features[nearest] - expected, float(delta[nearest])))
    # 32/54 minimum also tolerates a small foam ball masking multiple corners.
    # At least five fresh corners must survive in EACH quarter of the board.
    if len(matches) < 32:
        return None
    quadrants = [0, 0, 0, 0]
    for index, _, _ in matches:
        row, col = divmod(index, 9)
        quadrants[int(row >= 3) * 2 + int(col >= 4)] += 1
    if min(quadrants) < 5:
        return None
    distances = np.asarray([d for _, _, d in matches], dtype=np.float32)
    shifts = np.asarray([offset for _, offset, _ in matches], dtype=np.float32)
    median_shift = np.median(shifts, axis=0)
    # Deliberately much tighter than a movable-camera re-registration.
    if (float(np.median(distances)) > 5.0 or
        float(np.percentile(distances, 90)) > 8.0 or
        float(np.linalg.norm(median_shift)) > 4.0):
        return None
    return {
        "corners": stored.reshape(6, 9, 2).copy(),
        "visible_match_count": len(matches),
        "median_corner_error_px": round(float(np.median(distances)), 2),
        "median_shift_px": [round(float(v), 2) for v in median_shift],
        "reference_verified": True,
    }


def analyze_image(raw: bytes, square_mm: float | None = None, reference=None) -> dict:
    if square_mm is not None and (
        isinstance(square_mm, bool) or not math.isfinite(square_mm) or square_mm <= 0 or square_mm > 200
    ):
        raise VisionError("单格尺寸必须是有效的 0～200 mm 正数")
    image = decode_image(raw)
    height, width = image.shape[:2]
    grid, h = find_chessboard(image)
    ball = find_yellow_ball(image)
    warnings = []
    partial = None
    if grid is None and reference is not None:
        # A moving camera or missing live corner evidence must NEVER fall back
        # blindly to the saved transform, even if a file exists.
        if square_mm is not None and abs(float(reference.get("square_mm", -1)) - square_mm) < 0.05:
            partial = verify_visible_reference_corners(image, reference, ball)
        if partial:
            grid = partial["corners"]
            image_points = grid.reshape(-1, 2).astype(np.float32)
            targets = np.asarray(
                [(c, r) for r in range(PATTERN[1]) for c in range(PATTERN[0])],
                dtype=np.float32
            )
            h, _ = cv2.findHomography(image_points, targets, method=0)
            if h is None or not np.isfinite(h).all():
                grid, partial, h = None, None, None
    checker = {
        "detected": grid is not None,
        "pattern_inner": list(PATTERN),
        "source": "reference_partial_verified" if partial else ("live_54" if grid is not None else "not_detected"),
    }
    if partial:
        checker.update({k: v for k, v in partial.items() if k != "corners"})
        warnings.append(
            "完整棋盘识别受到球体遮挡；本帧已用"
            f"{partial['visible_match_count']}个未被遮挡的实时角点核对保存位置。"
            "相机或棋盘移动时必须重新进行完整标定"
        )
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
        warnings.append(
            "本帧未识别到全部54个内角点，也无法安全核实保存的标定参考。"
            "球可能遮挡了棋盘角点；请暂时移开球完成一次棋盘识别，"
            "再把球放回去。无法确认棋盘位置时禁止抓取规划"
        )
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
                    "球心不在9×6内角点围成的安全区域内。"
                    "即使球在棋盘纸最外圈黑白格上，也不代表位于内部标定凸包；"
                    "当前坐标仅供观察，禁止自动抓取"
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
