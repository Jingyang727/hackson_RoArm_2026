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


def find_chessboard_near_reference(image, reference):
    """Detect the known board in a SMALL ROI, not the full 1080p frame.

    The saved 54 corners define where the board is expected. This function
    only searches the region; a subsequent difference check still must verify
    that the live corners match the saved coordinates. No XYZ data is changed.
    """
    if not isinstance(reference, dict):
        return None, None
    height, width = image.shape[:2]
    if reference.get("image_size") != [width, height]:
        return None, None
    try:
        points = np.asarray(reference.get("corners_px"), dtype=np.float32)
    except (TypeError, ValueError):
        return None, None
    if points.shape != (54, 2) or not np.isfinite(points).all():
        return None, None
    grid = points.reshape(6, 9, 2)
    spacing = np.r_[
        np.linalg.norm(np.diff(grid, axis=0), axis=2).flatten(),
        np.linalg.norm(np.diff(grid, axis=1), axis=2).flatten(),
    ]
    if len(spacing) == 0 or float(np.median(spacing)) < 5:
        return None, None
    padding = max(22, int(np.percentile(spacing, 90) * 1.8))
    x0 = max(0, int(np.floor(points[:, 0].min())) - padding)
    y0 = max(0, int(np.floor(points[:, 1].min())) - padding)
    x1 = min(width, int(np.ceil(points[:, 0].max())) + padding + 1)
    y1 = min(height, int(np.ceil(points[:, 1].max())) + padding + 1)
    if x1 - x0 < 80 or y1 - y0 < 80:
        return None, None
    detected, _ = find_chessboard(image[y0:y1, x0:x1])
    if detected is None:
        return None, None
    detected = detected + np.array([x0, y0], dtype=np.float32)
    image_points = np.ascontiguousarray(detected.reshape(-1, 2), dtype=np.float32)
    target_points = np.array(
        [(col, row) for row in range(PATTERN[1]) for col in range(PATTERN[0])],
        dtype=np.float32,
    )
    homography, _ = cv2.findHomography(image_points, target_points, method=0)
    if homography is None or not np.isfinite(homography).all():
        return None, None
    return detected, homography


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


def reference_board_tolerance_mm(reference):
    """Convert image displacement to approximate plane displacement.

    The stored 9×6 adjacent-corner spacing is tied to the measured square
    dimension. Keep a finite conservative ceiling; do not mistake px for mm.
    """
    if not isinstance(reference, dict):
        return None
    try:
        pts = np.asarray(reference["corners_px"], dtype=np.float64).reshape(6, 9, 2)
        pitch_mm = float(reference["square_mm"])
        if not (0.1 < pitch_mm <= 200) or not np.isfinite(pts).all():
            return None
        segments = np.concatenate((
            np.linalg.norm(np.diff(pts, axis=0), axis=2).ravel(),
            np.linalg.norm(np.diff(pts, axis=1), axis=2).ravel(),
        ))
        pixel_pitch = float(np.median(segments))
        if pixel_pitch < 5 or not math.isfinite(pixel_pitch):
            return None
        return pixel_pitch / pitch_mm
    except (TypeError, ValueError, KeyError):
        return None


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
    px_per_mm = reference_board_tolerance_mm(reference)
    if px_per_mm is None:
        return None
    if (float(np.median(distances)) / px_per_mm > 8.0 or
        float(np.percentile(distances, 90)) / px_per_mm > 12.0 or
        float(np.linalg.norm(median_shift)) / px_per_mm > 8.0):
        return None
    return {
        "corners": stored.reshape(6, 9, 2).copy(),
        "visible_match_count": len(matches),
        "median_corner_error_px": round(float(np.median(distances)), 2),
        "median_shift_px": [round(float(v), 2) for v in median_shift],
        "reference_verified": True,
    }


def verify_confirmed_empty_board(image, reference, ball=None):
    """Verify a previously captured empty-board image against the current frame.

    The operator explicitly confirms the uncovered board once. Only the
    verified reference corner map is reused, and then only after *fresh*
    image-to-image template checks across the unobstructed checkerboard.
    This cannot command the arm and makes NO safety claim about 3D reachability.
    """
    if not isinstance(reference, dict):
        return None
    height, width = image.shape[:2]
    original = reference.get("baseline_gray")
    if not isinstance(original, np.ndarray) or original.shape != (height, width):
        return None
    if reference.get("image_size") != [width, height]:
        return None
    try:
        stored = np.asarray(reference.get("corners_px"), dtype=np.float32)
    except (ValueError, TypeError):
        return None
    if stored.shape != (54, 2) or not np.isfinite(stored).all():
        return None
    current = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    half = 12
    tolerance = 11
    seen = []
    for index, (cx, cy) in enumerate(stored):
        x, y = int(round(float(cx))), int(round(float(cy)))
        if not (x > half + tolerance and y > half + tolerance and
                x < width - half - tolerance and y < height - half - tolerance):
            continue
        if ball and ball.get("center_px"):
            bx, by = ball["center_px"]
            exclude = max(18.0, float(ball.get("radius_px", 0)) * 1.5)
            if math.hypot(x - bx, y - by) < exclude + half * 1.5:
                continue
        patch = original[y-half:y+half+1, x-half:x+half+1]
        search = current[
            y-half-tolerance:y+half+tolerance+1,
            x-half-tolerance:x+half+tolerance+1,
        ]
        # Uniform/blurred or saturated patches are not independent evidence.
        if patch.std() < 20 or search.std() < 20:
            continue
        scores = cv2.matchTemplate(search, patch, cv2.TM_CCOEFF_NORMED)
        _, score, _, pos = cv2.minMaxLoc(scores)
        dx, dy = pos[0] - tolerance, pos[1] - tolerance
        if score < 0.80 or math.hypot(dx, dy) > 5:
            continue
        seen.append((index, dx, dy, score))
    if len(seen) < 24:
        return None
    quadrants = [0, 0, 0, 0]
    for index, _, _, _ in seen:
        row, col = divmod(index, 9)
        quadrants[int(row >= 3) * 2 + int(col >= 4)] += 1
    if min(quadrants) < 4:
        return None
    offsets = np.asarray([[dx, dy] for _, dx, dy, _ in seen], dtype=np.float32)
    scores = np.asarray([score for _, _, _, score in seen], dtype=np.float32)
    distances = np.linalg.norm(offsets, axis=1)
    median_shift = np.median(offsets, axis=0)
    px_per_mm = reference_board_tolerance_mm(reference)
    if px_per_mm is None:
        return None
    if (float(np.median(distances)) / px_per_mm > 8.0 or
        float(np.percentile(distances, 90)) / px_per_mm > 12.0 or
        float(np.linalg.norm(median_shift)) / px_per_mm > 8.0 or
        float(np.median(scores)) < 0.88):
        return None
    return {
        "corners": stored.reshape(6, 9, 2).copy(),
        "visible_match_count": len(seen),
        "median_corner_error_px": round(float(np.median(distances)), 2),
        "median_shift_px": [round(float(v), 2) for v in median_shift],
        "reference_verified": True,
        "source": "empty_board_frame_verified",
    }


def confirm_empty_board(raw, square_mm, saved_reference):
    """Operator explicitly captures a ball-free board; read-only operation.

    A full live 9x6 detection is required and must match the *persisted* 4+1
    calibration corners. Nothing here changes the saved robot XY transform.
    """
    if not isinstance(saved_reference, dict):
        raise VisionError("没有已保存且验收通过的4+1点XY标定，不能确认棋盘参考")
    image = decode_image(raw)
    height, width = image.shape[:2]
    if saved_reference.get("image_size") != [width, height]:
        raise VisionError(
            f"摄像头当前分辨率 {width}×{height} 与已保存的"
            f" {saved_reference.get('image_size')} 不同；请选回标定时的相机分辨率"
        )
    if square_mm is None or abs(float(saved_reference.get("square_mm", -1)) - float(square_mm)) > 0.05:
        raise VisionError("棋盘单格尺寸与已保存XY标定不一致，请恢复原始单格尺寸")
    # Full-frame exhaustive SB on a largely empty 1080p image can take a
    # very long time. The saved reference already tells us where to look.
    grid, _ = find_chessboard_near_reference(image, saved_reference)
    if grid is None:
        raise VisionError("未看到完整的9×6内部角点。先拿开黄球，保持棋盘完整入镜，再点击确认棋盘")
    stored=np.asarray(saved_reference.get("corners_px"),dtype=np.float32)
    live=grid.reshape(-1,2)
    if stored.shape!=(54,2) or not np.isfinite(stored).all():
        raise VisionError("已保存的54角点参考无效，不得使用")
    diffs=np.linalg.norm(stored-live,axis=1)
    median=float(np.median(diffs))
    p90=float(np.percentile(diffs,90))
    px_per_mm=reference_board_tolerance_mm(saved_reference)
    if px_per_mm is None:
        raise VisionError("无法确定棋盘像素比例，不能验证已保存的位置")
    median_mm=median/px_per_mm
    p90_mm=p90/px_per_mm
    # The previous hard-coded 5px/8px had no physical meaning. Allow a small
    # measured alignment drift for board RECOGNITION; actual motor execution
    # keeps a separate, stricter gate.
    if median_mm>8 or p90_mm>12:
        raise VisionError(
            f"棋盘与保存位置的差异约为：中位{median_mm:.1f}mm，"
            f"90分位{p90_mm:.1f}mm。超过确认阈值（8/12mm）。"
            "请核对相机、棋盘或底座是否移动；旧XY文件仍保留"
        )
    visible_ball=find_yellow_ball(image)
    if visible_ball:
        bx,by=visible_ball["center_px"]
        corners=grid.reshape(-1,2)
        if (min(corners[:,0])<=bx<=max(corners[:,0]) and
                min(corners[:,1])<=by<=max(corners[:,1])):
            raise VisionError("黄球还在棋盘内部，先取走黄球再确认空棋盘")
    reference={
        "image_size":[width,height],
        "square_mm":float(square_mm),
        "corners_px":saved_reference["corners_px"],
        "baseline_gray":cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),
    }
    return reference,{
        "confirmed":True,
        "image_size":[width,height],
        "square_mm":float(square_mm),
        "corners_count":54,
        "median_reference_difference_px":round(median,2),
        "median_reference_difference_mm":round(median_mm,2),
        "p90_reference_difference_mm":round(p90_mm,2),
        "motion_stability_confirmed":median_mm<=4 and p90_mm<=6,
        "message":"空棋盘已确认。现在把黄球放回棋盘，点击下方一键识别和轨迹预览；不用重新机械臂五点标定",
    }


def analyze_image(raw: bytes, square_mm: float | None = None, reference=None) -> dict:
    if square_mm is not None and (
        isinstance(square_mm, bool) or not math.isfinite(square_mm) or square_mm <= 0 or square_mm > 200
    ):
        raise VisionError("单格尺寸必须是有效的 0～200 mm 正数")
    image = decode_image(raw)
    height, width = image.shape[:2]
    ball = find_yellow_ball(image)
    warnings = []
    grid, h, partial = None, None, None
    same_reference = (
        isinstance(reference, dict) and
        square_mm is not None and
        abs(float(reference.get("square_mm", -1)) - square_mm) < 0.05 and
        reference.get("image_size") == [width, height]
    )
    if same_reference and ball:
        # Never do costly EXHAUSTIVE full-frame chessboard detection on an
        # occluded ball frame. Verify observable evidence from known corners.
        partial = verify_confirmed_empty_board(image, reference, ball)
        if partial is None:
            partial = verify_visible_reference_corners(image, reference, ball)
        if partial:
            grid = partial["corners"]
            image_points = grid.reshape(-1, 2).astype(np.float32)
            targets = np.asarray(
                [(c, r) for r in range(PATTERN[1]) for c in range(PATTERN[0])],
                dtype=np.float32,
            )
            h, _ = cv2.findHomography(image_points, targets, method=0)
            if h is None or not np.isfinite(h).all():
                grid, partial, h = None, None, None
    elif same_reference:
        # An empty scene should be searched only around the saved chessboard,
        # and re-check its orientation/position before accepting the mapping.
        grid, h = find_chessboard_near_reference(image, reference)
        if grid is not None:
            stored = np.asarray(reference["corners_px"], dtype=np.float32)
            delta = np.linalg.norm(grid.reshape(-1, 2) - stored, axis=1)
            scale = reference_board_tolerance_mm(reference)
            if (scale is None or
                float(np.median(delta)) / scale > 8 or
                float(np.percentile(delta, 90)) / scale > 12):
                grid, h = None, None
    else:
        grid, h = find_chessboard(image)
    checker = {
        "detected": grid is not None,
        "pattern_inner": list(PATTERN),
        "source": partial.get("source", "reference_partial_verified") if partial else ("live_54" if grid is not None else "not_detected"),
    }
    if partial:
        checker.update({k: v for k, v in partial.items() if k != "corners"})
        px_per_mm = reference_board_tolerance_mm(reference)
        if px_per_mm:
            # This is independent live image evidence, not a blindly copied
            # set of saved coordinates. Require low drift for motor eligibility.
            dx, dy = partial.get("median_shift_px", [0.0, 0.0])
            shift_mm = math.hypot(dx, dy) / px_per_mm
            checker["live_reference_shift_mm"] = round(shift_mm, 2)
            checker["motion_stable"] = shift_mm <= 4.0
        warnings.append(
            "黄球遮挡部分角点；已使用"
            f"{partial['visible_match_count']}个实时可见棋盘角点核对保存位置，"
            "可以计算球的棋盘毫米坐标。相机或棋盘移动后需重新确认无球棋盘"
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
            inner = 0 <= gx <= PATTERN[0] - 1 and 0 <= gy <= PATTERN[1] - 1
            # An ordinary 9x6 INNER-corner grid has one surrounding ring
            # of physical squares. We may report the ball's table-plane
            # projection there, but this is OUTSIDE trained four-point hull.
            on_sheet = -1 <= gx <= PATTERN[0] and -1 <= gy <= PATTERN[1]
            ball["board_grid_xy"] = [round(gx, 3), round(gy, 3)]
            ball["inside_inner_board"] = inner
            ball["inside_board_sheet"] = on_sheet
            if square_mm is not None and on_sheet:
                ball["board_mm_xy"] = [round(gx * square_mm, 1), round(gy * square_mm, 1)]
            if not on_sheet:
                warnings.append(
                    "球心位于当前棋盘可确认的纸面范围之外，不能可靠地"
                    "将像素外推为桌面毫米坐标；需要额外标定桌面工作区"
                )
            elif not inner:
                warnings.append(
                    "球心位于棋盘最外圈格子：可以定位及查看准备结果，"
                    "但四点标定区域外的真实抓取需要额外验证"
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
