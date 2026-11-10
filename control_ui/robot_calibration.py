"""Measured chessboard-mm -> RoArm-M3 XY calibration, with a held-out checkpoint.

A planar mapping is NOT collision avoidance, a Z calibration, or a validated
tool-center-point (TCP) offset. This module never sends motor commands.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

TORQUE_KEYS = tuple("torswitch" + n for n in "BSETRG")
EXPECTED_POINT_IDS = ("P1", "P2", "P3", "P4", "V1")
TRAINING_IDS = ("P1", "P2", "P3", "P4")
MIN_TRAINING_SAMPLES = 4
TRAIN_RMS_LIMIT_MM = 6.0
TRAIN_MAX_LIMIT_MM = 9.0
VALIDATION_LIMIT_MM = 10.0
STILL_XYZ_LIMIT_MM = 2.0
STILL_JOINT_LIMIT_DEG = 0.8


class CalibrationError(ValueError):
    pass


def _finite_number(x, label):
    if type(x) not in (float, int) or not math.isfinite(x):
        raise CalibrationError(f"{label} 必须是有效数字")
    return float(x)


def _robot_xyz(feedback):
    if not isinstance(feedback, dict) or feedback.get("T") != 1051:
        raise CalibrationError("没有有效的 T=1051 状态")
    return np.array([_finite_number(feedback.get(i), "机械臂" + i) for i in "xyz"])


def fit_board_to_robot(records):
    """Rigid orthogonal planar mapping. Reflection is permitted: image +Y is down.

    Result: [robot_x,robot_y]^T = R @ [board_x,board_y]^T + t.
    Distances are preserved because the grid square dimension is measured in mm.
    A sign-flipped determinant is valid for camera conventions (not a robot fault).
    """
    training = [r for r in records if r["id"] in TRAINING_IDS]
    if len(training) < MIN_TRAINING_SAMPLES:
        raise CalibrationError("至少需要4个标定点")
    b = np.array([r["board_mm"] for r in training], dtype=np.float64)
    a = np.array([r["robot_xyz_mm"][:2] for r in training], dtype=np.float64)
    if not (np.isfinite(b).all() and np.isfinite(a).all()):
        raise CalibrationError("坐标包含非法值")
    centered = b - b.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    if singular[1] < 10.0 or singular[0] / singular[1] > 12:
        raise CalibrationError("四个标定点过于接近或近似共线，请扩大并分散标定范围")
    u, _, vt = np.linalg.svd((a - a.mean(axis=0)).T @ centered)
    matrix = u @ vt  # det +/-1: image axes can be mirrored w.r.t robot axes
    offset = a.mean(axis=0) - matrix @ b.mean(axis=0)
    predicted = b @ matrix.T + offset
    errors = np.linalg.norm(predicted - a, axis=1)
    result = {
        "matrix_2x2": [[float(v) for v in row] for row in matrix],
        "offset_mm": [float(v) for v in offset],
        "determinant": round(float(np.linalg.det(matrix)), 4),
        "train_rms_mm": round(float(np.sqrt(np.mean(errors ** 2))), 2),
        "train_max_mm": round(float(np.max(errors)), 2),
        "train_point_errors_mm": {r["id"]: round(float(e), 2)
                                  for r, e in zip(training, errors)},
        "validation_error_mm": None,
        "validated": False,
        "z_calibrated": False,
        "tcp_calibrated": False,
        "auto_move_enabled": False,
    }
    validation = next((r for r in records if r["id"] == "V1"), None)
    if validation is not None:
        target = np.asarray(validation["board_mm"], dtype=np.float64)
        observed = np.asarray(validation["robot_xyz_mm"][:2], dtype=np.float64)
        result["validation_error_mm"] = round(float(np.linalg.norm(matrix @ target + offset - observed)), 2)
    result["validated"] = bool(
        len(training) >= 4 and validation is not None
        and result["train_rms_mm"] <= TRAIN_RMS_LIMIT_MM
        and result["train_max_mm"] <= TRAIN_MAX_LIMIT_MM
        and result["validation_error_mm"] <= VALIDATION_LIMIT_MM
    )
    return result


def default_path():
    return Path.home() / ".evoarm" / "robot_xy_calibration.json"


def session_path():
    """Separate per-point draft so a process restart never destroys P1–V1."""
    return Path.home() / ".evoarm" / "robot_xy_session.json"


def _write_json_atomic(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    except (OSError, ValueError, TypeError) as exc:
        raise CalibrationError(f"标定数据写入磁盘失败，不能保证断电后恢复：{exc}") from exc


def _check_restored_records(raw, square_mm):
    if not isinstance(raw, list) or len(raw) > 5:
        raise CalibrationError("磁盘标定点列表格式错误")
    restored = {}
    locations = set()
    for row in raw:
        if not isinstance(row, dict) or row.get("id") not in EXPECTED_POINT_IDS:
            raise CalibrationError("磁盘标定点 ID 不合法")
        key = row["id"]
        if key in restored:
            raise CalibrationError("磁盘标定点包含重复 ID")
        grid = row.get("grid")
        if (not isinstance(grid, list) or len(grid) != 2 or
            any(type(n) is not int for n in grid) or
            not (0 <= grid[0] <= 8 and 0 <= grid[1] <= 5)):
            raise CalibrationError("磁盘标定点网格位置无效")
        if tuple(grid) in locations:
            raise CalibrationError("磁盘标定点有重复角点")
        locations.add(tuple(grid))
        board = row.get("board_mm")
        xyz = row.get("robot_xyz_mm")
        pose = row.get("tool_pose_rad")
        if (not isinstance(board, list) or len(board) != 2 or
            not isinstance(xyz, list) or len(xyz) != 3 or
            not isinstance(pose, list) or len(pose) != 2):
            raise CalibrationError("磁盘标定点坐标格式无效")
        for name, values in (("棋盘", board), ("机械臂", xyz), ("工具姿态", pose)):
            for n in values:
                _finite_number(n, name)
        if any(abs(board[i] - grid[i] * square_mm) > 0.05 for i in range(2)):
            raise CalibrationError("磁盘标定点与单格尺寸不一致")
        restored[key] = row
    return restored


class RobotCalibration:
    def __init__(self):
        self.lock = threading.RLock()
        self.session_id = None
        self.square_mm = None
        self.image_size = None
        self.corners_px = None
        self.records = {}
        self.saved = False
        self.restored_from = None
        self._restore_existing()

    def _restore_existing(self):
        """Recover exact prior draft; otherwise restore validated final file.

        Never overwrite a saved calibration with an incomplete new session.
        Reject malformed JSON rather than trusting it for motor motion.
        """
        with self.lock:
            for candidate, is_final in ((session_path(), False), (default_path(), True)):
                if not candidate.exists():
                    continue
                try:
                    raw = json.loads(candidate.read_text(encoding="utf-8"))
                    if not isinstance(raw, dict):
                        raise CalibrationError("磁盘标定文件必须是对象")
                    scale = _finite_number(raw.get("square_mm"), "磁盘单格尺寸")
                    dims = raw.get("image_size")
                    if not 0.1 <= scale <= 200 or not isinstance(dims, list) or len(dims) != 2 or any(type(v) is not int or v < 240 or v > 4096 for v in dims):
                        raise CalibrationError("磁盘棋盘尺寸或图像尺寸无效")
                    corners = raw.get("corners_px")
                    if corners is not None:
                        if not isinstance(corners, list) or len(corners) != 54:
                            raise CalibrationError("保存的54个角点不完整")
                        for point in corners:
                            if (not isinstance(point, list) or len(point) != 2 or
                                any(type(n) not in (int, float) or not math.isfinite(n) for n in point) or
                                not (0 <= point[0] < dims[0] and 0 <= point[1] < dims[1])):
                                raise CalibrationError("保存的角点像素坐标无效")
                    records = _check_restored_records(raw.get("records"), scale)
                    if is_final and (len(records) != 5 or
                                     not fit_board_to_robot(list(records.values()))["validated"]):
                        raise CalibrationError("已保存标定结果未通过重新校验")
                except (OSError, ValueError, KeyError, TypeError, CalibrationError):
                    # Do not delete or overwrite the unreadable file.
                    continue
                self.session_id = raw.get("session_id") if (not is_final and isinstance(raw.get("session_id"), str) and len(raw["session_id"]) <= 128) else uuid.uuid4().hex
                self.square_mm = scale
                self.image_size = dims.copy()
                self.corners_px = corners
                self.records = records
                self.saved = bool(is_final or raw.get("saved", False))
                self.restored_from = str(candidate)
                return

    def _persist_session(self):
        if not self.session_id:
            return
        _write_json_atomic(session_path(), {
            "version": 2,
            "session_id": self.session_id,
            "square_mm": self.square_mm,
            "image_size": self.image_size,
            "corners_px": self.corners_px,
            "records": [self.records[k] for k in EXPECTED_POINT_IDS if k in self.records],
            "saved": self.saved,
            "updated_at": datetime.now(timezone.utc).isoformat(),
        })

    def start(self, square_mm, image_size, corners_px=None):
        value = _finite_number(square_mm, "单格边长")
        if not (0.1 <= value <= 200):
            raise CalibrationError("棋盘单格边长需在0.1～200mm之间")
        if type(image_size) is not list or len(image_size) != 2 or any(type(i) is not int for i in image_size):
            raise CalibrationError("图像尺寸无效")
        if not (240 <= min(image_size) and max(image_size) <= 4096):
            raise CalibrationError("图像尺寸超出范围")
        if corners_px is not None:
            if (not isinstance(corners_px, list) or len(corners_px) != 54 or
                any(not isinstance(p, list) or len(p) != 2 for p in corners_px)):
                raise CalibrationError("棋盘角点必须包含54组像素坐标")
            validated_corners = []
            for p in corners_px:
                x = _finite_number(p[0], "角点X")
                y = _finite_number(p[1], "角点Y")
                if not 0 <= x < image_size[0] or not 0 <= y < image_size[1]:
                    raise CalibrationError("有棋盘角点位于图像边界之外")
                validated_corners.append([round(x,2),round(y,2)])
        else:
            validated_corners = None
        with self.lock:
            self.session_id = uuid.uuid4().hex
            self.square_mm = value
            self.image_size = image_size.copy()
            self.corners_px = validated_corners
            self.records = {}
            self.saved = False
            self.restored_from = None
            self._persist_session()
            return self.summary()

    def _session(self, token):
        if not self.session_id or token != self.session_id:
            raise CalibrationError("标定会话不匹配，请重新开始并重新测量标记")

    def capture(self, token, point_id, col, row, arm_service, *, sleep_fn=time.sleep):
        if point_id not in EXPECTED_POINT_IDS:
            raise CalibrationError("未知的标定点")
        if type(col) is not int or type(row) is not int or not (0 <= col <= 8 and 0 <= row <= 5):
            raise CalibrationError("必须使用棋盘9×6内部角点")
        with self.lock:
            self._session(token)
            if any((r["grid"] == [col, row] and r["id"] != point_id) for r in self.records.values()):
                raise CalibrationError("每个标定点必须是不同的棋盘角点")
            # Hold the serial owner's lock for the entire stable sample pair,
            # preventing a concurrent jog or torque transition mid-measurement.
            with arm_service.lock:
                health = arm_service.health()
                if not health["connected"] or health["pending"] or health["fault"]:
                    raise CalibrationError("机械臂未连接，或仍有待完成动作/异常")
                # Only read motor feedback. Never generate any motion command.
                first = arm_service.status()
                sleep_fn(0.65)
                second = arm_service.status()
            abc = np.stack((_robot_xyz(first), _robot_xyz(second)))
            delta = float(np.linalg.norm(abc[1] - abc[0]))
            from .safety import current_degrees
            j1, j2 = current_degrees(first), current_degrees(second)
            joint_delta = max(abs(j1[k] - j2[k]) for k in j1)
            if delta > STILL_XYZ_LIMIT_MM or joint_delta > STILL_JOINT_LIMIT_DEG:
                raise CalibrationError(f"机械臂反馈仍在变化（XYZ {delta:.1f}mm，关节 {joint_delta:.1f}°），先等静止")
            torque = [second.get(key) for key in TORQUE_KEYS]
            if any(v not in (0, 1) for v in torque):
                raise CalibrationError("无法确认扭矩状态，拒绝记录")
            if len(set(torque)) != 1:
                raise CalibrationError("关节扭矩状态不一致，拒绝记录")
            sample = np.mean(abc, axis=0)
            if float(np.max(np.abs(sample))) > 2000:
                raise CalibrationError("机械臂 XYZ 反馈超出合理范围")
            pitch = float(np.mean([
                _finite_number(first.get("tit"), "末端Pitch(tit)"),
                _finite_number(second.get("tit"), "末端Pitch(tit)")
            ]))
            roll = float(np.mean([
                _finite_number(first.get("r"), "末端Roll(r)"),
                _finite_number(second.get("r"), "末端Roll(r)")
            ]))
            other = [r for key, r in self.records.items() if key != point_id]
            if other:
                baseline = other[0]
                baseline_pitch, baseline_roll = baseline["tool_pose_rad"]
                pitch_diff = round(math.degrees(abs(pitch - baseline_pitch)), 1)
                roll_diff = round(math.degrees(abs(roll - baseline_roll)), 1)
                if pitch_diff > 5 or roll_diff > 5:
                    raise CalibrationError(
                        f"末端姿态与 {baseline['id']} 不一致：Pitch 相差 {pitch_diff}°，"
                        f"Roll 相差 {roll_diff}°（上限均为5°）。"
                        "请重新调整到和首次记录相同的夹爪朝向，再点击此点重录；已成功记录的其他点不会丢失"
                    )
                z_diff = round(abs(float(sample[2]) - baseline["robot_xyz_mm"][2]), 1)
                if z_diff > 15:
                    raise CalibrationError(
                        f"末端高度与 {baseline['id']} 相差 {z_diff} mm（上限15mm）。"
                        "请保持与首次标定相近的夹爪高度，再重新记录；其他已成功点会保留"
                    )
            self.records[point_id] = {
                "id": point_id,
                "role": "validate" if point_id == "V1" else "train",
                "grid": [col, row],
                "board_mm": [round(col * self.square_mm, 3), round(row * self.square_mm, 3)],
                "robot_xyz_mm": [round(float(v), 3) for v in sample],
                "torque_locked": bool(torque[0]),
                "tool_pose_rad": [round(pitch, 5), round(roll, 5)],
                "captured_at": datetime.now(timezone.utc).isoformat(),
            }
            self.saved = False
            self._persist_session()
            result = self.summary()
            # The final V1 record must survive even if the user cannot reach
            # a Save button due to layout. Only auto-save a validated 4+1 fit.
            if result["ready"]:
                self.save(token)
                result = self.summary()
            return result

    def summary(self):
        with self.lock:
            records = [self.records[p] for p in EXPECTED_POINT_IDS if p in self.records]
            result = {
                "session_id": self.session_id,
                "square_mm": self.square_mm,
                "image_size": self.image_size,
                "corners_saved": self.corners_px is not None,
                "records": records,
                "saved": self.saved,
                "restored_from": self.restored_from,
                "saved_path": str(default_path()) if self.saved else None,
                "next_id": next((p for p in EXPECTED_POINT_IDS if p not in self.records), None),
                "ready": False,
                "fit": None,
                "message": "",
            }
            if not self.session_id:
                result["message"] = "还未开始机器人XY标定"
                return result
            if len([r for r in records if r["role"] == "train"]) < 4:
                result["message"] = "请记录4个分散的标定角点"
                return result
            try:
                fit = fit_board_to_robot(records)
            except CalibrationError as error:
                result["message"] = str(error)
                return result
            result["fit"] = fit
            result["ready"] = fit["validated"]
            if fit["validation_error_mm"] is None:
                result["message"] = "训练点已拟合；还需要1个独立验证点 V1"
            elif fit["validated"]:
                result["message"] = "二维棋盘坐标 → 机械臂XY标定验收通过；仍未完成工具TCP和Z高度标定"
            else:
                result["message"] = "验收误差未达标，请检查对准点、夹爪中心及棋盘是否移动，然后重测"
            return result

    def save(self, token):
        with self.lock:
            self._session(token)
            result = self.summary()
            if not result["ready"]:
                raise CalibrationError("未达到4标定+1独立验证及误差门槛，无法保存")
            data = {
                "version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
                "square_mm": self.square_mm, "image_size": self.image_size,
                "corners_px": self.corners_px,
                "fit": result["fit"], "records": result["records"],
                "note": "Planar board XY->robot XY only; Z, TCP and collisions uncalibrated",
            }
            file = default_path()
            _write_json_atomic(file, data)
            self.saved = True
            self._persist_session()
            return {"saved": True, "path": str(file), "result": result}


calibration = RobotCalibration()
