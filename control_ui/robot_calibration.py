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


class RobotCalibration:
    def __init__(self):
        self.lock = threading.RLock()
        self.session_id = None
        self.square_mm = None
        self.image_size = None
        self.records = {}

    def start(self, square_mm, image_size):
        value = _finite_number(square_mm, "单格边长")
        if not (0.1 <= value <= 200):
            raise CalibrationError("棋盘单格边长需在0.1～200mm之间")
        if type(image_size) is not list or len(image_size) != 2 or any(type(i) is not int for i in image_size):
            raise CalibrationError("图像尺寸无效")
        if not (240 <= min(image_size) and max(image_size) <= 4096):
            raise CalibrationError("图像尺寸超出范围")
        with self.lock:
            self.session_id = uuid.uuid4().hex
            self.square_mm = value
            self.image_size = image_size.copy()
            self.records = {}
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
                if abs(pitch-baseline_pitch) > math.radians(5) or abs(roll-baseline_roll) > math.radians(5):
                    raise CalibrationError("请将所有标定点的夹爪姿态保持一致（Pitch/Roll差应≤5°）；否则末端工具偏移会影响XY对齐")
                if abs(float(sample[2]) - baseline["robot_xyz_mm"][2]) > 15:
                    raise CalibrationError("各标定点末端Z高度差超过15mm，无法保证是同一工具参考平面")
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
            return self.summary()

    def summary(self):
        with self.lock:
            records = [self.records[p] for p in EXPECTED_POINT_IDS if p in self.records]
            result = {
                "session_id": self.session_id,
                "square_mm": self.square_mm,
                "image_size": self.image_size,
                "records": records,
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
                "fit": result["fit"], "records": result["records"],
                "note": "Planar board XY->robot XY only; Z, TCP and collisions uncalibrated",
            }
            file = default_path()
            file.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            tmp = file.with_suffix(".tmp")
            tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            os.chmod(tmp, 0o600)
            os.replace(tmp, file)
            return {"saved": True, "path": str(file), "result": result}


calibration = RobotCalibration()
