"""Layout-specific, resumable board/robot calibration without taught task poses.

All controller access is GET-only. Alignment is performed by the local operator;
this module never opens serial, unlocks, changes torque or sends motor commands.
Four points fit a metric board-plane -> firmware-reference mapping; V1 is held
out. A passed planar fit is not a verified RGB-D extrinsic or grasping TCP.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
from pathlib import Path
import time
import uuid
from urllib import error

import numpy as np

from control_ui.robot_calibration import (
    CalibrationError, STILL_JOINT_LIMIT_DEG, STILL_XYZ_LIMIT_MM, TORQUE_KEYS,
    TRAIN_RMS_LIMIT_MM, TRAIN_MAX_LIMIT_MM, VALIDATION_LIMIT_MM,
    _finite_number, _robot_xyz, _write_json_atomic,
)
from control_ui.safety import current_degrees
from .real_probe import _base_url, _controller_get
from .task_memory import valid_id

POINTS = {"P1": [1, 1], "P2": [7, 1], "P3": [7, 4], "P4": [1, 4], "V1": [4, 2]}
SAMPLE_KIND = "stationary_robot_feedback_v1"
SESSION_KIND = "bench_calibration_session_v1"


def _idle(health, *, allow_manual=False):
    if (health.get("connected") is not True or health.get("fault") is not None
            or health.get("pending") is not None
            or (not allow_manual and health.get("teach_active") is not False)):
        raise CalibrationError("需要已连接、无故障且无待完成运动的控制器；手动采点需持续支撑")


def validate_sample(sample, *, allow_manual=False):
    if not isinstance(sample, dict) or sample.get("kind") != SAMPLE_KIND:
        raise CalibrationError("需要新版控制器的静止采样接口，请核对实际运行的服务代码")
    rows = sample.get("samples")
    if not isinstance(rows, list) or len(rows) != 3:
        raise CalibrationError("需要三次真实反馈")
    stamps = [row["host_monotonic_ns"] for row in rows]
    if any(type(s) is not int for s in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise CalibrationError("反馈时间戳无效")
    if stamps[-1] - stamps[0] < 600_000_000:
        raise CalibrationError("反馈采样时间过短")
    statuses = [row["robot_status"] for row in rows]
    xyz = np.stack([_robot_xyz(s) for s in statuses])
    angles = np.asarray([list(current_degrees(s).values()) for s in statuses])
    spread = max(float(np.linalg.norm(a-b)) for a in xyz for b in xyz)
    if spread > STILL_XYZ_LIMIT_MM or float(np.max(np.ptp(angles, axis=0))) > STILL_JOINT_LIMIT_DEG:
        raise CalibrationError("机械臂反馈未静止；未记录此点，不会重试运动")
    torque = [[s.get(k) for k in TORQUE_KEYS] for s in statuses]
    accepted = ([1]*6, [0]*6) if allow_manual else ([1]*6,)
    if torque[0] not in accepted or any(row != torque[0] for row in torque):
        raise CalibrationError("扭矩状态不完整、不一致或采样期间发生变化；未记录此点")
    if np.max(np.abs(xyz)) > 2000:
        raise CalibrationError("机械臂XYZ反馈超出合理范围")
    poses = np.asarray([[_finite_number(s.get(k), k) for k in ("tit", "r", "g")]
                        for s in statuses])
    return {"robot_xyz_mm": np.mean(xyz, axis=0).tolist(),
            "tool_pose_rad": dict(zip(("pitch", "roll", "gripper"), np.mean(poses, axis=0).tolist())),
            "xyz_spread_mm": spread, "torque_locked": torque[0] == [1]*6,
            "operator_support_required": torque[0] == [0]*6}


def stationary_sample(arm_service, *, sleep_fn=time.sleep, allow_manual=False):
    """Hold the serial owner's lock across all three samples, excluding jogs."""
    with arm_service.lock:
        _idle(arm_service.health(), allow_manual=allow_manual)
        rows = []
        for index in range(3):
            if index:
                sleep_fn(.35)
            status = arm_service.status()
            rows.append({"host_monotonic_ns": time.monotonic_ns(), "robot_status": status})
            _idle(arm_service.health(), allow_manual=allow_manual)
        result = {"kind": SAMPLE_KIND, "samples": rows,
                  "source": "T1051_encoder_forward_kinematics", "motion_commands_sent": False}
        result.update(validate_sample(result, allow_manual=allow_manual))
        return result


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load_session(path):
    path = Path(path).expanduser()
    session = json.loads(path.read_text(encoding="utf-8"))
    if session.get("kind") != SESSION_KIND:
        raise CalibrationError("不是新布置的标定会话")
    valid_id(session["setup_id"])
    if _digest(session["reference"]) != session["reference_sha256"]:
        raise CalibrationError("棋盘参考已改变；请为改变后的布置创建新会话")
    if session["points"] != POINTS:
        raise CalibrationError("五点计划与本次采点图不匹配")
    return path, session


@contextmanager
def _session_lock(path):
    # Two local invocations must not silently discard each other's points.
    with Path(path).expanduser().with_suffix(".lock").open("a") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise CalibrationError("该会话正在采样，请等当前采点结束") from exc
        try:
            yield
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def _overlay(session, path):
    import cv2
    cap = cv2.VideoCapture(str(Path(session["reference_trial"]) / "rgb.mp4"))
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, session["reference"]["source_frame_indices"][0])
        ok, frame = cap.read()
    finally:
        cap.release()
    if not ok:
        raise CalibrationError("无法读取原录像生成采点图")
    corners = session["reference"]["corners_px"]
    for key, grid in session["points"].items():
        x, y = [round(v) for v in corners[grid[1]*9+grid[0]]]
        color = (0, 200, 255) if key == "V1" else (30, 30, 240)
        cv2.circle(frame, (x, y), 9, color, 2)
        cv2.putText(frame, f"{key} ({grid[0]},{grid[1]})", (x+10, y-9),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    if not cv2.imwrite(str(path), frame):
        raise CalibrationError("采点图保存失败")


def start_session(trial, *, setup_id, square_mm, output):
    from .board_reference import create_reference
    valid_id(setup_id)
    path = Path(output).expanduser()
    if path.exists():
        raise CalibrationError("会话已经存在；使用 status/capture 继续，不覆盖已记录点")
    path.parent.mkdir(parents=True, exist_ok=True)
    # Reference extraction writes only into this new session's directory.
    reference_path = path.with_name(path.stem + "-reference.json")
    if reference_path.exists():
        raise CalibrationError("候选参考已存在；请使用新的会话文件名")
    reference = create_reference(trial, output=reference_path, square_mm=square_mm)
    reference.pop("output_file", None)
    session = {"kind": SESSION_KIND, "setup_id": setup_id, "session_id": uuid.uuid4().hex,
               "created_at": datetime.now(timezone.utc).isoformat(),
               "reference_trial": str(Path(trial).expanduser().resolve()),
               "reference": reference, "reference_sha256": _digest(reference),
               "points": {k: v.copy() for k, v in POINTS.items()}, "records": {}, "history": [],
               "tool_reference": "waveshare_stock_firmware_endpoint",
               "alignment": "same_endpoint_on_board_surface_without_pressure",
               "motion_authorized": False}
    _overlay(session, path.with_suffix(".png"))
    _write_json_atomic(path, session)
    return session_status(path)


def capture_point(path, point_id, *, aligned=False, controller="http://127.0.0.1:8765", replace=False,
                  arm_service=None, allow_manual=False):
    if aligned is not True:
        raise CalibrationError("先现场对准图中角点，再以 --aligned 声明已对准；程序不会移动机械臂")
    with _session_lock(path):
        with arm_service.lock if arm_service is not None else nullcontext():
            return _capture_point(path, point_id, controller=controller, replace=replace,
                                  arm_service=arm_service, allow_manual=allow_manual)


def _capture_point(path, point_id, *, controller, replace, arm_service, allow_manual):
    path, session = load_session(path)
    if point_id not in POINTS:
        raise CalibrationError("未知采点ID")
    if point_id in session["records"] and not replace:
        raise CalibrationError("该点已记录；重采需 --replace，旧原始记录仍保留")
    base = _base_url(controller)
    try:
        sample = (stationary_sample(arm_service, allow_manual=allow_manual) if arm_service is not None else
                  _controller_get(base, "/api/calibration/stationary-sample"))
    except error.HTTPError as exc:
        if exc.code == 404:
            raise CalibrationError("当前服务未加载新采样接口；需从新worktree重启实际8765控制服务") from exc
        raw = exc.read(8192).decode("utf-8", errors="replace")
        try:
            detail = str(json.loads(raw).get("detail", raw))
        except (ValueError, AttributeError):
            detail = raw
        raise CalibrationError(f"静止采样HTTP {exc.code}: {detail[:400]}") from exc
    measured = validate_sample(sample, allow_manual=allow_manual)
    warnings = []
    others = [r for k, r in session["records"].items() if k != point_id]
    if others:
        baseline = others[0]["tool_pose_rad"]
        if any(abs(math.degrees(math.remainder(measured["tool_pose_rad"][key]-baseline[key], 2*math.pi))) > 5
               for key in ("roll", "gripper")):
            raise CalibrationError("末端旋转或夹爪开合与已有点相差超过5°，请核对是否仍使用同一个触点；其他点保留")
        pitch_delta = abs(math.degrees(math.remainder(measured["tool_pose_rad"]["pitch"]-baseline["pitch"], 2*math.pi)))
        if pitch_delta > 5:
            warnings.append(f"俯仰与 {others[0]['id']} 相差 {pitch_delta:.1f}°；已保留真实XYZ，可尽量保持一致")
    grid = session["points"][point_id]
    record = {"id": point_id, "grid": grid, "role": "validate" if point_id == "V1" else "train",
              "board_mm": [v*session["reference"]["square_mm"] for v in grid],
              "pixel_xy": session["reference"]["corners_px"][grid[1]*9+grid[0]],
              "captured_at": datetime.now(timezone.utc).isoformat(),
              "operator_confirmed_endpoint_on_board": True, **measured, "pose_warnings": warnings, "raw_feedback": sample}
    # Save every raw sample before updating the resumable session. Never erase
    # an earlier attempt, even when --replace is explicitly requested.
    archive = path.parent / (path.stem + "-samples") / (point_id + "-" + uuid.uuid4().hex + ".json")
    _write_json_atomic(archive, record)
    if point_id in session["records"]:
        session["history"].append(session["records"][point_id])
    session["records"][point_id] = record
    session.pop("fit", None)
    _write_json_atomic(path, session)
    return session_status(path)


def fit_points(records):
    """Metric rigid 2D plane -> 3D firmware frame, with V1 excluded from fit."""
    if set(records) != set(POINTS):
        raise CalibrationError("需要4个拟合点和独立V1；缺失点不能用推算值代替")
    train = [records[k] for k in POINTS if k != "V1"]
    board = np.asarray([r["board_mm"] + [0.] for r in train], dtype=float)
    robot = np.asarray([r["robot_xyz_mm"] for r in train], dtype=float)
    heldout = records["V1"]
    if board.shape != (4, 3) or robot.shape != (4, 3) or not np.isfinite(board).all() or not np.isfinite(robot).all():
        raise CalibrationError("采点坐标无效")
    bc, rc = board-board.mean(axis=0), robot-robot.mean(axis=0)
    singular = np.linalg.svd(bc, compute_uv=False)
    if singular[1] < 10 or singular[0]/singular[1] > 12:
        raise CalibrationError("采点范围过小或近似共线")
    u, _, vt = np.linalg.svd(rc.T @ bc)
    rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    offset = robot.mean(axis=0) - rotation @ board.mean(axis=0)
    errors = np.linalg.norm(board @ rotation.T + offset - robot, axis=1)
    vb = np.asarray(heldout["board_mm"] + [0.], dtype=float)
    vr = np.asarray(heldout["robot_xyz_mm"], dtype=float)
    if vb.shape != (3,) or vr.shape != (3,) or not np.isfinite(vb).all() or not np.isfinite(vr).all():
        raise CalibrationError("独立验证点坐标无效")
    validation = float(np.linalg.norm(rotation @ vb + offset - vr))
    rms, largest = float(np.sqrt(np.mean(errors**2))), float(np.max(errors))
    normal = rotation[:, 2].copy()
    if normal[2] < 0:
        normal = -normal
    passed = rms <= TRAIN_RMS_LIMIT_MM and largest <= TRAIN_MAX_LIMIT_MM and validation <= VALIDATION_LIMIT_MM
    return {"board_to_robot_rotation_3x3": rotation.tolist(), "board_to_robot_translation_mm": offset.tolist(),
            "train_rms_mm": rms, "train_max_mm": largest, "validation_error_mm": validation,
            "train_errors_mm": dict(zip((r["id"] for r in train), errors.tolist())),
            "table_plane_robot": {"normal_up": normal.tolist(), "d_mm": -float(normal @ offset)},
            "planar_fit_validated": bool(passed), "motion_authorized": False,
            "scope": "board_plane_firmware_endpoint",
            "camera_to_robot_verified": False, "grasp_tcp_verified": False}


def session_status(path):
    path, session = load_session(path)
    points = [{"id": key, "grid": grid, "captured": key in session["records"],
               "robot_xyz_mm": session["records"].get(key, {}).get("robot_xyz_mm"),
               "pose_warnings": session["records"].get(key, {}).get("pose_warnings", [])}
              for key, grid in session["points"].items()]
    return {"setup_id": session["setup_id"], "session": str(path), "alignment_image": str(path.with_suffix(".png")),
            "points": points, "next_point": next((p["id"] for p in points if not p["captured"]), None),
            "reference_trial_id": session["reference"]["source_trial_id"], "fit": session.get("fit"),
            "motion_authorized": False}


def fit_session(path):
    with _session_lock(path):
        path, session = load_session(path)
        result = {"kind": "bench_calibration_candidate_v1", "setup_id": session["setup_id"],
                  "session_id": session["session_id"], "reference_sha256": session["reference_sha256"],
                  "source_session": str(path), **fit_points(session["records"])}
        output = path.parent/("candidate-"+uuid.uuid4().hex+".json")
        _write_json_atomic(output, result)
        session["fit"] = {**result, "output_file": str(output)}
        _write_json_atomic(path, session)
        return session_status(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    init = actions.add_parser("init")
    init.add_argument("reference_trial")
    init.add_argument("--setup-id", required=True)
    init.add_argument("--square-mm", required=True, type=float)
    init.add_argument("--output", required=True)
    for name in ("status", "capture", "fit"):
        action = actions.add_parser(name)
        action.add_argument("--session", required=True)
        if name == "capture":
            action.add_argument("--point", choices=POINTS, required=True)
            action.add_argument("--aligned", action="store_true")
            action.add_argument("--replace", action="store_true")
            action.add_argument("--controller", default="http://127.0.0.1:8765")
        if name == "fit":
            action.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        if args.action == "init":
            result = start_session(args.reference_trial, setup_id=args.setup_id,
                                   square_mm=args.square_mm, output=args.output)
        elif args.action == "capture":
            result = capture_point(args.session, args.point, aligned=args.aligned,
                                   controller=args.controller, replace=args.replace)
        elif args.action == "status":
            result = session_status(args.session)
        else:
            path, session = load_session(args.session)
            output = Path(args.output).expanduser()
            if output.exists():
                raise CalibrationError("候选结果已存在；请使用新的输出文件名，不覆盖已有标定")
            result = {"kind": "bench_calibration_candidate_v1", "setup_id": session["setup_id"],
                      "session_id": session["session_id"], "reference_sha256": session["reference_sha256"],
                      "source_session": str(path), **fit_points(session["records"])}
            _write_json_atomic(output, result)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        if args.action == "fit" and not result["planar_fit_validated"]:
            return 2
        return 0
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(json.dumps({"error": str(exc), "motion_commands_sent": False}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
