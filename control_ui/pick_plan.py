"""A reviewable pick-and-place plan in robot coordinates. Does not move hardware.

The arm's XYZ (T=1051) is an end-effector reference, not necessarily the center
between the jaws. XY registration absorbs a constant tool offset only if the
arm orientation is held fixed. Pick Z and gripper closure must be taught and
verified on a real ball before any motion can be authorized.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np

from .robot_calibration import CalibrationError, default_path as xy_calibration_path


class PickPlanningError(ValueError):
    pass


def _num(v, name):
    if type(v) not in (int, float) or not math.isfinite(v):
        raise PickPlanningError(f"{name} 必须是有限数字")
    return float(v)


def _read_file(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError) as exc:
        raise PickPlanningError(f"无法读取标定资料 {path}: {exc}") from exc


def profile_path():
    return Path.home() / ".evoarm" / "pick_profile.json"


def draft_path():
    return Path.home() / ".evoarm" / "pick_profile_draft.json"


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary=path.with_suffix(path.suffix+".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as output:
            json.dump(value, output, ensure_ascii=False, indent=2, allow_nan=False)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    except (OSError, ValueError, TypeError) as exc:
        raise PickPlanningError("示教数据无法持久化到本地磁盘："+str(exc)) from exc


_PICK_FIELDS={
    "grip_z_mm", "place_z_mm", "travel_z_mm", "place_xy_mm",
    "pitch_rad", "roll_rad", "open_g_rad", "close_g_rad",
    "ball_diameter_mm",
}


def save_pick_draft(partial):
    """Persist partial teaching after EACH actual feedback read, not only at the end.

    This is a record of measured values, not authorization for robot motion.
    """
    if not isinstance(partial, dict) or any(k not in _PICK_FIELDS for k in partial):
        raise PickPlanningError("示教草稿包含未知字段")
    clean={}
    for field, value in partial.items():
        if field=="place_xy_mm":
            if not isinstance(value, (list,tuple)) or len(value)!=2:
                raise PickPlanningError("示教放置 XY 格式不正确")
            clean[field]=[_num(x,"放置XY") for x in value]
        else:
            clean[field]=_num(value,field)
    _write_json(draft_path(),clean)
    return {"saved":True,"draft":True,"path":str(draft_path()),"fields":list(clean)}


def load_pick_progress():
    """Prefer unsaved partial draft, else restore completed profile."""
    for source, is_completed in ((draft_path(),False),(profile_path(),True)):
        if not source.exists():
            continue
        try:
            data=_read_file(source)
            if not isinstance(data,dict) or any(key not in _PICK_FIELDS for key in data):
                continue
            cleaned={}
            for key,value in data.items():
                if key=="place_xy_mm":
                    if not isinstance(value,(list,tuple)) or len(value)!=2:
                        raise PickPlanningError("不合法的放置XY草稿")
                    cleaned[key]=[_num(x,"放置XY") for x in value]
                else:
                    cleaned[key]=_num(value,key)
            if is_completed:
                cleaned=validate_pick_profile(cleaned)
            return {"profile":cleaned,"saved":is_completed,"source":str(source)}
        except (PickPlanningError,OSError,TypeError,ValueError):
            continue
    return {"profile":{},"saved":False,"source":None}


def _convex_hull(points):
    points = sorted(set((float(x), float(y)) for x, y in points))
    if len(points) < 3:
        raise PickPlanningError("标定点不足以覆盖一个平面区域")
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _inside_convex_polygon(point, polygon, clearance=0.0):
    # Clearance in mm from all polygon edges, not just vertices.
    if len(polygon) < 3:
        return False
    px, py = point
    for i, a in enumerate(polygon):
        b = polygon[(i+1) % len(polygon)]
        ex, ey = b[0]-a[0], b[1]-a[1]
        length = math.hypot(ex, ey)
        if length < 1e-6:
            return False
        cross = ex*(py-a[1]) - ey*(px-a[0])
        if cross / length < clearance-1e-4:
            return False
    return True


def validate_pick_profile(profile):
    required = ("grip_z_mm", "place_z_mm", "travel_z_mm",
                "place_xy_mm", "pitch_rad", "roll_rad", "open_g_rad",
                "close_g_rad", "ball_diameter_mm")
    if not isinstance(profile, dict) or any(k not in profile for k in required):
        raise PickPlanningError("缺少抓取/放置示教数据：需要抓取Z、放置Z、安全Z、姿态和开合角")
    out = {k: _num(profile[k], k) for k in required if k != "place_xy_mm"}
    xy = profile["place_xy_mm"]
    if not isinstance(xy, (list, tuple)) or len(xy) != 2:
        raise PickPlanningError("放置 XY 格式不正确")
    out["place_xy_mm"] = [_num(v, "放置XY") for v in xy]
    if not (8 <= out["ball_diameter_mm"] <= 90):
        raise PickPlanningError("球直径必须实测，当前仅支持直径 8～90mm 的轻质球")
    if not (40 <= out["grip_z_mm"] <= 350 and 40 <= out["place_z_mm"] <= 350):
        raise PickPlanningError("抓取和放置高度超出保守的示教范围")
    if not (70 <= out["travel_z_mm"] <= 380):
        raise PickPlanningError("安全高度超出保守范围")
    if out["travel_z_mm"] - max(out["grip_z_mm"], out["place_z_mm"]) < 45:
        raise PickPlanningError("安全抬升高度至少应高于抓取/放置高度45mm")
    if not (-math.pi/2 <= out["pitch_rad"] <= math.pi/2 and -math.pi <= out["roll_rad"] <= math.pi):
        raise PickPlanningError("末端姿态范围异常")
    if not (1.08 <= out["open_g_rad"] < out["close_g_rad"] <= 3.14):
        raise PickPlanningError("夹爪角度应满足张开角度小于闭合角度，且在官方范围内")
    if out["close_g_rad"]-out["open_g_rad"] < math.radians(8):
        raise PickPlanningError("夹爪开合差不足8°，请重新示教")
    return out


def _verify_board_unchanged(calibration_data, image_size, corners_px):
    baseline = calibration_data.get("corners_px")
    if baseline is None:
        raise PickPlanningError("旧标定未保存棋盘角点指纹，请重新完成4+1坐标标定")
    if image_size != calibration_data.get("image_size"):
        raise PickPlanningError("摄像头分辨率与标定时不一致，需要重新标定")
    expected = np.asarray(baseline, dtype=float)
    observed = np.asarray(corners_px, dtype=float)
    if expected.shape != (54, 2) or observed.shape != (54, 2):
        raise PickPlanningError("需要当前图像54个棋盘角点以确认相机和棋盘没有移动")
    if not np.isfinite(expected).all() or not np.isfinite(observed).all():
        raise PickPlanningError("棋盘角点坐标无效")
    distances = np.linalg.norm(observed - expected, axis=1)
    rms = float(np.sqrt(np.mean(distances ** 2)))
    max_shift = float(np.max(distances))
    if rms > 6 or max_shift > 13:
        raise PickPlanningError(
            f"当前棋盘位置与标定时不同：角点偏移 RMS={rms:.1f}px，最大={max_shift:.1f}px。"
            "请检查相机/棋盘是否移动，重新标定后再使用"
        )
    return {"rms_px": round(rms,2), "max_shift_px": round(max_shift,2)}


def plan_pick_place(ball_board_mm, profile, calibration_data, *,
                    image_size=None, corners_px=None, step_xy_mm=35):
    profile = validate_pick_profile(profile)
    if not isinstance(calibration_data, dict) or not calibration_data.get("fit", {}).get("validated"):
        raise PickPlanningError("没有通过 4+1 验证的机器人 XY 坐标标定")
    fingerprint = _verify_board_unchanged(calibration_data, image_size, corners_px)
    fit = calibration_data["fit"]
    M = np.asarray(fit.get("matrix_2x2"), dtype=float)
    t = np.asarray(fit.get("offset_mm"), dtype=float)
    if M.shape != (2, 2) or t.shape != (2,) or not np.isfinite(M).all() or not np.isfinite(t).all():
        raise PickPlanningError("机器人XY变换参数损坏")
    if not np.allclose(M.T @ M, np.eye(2), atol=0.025):
        raise PickPlanningError("机器人XY映射不是刚体变换，拒绝运动规划")
    if not isinstance(ball_board_mm, (list, tuple)) or len(ball_board_mm) != 2:
        raise PickPlanningError("黄色球必须有棋盘格毫米坐标")
    board_xy = [_num(v, "黄色球坐标") for v in ball_board_mm]
    records = calibration_data.get("records") or []
    training = [r["board_mm"] for r in records if r.get("id") in ("P1", "P2", "P3", "P4")]
    hull = _convex_hull(training)
    if not _inside_convex_polygon(board_xy, hull, clearance=6):
        raise PickPlanningError("球心未处于已验证标定区域的内部（需离边界≥6mm），禁止坐标外推抓取")
    ball_xy = (M @ np.asarray(board_xy) + t)
    place_xy = np.asarray(profile["place_xy_mm"])
    # Known point from manual taught place, not inferred from image.
    for point in [ball_xy, place_xy]:
        radius = float(np.linalg.norm(point))
        if not (70 <= radius <= 370):
            raise PickPlanningError("目标离机器人底座过近或过远，可能超出受控工作区域")
    if float(np.linalg.norm(ball_xy - place_xy)) > 280:
        raise PickPlanningError("放置点与抓取点跨度太大，请重新选择较近的安全放置位置")
    # Multi-waypoint travel at manually taught elevated Z; each XY hop bounded.
    pitch=profile["pitch_rad"]; roll=profile["roll_rad"]
    open_g=profile["open_g_rad"]; closed_g=profile["close_g_rad"]
    zhigh=profile["travel_z_mm"]
    waypoint=[]
    def go(stage, x, y, z, grip, reason):
        waypoint.append({
          "stage": stage, "kind": "cartesian",
          "xyz_mm": [round(float(x),2), round(float(y),2), round(float(z),2)],
          "t": round(pitch,5), "r": round(roll,5), "g": round(grip,5),
          "spd": 0.12, "reason": reason,
          "requires_operator_confirm": True,
        })
    def hand(stage, angle, reason):
        waypoint.append({
          "stage": stage, "kind": "gripper", "g": round(angle,5),
          "spd": 100, "acc": 10, "reason": reason,
          "requires_operator_confirm": True,
        })
    def vertical(last_stage, x, y, from_z, to_z, grip, reason):
        # Even slow Cartesian goals can travel unpredictably if far apart.
        segments = max(1, math.ceil(abs(to_z-from_z)/20.0))
        for segment in range(1,segments+1):
            z=from_z+(to_z-from_z)*segment/segments
            stage=last_stage if segment==segments else last_stage+"_part"+str(segment)
            go(stage,x,y,z,grip,reason)
    # Initial travel to ball cannot be inferred safe without a manually taught
    # safe start; do NOT move from arbitrary current position to high-Z.
    go("hover_ball", ball_xy[0],ball_xy[1],zhigh,open_g,"从已确认的高位安全姿态移动到球上方")
    hand("open_before_pick",open_g,"确认夹爪张开")
    vertical("descend_pick",ball_xy[0],ball_xy[1],zhigh,profile["grip_z_mm"],open_g,"在球上方小步垂直下降到示教抓取高度")
    hand("close_ball",closed_g,"夹住泡沫球；角度不能证明已夹住")
    vertical("lift_ball",ball_xy[0],ball_xy[1],profile["grip_z_mm"],zhigh,closed_g,"垂直分段抬升并目视核验球是否随动")
    number_segments = max(1, math.ceil(float(np.linalg.norm(place_xy - ball_xy))/step_xy_mm))
    for i in range(1,number_segments+1):
        pos=ball_xy + (place_xy-ball_xy)*(i/number_segments)
        go(f"travel_{i:02d}",pos[0],pos[1],zhigh,closed_g,"高位水平转运，分段控制")
    vertical("descend_place",place_xy[0],place_xy[1],zhigh,profile["place_z_mm"],closed_g,"分段下降到示教放置高度")
    hand("release_ball",open_g,"松开球，观察是否落在指定位置")
    vertical("retreat",place_xy[0],place_xy[1],profile["place_z_mm"],zhigh,open_g,"分段垂直撤离")
    return {
      "ball_board_mm": [round(v,2) for v in board_xy],
      "ball_robot_xy_mm": [round(float(v),2) for v in ball_xy],
      "place_robot_xy_mm": [round(float(v),2) for v in place_xy],
      "validated_region_board_mm": [list(p) for p in hull],
      "waypoints": waypoint,
      "number_of_steps": len(waypoint),
      "board_stability_check": fingerprint,
      "auto_execute": False,
      "requirements": [
        "必须从已示教的高位安全起点开始；规划器不会自动把任意初始姿态提升到安全高度",
        "末端姿态在各点保持一致，棋盘平面水平，机器人底座和相机保持固定",
        "仅完成XY标定，不代表真实抓球Z、TCP偏移或路径避障已验证",
        "每段T=104需获得实际 XYZ 反馈并由操作者放行，才可尝试下一段",
        "未通过接触安全性、夹爪力度和球体保持测试前，禁止全自动运行",
      ]
    }


def build_from_saved_files(ball_board_mm, *, profile_file=None, calibration_file=None,
                           image_size=None, corners_px=None):
    # A new/changed XY session invalidates the old final transform. Do NOT
    # quietly execute a pick using an obsolete, previously validated map.
    if calibration_file is None:
        from .robot_calibration import session_path
        draft = session_path()
        if draft.exists():
            current = _read_file(draft)
            if not isinstance(current, dict) or not current.get("saved"):
                raise PickPlanningError(
                    "存在尚未验收或已修改的 XY 标定草稿，请先通过4+1点验证并保存；"
                    "不能沿用旧标定结果规划动作"
                )
    calibration=_read_file(calibration_file or xy_calibration_path())
    profile=_read_file(profile_file or profile_path())
    return plan_pick_place(ball_board_mm,profile,calibration,
                           image_size=image_size,corners_px=corners_px)


def save_pick_profile(profile):
    valid=validate_pick_profile(profile)
    path=profile_path()
    _write_json(path,valid)
    # A completed and validated profile supersedes the unsaved draft.
    try:
        draft_path().unlink(missing_ok=True)
    except OSError:
        pass
    return {"saved":True,"path":str(path),"profile":valid}
