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
    # T=1051 Z is measured in the RoArm BASE frame and may be NEGATIVE:
    # zero is not necessarily the tabletop. These bounds are only a wide
    # finite-coordinate plausibility check, NOT collision/reachability approval.
    # Both positive and negative values are supported; clearance is relative.
    for name, limit in (("抓取", 350.0), ("放置", 350.0), ("安全抬升", 380.0)):
        field = {"抓取": "grip_z_mm", "放置": "place_z_mm",
                 "安全抬升": "travel_z_mm"}[name]
        if abs(out[field]) > limit:
            raise PickPlanningError(
                f"{name}高度 Z={out[field]:.2f}mm 超出基座坐标的暂定绝对值范围"
                f" ±{limit:.0f}mm。负 Z 本身允许，但必须现场确认工作空间和桌面间隙"
            )
    clearance = out["travel_z_mm"] - max(out["grip_z_mm"], out["place_z_mm"])
    if clearance < 45:
        raise PickPlanningError(
            f"安全抬升 Z={out['travel_z_mm']:.2f}mm，相对较高的抓取/放置点"
            f"只有 {clearance:.2f}mm 净抬升（要求至少45mm）。"
            " Z 是相对机械臂基座的有符号坐标，不能当作桌面离地高度"
        )
    if not (-math.pi/2 <= out["pitch_rad"] <= math.pi/2 and -math.pi <= out["roll_rad"] <= math.pi):
        raise PickPlanningError("末端姿态范围异常")
    if not (1.08 <= out["open_g_rad"] < out["close_g_rad"] <= 3.14):
        raise PickPlanningError("夹爪角度应满足张开角度小于闭合角度，且在官方范围内")
    if out["close_g_rad"]-out["open_g_rad"] < math.radians(8):
        raise PickPlanningError("夹爪开合差不足8°，请重新示教")
    return out


def _verify_board_unchanged(calibration_data, image_size, corners_px, *, preview_only=False):
    baseline = calibration_data.get("corners_px")
    if baseline is None:
        raise PickPlanningError("缺少已保存的棋盘角点参考，请确认XY标定文件")
    if image_size != calibration_data.get("image_size"):
        raise PickPlanningError("当前摄像头分辨率与标定时不一致，请恢复原始分辨率")
    expected = np.asarray(baseline, dtype=float)
    observed = np.asarray(corners_px, dtype=float)
    if expected.shape != (54, 2) or observed.shape != (54, 2):
        raise PickPlanningError("需要54个已核验的棋盘角点，不能使用未知相机位置")
    if not np.isfinite(expected).all() or not np.isfinite(observed).all():
        raise PickPlanningError("棋盘角点坐标无效")
    grid = expected.reshape(6, 9, 2)
    pitch_px = np.r_[
        np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
        np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel(),
    ]
    square_mm = float(calibration_data.get("square_mm", 0))
    median_pitch = float(np.median(pitch_px))
    if square_mm <= 0 or median_pitch < 5 or not math.isfinite(median_pitch):
        raise PickPlanningError("棋盘真实尺寸或图像角点比例无效")
    px_per_mm = median_pitch / square_mm
    distances = np.linalg.norm(observed - expected, axis=1)
    rms_mm = float(np.sqrt(np.mean(distances**2))) / px_per_mm
    max_mm = float(np.max(distances)) / px_per_mm
    # Pixel-based 6px/13px cutoffs were arbitrary. Preview may tolerate
    # slight measured displacement, but motion must still use tighter limits.
    if rms_mm > 12 or max_mm > 20:
        raise PickPlanningError(
            f"棋盘与保存位置变化较大：RMS约{rms_mm:.1f}mm，最大约{max_mm:.1f}mm。"
            "请先核对相机、棋盘和底座是否移动"
        )
    motion_stable = rms_mm <= 4 and max_mm <= 8
    if not preview_only and not motion_stable:
        raise PickPlanningError(
            f"棋盘确认可用于定位，但尚不能用于真实运动：RMS偏移约{rms_mm:.1f}mm，"
            f"最大约{max_mm:.1f}mm。运动需要更严格的4/8mm位置核验"
        )
    return {
        "rms_mm": round(rms_mm, 2),
        "max_shift_mm": round(max_mm, 2),
        "motion_stable": motion_stable,
    }


def plan_pick_place(ball_board_mm, profile, calibration_data, *,
                    image_size=None, corners_px=None, step_xy_mm=35,
                    preview_only=False):
    profile = validate_pick_profile(profile)
    if not isinstance(calibration_data, dict) or not calibration_data.get("fit", {}).get("validated"):
        raise PickPlanningError("没有通过 4+1 验证的机器人 XY 坐标标定")
    fingerprint = _verify_board_unchanged(calibration_data, image_size, corners_px, preview_only=preview_only)
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
    # The previous "at least 6mm from the boundary" was arbitrary.
    # P1–P4 training hull, sheet localization and robot reach are separate:
    # a point at the training boundary should not be refused just for margin.
    inside_training_hull = _inside_convex_polygon(board_xy, hull, clearance=0)
    side = float(calibration_data.get("square_mm", 0))
    on_board_sheet = (
        side > 0 and -side <= board_xy[0] <= 9*side
        and -side <= board_xy[1] <= 6*side
    )
    if not on_board_sheet:
        raise PickPlanningError(
            "黄球不在已识别棋盘纸面范围内；没有该桌面位置的可靠像素→毫米映射，"
            "请扩展桌面标定后再抓取"
        )
    if not inside_training_hull and not preview_only:
        raise PickPlanningError(
            "黄球超出P1～P4已实测覆盖区域。可以定位球，但该处真实抓取"
            "尚未验证工具姿态、TCP及碰撞路径；请先扩展已验证工作区"
        )
    ball_xy = (M @ np.asarray(board_xy) + t)
    place_xy = np.asarray(profile["place_xy_mm"])
    # The 70–370mm circle was an internal conservative SW guard, NOT a
    # manufacturer-stated RoArm-M3 safety/reachability limit. The actual
    # 3D IK workspace depends on Z, tool pitch, roll and servo joint limits.
    # Do NOT apply this invented bound to a purely read-only preview.
    # Keep execution fail-closed until real IK/collision checks exist.
    preview_warnings = []
    if not inside_training_hull:
        preview_warnings.append(
            "球在棋盘纸面上，但超出P1～P4实测覆盖范围；"
            "定位可用，真实执行需要额外安全验证"
        )
    if not fingerprint["motion_stable"]:
        preview_warnings.append(
            f"棋盘有轻微偏移（约RMS {fingerprint['rms_mm']}mm），"
            "仍允许定位，但真实运动需确认相机、棋盘、底座没有变化"
        )
    operator_cautions = []
    for label, point in (("抓取", ball_xy), ("放置", place_xy)):
        radius = float(np.linalg.norm(point))
        # 370mm was an invented radius, not a manufacturer safety limit.
        # A manually taught place 396mm away must not be mislabeled as
        # unreachable. Real motion is still opt-in, one step/feedback at a
        # time, starting from a manually checked safe high pose.
        if not (70 <= radius <= 500):
            message = (
                f"{label}位置距基座半径 {radius:.1f}mm，超出当前"
                "70～500mm粗略几何候选范围。该范围并非逐姿态IK验证"
            )
            if not preview_only:
                raise PickPlanningError(message + "；暂不能执行真实运动")
            preview_warnings.append(message + "；只允许查看定位结果")
        elif radius > 370:
            operator_cautions.append(
                f"{label}位置半径 {radius:.1f}mm：已超过以前自定的370mm"
                "试验阈值，但不能据此认定不可达。"
                "实际高位XYZ与Pitch/Roll可达性仍须逐段观察反馈"
            )

    span = float(np.linalg.norm(ball_xy - place_xy))
    if span > 280:
        operator_cautions.append(
            f"抓放点跨距 {span:.1f}mm：会分解为不超过35mm的高位水平小段；"
            "每段都需人工确认和实际到位反馈。"
        )
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
      "preview_only": bool(preview_only),
      "execution_eligible": not preview_warnings,
      "ball_inside_training_hull": inside_training_hull,
      "ball_on_board_sheet": on_board_sheet,
      "reachability_warnings": preview_warnings,
      "operator_cautions": operator_cautions,
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


def preview_v2_poses(ball_board_mm, calibration_data, poses, *,
                     image_size, corners_px):
    """Read-only camera-target overlay for the v2 teach snapshots.

    These are geometric candidate targets, not a certified collision-free
    trajectory. The robot's T=1051 XYZ may not be its fingertip TCP; in
    particular the Home height is not transferable to arbitrary XY positions.
    Never promote this preview to a T=104 motion plan.
    """
    required = ("home", "hover", "grasp", "place", "gripper_open", "gripper_close")
    if not isinstance(poses, dict) or any(k not in poses for k in required):
        missing = [k for k in required if not isinstance(poses, dict) or k not in poses]
        raise PickPlanningError("v2示教尚未完成：" + ", ".join(missing))
    if not isinstance(calibration_data, dict) or not calibration_data.get("fit", {}).get("validated"):
        raise PickPlanningError("没有通过4+1验收的XY标定")
    fingerprint = _verify_board_unchanged(
        calibration_data, image_size, corners_px, preview_only=True
    )
    if not isinstance(ball_board_mm, (list, tuple)) or len(ball_board_mm) != 2:
        raise PickPlanningError("缺少黄球棋盘XY")
    board_xy = [_num(v, "黄球棋盘XY") for v in ball_board_mm]
    square = _num(calibration_data.get("square_mm"), "棋盘单格尺寸")
    if not (-square <= board_xy[0] <= 9*square and
            -square <= board_xy[1] <= 6*square):
        raise PickPlanningError("黄球超出棋盘纸面，无法可靠映射到机械臂XY")
    fit = calibration_data["fit"]
    matrix = np.asarray(fit.get("matrix_2x2"), dtype=float)
    offset = np.asarray(fit.get("offset_mm"), dtype=float)
    if (matrix.shape != (2, 2) or offset.shape != (2,) or
            not np.isfinite(matrix).all() or not np.isfinite(offset).all() or
            not np.allclose(matrix.T @ matrix, np.eye(2), atol=0.025)):
        raise PickPlanningError("XY变换矩阵不合法")
    ball = matrix @ np.asarray(board_xy) + offset

    def xy_z(stage):
        pose = poses[stage]
        xyz = pose.get("xyz_mm") if isinstance(pose, dict) else None
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise PickPlanningError("v2示教 " + stage + " 缺少XYZ")
        xyz = [_num(v, stage+" XYZ") for v in xyz]
        pitch = _num(pose.get("pitch_rad"), stage+" Pitch")
        roll = _num(pose.get("roll_rad"), stage+" Roll")
        return xyz, pitch, roll

    home, ht, hr = xy_z("home")
    hover, vt, vr = xy_z("hover")
    grasp, gt, gr = xy_z("grasp")
    place, pt, pr = xy_z("place")
    opened = _num(poses["gripper_open"].get("gripper_rad"), "夹爪张开")
    closed = _num(poses["gripper_close"].get("gripper_rad"), "夹爪闭合")
    if not (1.08 <= opened < closed <= 3.14):
        raise PickPlanningError("v2夹爪开闭角不合法")
    if hover[2] <= grasp[2]:
        raise PickPlanningError("Hover必须高于Grasp；检查示教顺序")
    if math.dist(hover[:2], grasp[:2]) > 15:
        raise PickPlanningError("Hover与Grasp XY偏移过大；请核查是否为同一个球心")
    training = [r["board_mm"] for r in calibration_data.get("records", [])
                if r.get("id") in ("P1", "P2", "P3", "P4")]
    hull = _convex_hull(training)
    inside = _inside_convex_polygon(board_xy, hull, clearance=0)
    warnings = [
        "v2六组示教已读取；轨迹仅为候选目标，当前不下发真实运动",
        "尚未验证夹爪TCP相对T=1051 XYZ的偏移；球心XY不能直接当作夹爪抓取TCP",
        "尚未执行各XY/Z/Pitch的逆运动学、关节限位和连续碰撞检测",
        "Home的Z=%.1fmm仅在示教Home位置有效，不代表球上方或放置点可用的转运高度" % home[2],
    ]
    if hover[2] - place[2] < 45:
        warnings.append(
            "Hover Z=%.1fmm比Place Z=%.1fmm高%.1fmm，未达到45mm的示教转运净空；"
            "需验证放置点上方的独立安全高度" %
            (hover[2], place[2], hover[2]-place[2])
        )
    if not inside:
        warnings.append("黄球位于P1–P4实测凸包之外，尚未验收该处真实运动")
    if not fingerprint["motion_stable"]:
        warnings.append("棋盘相对保存角点有偏移，尚不能用于真实运动")

    def cart(stage, xyz, pitch, roll, grip, reason):
        return {
            "stage": stage, "kind": "cartesian",
            "xyz_mm": [round(float(v), 2) for v in xyz],
            "t": round(pitch, 5), "r": round(roll, 5),
            "g": round(grip, 5), "reason": reason,
            "requires_operator_confirm": True,
            "preview_only": True,
        }
    def hand(stage, angle, reason):
        return {
            "stage": stage, "kind": "gripper", "g": round(angle, 5),
            "reason": reason, "preview_only": True,
        }

    # XY is dynamic; stage-specific Z and orientation come from the v2
    # snapshots. No generic high-Z bridging or motion interpolation is
    # invented: transitions must first be IK/collision validated.
    target_hover = [float(ball[0]), float(ball[1]), hover[2]]
    target_grasp = [float(ball[0]), float(ball[1]), grasp[2]]
    waypoints = [
        cart("home_reference", home, ht, hr, opened, "示教Home参照，非自动起始移动"),
        cart("hover_ball_candidate", target_hover, vt, vr, opened, "视觉动态XY + Hover示教高度"),
        hand("open_before_pick", opened, "示教张开角"),
        cart("grasp_ball_candidate", target_grasp, gt, gr, opened, "视觉动态XY + Grasp示教高度"),
        hand("close_ball", closed, "示教闭合角；未验证球是否夹稳"),
        cart("lift_candidate", target_hover, vt, vr, closed, "返回Hover高度；待验证路径"),
        cart("place_candidate", place, pt, pr, closed, "示教Place目标，转运轨迹尚未生成"),
        hand("release_ball", opened, "放开夹爪"),
    ]
    return {
        "source": "pick_poses_v2.json",
        "preview_only": True, "execution_eligible": False,
        "ball_board_mm": [round(v, 2) for v in board_xy],
        "ball_robot_xy_mm": [round(float(v), 2) for v in ball],
        "place_robot_xy_mm": [round(float(v), 2) for v in place[:2]],
        "ball_inside_training_hull": inside,
        "board_stability_check": fingerprint,
        "validated_region_board_mm": [list(p) for p in hull],
        "reachability_warnings": warnings,
        "operator_cautions": [],
        "waypoints": waypoints,
        "number_of_steps": len(waypoints),
        "auto_execute": False,
        "requirements": warnings,
    }


def build_from_saved_files(ball_board_mm, *, profile_file=None, calibration_file=None,
                           image_size=None, corners_px=None, preview_only=False,
                           execution_intent=False):
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
    # A present v2 snapshot takes priority over stale legacy pick_profile.json.
    # Until tool TCP, per-target IK and collision checks are implemented, v2
    # supports read-only preview ONLY. Never silently fall back for motors.
    if profile_file is None:
        from .fast_teach import pose_path, read_poses
        if pose_path().exists():
            if execution_intent:
                raise PickPlanningError(
                    "检测到新版六组示教姿态，旧版抓取执行器尚未支持v2的TCP/IK/安全轨迹；"
                    "已拒绝运动，禁止回退使用旧pick_profile.json"
                )
            # All ordinary callers, including older code which omitted
            # preview_only, get an explicitly non-executable v2 preview.
            return preview_v2_poses(
                ball_board_mm, calibration, read_poses().get("poses"),
                image_size=image_size, corners_px=corners_px
            )
    profile=_read_file(profile_file or profile_path())
    return plan_pick_place(ball_board_mm,profile,calibration,
                           image_size=image_size,corners_px=corners_px,
                           preview_only=preview_only)


def save_pick_profile(profile):
    valid=validate_pick_profile(profile)
    path=profile_path()
    _write_json(path,valid)
    # A completed and validated profile supersedes the unsaved draft.
    try:
        draft_path().unlink(missing_ok=True)
    except OSError:
        pass
    radius=math.hypot(*valid["place_xy_mm"])
    warning=(
        f"放置点距基座的水平半径 {radius:.1f}mm，超过程序此前自定的370mm试验阈值"
        "（这不是厂商官方工作区边界）。示教参数已保存、允许查看只读轨迹预览，"
        "但实际XYZ轨迹在该姿态下还未通过逆运动学与避障验证，禁止自动执行"
        if radius>370 or radius<70 else None
    )
    return {"saved":True,"path":str(path),"profile":valid,
            "warnings":[warning] if warning else []}
