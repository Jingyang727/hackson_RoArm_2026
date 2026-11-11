"""Scene-derived TCP task targets; no Home/Hover/Grasp/Place teaching inputs.

This is a task planner, not an IK/whole-arm collision checker or serial
driver. It consumes a fresh, calibrated world state and emits read-only
skills. The execution adapter must still validate trajectories and timing.
"""
from __future__ import annotations

import math
import time

from .task_agent import timing_preflight


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _point(value, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain robot-base TCP XYZ")
    return [_number(v, name) for v in value]


def _inside(point, polygon, margin):
    """Convex, ordered calibration region, with physical clearance."""
    if not isinstance(polygon, list) or len(polygon) < 3:
        raise ValueError("validated convex workspace polygon required")
    signs = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if len(a) != 2 or len(b) != 2:
            raise ValueError("workspace polygon must contain XY points")
        ax, ay = [_number(v, "workspace") for v in a]
        bx, by = [_number(v, "workspace") for v in b]
        length = math.hypot(bx-ax, by-ay)
        if length <= 0:
            raise ValueError("degenerate workspace edge")
        signed = ((bx-ax)*(point[1]-ay) - (by-ay)*(point[0]-ax)) / length
        if abs(signed) < margin:
            return False
        signs.append(signed)
    return all(v >= 0 for v in signs) or all(v <= 0 for v in signs)


def plan_from_world(world, geometry, *, now_ns=None):
    """Compute a new plan from THIS ball and THIS funnel, in base/TCP frame.

    Calibrated geometry must supply table height, tool clearances, aperture
    sizes and a validated workspace. No task position is supplied by humans.
    Merely detecting image boxes or measuring the funnel interior does not
    satisfy this schema's calibrated mouth-center requirements.
    """
    if geometry.get("verified") is not True or geometry.get("frame") != "robot_base_tcp":
        raise ValueError("verified camera/base/TCP geometry required")
    if any(not geometry.get(k) or world.get(k) != geometry[k]
           for k in ("setup_id", "calibration_id")):
        raise ValueError("world state and calibration versions differ")
    stamp = world.get("host_monotonic_ns")
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    max_age = _number(geometry.get("max_scene_age_s"), "verified scene age limit")
    if max_age <= 0:
        raise ValueError("scene age limit must be positive")
    if type(stamp) is not int or not 0 <= now_ns-stamp <= max_age*1e9:
        raise ValueError("fresh same-host world state required; re-observe moved objects")
    if world.get("gripper_empty") is not True:
        raise ValueError("initial gripper state is not confirmed empty")
    current = _point(world.get("current_tcp_robot_mm"), "current TCP")
    ball = world.get("ball", {})
    funnel = world.get("funnel", {})
    if ball.get("stable") is not True or funnel.get("stable") is not True:
        raise ValueError("stable current ball and funnel observations required")
    if funnel.get("measurement") != "calibrated_mouth_center":
        raise ValueError("funnel interior depth is not its mouth/release plane")
    center = _point(ball.get("center_robot_mm"), "ball center")
    mouth = _point(funnel.get("mouth_center_robot_mm"), "funnel mouth")
    radius = _number(ball.get("radius_mm"), "ball radius")
    aperture = _number(funnel.get("mouth_radius_mm"), "mouth radius")
    tool_radius = _number(geometry.get("tool_radius_mm"), "tool radius")
    release_clearance = _number(geometry.get("release_clearance_mm"), "release clearance")
    transit_clearance = _number(geometry.get("transit_clearance_mm"), "transit clearance")
    table_z = _number(geometry.get("table_z_mm"), "table height")
    if min(radius, aperture, tool_radius, release_clearance, transit_clearance) <= 0:
        raise ValueError("object dimensions and verified clearances must be positive")
    if aperture <= radius:
        raise ValueError("ball does not fit through funnel mouth")
    if center[2] <= table_z or mouth[2] <= table_z:
        raise ValueError("object geometry is inconsistent with table height")
    polygon = geometry.get("workspace_xy_mm")
    if any(not _inside(p, polygon, tool_radius+radius) for p in (current, center, mouth)):
        raise ValueError("task point outside validated TCP workspace/clearance")
    zrange = geometry.get("tcp_z_range_mm")
    if not isinstance(zrange, list) or len(zrange) != 2:
        raise ValueError("verified TCP height range required")
    low, high = [_number(v, "TCP Z limit") for v in zrange]
    if low >= high:
        raise ValueError("invalid TCP height range")
    release_z = mouth[2]+radius+release_clearance
    transit_z = max(current[2], max(center[2]+radius, release_z)+transit_clearance)
    if min(current[2], center[2], release_z) < low or transit_z > high:
        raise ValueError("scene-derived task exceeds validated height range")
    opening = _number(geometry.get("gripper_open_rad"), "gripper open")
    closing = _number(geometry.get("gripper_close_rad"), "gripper close")
    if not 1.08 <= opening < closing <= 3.14:
        raise ValueError("invalid verified gripper geometry")
    attitude = [_number(geometry.get(k), k) for k in ("pitch_rad", "roll_rad")]
    skills = []

    def move(stage, point, grip):
        skills.append({"stage": stage, "kind": "move_tcp", "tcp_xyz_mm": point,
                       "pitch_rad": attitude[0], "roll_rad": attitude[1], "gripper_rad": grip})

    def hand(stage, grip):
        skills.append({"stage": stage, "kind": "gripper", "gripper_rad": grip})

    # Heights are derived from this scene and verified clearances. No
    # previously taught parking or transfer position enters the plan.
    move("raise_current", [*current[:2], transit_z], opening)
    move("approach_ball", [*center[:2], transit_z], opening)
    move("descend_ball", center, opening)
    hand("close", closing)
    move("lift", [*center[:2], transit_z], closing)
    move("transport", [*mouth[:2], transit_z], closing)
    move("lower_to_funnel", [*mouth[:2], release_z], closing)
    hand("release", opening)
    move("retreat", [*mouth[:2], transit_z], opening)
    return {"kind": "dynamic_pick_place_task_v1", "setup_id": world["setup_id"],
            "calibration_id": world["calibration_id"], "scene_time_ns": stamp,
            "frame": "robot_base_tcp", "skills": skills,
            "ball_target_mm": center, "funnel_target_mm": mouth,
            "taught_positions_used": False, "motion_authorized": False,
            "execution_eligible": False, "timing": timing_preflight(skills),
            "remaining_checks": ["whole-arm IK and swept-volume validation for current scene",
                                 "TCP-to-firmware reference conversion",
                                 "actual per-skill hardware timing and feedback confirmation"]}
