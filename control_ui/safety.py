"""Safety gate: only very small absolute-angle commands. NOT collision avoidance."""
import math
from dataclasses import dataclass


class UnsafeMotion(ValueError):
    pass


# Software-only training envelope, NOT hardware limits or collision-free positions.
JOINTS = {
    1: ("底座旋转", "b", -30.0, 30.0),
    2: ("肩部关节", "s", -30.0, 30.0),
    3: ("肘部关节", "e", 60.0, 120.0),
    4: ("手腕俯仰", "t", -30.0, 30.0),
    5: ("末端自转", "r", -30.0, 30.0),
    6: ("夹爪开合", "g", 160.0, 180.0),
}


@dataclass(frozen=True)
class Movement:
    joint: int
    current_deg: float
    target_deg: float
    payload: dict


def plan_single_step(status: dict, joint: int, delta_deg: float) -> Movement:
    if type(joint) is not int or joint not in JOINTS:
        raise UnsafeMotion("未知关节")
    if type(delta_deg) not in (float, int) or not math.isfinite(delta_deg) or abs(delta_deg) != 1:
        raise UnsafeMotion("仅允许单次 ±1°，不可持续运动")
    label, field, lo, hi = JOINTS[joint]
    if status.get("T") != 1051:
        raise UnsafeMotion("状态反馈无效")
    angle = status.get(field)
    if type(angle) not in (int, float) or not math.isfinite(angle):
        raise UnsafeMotion(f"{label} 角度反馈无效")
    current = math.degrees(angle)
    target = current + delta_deg
    if not (lo <= current <= hi and lo <= target <= hi):
        raise UnsafeMotion(f"{label} 当前或目标角度超出保守范围 [{lo}, {hi}]°")
    return Movement(joint, round(current, 3), round(target, 3), {
        "T": 121, "joint": joint, "angle": round(target, 3), "spd": 3, "acc": 10,
    })
