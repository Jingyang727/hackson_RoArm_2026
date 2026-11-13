"""Bounded single-joint manual control. Software limits are NOT collision avoidance."""
from dataclasses import dataclass
import math


class UnsafeMotion(ValueError):
    pass


# Training limits are intentionally narrower than official RoArm-M3 joint limits.
# (Chinese name, T=1051 radians feedback key, min degrees, max degrees)
JOINTS = {
    1: ("底座旋转", "b", -75.0, 75.0),
    2: ("肩部前后", "s", -50.0, 50.0),
    3: ("肘部上下", "e", 35.0, 145.0),
    4: ("手腕俯仰", "t", -60.0, 60.0),
    5: ("末端旋转", "r", -90.0, 90.0),
    6: ("夹爪开合", "g", 65.0, 180.0),
}
JOG_STEPS = (1, 3, 5, 10)
GRIPPER_STEPS = (5, 15, 30)
# Feedback may differ slightly from the official command range at a hard stop.
# A small out-of-range feedback value is recovery-only, never permission to go further out.
GRIPPER_RECOVERY_TOLERANCE_DEG = 2.0
GRIPPER_SPEED_STEPS_S = 120
GRIPPER_ACCEL_100_STEPS_S2 = 10
HOME_MAX_STEP = 5.0
SPEED_DEG_S = 15
ACC_DEG_S2 = 20
TORQUE_FIELDS = {1: "torswitchB", 2: "torswitchS", 3: "torswitchE",
                 4: "torswitchT", 5: "torswitchR", 6: "torswitchG"}
FACTORY_DEGREES = {1: 0.0, 2: 0.0, 3: 90.0, 4: 0.0, 5: 0.0, 6: 180.0}


@dataclass(frozen=True)
class Movement:
    joint: int
    current_deg: float
    target_deg: float
    payload: dict


def current_degrees(status):
    if not isinstance(status, dict) or status.get("T") != 1051:
        raise UnsafeMotion("没有有效的 T=1051 状态反馈")
    degrees = {}
    for joint, (name, field, lo, hi) in JOINTS.items():
        value = status.get(field)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise UnsafeMotion(f"{name} 的反馈不存在或无效")
        degrees[joint] = math.degrees(value)
    return degrees


def validate_angle_range(status, joint, target_deg):
    """Pure feedback/target range validation; this never authorizes actuation."""
    if type(joint) is not int or joint not in JOINTS:
        raise UnsafeMotion("未知的关节编号")
    if type(target_deg) not in (int, float) or not math.isfinite(target_deg):
        raise UnsafeMotion("目标角度无效")
    name, _, low, high = JOINTS[joint]
    current = current_degrees(status)[joint]
    lower_feedback = low - (GRIPPER_RECOVERY_TOLERANCE_DEG if joint == 6 else 0)
    upper_feedback = high + (GRIPPER_RECOVERY_TOLERANCE_DEG if joint == 6 else 0)
    if not (lower_feedback <= current <= upper_feedback):
        raise UnsafeMotion(f"{name} 当前角度 {current:.1f}° 超出允许偏差，需检查反馈和零位")
    if not low <= target_deg <= high:
        raise UnsafeMotion(f"{name} 目标角度 {target_deg:.1f}° 超出了手动训练范围 [{low}, {high}]°")
    if current < low and target_deg <= current or current > high and target_deg >= current:
        raise UnsafeMotion(f"{name} 已轻微越界，只允许朝安全区间恢复")
    return current


def plan_to_angle(status, joint, target_deg):
    current = validate_angle_range(status, joint, target_deg)
    name = JOINTS[joint][0]
    if status.get(TORQUE_FIELDS[joint]) != 1:
        raise UnsafeMotion(f"{name} 的扭矩使能状态未确认，拒绝运动")
    if joint == 6:
        # Official M3 T=106 gripper command accepts radians in [1.08, 3.14].
        # Its speed is servo steps/s, unlike T=121 degrees/s.
        cmd_rad = round(min(math.radians(target_deg), 3.14), 5)
        return Movement(joint, round(current, 2), round(target_deg, 2), {
            "T": 106, "cmd": cmd_rad,
            "spd": GRIPPER_SPEED_STEPS_S, "acc": GRIPPER_ACCEL_100_STEPS_S2,
        })
    return Movement(joint, round(current, 2), round(target_deg, 2), {
        "T": 121, "joint": joint, "angle": round(target_deg, 2),
        "spd": SPEED_DEG_S, "acc": ACC_DEG_S2,
    })


def plan_single_step(status, joint, delta_deg):
    if type(delta_deg) not in (float, int) or not math.isfinite(delta_deg):
        raise UnsafeMotion("步长无效")
    if abs(delta_deg) not in JOG_STEPS:
        raise UnsafeMotion("单次仅允许 ±1° / ±3° / ±5° / ±10°")
    current = current_degrees(status).get(joint)
    if current is None:
        raise UnsafeMotion("未知关节")
    return plan_to_angle(status, joint, current + delta_deg)


def plan_gripper_step(status, delta_deg):
    if type(delta_deg) not in (int, float) or not math.isfinite(delta_deg):
        raise UnsafeMotion("夹爪步长无效")
    if abs(delta_deg) not in GRIPPER_STEPS:
        raise UnsafeMotion("夹爪仅支持 5° / 15° / 30° 有限步进")
    current = current_degrees(status)[6]
    return plan_to_angle(status, 6, current + delta_deg)


def plan_home_step(status, joint, home_degrees):
    if joint not in JOINTS or type(joint) is not int:
        raise UnsafeMotion("未知关节")
    if joint not in home_degrees:
        raise UnsafeMotion("复位姿态不完整")
    target = home_degrees[joint]
    if type(target) not in (int, float) or not math.isfinite(target):
        raise UnsafeMotion("复位目标无效")
    current = current_degrees(status)[joint]
    difference = target - current
    if abs(difference) < 0.5:
        raise UnsafeMotion("该关节已接近目标姿态")
    delta = math.copysign(min(abs(difference), HOME_MAX_STEP), difference)
    return plan_to_angle(status, joint, current + delta)


def checked_home_pose(status):
    angles = current_degrees(status)
    for j, angle in angles.items():
        name, _, lo, hi = JOINTS[j]
        if not lo <= angle <= hi:
            raise UnsafeMotion(f"无法保存：{name} 超出了手动训练范围")
    return {j: round(value, 3) for j, value in angles.items()}
