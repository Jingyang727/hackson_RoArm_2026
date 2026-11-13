"""Read-only supervised teaching of tool height, pitch, roll and gripper angle."""
import math
import time

from .pick_plan import PickPlanningError


def observe_tool_pose(arm_service, *, sleep_fn=time.sleep):
    """Two quiet T1051 samples; never send motion commands.

    T1051.t is a wrist *joint* angle. Cartesian T104 expects tool Pitch,
    which is T1051.tit. Confusing them can cause dangerous tool orientation.
    """
    with arm_service.lock:
        h = arm_service.health()
        if not h["connected"] or h["pending"] is not None or h["fault"] is not None:
            raise PickPlanningError("机械臂未连接或有待完成动作/异常")
        first = arm_service.status()
        sleep_fn(0.65)
        second = arm_service.status()
        def validate(result):
            if result.get("T") != 1051:
                raise PickPlanningError("缺少 T1051 真实反馈")
            for field in ("x", "y", "z", "tit", "r", "g"):
                v = result.get(field)
                if type(v) not in (float, int) or not math.isfinite(v):
                    raise PickPlanningError("末端" + field + "反馈无效（必须有 tit 末端Pitch）")
            flags = [result.get("torswitch" + k) for k in "BSETRG"]
            if any(x != 1 for x in flags):
                raise PickPlanningError("请先托住机械臂、恢复全部扭矩锁定，再记录示教姿态")
        validate(first); validate(second)
        displacement = math.dist(
            [first[k] for k in "xyz"], [second[k] for k in "xyz"])
        if displacement > 2.0:
            raise PickPlanningError("机械臂还在运动，请等姿态稳定后再记录")
        result = {
            "x_mm": round((first["x"] + second["x"])/2, 2),
            "y_mm": round((first["y"] + second["y"])/2, 2),
            "z_mm": round((first["z"] + second["z"])/2, 2),
            "pitch_rad": round((first["tit"] + second["tit"])/2, 5),
            "roll_rad": round((first["r"] + second["r"])/2, 5),
            "gripper_rad": round((first["g"] + second["g"])/2, 5),
        }
        return result
