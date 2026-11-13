"""Stable, read-only teaching snapshots. Separate from validated XY mapping and legacy pick profile."""
import json
import math
import os
import time
from pathlib import Path

from .safety import current_degrees, UnsafeMotion

STAGES = ("home", "grasp", "hover", "place", "gripper_open", "gripper_close")
FIELDS = ("x", "y", "z", "tit", "r", "g", "b", "s", "e", "t")


def pose_path():
    return Path.home() / ".evoarm" / "pick_poses_v2.json"


def read_poses():
    path = pose_path()
    if not path.exists():
        return {"version": 2, "poses": {}, "note": "独立示教快照；不覆盖XY标定或旧抓取参数"}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        if value.get("version") != 2 or not isinstance(value.get("poses"), dict):
            raise ValueError("invalid v2 pose schema")
        return value
    except (ValueError, OSError, TypeError) as exc:
        raise UnsafeMotion("无法加载示教快照，请先备份检查：" + str(exc)) from exc


def capture_pose(service, stage, *, sleep_fn=time.sleep):
    if stage not in STAGES:
        raise UnsafeMotion("未知的示教阶段")
    with service.lock:
        h = service.health()
        if not h["connected"] or h["pending"] or h["fault"] or h["teach_active"]:
            raise UnsafeMotion("未连接、动作未完成或扭矩释放；暂不能记录示教")
        a = service.status()
        sleep_fn(0.65)
        b = service.status()
        for sample in (a, b):
            if sample.get("T") != 1051 or any(
                type(sample.get(k)) not in (int, float) or not math.isfinite(sample[k])
                for k in FIELDS
            ):
                raise UnsafeMotion("缺少有效 T=1051 XYZ、姿态或六关节反馈")
            if any(sample.get("torswitch" + k) != 1 for k in "BSETRG"):
                raise UnsafeMotion("六关节扭矩未全部锁定，禁止记录姿态")
        if math.dist([a[k] for k in "xyz"], [b[k] for k in "xyz"]) > 2:
            raise UnsafeMotion("末端还在移动，请等反馈稳定")
        if max(abs(math.degrees(a[k]-b[k])) for k in "bsetrg") > 0.8:
            raise UnsafeMotion("关节还未稳定，稍后再保存")
        joint_deg = current_degrees(b)
        if stage.startswith("gripper_"):
            # Only gripper angular feedback is required for the stored datum.
            pose = {"gripper_rad": round(b["g"], 6),
                    "gripper_deg": round(math.degrees(b["g"]), 2)}
        else:
            pose = {
                "xyz_mm": [round(b[k], 3) for k in "xyz"],
                "pitch_rad": round(b["tit"], 6),
                "roll_rad": round(b["r"], 6),
                "pitch_deg": round(math.degrees(b["tit"]), 2),
                "roll_deg": round(math.degrees(b["r"]), 2),
                "joint_deg": {str(k): round(v, 3) for k, v in joint_deg.items()},
            }
        data = read_poses()
        data["poses"][stage] = {"recorded_at": time.time(), **pose}
        path = pose_path()
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        tmp = path.with_suffix(".tmp")
        try:
            with tmp.open("w", encoding="utf-8") as output:
                json.dump(data, output, ensure_ascii=False, indent=2, allow_nan=False)
                output.flush()
                os.fsync(output.fileno())
            os.chmod(tmp, 0o600)
            os.replace(tmp, path)
        finally:
            if tmp.exists():
                tmp.unlink()
        return {"saved": True, "stage": stage, "value": data["poses"][stage],
                "path": str(path), "poses": data["poses"]}
