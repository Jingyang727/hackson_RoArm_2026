"""Save user-chosen pose locally; never write to arm firmware's boot configuration."""
import json
import os
from pathlib import Path
from datetime import datetime, timezone

from .safety import JOINTS, UnsafeMotion


def pose_file():
    return Path.home() / ".evoarm" / "home_pose.json"


def save_pose(degrees):
    path = pose_file()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    angles = {str(j): degrees[j] for j in JOINTS}
    data = {"version": 1, "created_at": datetime.now(timezone.utc).isoformat(), "degrees": angles}
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    os.chmod(temp, 0o600)
    os.replace(temp, path)
    return data


def load_pose():
    path = pose_file()
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("version") != 1:
            raise ValueError("版本不支持")
        raw = data["degrees"]
        degrees = {j: raw[str(j)] for j in JOINTS}
        import math
        for j, value in degrees.items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"关节 {j} 的目标角度无效")
        # Actual hard limits are separately enforced on every motion.
        return {"created_at": data["created_at"], "degrees": degrees}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise UnsafeMotion(f"本地复位姿态文件损坏：{exc}") from exc
