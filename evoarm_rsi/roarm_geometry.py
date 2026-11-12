"""RoArm-M3 default edge-tool geometry, matching the vendor 20260701 FK.

Firmware XYZ describes its nominal fingertip, not a generic grasp center.
This module is pure math and never opens a device.
"""
from __future__ import annotations

import math


def forward(status):
    if status.get("T") != 1051:
        raise ValueError("T1051 feedback required")
    fields = "bsetrg"
    if any(type(status.get(k)) not in (int, float) or not math.isfinite(status[k]) for k in fields):
        raise ValueError("six finite joint angles required")
    base, shoulder, elbow, wrist = (status[k] for k in "bset")
    l2, a2 = math.hypot(236.82, 30.), math.atan2(30., 236.82)
    le, ae = math.hypot(171.67, 13.69), math.atan2(13.69, 171.67)
    radius = z = 0.
    points = [[0., 0., 0.]]
    for length, angle in ((l2, math.pi/2-shoulder-a2),
                          (144.49, math.pi/2-shoulder-elbow),
                          (le, math.pi/2-shoulder-elbow-wrist-ae)):
        radius += length*math.cos(angle)
        z += length*math.sin(angle)
        points.append([radius*math.cos(base), radius*math.sin(base), z])
    return {"xyz_mm": points[-1], "joints_xyz_mm": points,
            "pitch_rad": shoulder+elbow+wrist-math.pi/2}


def height_above_plane(xyz, plane):
    normal = plane["normal_up"]
    if (len(normal) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in normal)
            or abs(math.sqrt(sum(v*v for v in normal))-1) > .001 or normal[2] < .95):
        raise ValueError("a finite, near-horizontal unit table normal is required")
    offset = plane["d_mm"]
    if type(offset) not in (int, float) or not math.isfinite(offset):
        raise ValueError("finite plane offset required")
    return sum(a*b for a, b in zip(xyz, normal))+offset
