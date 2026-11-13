"""Shared stock RoArm-M3 command ranges, not a collision-free workspace.

Source: vendor wiki and RoArm-M3_example_20260701 RoArm-M3_module.h.
Nominal degree endpoints are used for display/IK; dedicated gripper commands
are additionally bounded to the documented [1.08, 3.14] radians.
"""
import math

LIMIT_SOURCE = 'https://www.waveshare.net/wiki/RoArm-M3'
JOINTS = {
    1: ('底座旋转', 'b', -180., 180.),
    2: ('肩部前后', 's', -90., 90.),
    3: ('肘部上下', 'e', 0., 180.),
    4: ('手腕俯仰', 't', -90., 90.),
    5: ('末端旋转', 'r', -180., 180.),
    6: ('夹爪开合', 'g', math.degrees(1.08), 180.),
}
GRIPPER_MIN_RAD = 1.08
GRIPPER_MAX_RAD = 3.14


def joint_limits_info():
    return {'profile': 'roarm_m3_vendor_nominal_v1', 'source': LIMIT_SOURCE,
            'reference_firmware': 'RoArm-M3_example_20260701',
            'joints': [{'joint': j, 'name': name, 'field': field,
                        'min_deg': low, 'max_deg': high} for j, (name, field, low, high) in JOINTS.items()]}
