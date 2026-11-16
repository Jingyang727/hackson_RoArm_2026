"""Coordinate-page motion settings, separate from legacy grasp experiments."""
import math

DEFAULT_SPEED=.65
MIN_SPEED=.05
MAX_SPEED=1.
COMMAND_GAP_S=.12
WAYPOINT_STABLE_SPAN_S=.30
WAYPOINT_OFFSET_AFTER_S=.8


def motion_profile(speed=DEFAULT_SPEED):
    if type(speed) not in (int,float) or not math.isfinite(speed) or not MIN_SPEED<=speed<=MAX_SPEED:
        raise ValueError('坐标移动速度系数需在0.05～1.0之间')
    return {'speed':float(speed),'speed_unit':'controller_factor_not_mm_per_second',
            'minimum_command_gap_s':COMMAND_GAP_S,'waypoint_stable_span_s':WAYPOINT_STABLE_SPAN_S,
            'waypoint_offset_after_s':WAYPOINT_OFFSET_AFTER_S}
