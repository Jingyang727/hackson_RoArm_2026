"""Feedback-settling helpers.

Position stability alone is NOT proof that a move succeeded: the arm could be
stalled against an obstacle. This module only detects a repeatable plateau.
The service classifies the plateau separately using residual and progress.
"""
from math import isfinite

STABLE_MAX_DELTA_DEG = 0.6
MIN_SAMPLE_INTERVAL_S = 0.45
REQUIRED_STABLE_INTERVALS = 2   # 3 separate observations
MIN_SETTLE_ELAPSED_S = 2.5


def observe_stability(pending, angles, now):
    """Update a pending operation using an independent, time-spaced status sample."""
    last_at = pending.get("last_sample_at")
    if last_at is not None and now - last_at < MIN_SAMPLE_INTERVAL_S:
        return False
    previous = pending.get("last_angles")
    if previous is not None and all(isfinite(angles[k]) for k in angles):
        deviation = max(abs(angles[k] - previous[k]) for k in angles)
        pending["last_delta_deg"] = round(deviation, 3)
        pending["stable_intervals"] = (pending.get("stable_intervals", 0) + 1
                                        if deviation <= STABLE_MAX_DELTA_DEG else 0)
    else:
        pending["stable_intervals"] = 0
    pending["last_angles"] = angles.copy()
    pending["last_sample_at"] = now
    return (pending["stable_intervals"] >= REQUIRED_STABLE_INTERVALS
            and now - pending["since"] >= MIN_SETTLE_ELAPSED_S)
