"""Deterministic experiment orchestration, evidence capture and safe mock execution.

This package deliberately has NO import of the live RoArm serial driver.
Real hardware requires a separately reviewed adapter and safety preflight.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid


def _finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


@dataclass(frozen=True)
class TrialPolicy:
    offset_x_mm: float = 0.0
    offset_y_mm: float = 0.0
    grasp_z_offset_mm: float = 0.0
    gripper_close_deg: float = 160.75
    approach_speed: float = 0.10
    revision: int = 0

    def __post_init__(self):
        for key in ("offset_x_mm", "offset_y_mm", "grasp_z_offset_mm",
                    "gripper_close_deg", "approach_speed"):
            _finite(getattr(self, key), key)
        if not (-8 <= self.offset_x_mm <= 8 and -8 <= self.offset_y_mm <= 8
                and -8 <= self.grasp_z_offset_mm <= 8):
            raise ValueError("trial offsets must remain within +/-8 mm")
        if not (150 <= self.gripper_close_deg <= 170):
            raise ValueError("gripper angle outside provisional mock bounds")
        if not (0.02 <= self.approach_speed <= 0.15):
            raise ValueError("speed outside provisional mock bounds")
        if type(self.revision) is not int or self.revision < 0:
            raise ValueError("invalid policy revision")

    def updated(self, changes: dict) -> "TrialPolicy":
        # Strictly validate model-suggested parameter names and values.
        permitted = {"offset_x_mm", "offset_y_mm", "grasp_z_offset_mm",
                     "gripper_close_deg", "approach_speed"}
        if not isinstance(changes, dict) or not changes or set(changes) - permitted:
            raise ValueError("agent proposed unrecognized or empty policy changes")
        return replace(self, **changes, revision=self.revision + 1)


class TrialStore:
    """Atomic local JSON; every trial gets a manifest and append-only evidence."""
    def __init__(self, root=None):
        self.root = Path(root) if root is not None else (
            Path.home() / ".evoarm" / "rsi" / "experiments"
        )
        self.lock = threading.RLock()

    def create(self, trial_id, data):
        with self.lock:
            path = self.root / trial_id
            path.mkdir(parents=True, exist_ok=False)
            self.write(trial_id, "manifest.json", data)
            return path

    def write(self, trial_id, name, data):
        if name not in {"manifest.json", "policy_before.json", "policy_after.json",
                        "observation.json", "evaluation.json", "reflection.json"}:
            raise ValueError("unknown experiment artifact name")
        path = self.root / trial_id / name
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        with self.lock:
            try:
                with temporary.open("w", encoding="utf-8") as fp:
                    json.dump(data, fp, ensure_ascii=False, indent=2, allow_nan=False)
                    fp.flush()
                    os.fsync(fp.fileno())
                os.replace(temporary, path)
            finally:
                temporary.unlink(missing_ok=True)

    def event(self, trial_id, stream, item):
        if stream not in {"events", "commands", "trajectory"}:
            raise ValueError("unknown event stream")
        path = self.root / trial_id / (stream + ".jsonl")
        with self.lock:
            with path.open("a", encoding="utf-8") as fp:
                fp.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")
                fp.flush()

    def read(self, trial_id, name="manifest.json"):
        if not trial_id or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-" for c in trial_id):
            raise ValueError("invalid trial_id")
        if name not in {"manifest.json", "evaluation.json", "policy_before.json",
                        "policy_after.json", "observation.json", "reflection.json"}:
            raise ValueError("invalid artifact")
        return json.loads((self.root / trial_id / name).read_text(encoding="utf-8"))


class MockObserver:
    """Replace with RGB-D camera adapter; returns observations, not fake images."""
    def observe(self, policy, tick_s):
        err_x = round(4.0 - policy.offset_x_mm, 3)
        err_y = round(-2.0 - policy.offset_y_mm, 3)
        return {
            "timestamp_s": round(tick_s, 3),
            "target_class": "yellow_ball_simulated",
            "target_robot_xyz_mm": [342.3, -24.3, -86.787],
            "gripper_to_target_error_mm": [err_x, err_y],
            "simulated": True,
        }


class MockRobot:
    """P0 simulation. No physical commands, serial writes or motor imports."""
    hardware = False

    def __init__(self):
        self.is_home = True
        self.fault = False
        self.commands = []

    def preflight(self, policy):
        if self.fault:
            raise RuntimeError("mock arm fault")
        return {"ready": True, "hardware": False}

    def run_grasp(self, policy, max_seconds, on_feedback):
        if max_seconds <= 0 or max_seconds > 10:
            raise ValueError("trial action window must be 0..10 seconds")
        self.is_home = False
        for at, stage in ((0., "approach"), (3., "align"),
                          (6., "close"), (8.5, "lift")):
            if at > max_seconds:
                break
            command = {"timestamp_s": at, "stage": stage,
                       "policy": asdict(policy), "simulated": True}
            self.commands.append(command)
            on_feedback("commands", command)
            on_feedback("trajectory", {"timestamp_s": at, "stage": stage,
                                       "xyz_mm": [342.3, -24.3, -86.787],
                                       "joints_deg": None,
                                       "source": "simulator_not_real_feedback"})
        return {"status": "simulated_completion", "action_window_s": max_seconds}

    def return_home(self, on_feedback):
        if self.fault:
            raise RuntimeError("mock robot fault: cannot return home")
        self.is_home = True
        on_feedback("events", {"kind": "return_home", "simulated": True})
        return {"home": True}


class TrialRunner:
    def __init__(self, store=None, robot=None, observer=None, evaluator=None):
        self.store = store if store is not None else TrialStore()
        self.robot = robot if robot is not None else MockRobot()
        self.observer = observer if observer is not None else MockObserver()
        self.evaluator = evaluator
        self.lock = threading.Lock()

    def run_once(self, policy=None, *, action_window_s=10, propose_next=None):
        """One bounded trial. Agent output changes policy for NEXT run only.

        No autonomous repeat, no real-arm adapter in P0. An exception always
        records the evidence; reset is attempted only if preflight succeeded.
        """
        if getattr(self.robot, "hardware", True):
            raise RuntimeError("P0 rejects hardware adapters; use simulator only")
        policy = policy if policy is not None else TrialPolicy()
        if not isinstance(policy, TrialPolicy):
            raise TypeError("policy must be TrialPolicy")
        window = _finite(action_window_s, "action_window_s")
        if not 0 < window <= 10:
            raise ValueError("action window must be within 10 seconds")
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("another RSI trial is active")
        trial_id = uuid.uuid4().hex
        started = datetime.now(timezone.utc).isoformat()
        manifest = {
            "trial_id": trial_id, "created_at": started,
            "status": "created", "simulated": True,
            "action_window_s": window,
            "camera": "mock", "model_gateway": "not_called",
            "robot": "mock", "policy_revision": policy.revision,
        }
        created = False
        try:
            self.store.create(trial_id, manifest)
            created = True
            self.store.write(trial_id, "policy_before.json", asdict(policy))
            self.store.event(trial_id, "events", {"kind": "trial_started", "at": started})
            observation = self.observer.observe(policy, 0)
            self.store.write(trial_id, "observation.json", observation)
            preflight = self.robot.preflight(policy)
            if not preflight.get("ready"):
                raise RuntimeError("preflight rejected trial")
            manifest["status"] = "executing"
            self.store.write(trial_id, "manifest.json", manifest)
            attempt = self.robot.run_grasp(
                policy, window, lambda stream, item: self.store.event(trial_id, stream, item)
            )
            # The reset is deliberately simple for a cleared tabletop, but
            # motor faults must not trigger blind return-home commands.
            if getattr(self.robot, "fault", False):
                raise RuntimeError("robot reported fault before return-home")
            reset = self.robot.return_home(
                lambda stream, item: self.store.event(trial_id, stream, item)
            )
            outcome = self.observer.observe(policy, window)
            e = outcome["gripper_to_target_error_mm"]
            xy_error = math.hypot(*e)
            evaluation = {
                "simulated": True, "success": xy_error <= 2.0,
                "xy_error_mm": round(xy_error, 3),
                "failure_type": "none" if xy_error <= 2 else "xy_alignment",
                "attempt": attempt, "reset": reset,
                "observation_after": outcome,
            }
            self.store.write(trial_id, "evaluation.json", evaluation)
            proposed = None
            if propose_next is not None:
                proposal = propose_next(asdict(policy), evaluation)
                if not isinstance(proposal, dict):
                    raise ValueError("agent proposal must be a dict")
                proposed = policy.updated(proposal)
                self.store.write(trial_id, "policy_after.json", asdict(proposed))
                self.store.write(trial_id, "reflection.json", {
                    "source": "injected_strategy", "changes": proposal,
                    "agent_was_called": True,
                })
            manifest.update({"status": "completed", "success": evaluation["success"],
                             "finished_at": datetime.now(timezone.utc).isoformat()})
            self.store.write(trial_id, "manifest.json", manifest)
            return {"trial_id": trial_id, "manifest": manifest,
                    "evaluation": evaluation,
                    "next_policy": asdict(proposed) if proposed else None}
        except Exception as exc:
            if created:
                manifest.update({"status": "failed", "error": str(exc),
                                 "finished_at": datetime.now(timezone.utc).isoformat()})
                self.store.event(trial_id, "events", {"kind": "trial_error", "error": str(exc)})
                self.store.write(trial_id, "manifest.json", manifest)
            raise
        finally:
            self.lock.release()
