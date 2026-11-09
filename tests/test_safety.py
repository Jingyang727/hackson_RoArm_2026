"""Unit tests for the RoArm manual-command safety envelope (no hardware)."""
import math
import unittest
from control_ui.safety import (
    JOINTS, UnsafeMotion, current_degrees, plan_single_step,
    plan_home_step, FACTORY_DEGREES,
)


def feedback(**overrides):
    status = {
        "T": 1051, "b": 0.0, "s": 0.0, "e": math.pi / 2,
        "t": 0.0, "r": 0.0, "g": math.pi,
        "torswitchB": 1, "torswitchS": 1, "torswitchE": 1,
        "torswitchT": 1, "torswitchR": 1, "torswitchG": 1,
    }
    status.update(overrides)
    return status


class SafetyTests(unittest.TestCase):
    def test_six_joints(self):
        self.assertEqual(len(JOINTS), 6)

    def test_step_1_3_5_10(self):
        for step in (1, 3, 5, 10):
            plan = plan_single_step(feedback(), 1, step)
            self.assertAlmostEqual(plan.target_deg, step)
            self.assertEqual(plan.payload["T"], 121)
            self.assertEqual(plan.payload["joint"], 1)
            self.assertEqual(plan.payload["spd"], 15)

    def test_refuses_unlisted_step(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(), 1, 11)

    def test_refuses_nan(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(), 1, float("nan"))

    def test_refuses_missing_feedback(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step({"T": 1051, "b": 0}, 1, 1)

    def test_refuses_unknown_joint(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(), 100, 1)

    def test_refuses_outside_training_range(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(b=math.radians(74)), 1, 10)

    def test_refuses_torque_off(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(torswitchB=0), 1, 1)

    def test_home_step_maximum_five_degrees(self):
        target = dict(FACTORY_DEGREES)
        target[1] = 50
        plan = plan_home_step(feedback(), 1, target)
        self.assertEqual(plan.target_deg, 5.0)

    def test_home_step_small_remainder(self):
        target = dict(FACTORY_DEGREES)
        target[1] = 2.2
        plan = plan_home_step(feedback(), 1, target)
        self.assertEqual(plan.target_deg, 2.2)

    def test_factory_pose(self):
        self.assertEqual(FACTORY_DEGREES[3], 90)
        self.assertEqual(FACTORY_DEGREES[6], 180)


if __name__ == "__main__":
    unittest.main()
