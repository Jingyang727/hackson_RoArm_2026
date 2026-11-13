"""Unit tests for the RoArm manual-command safety envelope (no hardware)."""
import math
import unittest
from control_ui.safety import (
    JOINTS, UnsafeMotion, current_degrees, plan_single_step,
    plan_home_step, plan_gripper_step, FACTORY_DEGREES,
    validate_angle_range, plan_to_angle,
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

    def test_refuses_outside_vendor_range(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(b=math.radians(174)), 1, 10)

    def test_old_training_envelope_is_not_a_joint_limit(self):
        for joint,current,delta in ((1,74,10),(2,49,10),(3,144,10),(4,59,10),(5,89,10)):
            field=JOINTS[joint][1]
            plan=plan_single_step(feedback(**{field:math.radians(current)}),joint,delta)
            self.assertAlmostEqual(plan.target_deg,current+delta)

    def test_refuses_torque_off(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(feedback(torswitchB=0), 1, 1)

    def test_numeric_preview_validation_does_not_authorize_a_released_gripper(self):
        raw=feedback(torswitchG=0,g=math.radians(180.8))
        self.assertAlmostEqual(validate_angle_range(raw,6,179.9),180.8)
        with self.assertRaisesRegex(UnsafeMotion,'扭矩'):
            plan_to_angle(raw,6,179.9)

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

    def test_gripper_uses_dedicated_radians_command(self):
        plan = plan_gripper_step(feedback(g=math.radians(175)), -15)
        self.assertEqual(plan.payload["T"], 106)
        self.assertAlmostEqual(plan.payload["cmd"], math.radians(160), places=4)
        self.assertGreater(plan.payload["spd"], 0)

    def test_gripper_180_9_allows_only_inward_recovery(self):
        current = feedback(g=math.radians(180.9))
        plan = plan_gripper_step(current, -5)
        self.assertAlmostEqual(plan.target_deg, 175.9, places=1)
        self.assertEqual(plan.payload["T"], 106)
        with self.assertRaises(UnsafeMotion):
            plan_gripper_step(current, 5)

    def test_gripper_180_9_can_open_15_degrees(self):
        plan = plan_gripper_step(feedback(g=math.radians(180.9)), -15)
        self.assertAlmostEqual(plan.target_deg, 165.9, places=1)

    def test_gripper_190_refuses_even_inward(self):
        with self.assertRaises(UnsafeMotion):
            plan_gripper_step(feedback(g=math.radians(190)), -5)

    def test_gripper_small_lower_boundary_recovery(self):
        low=JOINTS[6][2];current=low-.8
        plan = plan_gripper_step(feedback(g=math.radians(current)), 5)
        self.assertAlmostEqual(plan.target_deg, current+5, places=1)
        with self.assertRaises(UnsafeMotion):
            plan_gripper_step(feedback(g=math.radians(current)), -5)

    def test_gripper_target_never_exceeds_3_14_radians(self):
        plan = plan_gripper_step(feedback(g=math.radians(175)), 5)
        self.assertLessEqual(plan.payload["cmd"], 3.14)

    def test_factory_pose(self):
        self.assertEqual(FACTORY_DEGREES[3], 90)
        self.assertEqual(FACTORY_DEGREES[6], 180)


if __name__ == "__main__":
    unittest.main()
