import math
import unittest
from control_ui.safety import plan_single_step, UnsafeMotion


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.status = {"T": 1051, "b": 0.0, "s": 0.0, "e": math.pi / 2,
                       "t": 0.0, "r": 0.0, "g": math.pi}

    def test_one_degree(self):
        plan = plan_single_step(self.status, 1, 1)
        self.assertEqual(plan.payload["T"], 121)
        self.assertEqual(plan.target_deg, 1)

    def test_refuse_big_step(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(self.status, 1, 2)

    def test_refuse_nan(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(self.status, 1, float("nan"))

    def test_refuse_stale_bad_status(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step({"T": 1, "b": 0.0}, 1, 1)

    def test_refuse_at_limit(self):
        self.status["b"] = math.radians(29.5)
        with self.assertRaises(UnsafeMotion):
            plan_single_step(self.status, 1, 1)

    def test_refuse_unknown_joint(self):
        with self.assertRaises(UnsafeMotion):
            plan_single_step(self.status, 9, 1)


if __name__ == "__main__":
    unittest.main()
