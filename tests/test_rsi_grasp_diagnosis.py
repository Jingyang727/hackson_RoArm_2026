"""Hardware-free tests for timestamped diagnosis of a stalled high T104."""
import math
import unittest
from evoarm_rsi.grasp_diagnosis import summarize_grasp_step,diagnose_report


def feedback(x,y,z,base,shoulder,pitch,voltage=1210):
    return {
        "x":x,"y":y,"z":z,
        "b":math.radians(base),"s":math.radians(shoulder),
        "e":math.pi/2,"t":0.0,"r":0.0,"g":3.10,
        "tit":math.radians(pitch),
        "v":voltage,"tB":11,"tS":84,"tE":20,
        "tT":6,"tR":4,"tG":4,
    }


class DiagnosticTests(unittest.TestCase):
    def test_real_first_stage_error_is_reported_not_falsely_successful(self):
        start=feedback(360.1,-2.1,192.7,0.3,-4.0,7.4)
        end=feedback(363.32,-2.23,181.14,0.4,-4.4,7.38,1195)
        step={
            "stage":"current_high_align",
            "target_xyz_mm":[360.88,-11.28,192.74],
            "feedback_trace":[start,end],
            "fault_snapshot":{"code":"cartesian_stalled"}
        }
        result=summarize_grasp_step(step)
        self.assertAlmostEqual(result["euclidean_residual_mm"],14.91,delta=.03)
        self.assertAlmostEqual(result["residual_target_minus_actual_xyz_mm"][1],-9.05)
        self.assertAlmostEqual(result["residual_target_minus_actual_xyz_mm"][2],11.6)
        self.assertFalse(result["reached_target"])
        self.assertAlmostEqual(result["joint_feedback_delta_deg"]["J1_base"],.1)
        self.assertAlmostEqual(result["min_voltage_v"],11.95)
        self.assertAlmostEqual(result["angular_y_difference_deg_at_xy_radius"],-1.39,delta=.10)

    def test_old_real_command_payload_can_be_inspected_offline(self):
        stage={
            "stage":"current_high_align",
            "target_xyz_mm":[360.88,-11.28,192.75],
            "ack":{"sent":True,"command":{
                "T":104,"x":360.88,"y":-11.28,"z":192.75,
                "t":0.129,"r":0.01,"g":2.588,"spd":0.08,
            }},
        }
        diag=summarize_grasp_step(stage)
        self.assertTrue(diag["motor_commands_sent"])
        self.assertEqual(diag["actual_sent_motor_payload"]["T"],104)
        self.assertAlmostEqual(diag["command_vs_stage_target"]["g"],2.588)
        self.assertIsNone(diag["reached_target"])

    def test_previous_fault_snapshot_needs_no_new_physical_trial(self):
        old={"stage":"current_high_align",
             "target_xyz_mm":[360.88,-11.28,192.74],
             "fault_snapshot":{
                 "code":"cartesian_stalled",
                 "actual_xyz_mm":[363.32,-2.23,181.14],
             }}
        report=summarize_grasp_step(old)
        self.assertEqual(report["valid_xyz_samples"],0)
        self.assertAlmostEqual(report["euclidean_residual_mm"],14.91,delta=.03)
        self.assertAlmostEqual(report["residual_target_minus_actual_xyz_mm"][1],-9.05)
        self.assertIsNone(report.get("joint_feedback_delta_deg"))
        self.assertFalse(report["reached_target"])

    def test_old_report_does_not_fabricate_kinematics(self):
        report={
            "kind":"rsi_v2_physical_grasp_and_lift",
            "source_ball_trial_id":"previous",
            "steps":[{"stage":"current_high_align",
                      "target_xyz_mm":[360,-11,204.4],
                      "fault_snapshot":{"code":"cartesian_stalled"}}],
        }
        result=diagnose_report(report)
        self.assertEqual(result["steps"][0]["valid_xyz_samples"],0)
        self.assertIsNone(result["steps"][0]["reached_target"])
        self.assertTrue(result["read_only"])


if __name__=="__main__":
    unittest.main()
