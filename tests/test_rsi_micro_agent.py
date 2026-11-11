"""Real RSI micro-loop checks: no camera, serial or network used."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from evoarm_rsi.real_probe import _ready, _base_url, run_real_probe
from evoarm_rsi.micro_agent import run_micro_loop
from evoarm_rsi.vision_review import _validate_review


HOME={"joint_deg":{str(j):0.0 for j in range(1,7)}}
HEALTH={
    "connected":True,"motion_enabled":True,"armed":True,
    "teach_active":False,"pending":None,"fault":None,
    "torque_status":[1]*6,
}


class MicroAgentTests(unittest.TestCase):
    def test_controller_must_be_local_and_exact_port(self):
        self.assertEqual(_base_url("http://127.0.0.1:8765"),
                         "http://127.0.0.1:8765")
        with self.assertRaises(ValueError):
            _base_url("https://example.com:443")
        with self.assertRaises(ValueError):
            _base_url("http://127.0.0.1:8765/evil")

    def test_physical_preflight_needs_torque_home_and_manual_arm(self):
        with patch("evoarm_rsi.real_probe._degrees",
                   return_value={j:0.0 for j in range(1,7)}):
            accepted=_ready(HEALTH,{},HOME)
            self.assertEqual(accepted["1"],0)
            with self.assertRaisesRegex(RuntimeError,"torque"):
                _ready({**HEALTH,"teach_active":True},{},HOME)
            with self.assertRaisesRegex(RuntimeError,"armed"):
                _ready({**HEALTH,"armed":False},{},HOME)
            with self.assertRaisesRegex(RuntimeError,"fault"):
                _ready({**HEALTH,"fault":{"code":"timeout"}},{},HOME)
            with self.assertRaisesRegex(RuntimeError,"torque feedback"):
                _ready({**HEALTH,"torque_status":[0]*6},{},HOME)

    def test_gravity_offset_on_shoulder_not_block_j1_probe(self):
        with patch("evoarm_rsi.real_probe._degrees",
                   return_value={1:0.,2:-4.04,3:0.,4:0.,5:0.,6:0.}):
            offsets=_ready(HEALTH,{},HOME)
        self.assertAlmostEqual(offsets["2"],-4.04)

    def test_shoulder_offset_over_five_degrees_still_blocked(self):
        with patch("evoarm_rsi.real_probe._degrees",
                   return_value={1:0.,2:-5.1,3:0.,4:0.,5:0.,6:0.}):
            with self.assertRaisesRegex(RuntimeError,"allowed 5.0 deg"):
                _ready(HEALTH,{},HOME)

    def test_other_joint_home_limits_remain_three_degrees(self):
        with patch("evoarm_rsi.real_probe._degrees",
                   return_value={1:0.,2:-4.04,3:0.,4:3.1,5:0.,6:0.}):
            with self.assertRaisesRegex(RuntimeError,"allowed 3.0 deg"):
                _ready(HEALTH,{},HOME)

    def test_readonly_default_does_not_contact_camera_or_send_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp)/"poses.json"
            home.write_text(json.dumps({"poses":{"home":HOME}}))
            with (patch("evoarm_rsi.real_probe._controller_get",
                        side_effect=[{},HEALTH]) as get,
                  patch("evoarm_rsi.real_probe._degrees",
                        return_value={j:0.0 for j in range(1,7)}),
                  patch("evoarm_rsi.real_probe._controller_step") as move,
                  patch("evoarm_rsi.real_probe.client_request") as camera):
                info=run_real_probe(home_path=home,execute=False)
                self.assertFalse(info["robot_moved"])
                move.assert_not_called()
                camera.assert_not_called()
                self.assertEqual(get.call_count,2)

    def test_probe_defaults_to_five_degrees_and_rejects_one_degree(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp)/"poses.json"
            home.write_text(json.dumps({"poses":{"home":HOME}}))
            with (patch("evoarm_rsi.real_probe._controller_get",
                        side_effect=[{},HEALTH]),
                  patch("evoarm_rsi.real_probe._degrees",
                        return_value={j:0.0 for j in range(1,7)}),
                  patch("evoarm_rsi.real_probe._controller_step") as move):
                preview=run_real_probe(home_path=home,execute=False)
            self.assertEqual(preview["safe_jog_deg"],5)
            move.assert_not_called()
        with self.assertRaisesRegex(ValueError,"5 or 10"):
            run_real_probe(degrees=1)
        with self.assertRaisesRegex(ValueError,"5 or 10"):
            run_micro_loop(degrees=3)

    def test_manual_lease_is_30_minutes_not_auto_renewed(self):
        from control_ui.service import SESSION_SECONDS
        self.assertEqual(SESSION_SECONDS,1800)

    def test_motion_confirmation_needs_real_displacement_not_only_status(self):
        from evoarm_rsi.real_probe import _wait_settled
        known_status={"T":1051}
        healthy={**HEALTH,"pending":None,
                 "last_command":{"status":"feedback_confirmed"}}
        with (patch("evoarm_rsi.real_probe._controller_get",
                    side_effect=[known_status,healthy]),
              patch("evoarm_rsi.real_probe._degrees",
                    return_value={1:0.0})):
            with self.assertRaisesRegex(RuntimeError,"J1 moved"):
                _wait_settled("http://127.0.0.1:8765",
                              start_deg=0.0,expected_delta_deg=5.0)
        with (patch("evoarm_rsi.real_probe._controller_get",
                    side_effect=[known_status,healthy]),
              patch("evoarm_rsi.real_probe._degrees",
                    return_value={1:4.5})):
            rows=_wait_settled("http://127.0.0.1:8765",
                               start_deg=0.0,expected_delta_deg=5.0)
        self.assertEqual(len(rows),1)

    def test_agent_abstains_instead_of_repeating_bounded_trial(self):
        with tempfile.TemporaryDirectory() as temp:
            result={
                "camera_capture":{"folder":temp},
                "reverse_confirmed":True,
            }
            class FakeGateway:
                def configured(self):return True
            review={
                "observed_outcome":"not_attempted",
                "next_probe_direction":None,
                "confidence":1.0,
            }
            with (patch("evoarm_rsi.micro_agent.run_real_probe",return_value=result) as run,
                  patch("evoarm_rsi.micro_agent.review_trial",
                        return_value=review) as reviewer,
                  patch("pathlib.Path.home",return_value=Path(temp))):
                outcome=run_micro_loop(iterations=3,execute=True,
                                       gateway=FakeGateway())
            self.assertEqual(outcome["real_trials_completed"],1)
            self.assertFalse(outcome["model_selected_next_action"])
            run.assert_called_once()
            reviewer.assert_called_once()

    def test_motor_stall_is_reviewed_by_grok_but_stops_next_trial(self):
        with tempfile.TemporaryDirectory() as temp:
            physical={
                "camera_capture":{"folder":temp},
                "reverse_confirmed":False,
                "command_attempted":True,
                "motion_error":"motion_stalled_or_offset: no J1 movement",
            }
            class FakeGateway:
                def configured(self):return True
            review={
                "observed_outcome":"not_attempted",
                "failure_type":"no_attempt",
                "next_probe_direction":1,
                "confidence":1,
            }
            with (patch("evoarm_rsi.micro_agent.run_real_probe",return_value=physical) as run,
                  patch("evoarm_rsi.micro_agent.review_trial",return_value=review) as model,
                  patch("pathlib.Path.home",return_value=Path(temp))):
                output=run_micro_loop(iterations=3,degrees=5,execute=True,
                                      gateway=FakeGateway())
            self.assertEqual(output["real_trials_completed"],1)
            self.assertTrue(output["results"][0]["trial_failed"])
            model.assert_called_once()
            self.assertEqual(run.call_count,1)
            self.assertTrue(run.call_args.kwargs["return_on_failure"])
            self.assertEqual(run.call_args.kwargs["degrees"],5)

    def test_company_model_may_choose_one_degree_direction_only(self):
        base={
            "observed_outcome":"not_attempted","confidence":0.8,
            "failure_type":"no_attempt","evidence":["base rotated"],
            "changes":{}, "next_probe_direction":-1,
        }
        result=_validate_review(base,motion_recorded=True,
                                grasp_attempted=False)
        self.assertEqual(result["next_probe_direction"],-1)
        self.assertEqual(result["observed_outcome"],"not_attempted")
        with self.assertRaisesRegex(ValueError,"next_probe_direction"):
            _validate_review({**base,"next_probe_direction":10},
                             motion_recorded=True)
        with self.assertRaisesRegex(ValueError,"exploration budget"):
            _validate_review({**base,"changes":{"offset_x_mm":7}},
                             motion_recorded=True)


if __name__=="__main__":
    unittest.main()
