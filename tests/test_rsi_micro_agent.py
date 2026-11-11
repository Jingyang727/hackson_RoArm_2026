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
