"""First physical RSI trial: one bounded 5°/10° base-joint probe with real RGB-D video.

Uses the EXISTING controller as the only serial owner and the dedicated
camera UNIX service. NO raw serial and NO direct SDK in this user process.

It only moves if:
- controller already has an explicitly armed motion session,
- torque for all 6 joints is ON, no faults/pending operations,
- current joint feedback agrees with the previously physically taught HOME,
- the Gemini camera service has actually started recording,
- motion is a single 5°/10° BASE (joint 1) jog.

After confirmed feedback, it attempts a bounded reverse to the prior pose.
It DOES NOT try a blind Home/reset after any feedback uncertainty.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time
from urllib import request, error
from urllib.parse import urlsplit

from .camera_service import client_request
from .robot_telemetry import _validate_status_url


def _base_url(url):
    parts=urlsplit(url)
    _validate_status_url(url.rstrip("/")+"/api/status")
    # _validate_status_url requires precisely /api/status, not a base URL.
    return url.rstrip("/")


def _controller_get(base,endpoint):
    req=request.Request(base+endpoint,method="GET")
    with request.urlopen(req,timeout=8) as response:
        return json.loads(response.read(256*1024).decode("utf-8"))


def _controller_step(base,joint,delta):
    payload=json.dumps({"joint":joint,"delta":delta}).encode("utf-8")
    req=request.Request(base+"/api/step",method="POST",data=payload,
                        headers={"Content-Type":"application/json"})
    with request.urlopen(req,timeout=12) as response:
        if response.status!=200:
            raise RuntimeError("robot controller rejected jog")
        return json.loads(response.read(256*1024).decode("utf-8"))


def _degrees(status):
    from control_ui.safety import current_degrees
    return current_degrees(status)


def _ready(health,status,home):
    if not isinstance(health,dict) or not isinstance(status,dict):
        raise RuntimeError("robot controller returned invalid preflight")
    if not all(health.get(k) is True for k in ("connected","motion_enabled","armed")):
        raise RuntimeError("robot controller not connected, motion-enabled and manually armed")
    if health.get("teach_active") is not False:
        raise RuntimeError("robot torque released: restore all six joints manually first")
    if health.get("fault") or health.get("pending"):
        raise RuntimeError("robot controller fault or unresolved motion")
    if health.get("torque_status") != [1]*6:
        raise RuntimeError("all six real torque feedback flags must equal 1")
    baseline=home.get("joint_deg")
    if not isinstance(baseline,dict):
        raise RuntimeError("no recorded HOME joint baseline")
    current=_degrees(status)
    differences={}
    # The taught Home is a *reference*, not a command target. Only J2 has
    # observed ~4 degrees of static gravity-related deviation. Allow 5
    # degrees for J2 solely for a J1 micro-probe; leave all other joints
    # at 3 degrees. This does NOT loosen controller motion limits or
    # accept an actively drifting/unsupported arm.
    home_tolerances={j:(5.0 if j==2 else 3.0) for j in range(1,7)}
    for j in range(1,7):
        saved=baseline.get(str(j))
        if type(saved) not in (int,float) or not math.isfinite(saved):
            raise RuntimeError("invalid taught HOME joint angles")
        differences[str(j)]=round(current[j]-saved,3)
        if abs(differences[str(j)])>home_tolerances[j]:
            raise RuntimeError(
                f"robot joint {j} is {differences[str(j)]:+.2f} deg from taught "
                f"HOME (allowed {home_tolerances[j]:.1f} deg)"
            )
    return differences


def _wait_settled(base, *, start_deg, expected_delta_deg, timeout_s=10):
    until=time.monotonic()+timeout_s
    seen=[]
    while time.monotonic()<until:
        status=_controller_get(base,"/api/status")
        health=_controller_get(base,"/api/health")
        seen.append({
            "host_monotonic_ns":time.monotonic_ns(),
            "robot_status":status,
            "pending":health.get("pending"),
            "fault":health.get("fault"),
        })
        if health.get("fault"):
            raise RuntimeError("robot reported fault; no automatic reversal")
        if not health.get("pending"):
            # A settled position without real progress is never success.
            # Controller distinguishes feedback_confirmed from faults and
            # settled_with_offset; we independently compare actual J1 motion.
            last = health.get("last_command") or {}
            if last.get("status") != "feedback_confirmed":
                raise RuntimeError(
                    "controller did not confirm actual motion: "
                    + str(last.get("status", "no last command"))
                )
            actual = _degrees(status)[1]
            displacement = actual-start_deg
            required = 0.6 * abs(expected_delta_deg)
            if displacement*expected_delta_deg <= 0 or abs(displacement) < required:
                raise RuntimeError(
                    f"J1 moved {displacement:+.2f}° but needed at least "
                    f"{required:.2f}° in commanded direction"
                )
            return seen
        time.sleep(0.25)
    raise RuntimeError("robot move did not settle; no automatic reversal")


def run_real_probe(*,controller="http://127.0.0.1:8765",
                   direction=1, degrees=5, execute=False,
                   home_path=None, camera_home=None,
                   return_on_failure=False):
    # Distinct flag ensures invoking the module without --execute is
    # a read-only preflight even if the controller is armed.
    if direction not in (-1,1):
        raise ValueError("direction must be +1 or -1")
    if type(degrees) is not int or degrees not in (5,10):
        raise ValueError("probe step must be 5 or 10 degrees")
    base=_base_url(controller)
    home_path=Path(home_path).expanduser() if home_path else (
        Path.home()/".evoarm"/"pick_poses_v2.json"
    )
    saved=json.loads(home_path.read_text(encoding="utf-8"))["poses"]["home"]
    # Status refreshes the controller's actual torque feedback flags.
    status=_controller_get(base,"/api/status")
    health=_controller_get(base,"/api/health")
    differences=_ready(health,status,saved)
    if not execute:
        return {"status":"ready_preview","safe_jog_deg":direction*degrees,
                "probe_step_deg":degrees,
                "joint":1,"home_joint_difference_deg":differences,
                "home_tolerance_deg":{"2":5.0,"other_joints":3.0},
                "board_visibility_required":False,
                "robot_moved":False}
    home=camera_home or Path.home()
    cam=client_request(home,"GET","/health")
    if cam.get("robot_control") is not False or cam.get("camera_service")!="ready":
        raise RuntimeError("expected dedicated camera-only service")
    if cam.get("recording"):
        raise RuntimeError("another camera recording is already active")
    capture_result={}
    capture_errors=[]
    def capture():
        try:
            capture_result.update(
                client_request(home,"POST","/record",{"seconds":10.0})
            )
        except Exception as exc:
            capture_errors.append(type(exc).__name__)
    thread=threading.Thread(target=capture,daemon=True)
    thread.start()
    # Do not move until the camera recorder confirms it has the FIRST
    # complete RGB+Depth frame and its true recording clock is running.
    started=False
    deadline=time.monotonic()+14
    while time.monotonic()<deadline and thread.is_alive():
        state=client_request(home,"GET","/health")
        if state.get("phase")=="recording" and state.get("recording_started_monotonic_ns"):
            started=True
            break
        time.sleep(0.15)
    if not started:
        thread.join(timeout=15)
        raise RuntimeError("camera failed to confirm active RGB-D; robot did not move")
    history=[{"kind":"pre_move","host_monotonic_ns":time.monotonic_ns(),
              "status":status}]
    moved=False
    confirmed_forward=False
    command_attempted=False
    reverse_attempted=False
    reverse_confirmed=False
    failure=None
    try:
        # All controller state is re-read at execution time.
        fresh_status=_controller_get(base,"/api/status")
        fresh_health=_controller_get(base,"/api/health")
        _ready(fresh_health,fresh_status,saved)
        # HTTP timeout AFTER a POST may mean the controller ACTED but the
        # response was lost. Mark the attempt BEFORE sending the request.
        command_attempted=True
        history.append({"kind":"move_request_attempted",
                        "host_monotonic_ns":time.monotonic_ns(),
                        "joint":1,"delta_deg":direction*degrees})
        before_deg = _degrees(fresh_status)[1]
        command=_controller_step(base,1,direction*degrees)
        moved=True
        history.append({"kind":"move_command","host_monotonic_ns":time.monotonic_ns(),
                        "joint":1,"delta_deg":direction*degrees,"controller_ack":command})
        history.extend(_wait_settled(
            base,start_deg=before_deg,expected_delta_deg=direction*degrees
        ))
        confirmed_forward=True
        history.append({"kind":"forward_motion_confirmed",
                        "host_monotonic_ns":time.monotonic_ns(),
                        "requested_delta_deg":direction*degrees})
        # The reverse is attempted only after actual feedback settled,
        # and only if torque remains fully on and no faults are latched.
        fresh_health=_controller_get(base,"/api/health")
        if (fresh_health.get("fault") or fresh_health.get("pending") or
                fresh_health.get("torque_status") != [1]*6):
            raise RuntimeError("post-jog unsafe state; no automatic reversal")
        reverse_attempted=True
        history.append({"kind":"reverse_request_attempted",
                        "host_monotonic_ns":time.monotonic_ns(),
                        "joint":1,"delta_deg":-direction*degrees})
        reverse_start=_degrees(_controller_get(base,"/api/status"))[1]
        reverse_ack=_controller_step(base,1,-direction*degrees)
        history.append({"kind":"reverse_command","host_monotonic_ns":time.monotonic_ns(),
                        "joint":1,"delta_deg":-direction*degrees,"controller_ack":reverse_ack})
        history.extend(_wait_settled(
            base,start_deg=reverse_start,expected_delta_deg=-direction*degrees
        ))
        reverse_confirmed=True
    except Exception as exc:
        failure=type(exc).__name__+": "+str(exc)
    finally:
        thread.join(timeout=20)
    if capture_errors and not failure:
        failure="camera_capture_error: "+",".join(capture_errors)
    if thread.is_alive() and not failure:
        failure="camera_capture_thread_stalled"
    result={
        "kind":"physical_single_joint_probe",
        "requested_step_deg":direction*degrees,
        "command_attempted":command_attempted,
        "command_acknowledged":moved,
        "robot_moved":confirmed_forward,
        "robot_motion_confirmed":confirmed_forward,
        "forward_motion_confirmed":confirmed_forward,
        "motion_may_have_occurred":command_attempted,
        "reverse_attempted":reverse_attempted,
        "reverse_confirmed":reverse_confirmed,
        "motion_error":failure,
        "camera_capture_errors":capture_errors,
        "camera_capture":capture_result,
        "controller_feedback":history,
        "model_not_called":True,
        "grasp_not_attempted":True,
    }
    # Store derived motion evidence separately from camera-owned raw trial.
    # Camera-service file ownership is user-readable after success.
    trial_id=capture_result.get("trial_id")
    if trial_id:
        out=Path.home()/".evoarm"/"rsi"/"analysis"/trial_id/"physical_probe.json"
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
        result["report_file"]=str(out)
    if failure and not return_on_failure:
        raise RuntimeError(
            f"physical probe stopped: {failure}; command_attempted={command_attempted}; "
            f"reverse_confirmed={reverse_confirmed}; inspect physical_probe report"
        )
    return result


def main(argv=None):
    p=argparse.ArgumentParser(
        description="Camera-synchronized single 5°/10° real RoArm base probe"
    )
    p.add_argument("--controller",default="http://127.0.0.1:8765")
    p.add_argument("--direction",type=int,choices=(-1,1),default=1)
    p.add_argument("--degrees",type=int,choices=(5,10),default=5)
    p.add_argument("--execute",action="store_true",
                   help="send bounded motor step ONLY if existing controller is already armed")
    args=p.parse_args(argv)
    print(json.dumps(run_real_probe(
        controller=args.controller,direction=args.direction,
        degrees=args.degrees,execute=args.execute),ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
