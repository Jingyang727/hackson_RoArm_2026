"""Evidence-driven micro-RSI loop: physical probe -> RGB+Depth -> Grok -> next.

NOT a free-space motion planner. First production slice is bounded to
+/-1 degree joint-1 from an explicitly taught Home baseline. Existing
controller remains sole serial owner, operator unlock is never automated,
and model decisions are never trusted as direct motor payloads.

Each iteration captures real Gemini RGB-D footage via the owner-only camera
service and T=1051 controller telemetry. Uncertain evidence => stop.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gateway import CompanyModelGateway
from .real_probe import run_real_probe
from .vision_review import review_trial


def run_micro_loop(*, iterations=1, initial_direction=1, execute=False,
                   controller="http://127.0.0.1:8765", gateway=None):
    if type(iterations) is not int or not 1<=iterations<=3:
        raise ValueError("max 3 physical experiments per batch")
    if initial_direction not in (-1,1):
        raise ValueError("invalid first direction")
    if not execute:
        return run_real_probe(controller=controller,direction=initial_direction,
                              execute=False)
    gateway=gateway or CompanyModelGateway()
    if not gateway.configured():
        raise RuntimeError("company Grok gateway must be configured before ANY motion")
    direction=initial_direction
    results=[]
    for step in range(iterations):
        physical=run_real_probe(
            controller=controller,direction=direction,execute=True
        )
        trial=physical["camera_capture"].get("folder")
        if not trial:
            raise RuntimeError("physical trial has no saved RGB-D evidence")
        review=review_trial(trial,gateway=gateway,frames=5)
        out=Path.home()/".evoarm"/"rsi"/"analysis"/Path(trial).name/"visual_review.json"
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding="utf-8")
        results.append({
            "trial":Path(trial).name,
            "direction":direction,
            "motion_confirmed":physical["reverse_confirmed"],
            "vision_review":review,
            "review_file":str(out),
        })
        if not physical["reverse_confirmed"]:
            break
        proposed=review.get("next_probe_direction")
        if proposed not in (-1,1):
            # Model abstention / insufficient visual evidence is a normal,
            # inspectable stop, not a reason to repeat a guessed action.
            break
        direction=proposed
    return {"real_trials_completed":len(results),
            "model_selected_next_action":len(results)>1,
            "results":results,
            "robot_home_reset_unconditionally":False}


def main(argv=None):
    p=argparse.ArgumentParser(
        description="Small real RoArm RSI loop: Gemini RGB-D + model video-frame review"
    )
    p.add_argument("--iterations",type=int,default=1)
    p.add_argument("--direction",type=int,choices=(-1,1),default=1)
    p.add_argument("--controller",default="http://127.0.0.1:8765")
    p.add_argument("--execute",action="store_true",
                   help="run physical bounded trial ONLY after manual controller arming")
    args=p.parse_args(argv)
    result=run_micro_loop(
        iterations=args.iterations,initial_direction=args.direction,
        execute=args.execute,controller=args.controller
    )
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
