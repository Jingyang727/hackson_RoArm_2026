# EvoArm RSI — P0 experiment infrastructure

**Status: simulated trials only.** This module cannot command a RoArm. The
existing `/pick` and `/teach` flows and saved calibration files are unchanged.

## Local mock run

Start the existing FastAPI service as usual, then inspect:

```bash
curl -s http://127.0.0.1:8765/api/rsi/status
curl -s -X POST http://127.0.0.1:8765/api/rsi/mock-trial \
  -H 'Content-Type: application/json' \
  -d '{"action_window_s":10}'
```

Each mock run records a manifest, policy, observation, command and trajectory
events, evaluation, and simulated Home reset in
`~/.evoarm/rsi/experiments/<trial_id>/`. The 10-second period is a **logical
simulation budget**; the simulator does not sleep ten seconds. All observations
and joint trajectories are labeled simulated. No video or real joint feedback
is claimed at this stage.

A local read-only query is available at `GET /api/rsi/trials/<trial_id>`.
Do not expose the FastAPI instance on a public network; keep the service bound
to `127.0.0.1`.

## Optional company model gateway

The default mode makes **no model requests**. Use ONLY your organization's
approved OpenAI-compatible internal gateway. Keep credentials out of source
control and GitHub Actions:

```bash
export EVOARM_COMPANY_LLM_BASE_URL='https://<approved-internal-gateway>/v1'
export EVOARM_COMPANY_LLM_MODEL='<internal-model-id>'
read -rs EVOARM_COMPANY_LLM_TOKEN
export EVOARM_COMPANY_LLM_TOKEN
```

Then post `{"use_company_agent":true}` to `/api/rsi/mock-trial`.
The adapter sends only the structured mock policy and evaluation to the
configured endpoint. It has no external provider fallback and does not send
video. It accepts only a JSON `{"changes":{"offset_x_mm":...}}` result,
validates all fields and numeric bounds and stores the proposed policy for
the **next** trial. It never turns a model completion into a motor command.
If your gateway is not OpenAI-compatible, implement an adapter with the same
`propose_next(policy, evaluation)` interface.

## P1/P2 remaining work (NOT implemented)

- Orbbec Gemini 335L acquisition + RGB/depth calibration, preserving prior
  five-point mechanical XY samples but not reusing the old camera intrinsics.
- Capture real RGB/depth video with synchronized monotonic timestamps,
  actual T=1051 joint feedback, real command acknowledgments and event frames.
- Verified tabletop workspace, 3D TCP, IK and collision-checked motion
  adapter. P0 rejects any `hardware=True` robot unconditionally.
- Reset on an unobstructed tabletop after a normal trial; faults stop
  autonomous recurrence instead of blindly returning home.
- A repeat-until-success loop driven by objective visual grasp outcomes;
  reset the lightweight ball between trials where needed.
- Offline Langfuse spooler for company-authorized deployment; local evidence
  remains the primary record and should not upload images without approval.

## Tests

```bash
python -m unittest tests.test_rsi_core -v
python -m unittest discover -s tests -v
```

GitHub Actions is configured to run on `feature/**` pushes. Do not switch a
physical robot service solely because simulation tests pass.
