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

The adapter defaults to a **90-second response timeout**, configurable with
`EVOARM_COMPANY_LLM_TIMEOUT_S` (5–300). It reads up to **2 MiB** of
response data, ignores `reasoning_content`, and extracts exactly one final
`{"changes": {...}}` JSON object from `message.content`. Gateway transport
errors report the underlying timeout/TLS/DNS/proxy reason without displaying
request headers or the token. If running the service as an existing process,
restart it with the approved environment loaded; sourcing the env file into
a different terminal does not change a process that is already running.

Then post `{"use_company_agent":true}` to `/api/rsi/mock-trial`.
The adapter sends only the structured mock policy and evaluation to the
configured endpoint. It has no external provider fallback and does not send
video. It accepts only a JSON `{"changes":{"offset_x_mm":...}}` result,
validates all fields and numeric bounds and stores the proposed policy for
the **next** trial. It never turns a model completion into a motor command.
If your gateway is not OpenAI-compatible, implement an adapter with the same
`propose_next(policy, evaluation)` interface.

## P1: first REAL RGB-D observation (ready for local verification)

The physical Gemini is supported by the new
`evoarm_rsi.orbbec.Gemini335LProvider`. It opens the Orbbec v2 SDK,
collects RGB and Depth, software-aligns depth to RGB, converts raw scaled
depth values into millimeters, and timestamps frames.

For your first **camera-only 10-second real observation** on your Mac:

```bash
conda activate evoarm
python -m unittest tests.test_rsi_observe -v
python -m evoarm_rsi.observe --camera gemini --seconds 10
```

Output: `~/.evoarm/rsi/experiments/<trial_id>/` with `rgb.mp4`,
`frames.jsonl`, `depth_samples.npz`, `manifest.json`,
`observation.json`, and `events.jsonl`.

- `simulated: false` means real camera footage only.
- `robot_motion_executed: false` and `agent_called: false` are explicit;
  this is **not** yet a grasp attempt or self-learning outcome.
- A 10-second live capture is a real elapsed window, unlike P0's
  logical mock timer.
- The recorder uses 20fps as a **target**, and preserves actual frame
  timestamps; hardware frames may be delayed or dropped.
- The optional `--store /absolute/path` controls evidence destination.
  If the macOS SDK still needs elevated permissions, do not run the whole
  FastAPI/robot process as root. Check the experiment directory ownership
  before using privileged capture.
- Depth samples are sparse (up to 32 aligned frames). This is sufficient
  for initial SDK/geometry validation but not yet full-rate RGB-D replay.
- The camera frame must include supported BGR/RGB/MJPG/YUYV color bytes;
  unsupported formats fail explicitly rather than writing invalid video.

Do not reuse the former USB camera's pixel calibration for Gemini. The
saved robot XY calibration and poses remain untouched.

## P1/P2 remaining work (NOT implemented)

- Orbbec Gemini 335L RGB/depth calibration, preserving prior
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
