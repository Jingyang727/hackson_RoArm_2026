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
- The camera provider discards transient RGB-only, Depth-only, null-alignment
  and incomplete aligned frame sets for up to **8 seconds per read** (maximum
  240 discarded sets), then fails with the last incomplete-frame reason.
  Valid RGB-D frames are required; persistent SDK faults are not concealed.
- The 10-second recording clock begins **after** the first valid aligned
  RGB-D frame, so startup warmup does not shorten the evidence window.
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

## P1.5: synchronized real RGB-D and read-only RoArm data

Your first camera-only trial captured real RGB and depth successfully. The
next trial can also poll the **existing FastAPI serial owner** without issuing
motor commands or opening the serial device twice.

1. Start the existing EvoArm control service as a regular user, bound to
   127.0.0.1. Connect the RoArm as usual.
2. Verify the actual API port and that `curl -s http://127.0.0.1:8765/api/status`
   returns one T=1051 feedback JSON (substitute your real service port).
3. In the separately authorized camera terminal, run:

```bash
python -m evoarm_rsi.observe --camera gemini --seconds 10 \
  --robot-status-url http://127.0.0.1:8765/api/status
```

If you still need elevated permissions for **camera capture only**, use the
same Python module via the manually authorized terminal. Never launch the
entire robot control API as root. The status URL only accepts localhost
`GET /api/status`; no motor control endpoint can be provided.

The recorded `trajectory.jsonl` samples have wall-clock and monotonic
request start/end timestamps. The capture source keeps per-frame monotonic
timestamps, so camera and robot data can be aligned approximately on the
same host. A status request may be slow: the sample interval is a target,
not an assumed device sampling rate. Missing status data are explicitly
recorded rather than fabricated.

For any saved trial, inspect evidence coverage without network access:

```bash
python -m evoarm_rsi.audit ~/.evoarm/rsi/experiments/<trial_id>
```

This remains a **read-only experiment**. No grasp attempts, autonomous retry,
3D TCP registration or motion control have been implemented yet. The
camera-only trial and the robot-data trial are preparatory milestones for
the first true learning loop.

## Trial 003 — offline Gemini RGB-D localization (NO motion)

Use Trial 002 or another real Gemini capture containing the yellow ball:

```bash
python -m unittest tests.test_rsi_localize -v
python -m evoarm_rsi.localize \
  ~/.evoarm/rsi/experiments/f830fd33ce6b41869d85285a48f7274c
```

Produces `rgbd_localization.json` in the same trial directory. It aligns
sparse depth samples to RGB video **by recorded monotonic timestamps** rather
than assuming 20fps; runs the existing yellow-ball detector and, when the
full 9×6 board is independently detected in the same RGB image, estimates
board-plane XY. For each observation it stores center pixel, ball surface
depth, nearby-background depth, depth contrast and pairing time offset.

**Important:** Background-vs-sphere depth contrast is a rough apparent
depth difference, not a fitted tabletop plane or certified ball height.
Occluded checkerboard corners may prevent live board XY; the program
reports this rather than loading stale old-camera corners.

A 3D camera-space point can optionally be calculated using a separately
verified **Gemini COLOR-aligned** intrinsics JSON:

```json
{
  "fx": 0, "fy": 0, "cx": 0, "cy": 0,
  "aligned_to": "color",
  "verified": false
}
```

Those zeros are placeholders only: **do not use this file as-is**. Obtain
actual fx/fy/cx/cy from the SDK's active color stream calibration and
verify them, set `verified` true after checking image resolution and
alignment, then pass `--intrinsics /path/to/gemini_intrinsics.json`.
A camera-space point is still **not a robot-base target**. This stage
intentionally does not produce robot XYZ or move any actuators until
camera-to-robot extrinsics, tool TCP and collision constraints have been
validated.

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
