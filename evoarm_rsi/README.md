# EvoArm RSI — real RGB-D, evidence-driven micro-experiments

**Status:** RGB-D recording and read-only RoArm telemetry are physically
verified. The bounded physical single-joint probe and company-model
multimodal review are now implemented but **NOT YET hardware-validated**
or CI-verified. The original `/pick` and `/teach` flows are unchanged.
No unattended open-space grasping is claimed.

## RSI v0.1 — real movement, vision feedback, learn next direction

### Why this is not another mock-only milestone

The real loop (not run automatically on install):

1. The **dedicated Gemini capture process** starts recording actual RGB-D.
2. Only after its first valid aligned frame, the existing RoArm controller
   receives exactly **one ±1° base-joint step**, starting from your
   manually taught Home joint configuration (within 3°).
3. The existing controller verifies real feedback. If it reports no fault
   and the move is settled, a bounded reverse step is attempted.
   **No blind Home motion occurs on any error.**
4. The real RGB video, per-frame timestamps, depth statistics and
   controller responses are linked in an analysis file.
5. Your company-only Grok endpoint receives 5 chronologically ordered
   RGB keyframes (small JPEGs via OpenAI-compatible `image_url`) plus
   structured depth and motion facts. Grok may suggest the next
   **±1° base direction**, or abstain. An abstention ends the loop.

This is **real exploratory movement**, NOT a grasp or general trajectory.
Nothing is uploaded to a public fallback. Model vision support on
`grok-4.7-build` must be empirically verified through your company gateway.
The explicit image+metadata request distinguishes this from uploading a raw
MP4; video/file input support is not assumed. RGB frames are sent; for now
depth is provided as numeric scene statistics rather than large colored
depth maps. Later add cropped depth near ball/TCP when grasp evaluation
requires it.

### Persistent camera service (eliminates repeated sudo in OpenCode)

From a terminal where Gemini 335L has previously opened under sudo:

```bash
cd ~/workdir/hackson_RoArm_2026
sudo /Users/jyang51/miniconda3/envs/evoarm/bin/python \
  -m evoarm_rsi.camera_service serve
```

Leave that camera-only terminal running. Its socket lives at
`~/.evoarm/rsi_camera.sock`, owned by the invoking user, mode `0600`.
There is **no TCP listener and no robot motion endpoint**. In a separate
ordinary user terminal or OpenCode:

```bash
python -m evoarm_rsi.camera_service health
python -m evoarm_rsi.camera_service record --seconds 10
```

New trial data are returned to the user's ownership after capture, including
failed trial artifacts when possible. This does NOT remove macOS's UVC
restriction; it isolates the required privileged camera access into a
manually started process instead of granting OpenCode root.

### First bounded real RSI action (requires operator preparation ONCE)

The arm is presently torque-released; **physically support it** until
torque has been restored using the normal `/teach` controls. Place it
in the previously recorded Home pose, clear the arm's swept region, start
the ordinary user control UI with `EVOARM_ENABLE_MOTION=1`, connect to
the RoArm, and explicitly unlock its existing timed motion session.
Do not enable torque or unlock via AI code. The existing arm service is
the only serial owner.

Run the new tests first:

```bash
python -m unittest tests.test_rsi_micro_agent \
  tests.test_rsi_camera_service tests.test_rsi_vision_review -v
```

Then check the controller preflight **without motion**:

```bash
python -m evoarm_rsi.real_probe --controller http://127.0.0.1:8765
```

When all prerequisites pass and a person is physically present at the
robot's emergency stop, start **one** real trial:

```bash
set -a
source ~/.evoarm/company_llm.env
set +a
python -m evoarm_rsi.micro_agent --execute --iterations 1
```

Only after this first real motion has been independently reviewed should
you raise `--iterations` to 2–3. **Never** exceed three trials per
batch. The controller's timed arming expires automatically, so this code
does not silently rearm it. If Grok does not support `image_url` or the
camera UVC stream fails, no further iteration starts.

Evidence is written to
`~/.evoarm/rsi/analysis/<trial_id>/physical_probe.json` and
`visual_review.json`. A single 1° movement must never be labeled
"grasp success" even if the ball is visible in RGB. There is still no
3D collision-free grasp execution until tool/scene geometry is validated.

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
verified **Gemini COLOR-aligned** intrinsics JSON. **New recordings** now
automatically save `gemini_color_intrinsics.json` from the *active*
`Pipeline.get_camera_param().rgb_intrinsic` if SDK calibration is valid.
The file has `verified: false` by default because an SDK value alone does
not verify RGB/depth optical alignment or camera-to-robot calibration.

Older trials (including Trial 002) do not contain that file; they can still
run offline 2D + depth analysis without repeating a capture.

Expected structure:

```json
{
  "fx": 0, "fy": 0, "cx": 0, "cy": 0,
  "aligned_to": "color",
  "verified": false
}
```

Those zeros are placeholders only: **do not use this file as-is**.
For a new trial, inspect the automatically saved SDK parameters and verify
their image resolution against `rgb.mp4` and the RGB-to-depth alignment.
Only after independent verification, copy the candidate to a separate
approved calibration file, change `verified` to true, and pass
`--intrinsics /path/to/approved_intrinsics.json`.
A camera-space point is still **not a robot-base target**. This stage
intentionally does not produce robot XYZ or move any actuators until
camera-to-robot extrinsics, tool TCP and collision constraints have been
validated.

## Trial 003.1 — new Gemini geometry + robust local analysis

Trial 002 already established 32/32 ball depth observations, but no 54-corner
board or RGB intrinsics were captured. No need to redo the old RoArm XY
calibration.

The offline analyzer now defaults to writing its report in the *current
user's* `~/.evoarm/rsi/analysis/<trial_id>/rgbd_localization.json`.
Raw trial directories (possibly root-owned) are left unchanged. Optional:
`--output /writable/path/report.json`.

To obtain a new same-camera board reference in a single recording:

1. Put the Gemini in its fixed experimental position; keep the entire 9×6
   inner-corner board visible and initially remove the yellow ball.
2. Start a new 10-second RGB-D recording using the authorized camera terminal.
   For the first ~2–3 seconds leave the board empty; then place the yellow
   ball on the board **without moving the camera or board**.
3. The camera code attempts to persist
   `gemini_color_intrinsics.json` from the active SDK stream. Its
   `verified` flag intentionally remains false until geometry is checked.
4. Analyze the new trial as a regular user:

```bash
python -m evoarm_rsi.localize ~/.evoarm/rsi/experiments/<new_trial_id>
```

The analyzer detects full checkerboard frames and can reuse a board mapping
seen **earlier within the same trial** for subsequent ball-obscured frames.
Those values are tagged `same_trial_board_reference_unverified` and are
**preview-only**, never an authorization to send a robot command. The RGB-D
localization will NOT silently use old USB camera corners. Even with
intrinsics, robot XYZ stays null until camera-to-robot extrinsics and TCP
validation have been completed.

## Trial 003.1 camera startup and countdown (November 11 fix)

If the SDK opens Pipeline but sends **zero frames**, the capture now
reports `Gemini stream stalled: received NO frames` separately from
transient RGB-only/depth-only/alignment warmup. Neither error should be
mistaken for successful recording, and both preserve the failed manifest.
Zero frames for 8 seconds can indicate a blocked USB data stream, another
camera owner, or SDK/device state; the error label is diagnostic, not a
guaranteed fix to the underlying UVC stream. Close other camera apps,
reconnect USB directly, and run capture from the terminal that previously
opened the device.

The CLI now prints `[CAM] Waiting...` while the first complete aligned
RGB-D frame is sought. **Only after that frame arrives** will it print
`[REC] Recording started` and begin a real-time countdown in a separate
thread. Keep the empty board visible for the first 3 seconds; at
`[REC] 3.0s elapsed — PLACE THE YELLOW BALL NOW` put the ball in place.
Remaining seconds are printed every second until 10s, independent of
blocking SDK frame reads.

```bash
python -m unittest tests.test_rsi_orbbec tests.test_rsi_observe -v
sudo /Users/jyang51/miniconda3/envs/evoarm/bin/python \
  -m evoarm_rsi.observe --camera gemini --seconds 10
```

No robot motion is involved; if the arm is torque-released, keep it
physically supported during this camera-only trial. Existing calibrations
are never overwritten.

## Trial 003.1 independent qualification from saved footage

A first analysis of real trial `bca3c4c1d5c449368a2e149f39b8479f`
found nine yellow-object false positives outside the 9×6 inner-corner
board. Repeated detection alone is not an on-board ball confirmation.

A new offline `qualify` command requires a stable on-board cluster and
a positive ball-to-background depth contrast, and combines that cluster
with the *old measured and held-out validated* 2D board→RoArm transform:

```bash
python -m unittest tests.test_rsi_qualify -v
python -m evoarm_rsi.qualify \
  ~/.evoarm/rsi/experiments/bca3c4c1d5c449368a2e149f39b8479f
```

Result is written in the user-owned analysis directory as
`ball_qualification.json` and includes the *preview-only* board XY and
RoArm-base XY. **Never directly command the robot to this XY.** Whether
the board's physical origin/orientation still match the old calibration
must be verified using known reference points. No robot-base Z,
tool-center-point or collision-free trajectory has been calibrated.

Independent geometric review of this trial reconstructed 93 neighboring
3D chessboard-corner distances: their median was about 23.88mm against
the printed 24mm grid pitch. This supports the SDK color intrinsics and
depth scale locally, **but does not verify a camera→robot 3D transform**.

## Next step: read-only grasp preflight using Trial 003.1

The complete captured trial includes `robot_xy_calibration.json` and
`pick_poses_v2.json` snapshots in its exported ZIP. Existing local paths
are used by default, so do not copy them into the live calibration files if
your newer values differ.

```bash
python -m unittest tests.test_rsi_qualify tests.test_rsi_grasp_preflight -v
TRIAL=~/.evoarm/rsi/experiments/bca3c4c1d5c449368a2e149f39b8479f
python -m evoarm_rsi.qualify "$TRIAL"
python -m evoarm_rsi.grasp_preflight "$TRIAL"
```

The preflight compares measured ball-to-board XY, the saved validated 2D
board-to-RoArm fit (held-out V1 error 3.94mm) and the recorded six
teach poses. In this trial, the preview ball point was approximately
(360.676, -11.034)mm and the saved grasp pose (361.050, -1.108)mm,
i.e. **9.933mm apart in XY**. The output does NOT provide any
grasp-Z, motion command, collision-free path, or permission to move.

Before hardware movement, verify the original board's physical origin and
axis orientation against the current Gemini view, and establish real TCP/
grasp height relative to the *robot* coordinate frame. The previous XY
validation is a measurement of the saved planar model, not proof that
the board did not move after it was originally measured.

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
