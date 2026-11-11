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
   receives exactly **one ±5°/±10° base-joint step**, starting from your
   manually taught Home joint configuration (within 3° for J1/J3–J6,\n   5° for J2 shoulder only during this J1 micro-probe).
3. The existing controller verifies real feedback. If it reports no fault
   and the move is settled, a bounded reverse step is attempted.
   **No blind Home motion occurs on any error.**
4. The real RGB video, per-frame timestamps, depth statistics and
   controller responses are linked in an analysis file.
5. Your company-only Grok endpoint receives 5 chronologically ordered
   RGB keyframes (small JPEGs via OpenAI-compatible `image_url`) plus
   structured depth and motion facts. Grok may suggest the next
   **±5°/±10° base direction**, or abstain. An abstention ends the loop.

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

## Trial 004: Home visibility, 30-minute authorization, shoulder gravity offset

- A **complete checkerboard is not required** during the first physical
  ±5°/±10° J1 base probe: RGB-D and robot feedback can record the motion when
  Home occludes some corners. The checkerboard is required again for
  ball localization and 3D calibration in subsequent *grasp* trials.
- Manually granted motion authorization now lasts **1800 seconds**, not
  120. It is never auto-renewed, and automatically expires or is revoked
  on fault / disarm. Restart the ordinary controller process after pulling
  the code before expecting the new duration.
- The physically observed **J2 shoulder -4.04°** from taught Home is
  accepted in the J1 probe's *starting-pose comparison* by widening
  J2-only tolerance to **5°**. Other joints remain **3°**.
  No saved Home, calibration or shoulder target is overwritten.
- This exception does **NOT** apply to arbitrary grasp trajectories or
  motion verification. J2 commands that fail to show real joint progress
  still stop and fault according to controller feedback. Do not blindly
  increase servo current or declare motion successful from a small-angle
  command alone.
- Avoid repeatedly jogging the gravity-loaded shoulder solely to chase
  a numeric Home difference when it would make the camera view worse.
  If J2 is continuously sagging, or its holding torque fails under load,
  physically support it, stop trials and inspect mounting, supply and
  mechanical load before continuing.
- The first physical probe deliberately has no board-derived target:
  `board_visibility_required=false` in its preflight result.

## Trial 004 failure diagnosis and bounded next attempt

Physical Trial `d124b8c3a0464c1299ee929c5c51e743` sent
J1 from -0.97° to +0.03° (+1°). The controller sampled a stable
**-0.97° (0.0° progress)** after 2.5s and correctly latched
`motion_stalled_or_offset`. Torque stayed ON; the fault revoked
the motion lease; no automatic reversal was sent. **An acknowledged
motor request is not proof the motor moved.**

From this commit:

- Manual operator lease lasts **1800s (30 min)**; fault/disarm still
  immediately revokes it, no automatic renewal.
- Trial physical probe accepts **`--degrees 5`** (default) or
  **`--degrees 10`** with existing controller allowlists and joint limits.
  Start at **5°**, only consider 10° after verifying motor operation.
- After an attempted physical action stalls, the real motion record is
  retained and passed to Grok with RGB keyframes, depth statistics and
  controller feedback, so failures contribute to the learning dataset.
  **No second action** is sent until an operator has explicitly cleared
  faults and re-authorized movement.
- The physical probe checks both the controller's
  `feedback_confirmed` and measured J1 displacement ≥60% of
  the requested angular change **in the correct direction**, before
  permitting a return step. A no-progress or stalled J1 must not
  trigger a blind reverse.
- `micro_agent --degrees 5` passes the same bounded step through;
  Grok suggestions do not bypass the controller's motion protections.
- **Before another movement:** operator visually inspects the robot,
  the stationary J1 joint and possible obstructions/power issues,
  clears the latched fault using the original recovery control, and
  manually unlocks again. Do not restart/reset the service to erase
  an uncleared fault.

```bash
# Read-only diagnostics after normal controller is running
curl -fsS http://127.0.0.1:8765/api/health | python -m json.tool
curl -fsS http://127.0.0.1:8765/api/status | python -m json.tool

# After an operator explicitly clears fault and authorizes motion
python -m evoarm_rsi.real_probe --degrees 5
# Then only with operator physically present and motion authorized:
python -m evoarm_rsi.micro_agent --degrees 5 --iterations 1 --execute
```

Official Waveshare JSON protocol documents `T=121` as absolute
single-joint **degree** target with `spd` in °/s and `acc`
in °/s². The known 1° stall alone does not identify a firmware
deadband vs mechanical/electrical/servo problem.

## Trial 004: J1 actually moved, but arrived short and did not return

Real experiment `82b1593ee7ff4d94804e2597165ff533`:
J1 started at -0.97°, received a +5° absolute target of +4.03°,
and moved to about +2.90° (observed +3.87°). The controller returned
`feedback_confirmed`, even though the final residual was about
**1.13°**. A reverse command was rejected with HTTP 409, and the
arm remained at about +2.90° without a fault or pending request.

Cause of the 409 **cannot be established** from old HTTPError text alone:
the FastAPI controller uses HTTP 409 for several conditions. Updated
real probes retain its actual `detail` string.

Previously the probe would proceed to reverse after as little as
60% of requested displacement, and blindly send exactly -5° even if
the forward move stopped short. Now:

- Controller acknowledgement, actual movement, precise arrival and
  return-to-start are separately evaluated.
- A measured residual **above 0.75°** halts before a return request;
  the original forward request and all sampled feedback remain saved.
- After a sufficiently accurate move, the reverse uses the **measured
  original J1 starting angle as an absolute joint target**, not a
  blindly mirrored relative angle. Existing controller checks still
  decide whether any return may execute.
- Any HTTP 409 from the controller is preserved with its human-readable
  JSON `detail`. There is no automatic retry.
- A read-only offline audit extracts the observed delta and final
  residual from saved data and includes that in Grok input.

To audit the earlier real run **without moving anything**:

```bash
python -m evoarm_rsi.motion_audit \
  82b1593ee7ff4d94804e2597165ff533
```

This creates `~/.evoarm/rsi/analysis/<trial_id>/motion_audit.json`.
It will not clear faults, unlock motion, or return the arm to Home.
**Do not run the next physical trial yet**: inspect the actual state
and confirm the observed robot return geometry first.

## Trial 005: real first FULL grasp/lift attempt (not more J1-only probing)

The current user goal is **actually try to pick up the yellow ball**, and
let Grok reflect on observed success/failure. The new v2/Gemini path uses:

1. Fresh 10s Gemini RGB-D capture with yellow ball on the *same stationary*
   9x6 chessboard; run `localize` and `qualify` as usual.
2. `grasp_candidate` converts measured board XY into robot XY using the
   previously validated five-point transform. It reuses the real v2 Hover,
   Grasp and gripper open/close Z/pitch/roll snapshots. It produces a
   **complete hover → open → descend → close → lift** sequence, with
   vertical increments ≤20mm, no invented robot Z or TCP.
3. For first real execution, the arm remains at the saved measured
   **Home**, NOT manually positioned at Hover. The controller reads live
   T=1051 and generates a short high-XY alignment followed by ≤20mm
   vertical Home→Hover segments; every stage is checked by the existing
   controller. The Home XY must be within 30mm of the ball's XY and its
   physical pose must match the recorded Home. The operator still checks
   full swept volume, camera/base stability and emergency power access.
   This is a restricted measured corridor, not arbitrary 3D path planning.
4. Only under opt-in `EVOARM_ENABLE_MOTION=1 EVOARM_ENABLE_PICK=1
   EVOARM_ENABLE_RSI_GRASP=1 EVOARM_ENABLE_RSI_BATCH=1`, one operator can
   approve ONE complete fixed grasp/lift attempt.
   Every T104/T106 stage still passes the existing `PickExecutor.step`
   pose/torque/limits and must return actual `T=1051 feedback_confirmed`
   before the next stage. Any error stops further motion; **no automatic
   Home move, replay, torque transition, or fault reset**.
5. Gemini records true RGB-D in 20s chunks during the sequence.
   `grasp_attempt.json` and the real stage log are provided to
   `vision_review` after completion. Grok must not equate a motor
   `close_ball` stage with proven pickup.

```bash
# 0. ONCE: extract the 54-point Gemini board from the ALREADY
#    RECORDED Trial 003.1. Do not film another empty board.
python -m evoarm_rsi.board_reference \
  ~/.evoarm/rsi/experiments/bca3c4c1d5c449368a2e149f39b8479f

# 1. From now on, just record the yellow ball IN PLACE and localize it:
python -m evoarm_rsi.camera_service record --seconds 10
TRIAL=~/.evoarm/rsi/experiments/<YOUR_NEW_TRIAL_ID>
python -m evoarm_rsi.localize "$TRIAL"
python -m evoarm_rsi.qualify "$TRIAL"
python -m evoarm_rsi.grasp_candidate "$(basename "$TRIAL")"

# 2. After controller restarted with all 4 explicit ENABLE flags,
#    arm is at measured Home (not Hover), and manually unlocked:
python -m evoarm_rsi.grasp_run "$(basename "$TRIAL")"
#    This command is READ-ONLY; inspect status and actual XYZ.

# 3. Only when physically present and wanting ONE entire attempt:
python -m evoarm_rsi.grasp_run "$(basename "$TRIAL")" --execute
#    Interactive typing RUN confirms this concrete physical trial.
```

**Note:** The existing old `/pick` browser workflow still refuses stale
legacy profile execution when v2 poses exist. This is intentional: the
new `/api/rsi/grasp/start` goes through the measured-v2 Gemini
candidate builder, never silently falls back to old USB image corners.

**No camera/board origin verification is automatic.** The physical
operator must confirm the camera/board/robot have not moved since the
five-point board→robot XY calibration. This is the final geometry
assumption for the first supervised grab, not a general 3D hand-eye
calibration or collision-free world model.

Software-only recovery: an explicitly rejected HTTP 409 whose `detail`
is **only** "minimum motion-command interval" is recognized by the
micro-probe; after re-reading real feedback and verifying the arm hasn't
moved, it can retry once. Real motor faults must not be automatically
cleared via `/api/recover`, since that endpoint requires a human
declaration of a physically inspected static workspace.

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

**Persistent Gemini reference — no repeated empty-board capture.**

Trial 003.1 (`bca3c4c1d5c449368a2e149f39b8479f`) ALREADY recorded
the full empty 9×6 checkerboard. Do **not** ask the operator to empty the
board or place the ball during a countdown again. Save that geometry ONCE:

```bash
python -m evoarm_rsi.board_reference \
  ~/.evoarm/rsi/experiments/bca3c4c1d5c449368a2e149f39b8479f
```

The result is `~/.evoarm/gemini_board_reference.json` with the original
54 corners and pixel-to-board transform, captured with this Gemini.
For EVERY future ball-only trial, simply leave the ball on the board:

```bash
python -m evoarm_rsi.camera_service record --seconds 10
python -m evoarm_rsi.localize ~/.evoarm/rsi/experiments/<new_trial_id>
python -m evoarm_rsi.qualify ~/.evoarm/rsi/experiments/<new_trial_id>
```

`localize` automatically uses the persistent reference when no entire
checkerboard can be seen behind the arm/ball. If a full checkerboard
is visible, it cross-checks the saved 54 corners and reports any large
movement; that movement must stop execution. If the board is occluded,
physical placement is **not independently verified by that recording**:
the original fixed camera, board and robot base must remain unchanged.
Gemini's stored image-to-board mapping never substitutes for a full
camera-to-robot 3D extrinsic calibration.

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
