"""Mac-local safe control UI. Bind only to 127.0.0.1."""
from pathlib import Path
import json
import math
import threading
import asyncio

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, Response, JSONResponse, RedirectResponse
from pydantic import BaseModel

from .service import ArmService
from .safety import UnsafeMotion
from .robot_calibration import CalibrationError, calibration, default_path as xy_calibration_path
from .pick_plan import PickPlanningError, plan_pick_place, build_from_saved_files, save_pick_profile, save_pick_draft, load_pick_progress
from .pick_teach import observe_tool_pose
from .fast_teach import read_poses, capture_pose
from .pick_execution import executor
from evoarm_rsi import TrialRunner, TrialStore, TrialPolicy, CompanyModelGateway
from evoarm.serial_arm import ArmCommunicationError

app = FastAPI(title="EvoArm 中文控制台", docs_url=None, redoc_url=None)
rsi_runner = TrialRunner(store=TrialStore())  # P0: strictly simulated, no serial commands
service = ArmService()
from .bench_ui import install as install_bench_ui
install_bench_ui(app, lambda: service)
from .start_motion import install as install_start_motion
install_start_motion(app, lambda: service)
from .ball_trial import install as install_ball_trial
install_ball_trial(app, lambda: service)
from .coordinates import install as install_coordinates
install_coordinates(app, lambda: service)
from .rsi_trial import install as install_rsi_trial
install_rsi_trial(app, lambda: service)
_board_reference_lock = threading.RLock()
_confirmed_empty_board = None  # only pixel evidence; no motor control


@app.exception_handler(ArmCommunicationError)
async def serial_communication_error(request: Request, exc: ArmCommunicationError):
    # Some read-only routes (e.g. calibration capture) do not use invoke().
    return JSONResponse(status_code=409, content={
        "detail": str(exc), "error_code": "serial_communication_error"})


class Connect(BaseModel):
    port: str


class Unlock(BaseModel):
    declaration: str


class Step(BaseModel):
    joint: int
    delta: float


class TeachTarget(BaseModel):
    joint: int
    target_deg: float


class TeachRelative(BaseModel):
    joint: int
    delta_deg: float


class TeachCapture(BaseModel):
    stage: str


class GripperMove(BaseModel):
    delta: float


class HomeMove(BaseModel):
    kind: str
    joint: int


class CalibrationStart(BaseModel):
    square_mm: float
    image_size: list[int]
    corners_px: list[list[float]] | None = None


class CalibrationCapture(BaseModel):
    session_id: str
    point_id: str
    col: int
    row: int


class CalibrationSave(BaseModel):
    session_id: str


class TeachTorque(BaseModel):
    enabled: bool
    declaration: str


class PickProfile(BaseModel):
    grip_z_mm: float
    place_z_mm: float
    travel_z_mm: float
    place_xy_mm: list[float]
    pitch_rad: float
    roll_rad: float
    open_g_rad: float
    close_g_rad: float
    ball_diameter_mm: float


class PickDraft(BaseModel):
    values: dict


class PickPlan(BaseModel):
    ball_board_mm: list[float]
    image_size: list[int]
    corners_px: list[list[float]]


class PickStart(PickPlan):
    declaration: str


class PickStepRequest(BaseModel):
    session_id: str
    step_index: int
    declaration: str


class RSIGraspStart(BaseModel):
    trial_id: str
    declaration: str



def invoke(fn, *args):
    try:
        return fn(*args)
    except (UnsafeMotion, ValueError, RuntimeError, OSError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/", response_class=HTMLResponse)
def home():
    return RedirectResponse("/calibration", status_code=307)


@app.get("/camera", response_class=HTMLResponse)
def camera_page():
    return RedirectResponse("/calibration#camera", status_code=307)


def _saved_visual_reference():
    """Use only already-validated 4+1 XY calibration, never an unverified draft."""
    state=calibration.summary()
    if not state.get("saved") or not state.get("ready"):
        raise HTTPException(
            status_code=409,
            detail="缺少已保存并验收通过的XY标定。这里仅需确认空棋盘，不需重复P1～V1",
        )
    try:
        data=json.loads(xy_calibration_path().read_text(encoding="utf-8"))
        if not (data.get("fit",{}).get("validated") and
                len(data.get("records",[]))==5 and
                len(data.get("corners_px") or [])==54):
            raise ValueError("磁盘XY标定数据不完整")
        return {
            "square_mm":data.get("square_mm"),
            "image_size":data.get("image_size"),
            "corners_px":data.get("corners_px"),
        }
    except (OSError,ValueError,TypeError) as exc:
        raise HTTPException(status_code=409,detail="无法读取保存的XY标定："+str(exc)) from exc


@app.get("/api/vision/board-reference")
def board_reference_status():
    with _board_reference_lock:
        current=_confirmed_empty_board
        if current is None:
            return {"confirmed":False,"message":"请拿走球，点击「确认空棋盘」一次；机械臂五点XY标定不需要重做"}
        return {
            "confirmed":True,
            "image_size":current["image_size"],
            "square_mm":current["square_mm"],
            "message":"空棋盘已确认。放回黄球即可检测，系统仍会逐帧核对可见角点",
        }


@app.post("/api/vision/confirm-board")
async def confirm_board(request: Request, square_mm: float):
    """Read-only one-time empty-board reference. Never changes XYZ or saved XY."""
    global _confirmed_empty_board
    try:
        from .vision import VisionError, confirm_empty_board
    except ImportError as exc:
        raise HTTPException(status_code=503,detail="缺少视觉依赖") from exc
    if not math.isfinite(square_mm) or not 0<square_mm<=200:
        raise HTTPException(status_code=422,detail="单格边长不合法")
    if request.headers.get("content-type","").split(";")[0].strip() not in ("image/png","image/jpeg","image/jpg"):
        raise HTTPException(status_code=415,detail="需要PNG或JPEG相机帧")
    with _board_reference_lock:
        _confirmed_empty_board=None  # a failed reconfirm must not keep stale evidence
    saved=_saved_visual_reference()
    raw = await request.body()
    if not raw or len(raw) > 12_000_000:
        raise HTTPException(status_code=413, detail="确认棋盘图片为空或超过12MB")
    try:
        # OpenCV is synchronous C++ and can stall the async event loop.
        # Run it off-thread with an absolute request deadline.
        ref, response = await asyncio.wait_for(
            asyncio.to_thread(confirm_empty_board, raw, square_mm, saved),
            timeout=8.0,
        )
    except asyncio.TimeoutError as exc:
        raise HTTPException(
            status_code=504,
            detail="棋盘识别超过8秒，已结束本次等待。请检查摄像头清晰度、棋盘是否完整入镜；旧的XY五点标定不会丢失",
        ) from exc
    except (VisionError,ValueError,TypeError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc
    with _board_reference_lock:
        _confirmed_empty_board=ref
    return response


@app.post("/api/vision/analyze")
async def vision_analyze(request: Request, square_mm: float | None = None, use_reference: bool = False):
    """Analyze ONE browser-uploaded PNG/JPEG frame; never accesses serial robot APIs."""
    try:
        from .vision import VisionError, analyze_image
    except ImportError as exc:
        raise HTTPException(
            status_code=503,
            detail="缺少视觉依赖。请先在 Mac 执行 pip install -r requirements-vision.txt",
        ) from exc
    if request.headers.get("content-type", "").split(";")[0].strip() not in (
        "image/png", "image/jpeg", "image/jpg"
    ):
        raise HTTPException(status_code=415, detail="请以 PNG 或 JPEG 格式上传单帧")
    if int(request.headers.get("content-length", "0") or "0") > 12_000_000:
        raise HTTPException(status_code=413, detail="图像超过12MB")
    raw = await request.body()
    reference = None
    if use_reference:
        try:
            saved=_saved_visual_reference()
            with _board_reference_lock:
                cached=_confirmed_empty_board
                if (cached is not None and
                        cached["image_size"]==saved["image_size"] and
                        cached["corners_px"]==saved["corners_px"] and
                        cached["square_mm"]==saved["square_mm"]):
                    reference=cached
                else:
                    reference=saved
        except HTTPException:
            reference=None
    try:
        return await asyncio.wait_for(
            asyncio.to_thread(analyze_image, raw, square_mm, reference),
            timeout=8.0,
        )
    except asyncio.TimeoutError as exc:
        raise HTTPException(
            status_code=504,
            detail="画面识别超过8秒，已结束本次等待。可以重新尝试取图，不会影响已经保存的XY标定",
        ) from exc
    except VisionError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/calibration", response_class=HTMLResponse)
def calibration_page():
    # Calibration UI changes rapidly during hardware bring-up. Never serve
    # a stale cached page with old onclick bindings or torque confirmations.
    return HTMLResponse(
        content=Path(__file__).with_name("manual_calibration.html").read_text(encoding="utf-8"),
        headers={"Cache-Control": "no-store, max-age=0", "Pragma": "no-cache"},
    )


@app.get("/api/calibration/status")
def calibration_status():
    return calibration.summary()


@app.get("/api/calibration/stationary-sample")
def calibration_stationary_sample():
    # Sampling uses the serial owner's lock, but does not touch the legacy
    # calibration session, unlock motion, open serial or change any torque.
    from evoarm_rsi.bench_calibration import stationary_sample
    try:
        return stationary_sample(service)
    except CalibrationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/calibration/start")
def calibration_start(req: CalibrationStart):
    try:
        return calibration.start(req.square_mm, req.image_size, req.corners_px)
    except CalibrationError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.post("/api/calibration/capture")
def calibration_capture(req: CalibrationCapture):
    try:
        return calibration.capture(req.session_id, req.point_id, req.col, req.row, service)
    except CalibrationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/calibration/save")
def calibration_save(req: CalibrationSave):
    try:
        return calibration.save(req.session_id)
    except CalibrationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/teach/torque")
def teach_torque(req: TeachTorque):
    return invoke(service.set_torque, req.enabled, req.declaration)


@app.get("/teach", response_class=HTMLResponse)
def fast_teaching_page():
    return RedirectResponse("/calibration", status_code=307)


@app.get("/api/teach/poses")
def teach_poses():
    return invoke(read_poses)


@app.post("/api/teach/capture")
def teach_capture(req: TeachCapture):
    return invoke(capture_pose, service, req.stage)


@app.post("/api/teach/move-relative")
def teach_move_relative(req: TeachRelative):
    return invoke(service.teach_move_relative, req.joint, req.delta_deg)


@app.post("/api/teach/move")
def teach_move(req: TeachTarget):
    return invoke(service.teach_move_to, req.joint, req.target_deg)


@app.get("/pick", response_class=HTMLResponse)
def pick_page():
    return RedirectResponse("/experiments", status_code=307)


@app.get("/experiments", response_class=HTMLResponse)
def experiments_page():
    return HTMLResponse(Path(__file__).with_name("experiments.html").read_text(encoding="utf-8"),
                        headers={"Cache-Control": "no-store"})


@app.get("/coordinates", response_class=HTMLResponse)
def coordinates_page():
    return HTMLResponse(Path(__file__).with_name("coordinates.html").read_text(encoding="utf-8"),
                        headers={"Cache-Control": "no-store"})


@app.get("/assets/coordinates.js")
def coordinates_script():
    return Response(Path(__file__).with_name("coordinates.js").read_text(encoding="utf-8"),
                    media_type="application/javascript",headers={"Cache-Control":"no-store"})


@app.get("/assets/coordinate_view.js")
def coordinate_view_script():
    return Response(Path(__file__).with_name("coordinate_view.js").read_text(encoding="utf-8"),
                    media_type="application/javascript",headers={"Cache-Control":"no-store"})


@app.get("/assets/workspace.css")
def workspace_styles():
    return Response(Path(__file__).with_name("workspace.css").read_text(encoding="utf-8"),
                    media_type="text/css", headers={"Cache-Control": "no-store"})


@app.get("/assets/workspace.js")
def workspace_script():
    return Response(Path(__file__).with_name("workspace.js").read_text(encoding="utf-8"),
                    media_type="application/javascript", headers={"Cache-Control": "no-store"})


@app.get("/assets/grasp.js")
def grasp_script():
    return Response(Path(__file__).with_name("grasp.js").read_text(encoding="utf-8"),
                    media_type="application/javascript", headers={"Cache-Control": "no-store"})


@app.get('/assets/rsi.js')
def rsi_script():
    return Response(Path(__file__).with_name('rsi.js').read_text(encoding='utf-8'),
                    media_type='application/javascript',headers={'Cache-Control':'no-store'})


@app.get("/api/pick/workspace")
def pick_workspace_geometry():
    """Read-only calibrated workspace geometry for camera overlay.

    P1–P4 are sampled robot-XY mappings, not a collision-free motion envelope.
    The outer sheet is an approximate image-space extension of the known
    9x6 inner chessboard corners, never a motion authorization.
    """
    import math
    state=calibration.summary()
    if not state.get("saved") or not state.get("ready"):
        raise HTTPException(status_code=409,detail="还没有已保存的4+1点XY标定")
    try:
        stored=json.loads(xy_calibration_path().read_text(encoding="utf-8"))
        corners=stored["corners_px"]
        if len(corners)!=54 or stored["image_size"]!=state.get("image_size"):
            raise ValueError("棋盘角点或图像尺寸不一致")
        pts=[[float(v) for v in c] for c in corners]
        if any(len(c)!=2 or not all(math.isfinite(v) for v in c) for c in pts):
            raise ValueError("角点坐标无效")
        def at(col,row):
            return pts[row*9+col]
        def extended(col,row,near_col,near_row,side_col,side_row):
            p=at(col,row);q=at(near_col,near_row);r=at(side_col,side_row)
            return [round(p[k]+(p[k]-q[k])+(p[k]-r[k]),1) for k in (0,1)]
        boundary=[
            extended(0,0,1,0,0,1),
            extended(8,0,7,0,8,1),
            extended(8,5,7,5,8,4),
            extended(0,5,1,5,0,4),
        ]
        records=[r for r in stored.get("records",[])
                 if r.get("id") in ("P1","P2","P3","P4")]
        if len(records)!=4:raise ValueError("P1–P4样本缺失")
        points=[]
        for r in records:
            col,row=r["grid"]
            if not (type(col) is int and type(row) is int and
                    0<=col<9 and 0<=row<6):
                raise ValueError("标定点网格不合法")
            xyz=r["robot_xyz_mm"];board=r["board_mm"]
            points.append({
                "id":r["id"],"pixel":at(col,row),
                "grid":[col,row],
                "board_mm":[float(board[0]),float(board[1])],
                "robot_xy_mm":[float(xyz[0]),float(xyz[1])],
            })
        # Convex hull in board-plane, with matching image / robot locations.
        # Four points suffice; sort by angle about the centroid.
        cx=sum(p["board_mm"][0] for p in points)/4
        cy=sum(p["board_mm"][1] for p in points)/4
        points.sort(key=lambda p:math.atan2(
            p["board_mm"][1]-cy,p["board_mm"][0]-cx))
        polygon=[p["board_mm"] for p in points]
        area=abs(sum(
            polygon[i][0]*polygon[(i+1)%4][1]-
            polygon[(i+1)%4][0]*polygon[i][1]
            for i in range(4)))/2
        cell=float(stored["square_mm"])
        board_area=(9*cell)*(6*cell)
        radii=[math.hypot(*p["robot_xy_mm"]) for p in points]
        return {
            "available":True,
            "image_size":stored["image_size"],
            "square_mm":cell,
            "board_outline_px":boundary,
            "sampled_polygon_px":[p["pixel"] for p in points],
            "samples":points,
            "sampled_area_mm2":round(area,1),
            "board_sheet_area_mm2":round(board_area,1),
            "sampled_area_percent":round(100*area/board_area,1),
            "sampled_robot_radius_mm":[round(min(radii),1),round(max(radii),1)],
            "labels":{
                "green":"P1–P4采样覆盖：XY映射优先验证区；不代表自动运动安全",
                "amber":"棋盘可定位：超出实测点，需验证末端高度与路径",
                "outside":"当前棋盘外：尚无可靠桌面坐标映射",
            },
        }
    except (OSError,KeyError,ValueError,TypeError,IndexError) as exc:
        raise HTTPException(status_code=409,
            detail="无法读取已保存工作区几何："+str(exc)) from exc


@app.get("/api/pick/profile")
def pick_profile_load():
    legacy = load_pick_progress()
    # Expose which source the preview will actually use; don't make users
    # repeat the legacy pick-profile workflow after completing v2 teaching.
    from .fast_teach import pose_path
    if pose_path().exists():
        snapshot = read_poses()
        stages = ("home", "hover", "grasp", "place", "gripper_open", "gripper_close")
        legacy["v2_ready"] = all(k in snapshot.get("poses", {}) for k in stages)
        legacy["v2_source"] = str(pose_path())
        legacy["v2_missing"] = [k for k in stages if k not in snapshot.get("poses", {})]
    else:
        legacy["v2_ready"] = False
        legacy["v2_source"] = None
        legacy["v2_missing"] = []
    return legacy


@app.post("/api/pick/draft")
def pick_draft_save(req: PickDraft):
    try:
        return save_pick_draft(req.values)
    except PickPlanningError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/pick/teach")
def pick_teach():
    try:
        return observe_tool_pose(service)
    except PickPlanningError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/pick/profile")
def pick_profile_save(req: PickProfile):
    try:
        return save_pick_profile(req.model_dump() if hasattr(req, "model_dump") else req.dict())
    except PickPlanningError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.post("/api/pick/preview")
def pick_plan_preview(req: PickPlan):
    try:
        return build_from_saved_files(req.ball_board_mm, image_size=req.image_size, corners_px=req.corners_px, preview_only=True)
    except (PickPlanningError, CalibrationError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


class RSIOnce(BaseModel):
    declaration: str


@app.post("/api/rsi/grasp/enable-once")
def rsi_grasp_enable_once(req: RSIOnce):
    """One local operator-authorized RSI trial; never unlocks the arm."""
    return invoke(executor.enable_rsi_once, service, req.declaration)


@app.post("/api/rsi/grasp/start")
def rsi_grasp_start(req: RSIGraspStart):
    """Operator-started fresh Gemini/v2 grasp, uses ordinary verified pick steps."""
    return invoke(executor.start_rsi_grasp, req.trial_id, service, req.declaration)


class RSIStallRecovery(BaseModel):
    session_id: str


@app.post("/api/rsi/grasp/recover-stall")
def rsi_grasp_recover_stall(req: RSIStallRecovery):
    """Automatically clear ONLY a stable, same-session RSI Cartesian fault.

    No motor command, torque change, rearm or continuation is performed.
    """
    return invoke(executor.recover_rsi_stall, req.session_id, service)


@app.get("/api/pick/execution")
def pick_execution_status():
    return executor.state(service)


@app.post("/api/pick/start")
def pick_execution_start(req: PickStart):
    return invoke(executor.start, req.ball_board_mm, req.image_size,
                  req.corners_px, service, req.declaration)


@app.post("/api/pick/step")
def pick_execution_step(req: PickStepRequest):
    return invoke(executor.step, req.session_id, req.step_index,
                  req.declaration, service)


@app.get("/api/pick/diagnose")
def pick_execution_diagnose():
    """Read-only T=1051 preflight: no T=104/T=106/T=210 commands."""
    return invoke(executor.diagnose, service)


@app.post("/api/pick/cancel")
def pick_execution_cancel():
    return executor.cancel(service)


@app.get("/api/ports")
def ports():
    return service.ports()


@app.get("/api/health")
def health():
    return service.health()


@app.post("/api/auto-connect")
def auto_connect():
    """Safe serial status connection, shared between the dashboard and calibration UI."""
    return service.auto_connect()


@app.post("/api/connect")
def connect(req: Connect):
    return {"status": invoke(service.connect, req.port)}


@app.post("/api/disconnect")
def disconnect():
    service.disconnect()
    return {"ok": True}


@app.get("/api/status")
def status():
    return invoke(service.status)


@app.post("/api/unlock-once")
def unlock(req: Unlock):
    return invoke(service.unlock_once, req.declaration)


@app.post("/api/disarm")
def disarm():
    return service.disarm()


@app.post("/api/recover")
def recover(req: Unlock):
    """Recover from a latched motion fault after manual safety check."""
    return invoke(service.recover_fault, req.declaration)


@app.post("/api/step")
def step(req: Step):
    return invoke(service.step, req.joint, req.delta)


@app.post("/api/gripper")
def gripper(req: GripperMove):
    return invoke(service.gripper_step, req.delta)


@app.post("/api/home/factory-reset")
def factory_reset():
    """Explicit official multi-joint init; requires preflight and UI confirmation."""
    return invoke(service.factory_reset)


@app.post("/api/home/save")
def home_save():
    return invoke(service.home_save)


@app.get("/api/home/preview")
def home_preview(kind: str = "saved"):
    return invoke(service.home_info, kind)


@app.post("/api/home/step")
def home_step(req: HomeMove):
    return invoke(service.home_step, req.kind, req.joint)


@app.on_event("shutdown")
def close_serial():
    service.disconnect()

# RSI P0 deliberately never takes a live ArmService or any serial handle.
class RSIMockTrialRequest(BaseModel):
    action_window_s: float = 10.0
    policy: dict | None = None
    use_company_agent: bool = False


@app.get("/api/rsi/status")
def rsi_status():
    gateway = CompanyModelGateway()
    return {
        "phase": "P0_mock_only",
        "hardware_enabled": False,
        "autonomous_hardware_loop": False,
        "max_action_window_s": 10,
        "company_gateway_configured": gateway.configured(),
        "storage": str(rsi_runner.store.root),
        "notice": "模拟实验不会读取相机或驱动机械臂；真实10秒视频与轨迹采集尚未接入",
    }


@app.post("/api/rsi/mock-trial")
def rsi_mock_trial(req: RSIMockTrialRequest):
    try:
        policy = TrialPolicy(**(req.policy or {}))
        agent = CompanyModelGateway() if req.use_company_agent else None
        return rsi_runner.run_once(
            policy, action_window_s=req.action_window_s,
            propose_next=agent.propose_next if agent is not None else None,
        )
    except (TypeError, ValueError, RuntimeError, OSError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/rsi/trials/{trial_id}")
def rsi_trial_info(trial_id: str):
    try:
        return {
            "manifest": rsi_runner.store.read(trial_id),
            "evaluation": rsi_runner.store.read(trial_id, "evaluation.json"),
        }
    except (ValueError, OSError) as exc:
        raise HTTPException(status_code=404, detail="trial not found") from exc

