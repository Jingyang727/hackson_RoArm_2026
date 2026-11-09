"""Mac-local safe control UI. Bind only to 127.0.0.1."""
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

from .service import ArmService
from .safety import UnsafeMotion

app = FastAPI(title="EvoArm 中文控制台", docs_url=None, redoc_url=None)
service = ArmService()


class Connect(BaseModel):
    port: str


class Unlock(BaseModel):
    declaration: str


class Step(BaseModel):
    joint: int
    delta: float


class GripperMove(BaseModel):
    delta: float


class HomeMove(BaseModel):
    kind: str
    joint: int


def invoke(fn, *args):
    try:
        return fn(*args)
    except (UnsafeMotion, ValueError, RuntimeError, OSError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/", response_class=HTMLResponse)
def home():
    return Path(__file__).with_name("index.html").read_text(encoding="utf-8")


@app.get("/camera", response_class=HTMLResponse)
def camera_page():
    return Path(__file__).with_name("camera.html").read_text(encoding="utf-8")


@app.get("/api/ports")
def ports():
    return service.ports()


@app.get("/api/health")
def health():
    return service.health()


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
