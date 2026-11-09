"""Mac-only safe control UI. Keep host bound to 127.0.0.1."""
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from .service import ArmService
from .safety import UnsafeMotion

app = FastAPI(title="EvoArm 安全控制台", docs_url=None, redoc_url=None)
service = ArmService()


class Connect(BaseModel):
    port: str


class Unlock(BaseModel):
    declaration: str


class Step(BaseModel):
    joint: int
    delta: float


def call(fn, *args):
    try:
        return fn(*args)
    except (UnsafeMotion, ValueError, RuntimeError, OSError) as e:
        raise HTTPException(409, str(e)) from e


@app.get("/", response_class=HTMLResponse)
def home():
    return Path(__file__).with_name("index.html").read_text(encoding="utf-8")


@app.get("/api/ports")
def ports():
    return service.ports()


@app.get("/api/health")
def health():
    return service.health()


@app.post("/api/connect")
def connect(req: Connect):
    return {"status": call(service.connect, req.port)}


@app.post("/api/disconnect")
def disconnect():
    service.disconnect()
    return {"ok": True}


@app.get("/api/status")
def status():
    return call(service.status)


@app.post("/api/unlock-once")
def unlock(req: Unlock):
    return call(service.unlock_once, req.declaration)


@app.post("/api/step")
def step(req: Step):
    return call(service.step, req.joint, req.delta)


@app.on_event("shutdown")
def close_serial():
    service.disconnect()
