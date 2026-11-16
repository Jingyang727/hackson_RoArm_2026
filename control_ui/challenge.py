"""Desktop multi-ball challenge, sharing the one controller and camera daemon."""
import json
from pathlib import Path
import re
import subprocess

from fastapi import APIRouter,HTTPException,Request
from fastapi.responses import FileResponse,HTMLResponse,Response,StreamingResponse
from pydantic import BaseModel,ConfigDict

from . import bench_ui
from evoarm_rsi.challenge_loop import Challenge
from evoarm_rsi import grok_review
from evoarm_rsi.browser_video import browser_video
from evoarm_rsi.rsi_events import stream_events


class Start(BaseModel):
    model_config=ConfigDict(extra='forbid')
    declaration:str


def valid_id(value):return bool(re.fullmatch('[0-9a-f]{32}',value))


def install(app,service_provider):
    runner=Challenge(service_provider);router=APIRouter(prefix='/api/challenge')
    app.state.challenge=runner

    def fail(exc):raise HTTPException(status_code=409,detail=str(exc))

    def folder(key):
        if not valid_id(key):raise HTTPException(status_code=404)
        return runner.root(key)

    def file_response(path,media):
        base=Path.home()/'.evoarm/rsi';path=Path(path)
        if not path.is_file() or any(p.is_symlink() for p in (path,*path.parents)) or not path.resolve().is_relative_to(base.resolve()):
            raise HTTPException(status_code=404)
        return FileResponse(path,media_type=media,headers={'Cache-Control':'no-store','X-Content-Type-Options':'nosniff'})

    @router.get('/state')
    def state():
        service=service_provider();feedback_error=None
        with service.lock:
            if service.arm and not service.automation_owner and not service.pending:
                try:service.status()
                except Exception as exc:feedback_error=str(exc)[:500]
            health=service.health()
        try:runner.session();calibrated=True;calibration_error=None
        except (OSError,ValueError,KeyError) as exc:calibrated=False;calibration_error=str(exc)[:500]
        try:revision=subprocess.check_output(['git','rev-parse','--short','HEAD'],cwd=Path(__file__).parent,text=True,stderr=subprocess.DEVNULL).strip()
        except (OSError,subprocess.SubprocessError):revision=None
        return {'challenge_api_version':1,'expected_balls':3,'ball_diameter_mm':34,'container_unchanged':True,
            'health':health,'feedback_error':feedback_error,'calibrated':calibrated,'calibration_error':calibration_error,
            'model':grok_review.configuration(),'challenge':runner.snapshot(),'process':runner.events.after(),
            'code_revision':revision,'recording_seconds':20}

    @router.post('/observe')
    def observe():
        try:return {'scene':runner.observe(),'robot_moved':False}
        except (OSError,ValueError,RuntimeError,KeyError) as exc:fail(exc)

    @router.post('/start',status_code=202)
    def start(req:Start):
        try:return runner.start(req.declaration)
        except (OSError,ValueError,RuntimeError,KeyError) as exc:fail(exc)

    @router.post('/stop')
    def stop():return runner.cancel()

    @router.get('/history')
    def history():
        rows=[];base=Path.home()/'.evoarm/rsi/challenge_sessions'
        for file in sorted(base.glob('*/state.json'),key=lambda p:p.stat().st_mtime,reverse=True):
            try:
                value=json.loads(file.read_text())
                rows.append({k:value.get(k) for k in ('session_id','status','started_at_unix','confirmed_colors','trials','error')})
                if len(rows)>=10:break
            except (OSError,ValueError):continue
        return {'sessions':rows}

    @router.get('/events')
    async def events(request:Request):
        return StreamingResponse(stream_events(request,runner.events),media_type='text/event-stream',
            headers={'Cache-Control':'no-cache','X-Accel-Buffering':'no'})

    @router.get('/sessions/{key}/observations/{observation}')
    def observation(key:str,observation:str):
        if not valid_id(observation):raise HTTPException(status_code=404)
        return file_response(folder(key)/'observations'/observation/'preview.jpg','image/jpeg')

    @router.get('/sessions/{key}/trials/{trial}/{kind}')
    def asset(key:str,trial:str,kind:str):
        if not valid_id(trial):raise HTTPException(status_code=404)
        root=folder(key)/'attempts'/trial
        if not (root/'state.json').is_file():raise HTTPException(status_code=404)
        if kind=='archive':return file_response(Path.home()/'.evoarm/rsi/backups'/f'{trial}.zip','application/zip')
        if kind=='state':return file_response(root/'state.json','application/json')
        if kind=='observation':return file_response(root/'observation.jpg','image/jpeg')
        if kind not in ('video','keyframes'):raise HTTPException(status_code=404)
        saved=json.loads((root/'state.json').read_text());capture=saved.get('result',{}).get('capture_trial_id','')
        if not valid_id(capture):raise HTTPException(status_code=404)
        captured=bench_ui.experiment_root()/capture
        try:path=browser_video(captured) if kind=='video' else captured/'grasp-keyframes.jpg'
        except (OSError,ValueError,RuntimeError) as exc:fail(exc)
        return file_response(path,'video/mp4' if kind=='video' else 'image/jpeg')

    app.include_router(router)

    @app.get('/challenge',response_class=HTMLResponse)
    def page():
        return HTMLResponse(Path(__file__).with_name('challenge.html').read_text(),headers={'Cache-Control':'no-store'})

    @app.get('/assets/challenge.js')
    def script():
        return Response(Path(__file__).with_name('challenge.js').read_text(),media_type='application/javascript',headers={'Cache-Control':'no-store'})

    return runner
