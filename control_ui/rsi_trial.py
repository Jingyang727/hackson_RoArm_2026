"""Browser pick/place trials and qualitative RSI review, using one ArmService."""
import copy
import json
import math
from pathlib import Path
import threading
import time
import uuid
import subprocess
import shutil
import re

import cv2
import numpy as np
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from . import bench_ui
from .vision import find_yellow_ball
from evoarm_rsi.bench_calibration import load_session,fit_points
from evoarm_rsi.coordinate_test import fingerprint,calibrated_table
from evoarm_rsi.bench_start import _ready
from evoarm_rsi.camera_service import client_request, preview_frame
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS, observe_scene, build_plan, preview, detect_funnel
from evoarm_rsi.rsi_policy import propose, target_audit, application, policy_id
from evoarm_rsi.rsi_execution import execute, reservation
from evoarm_rsi.browser_video import browser_video
from evoarm_rsi.task_agent import _write
from evoarm_rsi.task_scene import load_rgbd
from evoarm_rsi import grok_review
from evoarm_rsi.rsi_journal import append_event, archive_trial
from evoarm_rsi.rsi_learning import LearningLoop, LEARNING_DECLARATION,error_info
from evoarm_rsi.rsi_recovery import recover as recover_trial
from evoarm_rsi.rsi_events import stream_events
from evoarm_rsi.rsi_comparison import build_comparison,public_comparison
from evoarm_rsi.image_directions import CONTRACT,OPPOSITE
from evoarm_rsi.policy_repair import repair_clipped_policy
from evoarm_rsi import agent_vision
from evoarm_rsi.agent_readiness import describe as describe_readiness,AUTHORIZATION_CONTRACT
from evoarm_rsi.execution_diagnosis import diagnose
from evoarm_rsi.roarm_geometry import MotionConstraintError

DECLARATION='已核对黄球、漏斗和完整抓放路径，现场有人看护且双手离开运动范围'


def _launch(target):
    threading.Thread(target=target,name='evoarm-rsi-pick-place',daemon=True).start()


class Empty(BaseModel):
    model_config=ConfigDict(extra='forbid')


class PlanID(Empty):
    plan_id:str=Field(pattern=r'^[a-f0-9]{32}$')


class Run(PlanID):
    declaration:str
    vision_packet_id:str|None=Field(default=None,pattern=r'^[a-f0-9]{16}$')
    scene_review:dict|None=None


class Prepare(Empty):
    source:str=Field(default='web',pattern=r'^(web|opencode_agent)$')


class ComparisonReview(Empty):
    reference_plan_id:str|None=Field(pattern=r'^[a-f0-9]{32}$')
    effect:str=Field(pattern=r'^(improved|worsened|overshot|unchanged|unknown|not_comparable|not_compared)$')
    movement:str=Field(pattern=r'^(as_expected|opposite|no_visible_motion|unknown)$')
    note:str=Field(max_length=2000)


class ParameterTrial(Empty):
    parameter:str=Field(pattern=r'^(grasp_z_offset_mm|finger_offset_mm|release_clearance_mm|open_deg|close_deg|squeeze_deg|travel_speed|approach_speed|lift_height_mm)$')
    value:float=Field(allow_inf_nan=False)


class Review(PlanID):
    outcome:str=Field(pattern=r'^(placed|grasp_missed|pushed_away|slipped|placement_missed|bounced_out|execution_failed|unknown)$')
    direction:str=Field(default='none',pattern=r'^(none|image_left|image_right|image_up|image_down|too_high|too_low)$')
    source:str=Field(default='human',pattern=r'^(human|company_grok|opencode_agent)$')
    requested_step_mm:float|None=Field(default=None,gt=0,allow_inf_nan=False)
    correction_direction:str|None=Field(default=None,pattern=r'^(image_left|image_right|image_up|image_down|lower|higher)$')
    vision_packet_id:str|None=Field(default=None,pattern=r'^[a-f0-9]{16}$')
    visual_review:dict|None=None
    parameter_trial:ParameterTrial|None=None
    note:str=Field(default='',max_length=2000)
    confidence:float|None=Field(default=None,ge=0,le=1)
    ball_state:str|None=Field(default=None,pattern=r'^(table|gripper|funnel|unknown)$')
    magnitude:str=Field(default='medium',pattern=r'^(small|medium|large|unknown)$')
    adjustment:str=Field(default='unknown',pattern=r'^(keep|pickup_xy|grasp_height|grip_tighten|transport_slower|release_xy|release_height|unknown)$')
    comparison:ComparisonReview|None=None


class Learn(Empty):
    declaration:str
    max_attempts:int=Field(default=5,ge=1,le=20,strict=True)


class Reset(Empty):
    declaration:str


class AgentMemory(Empty):
    note:str=Field(min_length=1,max_length=4000)
    plan_id:str|None=Field(default=None,pattern=r'^[a-f0-9]{32}$')


def install(app,service_provider):
    router=APIRouter(prefix='/api/bench/rsi');lock=threading.RLock();preparing=threading.Lock()
    plans={};job={'status':'idle','phase':'idle'};stop=threading.Event();latest=None
    agent_stream=None;agent_phase=None
    try:code_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=Path(__file__).resolve().parent.parent,text=True,stderr=subprocess.DEVNULL,timeout=3).strip()
    except (OSError,subprocess.SubprocessError):code_revision='unknown'
    def root(key):return Path.home()/'.evoarm/rsi/pick_place_runs'/key
    def session():
        _,saved=load_session(bench_ui.session_path())
        if saved['setup_id']!=bench_ui.setup_id():raise ValueError('当前布置与保存标定不一致')
        return saved
    def policy_path(saved):return Path.home()/'.evoarm/rsi/pick_place_policies'/saved['setup_id']/f'{fingerprint(saved)}.json'
    def policy(saved,*,repair=False):
        path=policy_path(saved)
        if not path.exists():return {'version':1,'parameters':copy.deepcopy(DEFAULT_PARAMETERS),'reviews':[]}
        value=json.loads(path.read_text())
        if set(value['parameters'])-set(DEFAULT_PARAMETERS):raise ValueError('本地RSI参数版本不兼容')
        missing=set(DEFAULT_PARAMETERS)-set(value['parameters'])
        if missing-{'squeeze_deg'}:raise ValueError('本地RSI参数不完整')
        if missing:
            value['parameters']={**copy.deepcopy(DEFAULT_PARAMETERS),**value['parameters']}
            value['version']+=1
            value.setdefault('migrations',[]).append({'added_defaults':sorted(missing),'at_unix':time.time()})
            _write(path,value)
        if repair:
            repaired,record=repair_clipped_policy(value)
            if record:
                backup=path.with_name(path.stem+f".before-vector-repair-{record['id']}.json")
                _write(backup,value)
                record['backup_file']=str(backup)
                _write(path,repaired);value=repaired
        return value
    def known(key,*,historical=False):
        if key not in plans:
            if historical and len(key)==32 and all(c in '0123456789abcdef' for c in key):
                path=root(key)/'state.json'
                if path.is_file() and not path.is_symlink():
                    value=json.loads(path.read_text())
                    if value.get('status') in ('completed','failed'):return value
                    attempt=root(key)/'attempt.json'
                    if attempt.is_file():
                        value.update(status='failed',result=json.loads(attempt.read_text()),interrupted_by_restart=True)
                        return value
            raise ValueError('计划不在当前服务中，请重新观察；不会重放旧计划')
        return plans[key]
    def save(state):_write(root(state['id'])/'state.json',{k:v for k,v in state.items() if k not in ('created','geometry')})
    def recent_trials(saved):
        scope=fingerprint(saved);rows=[]
        files=sorted((Path.home()/'.evoarm/rsi/pick_place_runs').glob('*/state.json'),key=lambda p:p.stat().st_mtime,reverse=True)
        for file in files[:30]:
            try:
                value=known(file.parent.name,historical=True)
                if value.get('calibration_fingerprint')!=scope or not value.get('result'):continue
                rows.append(value)
            except (OSError,ValueError,KeyError,TypeError):continue
        rows.sort(key=lambda r:r.get('result',{}).get('created_at_unix',r.get('created_at_unix',0)),reverse=True)
        return rows[:3]
    def compare(current,pol):
        prior=pol['reviews'][-1] if pol['reviews'] else None
        try:previous=known(prior['plan_id'],historical=True) if prior else None
        except ValueError:previous=None
        return build_comparison(current,previous,prior,bench_ui.experiment_root())
    def archive(state):
        save(state)
        path=archive_trial(root(state['id']))
        state['backup_file']=path
        save(state)
        return path
    def fail(exc):raise HTTPException(status_code=409,detail=getattr(exc,'detail',str(exc))) from exc
    def publish(phase,**values):
        nonlocal agent_phase
        with lock:job.update(phase='stopping' if stop.is_set() and phase not in ('finished','saving_recording') else phase,**values)
        learner.record_motion_phase(phase)
        if agent_stream and learner.snapshot().get('status')!='running':
            signature=(job.get('plan_id'),phase)
            if signature!=agent_phase:
                agent_phase=signature;learner.events.emit('motion',phase=phase,plan_id=job.get('plan_id'))
    def validate_live(state):
        packet=preview_frame(Path.home());image=cv2.imdecode(np.frombuffer(packet['jpeg'],np.uint8),cv2.IMREAD_COLOR)
        if image is None or list(image.shape[1::-1])!=state['plan']['scene']['image_size']:raise ValueError('当前RGB画面或分辨率发生变化')
        ball=find_yellow_ball(image);expected=state['plan']['scene']['ball']
        if not ball or math.dist(ball['center_px'],expected['pixel_center'])>max(6.,expected['radius_px']*.4):
            raise ValueError('黄球位置已改变，请重新观察本轮目标')
        funnel=detect_funnel(image,state['geometry']);old=state['plan']['scene']['funnel']
        if math.dist(funnel['pixel_center'],old['pixel_center'])>max(6.,old['expected_diameter_px']*.15):
            raise ValueError('漏斗位置已改变，请重新观察本轮目标')

    @router.get('/state')
    def state():
        automatic=learner.snapshot()
        with lock:
            snapshot=dict(job);current=plans.get(latest);plan=None
            if current:
                remaining=max(0.,180-(time.monotonic()-current['created']))
                p=current['plan'];plan={'plan_id':current['id'],'status':current['status'],'seconds_left':remaining,
                    'preview_url':f"/api/bench/rsi/plans/{current['id']}/preview",'depth_url':f"/api/bench/rsi/plans/{current['id']}/depth",
                    'scene':p['scene'],'parameters':p['parameters'],'policy_version':current['policy_version'],
                    'target_audit':p.get('target_audit'),'policy_application':p.get('policy_application'),
                    'route_policy':p.get('route_policy'),'object_geometry':p.get('object_geometry'),
                    'steps':[s['stage'] for s in p['steps']],'pitch_deg':p['grasp_pitch_deg'],
                    'release_pitch_deg':p.get('release_pitch_deg',p['grasp_pitch_deg']),'warnings':p['warnings'],
                    'review':current.get('review')}
                plan['backup_url']=f"/api/bench/rsi/plans/{current['id']}/archive" if (root(current['id'])/'backup.json').is_file() else None
        service=service_provider()
        if snapshot.get('status') not in ('running','preparing') and automatic['status']!='running':
            try:
                if service.arm:service.status()
            except (OSError,RuntimeError,ValueError,KeyError,TypeError):pass
        health=snapshot.get('health') if snapshot.get('status')=='running' or automatic['status']=='running' else None
        return {'api_version':10,'setup_id':bench_ui.setup_id(),'health':health or service.health(),'job':snapshot,'plan':plan,
                'process':learner.events.after(),
                'learning':automatic,'grok':grok_review.configuration(),'code_revision':code_revision}

    @router.get('/learning/events')
    async def learning_events(request:Request):
        return StreamingResponse(stream_events(request,learner.events),media_type='text/event-stream',
                                 headers={'Cache-Control':'no-cache, no-store','X-Accel-Buffering':'no','X-Content-Type-Options':'nosniff'})

    @router.post('/observe')
    def observe(req:Empty):
        """Fresh object coordinates without a grasp plan, robot feedback or motion."""
        try:learner.assert_interactive_allowed()
        except ValueError as exc:fail(exc)
        if not preparing.acquire(blocking=False):raise HTTPException(status_code=409,detail='正在观察，请等待')
        try:
            with lock:
                if job.get('status') in ('running','preparing'):raise ValueError('请等本轮动作或观察完成')
            saved=session();cam=client_request(Path.home(),'GET','/health')
            if cam.get('recording') or cam.get('robot_control') is not False or cam.get('camera_service')!='ready':
                raise ValueError('独立相机服务需空闲并处于ready状态')
            capture=client_request(Path.home(),'POST','/record',{'seconds':2});key=capture['trial_id']
            if len(key)!=32 or any(c not in '0123456789abcdef' for c in key):raise ValueError('相机返回无效录像ID')
            scene,geometry,image,_=observe_scene(bench_ui.experiment_root()/key,saved)
            folder=Path.home()/'.evoarm/rsi/pick_place_observations'/key;folder.mkdir(parents=True,exist_ok=True)
            if not cv2.imwrite(str(folder/'observation.jpg'),image):raise ValueError('定位原图保存失败')
            marked=image.copy()
            for name,color in (('ball',(0,210,255)),('funnel',(100,230,100))):
                pixel=tuple(np.round(scene[name]['pixel_center']).astype(int))
                cv2.circle(marked,pixel,25,color,3);cv2.putText(marked,name.upper(),pixel,cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
            if not cv2.imwrite(str(folder/'preview.jpg'),marked):raise ValueError('定位预览保存失败')
            result={'observation_id':key,'observed_at_unix':time.time(),'scene':scene,
                    'calibration_fingerprint':fingerprint(saved),'frame':'robot_firmware_mm','units':'mm',
                    'robot_moved':False,'robot_sampled':False,'live_tracking':False,
                    'image_url':f'/api/bench/rsi/observations/{key}/preview',
                    'note':'本次采集快照，不是实时跟踪。球或漏斗移动后重新采集；手动验证时保持物体原位。球心不是表面或固件指尖目标。'}
            _write(folder/'scene.json',result);_write(folder/'calibration-snapshot.json',saved)
            return result
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)
        finally:preparing.release()

    @router.get('/observations/{observation_id}/preview')
    def observation_image(observation_id:str):
        if len(observation_id)!=32 or any(c not in '0123456789abcdef' for c in observation_id):raise HTTPException(status_code=404)
        path=Path.home()/'.evoarm/rsi/pick_place_observations'/observation_id/'preview.jpg'
        if not path.is_file() or path.is_symlink():raise HTTPException(status_code=404)
        return FileResponse(path,media_type='image/jpeg',headers={'Cache-Control':'no-store','X-Content-Type-Options':'nosniff'})

    @router.post('/prepare')
    def prepare(req:Prepare):
        nonlocal latest
        draft_id=None
        try:learner.assert_interactive_allowed()
        except ValueError as exc:fail(exc)
        if not preparing.acquire(blocking=False):raise HTTPException(status_code=409,detail='正在观察，请等待')
        try:
            with lock:
                if job.get('status')=='running':raise ValueError('本轮仍在执行或保存录像')
                if req.source=='opencode_agent':
                    previous=recent_trials(session())
                    if previous and not previous[0].get('review'):
                        raise MotionConstraintError('上一轮尚无结果复核与决策，先用vision和decision闭环，不重复执行。',
                            code='trial_review_required',plan_id=previous[0]['id'])
                old=plans.get(latest)
                if old and old['status']=='prepared':old['status']='superseded';save(old)
                latest=None
                job.update(status='preparing',phase='observing',error=None,result=None,diagnostic_url=None)
            service=service_provider()
            with service.lock:
                initial=service.status();_ready(service.health());saved=session()
            cam=client_request(Path.home(),'GET','/health')
            if cam.get('recording') or cam.get('robot_control') is not False or cam.get('camera_service')!='ready':raise ValueError('独立相机服务需空闲并处于ready状态')
            capture=client_request(Path.home(),'POST','/record',{'seconds':2})
            key=capture['trial_id']
            if len(key)!=32 or any(c not in '0123456789abcdef' for c in key):raise ValueError('相机返回无效录像ID')
            draft_id=uuid.uuid4().hex;folder=root(draft_id);folder.mkdir(parents=True,exist_ok=False)
            frame=load_rgbd(bench_ui.experiment_root()/key)
            cv2.imwrite(str(folder/'preview.jpg'),frame['bgr'])
            scene,geometry,image,depth=observe_scene(bench_ui.experiment_root()/key,saved)
            if not cv2.imwrite(str(folder/'observation.jpg'),image):raise ValueError('原始观察图保存失败')
            with lock:pol=policy(saved,repair=True)
            if req.source=='opencode_agent':
                previous=recent_trials(saved)
                if previous:
                    last=previous[0];diagnosis=diagnose(last['result'],last['plan'])
                    same_objects=all(math.dist(scene[name][field],last['plan']['scene'][name][field])<17 for name,field in (
                        ('ball','center_robot_mm'),('funnel','mouth_center_robot_mm')))
                    if (diagnosis['category'] in ('time_budget','clearance_shortfall') and same_objects and
                            last['plan']['parameters']==pol['parameters'] and last['plan'].get('route_policy',{}).get('version',0)>=4):
                        raise MotionConstraintError('上一轮执行故障尚无参数变化，不重复同一路径；先查看diagnosis并提交单项执行试验。',
                            code='execution_retry_requires_change',plan_id=last['id'],execution_diagnosis=diagnosis)
            with service.lock:
                fresh=service.status();_ready(service.health())
                if any(abs(initial[k]-fresh[k])>math.radians(1) for k in 'bsetrg'):raise ValueError('观察时机械臂发生移动，请重新规划')
                _write(folder/'calibration-snapshot.json',saved)
                _write(folder/'planning-input.json',{'schema_version':1,'code_revision':code_revision,
                    'initial_status':fresh,'scene':scene,'parameters':pol['parameters'],
                    'policy_version':pol['version'],'calibration_fingerprint':fingerprint(saved)})
                plan=build_plan(saved,fresh,scene,pol['parameters'])
            plan['target_audit']=target_audit(scene,pol['parameters'])
            plan['policy_application']=application(pol['parameters'],plan['parameters'],pol['reviews'],pol.get('baseline_resets',[]))
            plan_id=draft_id
            if not cv2.imwrite(str(folder/'preview.jpg'),preview(image,plan,geometry)):raise ValueError('预览保存失败')
            valid=depth>0;colored=np.zeros((*depth.shape,3),np.uint8)
            if np.any(valid):
                low,high=np.percentile(depth[valid],[2,98]);scaled=np.clip((depth-low)/max(1.,high-low)*255,0,255).astype(np.uint8)
                colored=cv2.applyColorMap(scaled,cv2.COLORMAP_TURBO);colored[~valid]=0
                scene['depth_scale']={'near_mm':float(low),'far_mm':float(high)}
            cv2.imwrite(str(folder/'depth.jpg'),colored)
            state={'id':plan_id,'status':'prepared','created':time.monotonic(),'plan':plan,'policy_version':pol['version'],
                   'calibration_fingerprint':fingerprint(saved),'geometry':geometry,'code_revision':code_revision,'source':req.source,
                   'created_at_unix':time.time()}
            _write(folder/'calibration-snapshot.json',saved)
            if memory_path(saved).is_file():shutil.copy2(memory_path(saved),folder/'agent-memory-snapshot.json')
            append_event(folder,'prepared',parameters=pol['parameters'],policy_version=pol['version'],code_revision=code_revision)
            save(state)
            with lock:
                old=plans.get(latest)
                if old and old['status']=='prepared':old['status']='superseded';save(old)
                plans[plan_id]=state;latest=plan_id;job.update(status='ready',phase='ready',plan_id=plan_id)
            return {'plan_id':plan_id,'preview_url':f'/api/bench/rsi/plans/{plan_id}/preview'}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
            diagnostic=f'/api/bench/rsi/plans/{draft_id}/preview' if draft_id and (root(draft_id)/'preview.jpg').exists() else None
            publish('failed',status='failed',error=str(exc),diagnostic_url=diagnostic)
            if diagnostic:
                detail=getattr(exc,'detail',{'message':str(exc)})
                try:_write(root(draft_id)/'prepare-error.json',{'error':str(exc),'detail':detail,'robot_moved':False,'observation_trial_id':key})
                except OSError:pass
                raise HTTPException(status_code=409,detail={**detail,'message':str(exc),'image_url':diagnostic}) from exc
            fail(exc)
        finally:preparing.release()

    @router.post('/run',status_code=202)
    def run(req:Run):
        try:learner.assert_interactive_allowed()
        except ValueError as exc:fail(exc)
        automation_id=service_provider().automation_owner if learner.is_worker() else None
        with lock:
            if job.get('status') in ('running','preparing') or preparing.locked():raise HTTPException(status_code=409,detail='正在观察或执行')
            if req.plan_id not in plans:raise HTTPException(status_code=409,detail='计划不在当前服务中，请重新观察；不会重放旧计划')
            current=known(req.plan_id)
            if current['status']!='prepared' or time.monotonic()-current['created']>180:raise HTTPException(status_code=409,detail='计划已过期或消耗，请重新观察')
            if req.declaration!=DECLARATION:raise HTTPException(status_code=409,detail='请核对完整抓放路径并确认现场')
            if current.get('source')=='opencode_agent' and (req.scene_review is None or not req.vision_packet_id):
                raise HTTPException(status_code=409,detail='执行Agent必须附上本轮绑定的场景视觉复核')
            if req.scene_review is not None or req.vision_packet_id:
                try:
                    if not req.vision_packet_id or req.scene_review is None:raise ValueError('场景复核需要完整图片包与结论')
                    agent_vision.validate_binding(current,root(req.plan_id),req.vision_packet_id,phase='scene')
                    scene_review=grok_review._scene_verdict(req.scene_review)
                    if not scene_review['ready']:raise ValueError('视觉子Agent尚未确认本轮目标：'+scene_review['note'])
                    current['scene_review']={'vision_packet_id':req.vision_packet_id,**scene_review}
                    append_event(root(req.plan_id),'agent_scene_review',review=current['scene_review'])
                    learner.events.emit('grok_validated',plan_id=req.plan_id,verdict=scene_review)
                except (OSError,ValueError,KeyError,TypeError) as exc:fail(exc)
            current['status']='starting'
            try:save(current)
            except (OSError,ValueError) as exc:fail(exc)
            job.update(status='running',phase='recording_warmup',plan_id=req.plan_id,error=None,result=None)
        service=service_provider();stop.clear()
        try:
            with service.lock:
                actual=service.status();_ready(service.health())
                if service.allowed_joints!=set(range(1,7)):raise ValueError('完整抓放需要现有六关节授权')
                if fingerprint(session())!=current['calibration_fingerprint']:raise ValueError('标定已变化，请重新观察')
                active_policy=policy(session())
                if current['plan'].get('route_policy',{}).get('version')!=4:
                    raise ValueError('请重新观察，使用抓取与运输/投放独立姿态的新路径')
                if active_policy['version']!=current['policy_version']:raise ValueError('RSI参数已更新，请用新参数重新观察')
                if policy_id(current['plan']['parameters'])!=current['plan']['policy_application']['policy_id']:
                    raise ValueError('执行参数与本轮已核对的RSI参数不一致')
                application(active_policy['parameters'],current['plan']['parameters'],active_policy['reviews'],active_policy.get('baseline_resets',[]))
                if any(abs(actual[k]-current['plan']['initial_status'][k])>math.radians(1) for k in 'bsetrg'):raise ValueError('机械臂起点已变化，请重新观察')
                if (root(req.plan_id)/'attempt.json').exists():raise ValueError('本轮已有执行收据，不会重放')
                cam=client_request(Path.home(),'GET','/health')
                if cam.get('recording') or cam.get('robot_control') is not False or cam.get('recording_request_ids') is not True:raise ValueError('独立相机服务需要空闲并支持录像请求ID')
                validate_live(current);service.unlock_once('已检查设备并清空运动区域',automation_id=automation_id)
                service.pending=reservation(req.plan_id);current['status']='active';current['worker_finished']=False;save(current)
                publish('recording_warmup',health=service.health())
            def worker():
                try:
                    def validate_plan():
                        if fingerprint(session())!=current['calibration_fingerprint']:raise ValueError('执行中标定发生变化，停止后续动作')
                    result=execute(service,current['plan'],root(req.plan_id),stop,publish,camera=client_request,validate_plan=validate_plan,automation_id=automation_id)
                    result['policy_application']=current['plan']['policy_application']
                    current['result']=result;current['status']='completed' if result['motion_completed'] and not result.get('camera_error') and result.get('recording_covers_motion') else 'failed';save(current)
                    try:archive(current)
                    except (OSError,ValueError,KeyError) as exc:current['backup_error']=str(exc);save(current)
                    publish('finished',status=current['status'],result=result,error=result['motion_error'] or result.get('camera_error'))
                except Exception as exc:
                    with service.lock:
                        service.disarm()
                        if (service.pending or {}).get('run_id')==req.plan_id:service._latch_fault('rsi_execution_failed','本轮执行异常，请核对实物静止后复核')
                    current['status']='failed'
                    try:save(current)
                    except (OSError,ValueError):pass
                    publish('finished',status='failed',error=f'{type(exc).__name__}: {str(exc)[:500]}')
                finally:
                    current['worker_finished']=True
                    try:save(current)
                    except (OSError,ValueError):pass
            _launch(worker)
            return {'started':True,'plan_id':req.plan_id}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
            with service.lock:
                service.disarm()
                if (service.pending or {}).get('run_id')==req.plan_id:service.pending=None
            current['status']='failed';publish('failed',status='failed',error=str(exc))
            try:save(current)
            except (OSError,ValueError):pass
            fail(exc)

    @router.post('/stop')
    def stop_run(req:PlanID):
        with lock:
            if job.get('status')!='running' or job.get('plan_id')!=req.plan_id:raise HTTPException(status_code=409,detail='没有匹配的执行中试次')
            stop.set();job['phase']='stopping'
        threading.Thread(target=service_provider().disarm,daemon=True).start()
        return {'stop_requested':True,'notice':'仅阻止后续动作；已发指令可能继续'}

    @router.post('/review')
    def review(req:Review):
        try:
            learner.assert_interactive_allowed()
            with lock:
                if job.get('status') in ('running','preparing'):raise ValueError('请等当前执行或观察完成后再更新参数')
                current=known(req.plan_id,historical=True)
                if current['status'] not in ('completed','failed'):raise ValueError('请等本轮结束后复核')
                result=current.get('result',{})
                if req.outcome=='placed' and not result.get('placement_attempted'):raise ValueError('本轮未执行投放，不能标记抓放成功')
                pushed_before_close=req.outcome=='pushed_away' and any(d.get('stage')=='descend' for d in result.get('dispatches',[]))
                if req.outcome not in ('unknown','execution_failed') and not result.get('grasp_attempted') and not pushed_before_close:raise ValueError('本轮未执行闭爪，先记录控制/录像故障')
                if req.source in ('company_grok','opencode_agent') and req.outcome=='placed' and (result.get('camera_error') or not result.get('recording_covers_motion')):
                    raise ValueError('录像证据不完整，Grok不能据此宣称投放成功')
                saved=session()
                if fingerprint(saved)!=current['calibration_fingerprint']:raise ValueError('标定已变化，不能把旧轮次修正应用到新坐标系')
                pol=policy(saved)
                existing=next((r for r in pol['reviews'] if r['plan_id']==req.plan_id),None)
                if existing:
                    current['review']=existing;save(current);return existing
                if pol['parameters']!=current['plan']['parameters']:raise ValueError('本轮实际参数与当前策略不一致，不能重复应用旧试次修正')
                comparison=compare(current,pol);assessment=req.comparison.model_dump() if req.comparison else None
                execution_diagnosis=diagnose(result,current['plan'])
                if req.visual_review is not None:
                    if req.source!='opencode_agent' or not req.vision_packet_id:raise ValueError('视觉子Agent结论需要本轮图片包编号')
                    agent_vision.validate_binding(current,root(req.plan_id),req.vision_packet_id)
                    visual=grok_review.validate_verdict(req.visual_review)
                    for field in ('outcome','direction','ball_state','magnitude','confidence','adjustment'):
                        if visual.get(field,'unknown')!=getattr(req,field):raise ValueError('主Agent不得改写视觉子Agent的定性结论：'+field)
                    if visual.get('comparison')!=(assessment or None):raise ValueError('前后比较结论与视觉子Agent不一致')
                if req.requested_step_mm is not None and req.source!='opencode_agent':raise ValueError('数值试验步长只接受执行Agent工具，不接受视觉模型测量值')
                if req.requested_step_mm is not None and execution_diagnosis['category'] in ('time_budget','clearance_shortfall'):
                    raise ValueError('先解决执行预算或抬升不足；XY补偿不用于修复这类执行故障')
                if req.correction_direction is not None and req.correction_direction!=OPPOSITE.get(req.direction):
                    raise ValueError('纠正方向必须与原图观察偏差相反；不要再次镜像机械臂坐标')
                expected_adjustment={'grasp_missed':'pickup_xy','placement_missed':'release_xy'}.get(req.outcome)
                if req.requested_step_mm is not None and (req.adjustment!=expected_adjustment or req.direction not in CONTRACT['observed_side_to_correction']):
                    raise ValueError('XY试验步长必须对应图像XY偏差与相应调整项')
                parameters,changes,proposal=propose(pol['parameters'],current['plan']['scene'],req.outcome,req.direction,
                    req.magnitude,req.adjustment,history=pol['reviews'],comparison=comparison,assessment=assessment,requested_step_mm=req.requested_step_mm)
                execution_fault=execution_diagnosis['category'] in ('time_budget','clearance_shortfall')
                if execution_fault and not req.parameter_trial:
                    # Keep the visual finding, but do not let automatic visual
                    # adaptation silently turn an execution failure into XY learning.
                    parameters=copy.deepcopy(pol['parameters'])
                    changes=['先解决执行故障，保留视觉结论，暂不应用抓取坐标或夹持补偿。']
                    proposal={k:v for k,v in proposal.items() if k not in ('robot_delta_mm','predicted_pixel_delta','saved_xy_step_mm',
                        'projection_direction_verified','projection_direction_alignment','requested_step_preserved','candidate_robot_delta_mm','candidate_pixel_delta','rejected_policy_id')}
                    proposal.update(status='unchanged',decision='execution_change_required',parameters_changed=[],before={},after={},
                        step_mm=None,allow_retry=False,after_policy_id=policy_id(parameters),reason=changes[0])
                if req.parameter_trial:
                    if req.source!='opencode_agent' or req.visual_review is None:raise ValueError('标量参数试验需要视觉子Agent结论与执行Agent决策')
                    if req.requested_step_mm is not None:raise ValueError('每轮只选择XY步长或一个标量参数')
                    execution_trial=req.parameter_trial.parameter in execution_diagnosis['execution_trial_parameters']
                    if execution_fault and not execution_trial:raise ValueError('先按执行诊断调整速度或抬升高度，再学习抓取偏差')
                    if req.outcome=='placed' or req.outcome in ('unknown','execution_failed') and not execution_trial:
                        raise ValueError('本轮结果不支持该项参数学习；执行故障只能按诊断调整速度或抬升高度')
                    if proposal['status']=='rolled_back':raise ValueError('本轮先保留回滚，下一轮再选择新参数')
                    name=req.parameter_trial.parameter;value=req.parameter_trial.value
                    allowed={'grasp_missed':{'grasp_z_offset_mm','finger_offset_mm','open_deg','close_deg','approach_speed'},
                        'pushed_away':{'grasp_z_offset_mm','finger_offset_mm','open_deg','approach_speed'},
                        'slipped':{'squeeze_deg','travel_speed','close_deg'},
                        'placement_missed':{'release_clearance_mm'},'bounced_out':{'release_clearance_mm','travel_speed'}}
                    if name not in allowed.get(req.outcome,set()) and not execution_trial:raise ValueError('本轮观察结果与所选试验参数不对应')
                    parameters=copy.deepcopy(pol['parameters']);parameters[name]=value
                    changed=value!=pol['parameters'][name]
                    changes=[f'执行Agent选择试验：{name}从{pol["parameters"][name]}改为{value}；{req.note}'] if changed else ['参数未改变，不重复原策略']
                    proposal={k:v for k,v in proposal.items() if k not in ('robot_delta_mm','predicted_pixel_delta','saved_xy_step_mm',
                        'projection_direction_verified','projection_direction_alignment','requested_step_preserved','candidate_robot_delta_mm','candidate_pixel_delta')}
                    proposal.update(status='applied' if changed else 'unchanged',decision='execution_parameter_trial' if execution_trial else 'agent_parameter_trial',
                        parameters_changed=[name] if changed else [],before={name:pol['parameters'][name]} if changed else {},
                        after={name:value} if changed else {},step_mm=None,allow_retry=changed,
                        after_policy_id=policy_id(parameters),reason='\n'.join(changes))
                if execution_diagnosis['category'] in ('time_budget','clearance_shortfall') and parameters==pol['parameters']:
                    updated_recipe=current['plan'].get('route_policy',{}).get('version',0)<4
                    proposal.update(allow_retry=updated_recipe,decision='updated_execution_recipe' if updated_recipe else 'execution_change_required')
                    changes.append('下一轮验证新的合并接近/运输姿态抬升路径。' if updated_recipe else '执行问题未解决，不重复相同参数。')
                if req.source=='opencode_agent' and parameters!=pol['parameters']:
                    # Check the complete candidate recipe with real IK before persisting an Agent trial.
                    service=service_provider()
                    with service.lock:
                        fresh=service.status();_ready(service.health())
                        build_plan(saved,fresh,current['plan']['scene'],parameters)
                entry={'plan_id':req.plan_id,'outcome':req.outcome,'direction':req.direction,'source':req.source,
                       'note':req.note,'before_version':pol['version'],'after_version':pol['version']+1,
                       'before':pol['parameters'],'after':parameters,'changes':changes,'proposal':proposal,'at_unix':time.time(),
                       'comparison':public_comparison(comparison,assessment,plan_id=req.plan_id,panel=(root(req.plan_id)/'comparison.jpg').is_file())}
                entry.update(confidence=req.confidence,ball_state=req.ball_state,magnitude=req.magnitude,adjustment=req.adjustment)
                entry['execution_diagnosis']=execution_diagnosis
                if req.visual_review is not None:entry.update(vision_packet_id=req.vision_packet_id,visual_review=req.visual_review)
                if req.parameter_trial:entry['parameter_trial']=req.parameter_trial.model_dump()
                entry.update(ball_xyz_mm=current['plan']['scene']['ball']['center_robot_mm'],
                             grasp_pitch_deg=current['plan']['grasp_pitch_deg'],
                             funnel_xyz_mm=current['plan']['scene']['funnel']['mouth_center_robot_mm'])
                if req.source=='opencode_agent':
                    _write(root(req.plan_id)/'agent-review.json',{'request':req.model_dump(),'direction_contract':CONTRACT,
                           'candidate_policy_id':policy_id(parameters),'ik_preflight':parameters!=pol['parameters']})
                pol.update(version=entry['after_version'],parameters=parameters);pol['reviews'].append(entry)
                pol['statistics']={'reviewed_trials':len(pol['reviews']),
                    'successes':sum(r['outcome']=='placed' for r in pol['reviews'])}
                if req.outcome=='placed':
                    pol['successful_policy']={'parameters':copy.deepcopy(parameters),'plan_id':req.plan_id,
                        'version':entry['after_version'],'ball_xyz_mm':entry['ball_xyz_mm'],
                        'funnel_xyz_mm':entry['funnel_xyz_mm'],'at_unix':time.time()}
                _write(policy_path(saved),pol)  # Atomic policy + applied ID; retries cannot apply twice.
                current['review']=entry;save(current)
                append_event(root(req.plan_id),'policy_updated',review=entry)
                archive(current)
                if req.source=='opencode_agent':
                    learner.events.emit('grok_validated',plan_id=req.plan_id,verdict=req.visual_review or entry)
                    learner.events.emit('policy',plan_id=req.plan_id,review=entry,changes=changes)
                for old in plans.values():
                    if old['status']=='prepared':old['status']='superseded';save(old)
                return entry
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/context/{plan_id}')
    def context(plan_id:str):
        try:
            with lock:
                current=known(plan_id,historical=True);pol=policy(session())
                result=current.get('result',{});trial=result.get('capture_trial_id')
                compact={k:result.get(k) for k in ('motion_completed','grasp_attempted','placement_attempted','physical_elapsed_s','within_10s',
                                                  'recording_covers_motion','motion_error','camera_error','events','dispatches','execution_diagnosis')}
                compact['execution_diagnosis']=result.get('execution_diagnosis') or diagnose(result,current['plan'])
                return {'plan_id':plan_id,'plan':current['plan'],'result':compact,'review':current.get('review'),
                    'direction_contract':CONTRACT,
                    'comparison':compare(current,pol),
                    'recent_reviews':[{k:r.get(k) for k in ('plan_id','outcome','direction','source','changes','after_version','note','confidence','magnitude','proposal','ball_xyz_mm','funnel_xyz_mm')} for r in pol['reviews'][-5:]],
                    'current_policy':{'version':pol['version'],'parameters':pol['parameters'],'statistics':pol.get('statistics'),
                                      'successful_policy':pol.get('successful_policy')},
                    'video_path':str(bench_ui.experiment_root()/trial/'rgb.mp4') if trial else None,
                    'keyframes_path':result.get('keyframes_file'),
                    'instructions':'复核未镜像RGB录像，u向画面右、v向画面下；方向描述夹爪在哪里，不是建议移动方向。机械臂与相机相对摆放，不能把画面左右直接当成机械臂XYZ正负。给定性结果即可；补偿由程序经标定转换。遮挡用unknown，数值到位不证明抓住或入漏斗。',
                    'review_template':{'plan_id':plan_id,'outcome':'unknown','direction':'none','source':'company_grok','note':''},
                    'review_endpoint':'/api/bench/rsi/review','allowed_outcomes':['placed','grasp_missed','pushed_away','slipped','placement_missed','bounced_out','execution_failed','unknown'],
                    'allowed_directions':['none','image_left','image_right','image_up','image_down','too_high','too_low']}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/vision-context/{plan_id}')
    def vision_context(plan_id:str):
        nonlocal agent_stream,agent_phase
        try:
            with lock:
                current=known(plan_id,historical=True);pol=policy(session())
                if current['calibration_fingerprint']!=fingerprint(session()):raise ValueError('本轮标定已变化')
                if current['status'] not in ('prepared','completed','failed'):raise ValueError('请等待本轮观察或执行完成')
                packet=agent_vision.packet(current,compare(current,pol),root(plan_id),bench_ui.experiment_root())
                if learner.snapshot().get('status')!='running':
                    if agent_stream is None or learner.events.session_id!=agent_stream:
                        agent_stream=uuid.uuid4().hex;agent_phase=None;learner.events.reset(agent_stream)
                    learner.events.emit('phase',phase='scene_check' if packet['phase']=='scene' else 'analyzing',
                                        plan_id=plan_id,message='图片包已准备，等待只读Grok视觉子Agent复核')
                return packet
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    def memory_path(saved):return Path.home()/'.evoarm/rsi/agent_memory'/saved['setup_id']/f'{fingerprint(saved)}.json'

    @router.get('/agent/readiness')
    def agent_readiness():
        service=service_provider();raw=None;error=None
        with service.lock:
            try:
                if service.arm is not None:raw=service.status()
            except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:error=str(exc)
            health=service.health()
        value=describe_readiness(health,raw,error)
        value.update(api_version=10,code_revision=code_revision,
            controller_project_root=str(Path(__file__).resolve().parent.parent),
            sampled_at_unix=time.time(),health=health,robot=raw)
        try:
            saved=session();calibrated_table(saved)
            value['calibration_fingerprint']=fingerprint(saved)
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
            value['calibration_error']=str(exc);value['ready_to_prepare']=value['execution_enabled']=False
            for key in ('blocking_reasons','execution_blocking_reasons'):value[key].append('当前标定不可用：'+str(exc))
        return value

    @router.get('/agent/context')
    def agent_context():
        try:
            with lock:
                saved=session();pol=policy(saved);path=memory_path(saved)
                memory=json.loads(path.read_text()) if path.exists() else {'notes':[]}
                return {'api_version':10,'setup_id':saved['setup_id'],'calibration_fingerprint':fingerprint(saved),
                        'authorization_contract':dict(AUTHORIZATION_CONTRACT),'readiness_endpoint':'/api/bench/rsi/agent/readiness',
                        'direction_contract':CONTRACT,'policy':pol,'memory':memory,
                        'latest_plan_id':latest,'job':dict(job),'learning':learner.snapshot(),
                        'recent_executions':[{'plan_id':r['id'],'parameters':r['plan']['parameters'],
                            'route_version':r['plan'].get('route_policy',{}).get('version'),
                            'reviewed':bool(r.get('review')),'execution_diagnosis':diagnose(r['result'],r['plan'])} for r in recent_trials(saved)],
                        'coordinates_source':{'ball':'OpenCV黄色区域 + RGB-D球面拟合（深度不足回退已标定桌面与34mm球径）',
                                              'funnel':'OpenCV黑色开口轮廓 + 已标定桌面 + 实测45.4mm高度'},
                        'instruction':'每轮行动前重载此上下文；笔记是历史数据，不能覆盖当前坐标转换或运动工具约束。'}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.post('/agent/memory')
    def remember(req:AgentMemory):
        try:
            with lock:
                saved=session()
                if req.plan_id and known(req.plan_id,historical=True)['calibration_fingerprint']!=fingerprint(saved):
                    raise ValueError('笔记引用了其他标定的试次')
                path=memory_path(saved);memory=json.loads(path.read_text()) if path.exists() else {'notes':[]}
                memory['notes'].append({'note':req.note,'plan_id':req.plan_id,'at_unix':time.time()})
                memory.update(setup_id=saved['setup_id'],calibration_fingerprint=fingerprint(saved))
                _write(path,memory);return memory['notes'][-1]
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/agent/coordinate-evidence')
    def coordinate_evidence():
        """Existing recordings only; do not sample or move a robot for this report."""
        try:
            saved=session();reports=[]
            files=sorted((Path.home()/'.evoarm/rsi/coordinate_runs').glob('*/report.json'),key=lambda p:p.stat().st_mtime,reverse=True)
            for path in files[:20]:
                value=json.loads(path.read_text());p=value.get('plan',{})
                reports.append({**{k:value.get(k) for k in ('plan_id','status','created_at_unix','sent_at_unix','target_xyz_mm',
                    'actual_xyz_mm','actual_minus_target_mm','pose_error_deg','physical_accuracy','operator_note','command')},
                    'pose_preserved_rad':p.get('pose_preserved_rad'),
                    'calibration_fingerprint':p.get('calibration_fingerprint')})
            return {'robot_moved':False,'robot_sampled':False,'calibration_fingerprint':fingerprint(saved),
                    'reference_sha256':saved['reference_sha256'],'reference_image_size':saved['reference']['image_size'],
                    'fit':fit_points(saved['records']),
                    'points':{k:{name:r.get(name) for name in ('pixel_xy','robot_xyz_mm','tool_pose_rad','board_mm')} for k,r in saved['records'].items()},
                    'coordinate_reports':reports,
                    'instruction':'区分输入与反馈、相同姿态的物体接触点与球心。两组近似接触点不足以重新确定完整相机外参；不要自动拟合后覆盖标定。'}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/history')
    def history():
        base=Path.home()/'.evoarm/rsi/pick_place_runs';rows=[]
        for file in sorted(base.glob('*/state.json'),key=lambda f:f.stat().st_mtime,reverse=True):
            try:
                current=json.loads(file.read_text())
                if not current.get('result') and not (file.parent/'attempt.json').exists():continue
                if current['plan']['setup_id']==bench_ui.setup_id():
                    row={k:current.get(k) for k in ('id','status','policy_version','review')}
                    row['backup_url']=f"/api/bench/rsi/plans/{current['id']}/archive" if (file.parent/'backup.json').is_file() else None
                    row['result']={k:current.get('result',{}).get(k) for k in ('motion_completed','grasp_attempted','placement_attempted','physical_elapsed_s','within_10s','motion_error','camera_error','capture_trial_id')}
                    rows.append(row)
                    if len(rows)>=30:break
            except (OSError,ValueError,KeyError,TypeError):continue
        return {'trials':rows}

    @router.get('/plans/{plan_id}/{kind}')
    def asset(plan_id:str,kind:str):
        if len(plan_id)!=32 or any(c not in '0123456789abcdef' for c in plan_id):raise HTTPException(status_code=404)
        vision=re.fullmatch(r'vision-([a-f0-9]{16})-([0-9]{1,2})',kind)
        if vision:path=root(plan_id)/'vision-evidence'/vision[1]/f'frame-{int(vision[2]):02d}.jpg';media='image/jpeg'
        elif kind in ('preview','depth','comparison'):path=root(plan_id)/f'{kind}.jpg';media='image/jpeg'
        elif kind=='archive':path=Path.home()/'.evoarm/rsi/backups'/f'{plan_id}.zip';media='application/zip'
        elif kind in ('video','raw-video','keyframes'):
            try:trial=known(plan_id,historical=True).get('result',{}).get('capture_trial_id')
            except ValueError:trial=None
            if not trial:raise HTTPException(status_code=404)
            folder=bench_ui.experiment_root()/trial
            if kind=='video':
                try:path=browser_video(folder)
                except (OSError,RuntimeError,ValueError) as exc:fail(exc)
            else:path=folder/('rgb.mp4' if kind=='raw-video' else 'grasp-keyframes.jpg')
            media='image/jpeg' if kind=='keyframes' else 'video/mp4'
        else:raise HTTPException(status_code=404)
        if not path.is_file() or path.is_symlink():raise HTTPException(status_code=404)
        return FileResponse(path,media_type=media,headers={'Cache-Control':'no-store','X-Content-Type-Options':'nosniff'})

    def recover_automatic(current,verdict,stop_event,automation_id):
        report=recover_trial(service_provider(),current['plan'],current.get('result',{}),verdict,root(current['id']),
                             stop_event,learner.publish,automation_id=automation_id)
        if not stop_event.is_set():
            try:
                capture=client_request(Path.home(),'POST','/record',{'seconds':2})
                key=capture['trial_id']
                if len(key)!=32 or any(c not in '0123456789abcdef' for c in key):raise ValueError('恢复观察ID无效')
                frame=load_rgbd(bench_ui.experiment_root()/key)
                if not cv2.imwrite(str(root(current['id'])/'recovery-final.jpg'),frame['bgr']):raise ValueError('恢复观察图保存失败')
                report['verification_trial_id']=key
            except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
                report['camera_error']=str(exc)
        return report

    def reset_failure(key,current,exc,phase,stop_event,automation_id):
        folder=Path.home()/'.evoarm/rsi/learning_sessions'/automation_id/'recoveries'/key
        info=error_info(exc)
        _write(folder/'cause.json',{'phase':phase,'error_type':type(exc).__name__,**info,
                                   'related_plan_id':current['id'] if current else None,'code_revision':code_revision})
        # Preserve the rejected planning observation, without manufacturing a
        # successful plan or counting it as another physical grasp.
        image_url=info.get('image_url')
        if isinstance(image_url,str):
            parts=image_url.split('/')
            draft=parts[-2] if len(parts)>2 else ''
            if len(draft)==32 and all(c in '0123456789abcdef' for c in draft):
                for name in ('preview.jpg','prepare-error.json','planning-input.json','calibration-snapshot.json'):
                    file=root(draft)/name
                    if file.is_file() and not file.is_symlink():shutil.copy2(file,folder/name)
                diagnostic=folder/'prepare-error.json'
                if diagnostic.is_file():
                    observation=json.loads(diagnostic.read_text()).get('observation_trial_id','')
                    if len(observation)==32 and all(c in '0123456789abcdef' for c in observation):
                        target=folder/'observation';target.mkdir(exist_ok=True)
                        for name in ('rgb.mp4','frames.jsonl','depth_samples.npz','gemini_color_intrinsics.json','manifest.json'):
                            file=bench_ui.experiment_root()/observation/name
                            if file.is_file() and not file.is_symlink():shutil.copy2(file,target/name)
        result=current.get('result',{}) if current and current.get('recovery',{}).get('status')!='completed' else {}
        try:
            return recover_trial(service_provider(),None,result,{'ball_state':'unknown'},folder,stop_event,
                                 learner.publish,automation_id=automation_id)
        finally:
            if current:
                target=root(current['id'])/'exception-recoveries'/key
                if folder.is_dir():shutil.copytree(folder,target,dirs_exist_ok=True)
                report=folder/'recovery.json'
                if report.is_file():current.setdefault('recovery_events',[]).append({'id':key,'phase':phase,'report':json.loads(report.read_text())})

    learner=LearningLoop(service_provider,prepare=lambda:prepare(Prepare())['plan_id'],
        run=lambda key:run(Run(plan_id=key,declaration=DECLARATION)),get_trial=known,context=context,
        apply_review=lambda key,value:review(Review(plan_id=key,source='company_grok',**value)),
        recover=recover_automatic,archive=archive,trial_stop=stop,reset_failure=reset_failure)

    @router.post('/learning/start',status_code=202)
    def start_learning(req:Learn):
        try:return learner.start(req.declaration,req.max_attempts)
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.post('/learning/stop')
    def stop_learning(req:Empty):return learner.stop()

    @router.post('/learning/reset',status_code=202)
    def reset_official(req:Reset):
        try:return learner.reset_to_official(req.declaration)
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/sessions/{session_id}/archive')
    def session_archive(session_id:str):
        if len(session_id)!=32 or any(c not in '0123456789abcdef' for c in session_id):raise HTTPException(status_code=404)
        path=Path.home()/'.evoarm/rsi/backups'/f'session-{session_id}.zip'
        if not path.is_file() or path.is_symlink():raise HTTPException(status_code=404)
        return FileResponse(path,media_type='application/zip',headers={'Cache-Control':'no-store'})

    app.include_router(router)
