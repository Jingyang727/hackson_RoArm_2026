"""Browser pick/place trials and qualitative RSI review, using one ArmService."""
import copy
import json
import math
from pathlib import Path
import threading
import time
import uuid
import subprocess

import cv2
import numpy as np
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel, ConfigDict, Field

from . import bench_ui
from .vision import find_yellow_ball
from evoarm_rsi.bench_calibration import load_session
from evoarm_rsi.coordinate_test import fingerprint
from evoarm_rsi.bench_start import _ready
from evoarm_rsi.camera_service import client_request, preview_frame
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS, observe_scene, build_plan, preview, detect_funnel, adapt
from evoarm_rsi.rsi_execution import execute, reservation
from evoarm_rsi.browser_video import browser_video
from evoarm_rsi.task_agent import _write
from evoarm_rsi.task_scene import load_rgbd
from evoarm_rsi import grok_review
from evoarm_rsi.rsi_journal import append_event, archive_trial
from evoarm_rsi.rsi_learning import LearningLoop, LEARNING_DECLARATION
from evoarm_rsi.rsi_recovery import recover as recover_trial

DECLARATION='已核对黄球、漏斗和完整抓放路径，现场有人看护且双手离开运动范围'


def _launch(target):
    threading.Thread(target=target,name='evoarm-rsi-pick-place',daemon=True).start()


class Empty(BaseModel):
    model_config=ConfigDict(extra='forbid')


class PlanID(Empty):
    plan_id:str=Field(pattern=r'^[a-f0-9]{32}$')


class Run(PlanID):
    declaration:str


class Review(PlanID):
    outcome:str=Field(pattern=r'^(placed|grasp_missed|pushed_away|slipped|placement_missed|bounced_out|execution_failed|unknown)$')
    direction:str=Field(default='none',pattern=r'^(none|image_left|image_right|image_up|image_down|too_high|too_low)$')
    source:str=Field(default='human',pattern=r'^(human|company_grok)$')
    note:str=Field(default='',max_length=2000)
    confidence:float|None=Field(default=None,ge=0,le=1)
    ball_state:str|None=Field(default=None,pattern=r'^(table|gripper|funnel|unknown)$')
    magnitude:str=Field(default='medium',pattern=r'^(small|medium|large|unknown)$')
    adjustment:str=Field(default='unknown',pattern=r'^(keep|pickup_xy|grasp_height|grip_tighten|transport_slower|release_xy|release_height|unknown)$')


class Learn(Empty):
    declaration:str
    max_attempts:int=Field(default=5,ge=1,le=20,strict=True)


def install(app,service_provider):
    router=APIRouter(prefix='/api/bench/rsi');lock=threading.RLock();preparing=threading.Lock()
    plans={};job={'status':'idle','phase':'idle'};stop=threading.Event();latest=None
    try:code_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=Path(__file__).resolve().parent.parent,text=True,stderr=subprocess.DEVNULL,timeout=3).strip()
    except (OSError,subprocess.SubprocessError):code_revision='unknown'
    def root(key):return Path.home()/'.evoarm/rsi/pick_place_runs'/key
    def session():
        _,saved=load_session(bench_ui.session_path())
        if saved['setup_id']!=bench_ui.setup_id():raise ValueError('当前布置与保存标定不一致')
        return saved
    def policy_path(saved):return Path.home()/'.evoarm/rsi/pick_place_policies'/saved['setup_id']/f'{fingerprint(saved)}.json'
    def policy(saved):
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
    def archive(state):
        save(state)
        path=archive_trial(root(state['id']))
        state['backup_file']=path
        save(state)
        return path
    def fail(exc):raise HTTPException(status_code=409,detail=str(exc)) from exc
    def publish(phase,**values):
        with lock:job.update(phase='stopping' if stop.is_set() and phase not in ('finished','saving_recording') else phase,**values)
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
                    'steps':[s['stage'] for s in p['steps']],'pitch_deg':p['grasp_pitch_deg'],'warnings':p['warnings'],
                    'review':current.get('review')}
                plan['backup_url']=f"/api/bench/rsi/plans/{current['id']}/archive" if (root(current['id'])/'backup.json').is_file() else None
        service=service_provider()
        if snapshot.get('status') not in ('running','preparing') and automatic['status']!='running':
            try:
                if service.arm:service.status()
            except (OSError,RuntimeError,ValueError,KeyError,TypeError):pass
        health=snapshot.get('health') if snapshot.get('status')=='running' or automatic['status']=='running' else None
        return {'api_version':2,'setup_id':bench_ui.setup_id(),'health':health or service.health(),'job':snapshot,'plan':plan,
                'learning':automatic,'grok':grok_review.configuration(),'code_revision':code_revision}

    @router.post('/prepare')
    def prepare(req:Empty):
        nonlocal latest
        draft_id=None
        try:learner.assert_interactive_allowed()
        except ValueError as exc:fail(exc)
        if not preparing.acquire(blocking=False):raise HTTPException(status_code=409,detail='正在观察，请等待')
        try:
            with lock:
                if job.get('status')=='running':raise ValueError('本轮仍在执行或保存录像')
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
            pol=policy(saved)
            with service.lock:
                fresh=service.status();_ready(service.health())
                if any(abs(initial[k]-fresh[k])>math.radians(1) for k in 'bsetrg'):raise ValueError('观察时机械臂发生移动，请重新规划')
                plan=build_plan(saved,fresh,scene,pol['parameters'])
            plan_id=draft_id
            if not cv2.imwrite(str(folder/'preview.jpg'),preview(image,plan,geometry)):raise ValueError('预览保存失败')
            valid=depth>0;colored=np.zeros((*depth.shape,3),np.uint8)
            if np.any(valid):
                low,high=np.percentile(depth[valid],[2,98]);scaled=np.clip((depth-low)/max(1.,high-low)*255,0,255).astype(np.uint8)
                colored=cv2.applyColorMap(scaled,cv2.COLORMAP_TURBO);colored[~valid]=0
                scene['depth_scale']={'near_mm':float(low),'far_mm':float(high)}
            cv2.imwrite(str(folder/'depth.jpg'),colored)
            state={'id':plan_id,'status':'prepared','created':time.monotonic(),'plan':plan,'policy_version':pol['version'],
                   'calibration_fingerprint':fingerprint(saved),'geometry':geometry,'code_revision':code_revision}
            _write(folder/'calibration-snapshot.json',saved)
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
                try:_write(root(draft_id)/'prepare-error.json',{'error':str(exc),'robot_moved':False,'observation_trial_id':key})
                except OSError:pass
                raise HTTPException(status_code=409,detail={'message':str(exc),'image_url':diagnostic}) from exc
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
                if policy(session())['version']!=current['policy_version']:raise ValueError('RSI参数已更新，请用新参数重新观察')
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
                if req.source=='company_grok' and req.outcome=='placed' and (result.get('camera_error') or not result.get('recording_covers_motion')):
                    raise ValueError('录像证据不完整，Grok不能据此宣称投放成功')
                saved=session()
                if fingerprint(saved)!=current['calibration_fingerprint']:raise ValueError('标定已变化，不能把旧轮次修正应用到新坐标系')
                pol=policy(saved)
                existing=next((r for r in pol['reviews'] if r['plan_id']==req.plan_id),None)
                if existing:
                    current['review']=existing;save(current);return existing
                parameters,changes=adapt(pol['parameters'],current['plan']['scene'],req.outcome,req.direction,req.magnitude,req.adjustment)
                entry={'plan_id':req.plan_id,'outcome':req.outcome,'direction':req.direction,'source':req.source,
                       'note':req.note,'before_version':pol['version'],'after_version':pol['version']+1,
                       'before':pol['parameters'],'after':parameters,'changes':changes,'at_unix':time.time()}
                entry.update(confidence=req.confidence,ball_state=req.ball_state,magnitude=req.magnitude,adjustment=req.adjustment)
                entry.update(ball_xyz_mm=current['plan']['scene']['ball']['center_robot_mm'],
                             funnel_xyz_mm=current['plan']['scene']['funnel']['mouth_center_robot_mm'])
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
                                                  'recording_covers_motion','motion_error','camera_error','events','dispatches')}
                return {'plan_id':plan_id,'plan':current['plan'],'result':compact,'review':current.get('review'),
                    'recent_reviews':[{k:r.get(k) for k in ('plan_id','outcome','direction','source','changes','after_version','note','confidence','ball_xyz_mm','funnel_xyz_mm')} for r in pol['reviews'][-5:]],
                    'current_policy':{'version':pol['version'],'parameters':pol['parameters'],'statistics':pol.get('statistics'),
                                      'successful_policy':pol.get('successful_policy')},
                    'video_path':str(bench_ui.experiment_root()/trial/'rgb.mp4') if trial else None,
                    'keyframes_path':result.get('keyframes_file'),
                    'instructions':'使用本地公司Grok复核未镜像RGB录像/关键帧，只给定性结果。方向指夹爪相对球或漏斗在画面中偏哪边。不要估计毫米值；遮挡或证据不足用unknown。数值到位不证明抓住或入漏斗。',
                    'review_template':{'plan_id':plan_id,'outcome':'unknown','direction':'none','source':'company_grok','note':''},
                    'review_endpoint':'/api/bench/rsi/review','allowed_outcomes':['placed','grasp_missed','pushed_away','slipped','placement_missed','bounced_out','execution_failed','unknown'],
                    'allowed_directions':['none','image_left','image_right','image_up','image_down','too_high','too_low']}
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
        if kind in ('preview','depth'):path=root(plan_id)/f'{kind}.jpg';media='image/jpeg'
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
        if fingerprint(session())!=current['calibration_fingerprint']:
            raise ValueError('标定已变化，停止使用旧坐标执行自动恢复')
        def fresh_release():
            packet=preview_frame(Path.home());image=cv2.imdecode(np.frombuffer(packet['jpeg'],np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('恢复时无法读取当前漏斗画面')
            funnel=detect_funnel(image,current['geometry'])
            if funnel['ambiguous']:raise ValueError('恢复时漏斗有多个候选，停止自动投放')
            old=np.asarray(current['plan']['scene']['funnel']['mouth_center_robot_mm'])
            delta=np.asarray(funnel['mouth_center_robot_mm'])-old
            return (np.asarray(current['plan']['release_tip_xyz_mm'])+delta).tolist()
        report=recover_trial(service_provider(),current['plan'],current.get('result',{}),verdict,root(current['id']),
                             stop_event,learner.publish,automation_id=automation_id,validate_funnel=fresh_release)
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

    def verify_recovery(current,stop_event):
        key=current['recovery'].get('verification_trial_id')
        if not key:raise ValueError('恢复后观察缺失，不能自动判断最终投放')
        frame=load_rgbd(bench_ui.experiment_root()/key)
        file=root(current['id'])/'recovery-final.jpg'
        evidence=context(current['id']);evidence['review_phase']='recovery';evidence['recovery']=current['recovery']
        start=current['result']['events'][0]['host_monotonic_ns']
        evidence['additional_frames']=[{'path':str(file),'elapsed_s':(frame['host_monotonic_ns']-start)/1e9,
                                        'label':'current_view_after_automatic_recovery'}]
        return grok_review.review(evidence,root(current['id']),stop_event)

    learner=LearningLoop(service_provider,prepare=lambda:prepare(Empty())['plan_id'],
        run=lambda key:run(Run(plan_id=key,declaration=DECLARATION)),get_trial=known,context=context,
        apply_review=lambda key,value:review(Review(plan_id=key,source='company_grok',**value)),
        recover=recover_automatic,archive=archive,trial_stop=stop,verify_recovery=verify_recovery)

    @router.post('/learning/start',status_code=202)
    def start_learning(req:Learn):
        try:return learner.start(req.declaration,req.max_attempts)
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.post('/learning/stop')
    def stop_learning(req:Empty):return learner.stop()

    app.include_router(router)
