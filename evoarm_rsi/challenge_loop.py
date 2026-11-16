"""Three-colour cleanup orchestration, reusing the main motion/recording path."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

import cv2

from control_ui import bench_ui
from .bench_calibration import load_session
from .bench_start import _ready
from .coordinate_test import fingerprint,calibrated_table
from .camera_service import client_request
from .pick_place import DEFAULT_PARAMETERS,build_plan
from .rsi_execution import execute,reservation
from .rsi_recovery import recover
from .rsi_events import LearningEvents
from .rsi_journal import archive_trial
from .rsi_policy import policy_id,propose
from .policy_repair import repair_clipped_policy
from .release_geometry import upgrade_release_policy
from .execution_diagnosis import diagnose
from .learning_basis import basis,execution_proposal
from . import challenge_scene,grok_review
from .task_agent import _write

DECLARATION='现场已清空，允许连续抓球入碗及故障时自动回位'
MOTION_PROFILE=2


def launch(target):threading.Thread(target=target,name='evoarm-challenge',daemon=True).start()


class Challenge:
    def __init__(self,service_provider):
        self.service_provider=service_provider;self.lock=threading.RLock();self.stop=threading.Event()
        self.value={'status':'idle','phase':'idle','confirmed_colors':[],'remaining':None,'attempt':0,'trials':[]}
        self.events=LearningEvents(folder=self.root)

    def root(self,key):return Path.home()/'.evoarm/rsi/challenge_sessions'/key

    def session(self):
        _,saved=load_session(bench_ui.session_path())
        if saved['setup_id']!=bench_ui.setup_id():raise ValueError('活动标定与当前布置不一致')
        calibrated_table(saved)
        return saved

    def policy_path(self,saved):return Path.home()/'.evoarm/rsi/challenge_policies'/saved['setup_id']/f'{fingerprint(saved)}.json'

    def policy(self,saved):
        path=self.policy_path(saved)
        if path.exists():
            pol=json.loads(path.read_text())
            if pol.get('challenge_motion_profile')==MOTION_PROFILE:return pol
            pol=copy.deepcopy(pol)
        else:
            source=Path.home()/'.evoarm/rsi/pick_place_policies'/saved['setup_id']/f'{fingerprint(saved)}.json'
            base=json.loads(source.read_text()) if source.exists() else {'parameters':copy.deepcopy(DEFAULT_PARAMETERS),'reviews':[],'version':1}
            base=copy.deepcopy(base);base['parameters']={**DEFAULT_PARAMETERS,**base['parameters']}
            base,_=repair_clipped_policy(base);base,_=upgrade_release_policy(base)
            pol={'version':1,'parameters':copy.deepcopy(base['parameters']),'reviews':[],
                 'source_single_ball_policy_id':policy_id(base['parameters'])}
        before=copy.deepcopy(pol['parameters'])
        pol['parameters'].update(travel_speed=1.,approach_speed=max(.6,pol['parameters']['approach_speed']))
        pol['challenge_motion_profile']=MOTION_PROFILE
        pol['speed_profile_migration']={'before':before,'after':copy.deepcopy(pol['parameters']),
            'reason':'仅挑战模式采用快速转移1.0、下探0.6；后续滑落学习可下调，不每轮强行覆盖。'}
        return pol

    def snapshot(self):
        with self.lock:return copy.deepcopy(self.value)

    def publish(self,phase,**values):
        with self.lock:
            previous=self.value.get('phase');self.value.update(phase=phase,**values)
            if phase!=previous or values.get('message'):
                self.events.emit('phase',phase=phase,attempt=self.value['attempt'],message=values.get('message'))
            if self.value.get('session_id'):_write(self.root(self.value['session_id'])/'state.json',self.value)

    def guard(self,saved=None):
        if self.stop.is_set():raise RuntimeError('用户已停止挑战')
        if saved is not None and fingerprint(self.session())!=fingerprint(saved):raise RuntimeError('活动标定已变化，停止后续动作')

    def _capture(self,saved,folder,*,fast=True,source=None,verified_colors=()):
        self.guard(saved)
        if source is None:
            health=client_request(Path.home(),'GET','/health')
            until=time.monotonic()+1
            while health.get('recording') and time.monotonic()<until:
                self.guard(saved);self.stop.wait(.03);health=client_request(Path.home(),'GET','/health')
            if health.get('recording') or health.get('camera_service')!='ready' or health.get('robot_control') is not False:
                raise RuntimeError('独立相机服务需空闲并处于ready状态')
            self.adaptive_recording=health.get('recording_finish') is True
            with self.lock:self.value['adaptive_recording']=self.adaptive_recording
            capture=client_request(Path.home(),'POST','/record',{'seconds':.45})
            key=capture['trial_id']
        else:key=source
        if len(key)!=32 or any(c not in '0123456789abcdef' for c in key):raise ValueError('相机返回无效录像ID')
        folder.mkdir(parents=True,exist_ok=True)
        value=challenge_scene.observe(bench_ui.experiment_root()/key,saved,folder,self.stop,
            lambda u:self.events.model(u,attempt=self.value['attempt'],plan_id=folder.name),fast=fast,verified_colors=verified_colors)
        _write(folder/'calibration-snapshot.json',saved)
        return value

    def observe(self):
        service=self.service_provider()
        with service.lock,self.lock:
            if self.value['status']=='running' or service.automation_owner or service.pending:raise ValueError('先等当前任务结束再单独识别')
            if self.value['status']=='observing':raise ValueError('正在识别')
            key=uuid.uuid4().hex;self.stop=threading.Event();self.events.reset(key)
            self.value={'status':'observing','phase':'observing','attempt':0,'trials':[],'error':None,
                        'confirmed_colors':[],'remaining':None}
            service.automation_owner=key  # Also reserves the shared camera during this read-only capture.
        try:
            scene,_,_=self._capture(self.session(),self.root(key)/'observations'/key)
            with self.lock:self.value.update(status='idle',phase='ready',scene=scene,remaining=len(scene['balls'])+len(scene['unresolved']),observation_session_id=key,observation_id=key)
            return scene
        finally:
            with service.lock:
                if service.automation_owner==key:service.automation_owner=None
            with self.lock:
                if self.value['status']=='observing':self.value['status']='idle'

    def start(self,declaration):
        if declaration!=DECLARATION:raise ValueError('请确认现场已清空并允许连续抓放及故障回位')
        config=grok_review.configuration()
        if not config['configured']:raise ValueError(config['reason'])
        saved=self.session();service=self.service_provider()
        with service.lock,self.lock:
            if self.value['status'] in ('running','observing') or service.automation_owner or service.pending:raise ValueError('已有任务占用控制器或相机')
            if not service.arm or not service.motion_enabled or service.teach_active:raise ValueError('请连接机械臂并开启实物执行，确认扭矩锁定')
            raw=service.status()
            if any(raw.get('torswitch'+k)!=1 for k in 'BSETRG'):raise ValueError('六关节锁定反馈尚未确认')
            key=uuid.uuid4().hex;self.stop=threading.Event();self.events.reset(key)
            self.value={'session_id':key,'status':'running','phase':'starting','attempt':0,'remaining':None,
                'initial_colors':[],'confirmed_colors':[],'trials':[],'error':None,
                'started_at_unix':time.time(),'calibration_fingerprint':fingerprint(saved)}
            service.automation_owner=key;self.publish('starting')
        try:launch(lambda:self._worker(key,saved))
        except Exception:
            with service.lock:service.automation_owner=None
            self.publish('failed',status='failed');raise
        return {'started':True,'session_id':key}

    def cancel(self):
        with self.lock:
            if self.value['status']!='running':return {'stop_requested':False}
            self.stop.set();self.publish('stopping')
        threading.Thread(target=self.service_provider().disarm,daemon=True).start()
        return {'stop_requested':True}

    def _archive(self,current,folder):
        _write(folder/'state.json',current);archive_trial(folder)
        outbox=Path.home()/'.evoarm/rsi/langfuse-outbox'/f"{current['id']}.json"
        packet=json.loads(outbox.read_text());packet['task']=current['task'];_write(outbox,packet)

    def _trial_folder(self,current):
        return self.root(current['task']['session_id'])/'attempts'/current['id']

    def _trial_row(self,current):
        row={'plan_id':current['id'],'target_color':current['task']['target_color'],
             'attempt':current['task']['attempt'],
             'outcome':current.get('review',{}).get('outcome','unknown'),
             'review_status':current.get('review_status','pending'),
             'motion_completed':current.get('result',{}).get('motion_completed'),
             'physical_elapsed_s':current.get('result',{}).get('physical_elapsed_s')}
        with self.lock:
            self.value['trials']=[r for r in self.value['trials'] if r['plan_id']!=current['id']]
            self.value['trials'].append(row);self.value['trials']=sorted(self.value['trials'],key=lambda r:r['attempt'])[-100:]

    def _attempt(self,key,saved,scene,g,image,ball,plan,pol):
        service=self.service_provider();trial=uuid.uuid4().hex;folder=self.root(key)/'attempts'/trial
        folder.mkdir(parents=True);count=self.value['attempt']+1
        current={'id':trial,'status':'prepared','plan':plan,'calibration_fingerprint':fingerprint(saved),
            'policy_version':pol['version'],'review_status':'pending',
            'task':{'kind':'challenge_upgrade','session_id':key,'target_color':ball['color'],
                'target_label':ball['label'],'attempt':count}}
        _write(folder/'calibration-snapshot.json',saved);_write(folder/'state.json',current)
        cv2.imwrite(str(folder/'observation.jpg'),image)
        self.publish('executing',attempt=count,target=ball,plan_id=trial,
            transfer_height_mm=plan['route_policy']['transfer_height_mm'],speeds={k:pol['parameters'][k] for k in ('travel_speed','approach_speed')},
            adaptive_recording=getattr(self,'adaptive_recording',False))
        self.events.emit('target',attempt=count,ball=ball,transfer_height_mm=plan['route_policy']['transfer_height_mm'])
        try:
            with service.lock:
                self.guard(saved);actual=service.status();_ready(service.health())
                if any(abs(actual[k]-plan['initial_status'][k])>math.radians(1) for k in 'bsetrg'):raise ValueError('规划时起点发生变化，尚未发送抓放')
                service.unlock_once('已检查设备并清空运动区域',automation_id=key)
                service.pending=reservation(trial)
            result=execute(service,plan,folder,self.stop,lambda phase,**v:self.publish(phase),
                camera=client_request,automation_id=key,validate_plan=lambda:self.guard(saved),
                finish_recording=getattr(self,'adaptive_recording',False))
            current.update(result=result,status='executed' if result.get('motion_completed') else 'failed')
            _write(folder/'state.json',current);self.guard(saved)
            diagnosis=diagnose(result,plan)
            if diagnosis['category']!='complete':
                # A missed grasp is a visual result, not a controller fault.
                # Only uncertain/failed execution uses the official reset.
                self.publish('recovering',message='本轮执行异常，自动恢复；正常逐球抓放不回位。')
                current['recovery']=recover(service,plan,result,{'ball_state':'unknown'},folder,self.stop,
                    lambda phase,**v:self.publish(phase),automation_id=key)
                self._apply_review(saved,current,{'outcome':'execution_failed','ball_state':'unknown','direction':'none','magnitude':'unknown'},pol,source='execution')
                if diagnosis['category'] in ('execution_error','camera_error','not_started') or not current['review']['proposal'].get('parameters_changed'):
                    raise RuntimeError('本轮执行未确认：'+str(diagnosis.get('error')))
            if not result.get('capture_trial_id') or result.get('camera_error'):
                raise RuntimeError('本轮缺少完整录像证据，未判定成功')
            return current
        except Exception as exc:
            current['error']=str(exc)[:1500];current['status']='stopped' if self.stop.is_set() else 'failed';raise
        finally:
            if (folder/'attempt.json').exists() and not current.get('result'):
                current['result']=json.loads((folder/'attempt.json').read_text());current['status']='failed'
            self._trial_row(current);self._archive(current,folder)

    def _apply_review(self,saved,current,verdict,pol,*,source):
        plan=current['plan'];result=current['result'];before=copy.deepcopy(pol['parameters']);folder=self._trial_folder(current)
        if verdict['outcome']=='placed' and (not result.get('placement_attempted') or not result.get('recording_covers_motion')):
            verdict={**verdict,'outcome':'unknown','ball_state':'unknown','note':'录像或实际投放记录不完整，不能确认进碗。'}
        diagnosis=diagnose(result,plan)
        evidence=basis(result,diagnosis,outcome=verdict['outcome'],ball_state=verdict.get('ball_state'))
        newer_success=any(r.get('session_id')==current['task']['session_id']
            and r.get('target_color')==current['task']['target_color'] and r.get('outcome')=='placed'
            and r.get('attempt',0)>current['task']['attempt'] for r in pol['reviews'])
        if before!=plan['parameters'] or newer_success:
            after=before;changes=['这份后台判断对应旧参数或已被后续成功覆盖，仅保留证据，不覆盖当前策略。']
            proposal={'status':'unchanged','parameters_changed':[],'decision':'stale_review','reason':changes[0]}
        elif evidence['allow_visual']:
            after,changes,proposal=propose(before,plan['scene'],verdict['outcome'],verdict['direction'],
                verdict.get('magnitude','unknown'),verdict.get('adjustment','unknown'),history=pol['reviews'])
        else:after,changes,proposal=execution_proposal(before,diagnosis)
        if after!=before:
            try:
                with self.service_provider().lock:build_plan(saved,self.service_provider().status(),plan['scene'],after)
            except ValueError as exc:
                after=before;changes=['候选参数未通过完整路径检查，保留原参数：'+str(exc)]
                proposal={**proposal,'status':'unchanged','decision':'candidate_unreachable','parameters_changed':[],
                          'before':{},'after':{},'reason':changes[0]}
        entry={'plan_id':current['id'],'session_id':current['task']['session_id'],'attempt':current['task']['attempt'],
            'before':before,'after':after,'proposal':proposal,'changes':changes,**verdict,
            'source':source,'ball_xyz_mm':plan['scene']['ball']['center_robot_mm'],
            'funnel_xyz_mm':plan['scene']['funnel']['mouth_center_robot_mm'],'grasp_pitch_deg':plan['grasp_pitch_deg'],
            'target_color':current['task']['target_color'],'at_unix':time.time()}
        pol['parameters']=copy.deepcopy(after);pol['reviews'].append(entry);pol['version']+=1
        _write(self.policy_path(saved),pol);current.update(review=entry,review_status='completed')
        self.events.emit('review',attempt=current['task']['attempt'],target_color=current['task']['target_color'],
            verdict=verdict,changes=changes,source=source)
        self._trial_row(current);self._archive(current,folder)
        self.publish('reobserving',last_review=verdict,review_source=source,changes=changes)
        return verdict

    def _model_review(self,current,stop):
        key=current['task']['session_id'];plan=current['plan'];ball=plan['scene']['ball']
        context={'plan_id':current['id'],'task_profile':'challenge_upgrade','result':current['result'],
            'video_path':str(bench_ui.experiment_root()/current['result']['capture_trial_id']/'rgb.mp4'),
            'visual_target':{'color':ball['color'],'label':ball['label'],'pixel_center':ball['pixel_center'],
                             'bowl_boundary_px':plan['scene']['funnel']['mouth_boundary_px']},
            'comparison':{'available':False,'reference_plan_id':None,'reason':'逐球任务后台复核本轮选中的球'}}
        def update(value):
            if not stop.is_set() and self.value.get('session_id')==key:
                self.events.model(value,attempt=current['task']['attempt'],plan_id=current['id'])
        return grok_review.review(context,self._trial_folder(current),stop,on_update=update)

    def _worker(self,key,saved):
        service=self.service_provider();confirmed=set();seen=set();tries={};cached=None
        pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='challenge-review');review_stop=threading.Event()
        pending=None;deferred={};pol=None
        def drain():
            nonlocal pending
            if pending and pending[0].done():
                future,current=pending;pending=None
                try:
                    verdict=self._apply_review(saved,current,future.result(),pol,source='model')
                    if verdict['outcome']=='placed':confirmed.add(current['task']['target_color'])
                except Exception as exc:
                    current.update(review_status='failed',review_error=str(exc)[:500]);self._archive(current,self._trial_folder(current));self._trial_row(current)
                    self.events.emit('review_error',message='后台视觉分析失败；本轮数据已保存，继续按新RGB-D定位。')
            if pending is None and deferred:
                color=next(iter(deferred));current=deferred.pop(color)
                pending=(pool.submit(self._model_review,current,review_stop),current)
        try:
            pol=self.policy(saved)
            _write(self.policy_path(saved),pol)
            # Begin from the actual idle pose. T100 is reserved for a real fault.
            if service.fault:
                self.publish('recovering')
                recover(service,None,{}, {'ball_state':'unknown'},self.root(key)/'startup',self.stop,
                    lambda phase,**v:self.publish(phase),automation_id=key)
            while True:
                self.guard(saved);drain();observation=uuid.uuid4().hex;self.publish('observing')
                if cached is not None and 0<=(time.monotonic_ns()-cached[0]['host_monotonic_ns'])/1e9<2:
                    scene,g,image=cached;observation=self.value['observation_id'];cached=None
                else:scene,g,image=self._capture(saved,self.root(key)/'observations'/observation)
                self.guard(saved);colors={b['color'] for b in scene['balls']};seen|=colors
                confirmed|=set(scene['bowl_ball_colors'])&seen;confirmed-=colors
                if not scene['view_clear']:
                    self.publish('waiting_view',message='当前画面不清晰，等待下一张新画面。');self.stop.wait(.2);continue
                # Normal nonempty scans use local RGB-D. Only an empty or
                # unresolved scan needs a foreground model census.
                if not scene['balls']:
                    self.publish('verifying_table',message='桌面候选已处理，正在做最后一次画面确认。')
                    observation=uuid.uuid4().hex
                    scene,g,image=self._capture(saved,self.root(key)/'observations'/observation,fast=False)
                    colors={b['color'] for b in scene['balls']};seen|=colors
                    confirmed|=set(scene['bowl_ball_colors'])&seen;confirmed-=colors
                    if not scene['balls'] and not scene['unresolved'] and scene['view_clear']:
                        drain();confirmed-=colors
                        self.publish('completed' if seen<=confirmed else 'cleared',status='completed',remaining=0,
                            confirmed_colors=sorted(confirmed),detected_total=len(seen),placement_confirmed=seen<=confirmed,
                            finished_at_unix=time.time(),message='桌面已清空；入碗记录已保存。',scene=scene,
                            observation_id=observation,observation_session_id=key)
                        return
                    if not scene['balls'] or not scene['view_clear']:
                        self.publish('waiting_view',message='画面有遮挡或未定位的球，继续观察。');self.stop.wait(.2);continue
                    # After model latency, return to a fresh measurement.
                    observation=uuid.uuid4().hex
                    scene,g,image=self._capture(saved,self.root(key)/'observations'/observation,
                        verified_colors={b['color'] for b in scene['balls']})
                    if not scene['balls']:continue
                    colors={b['color'] for b in scene['balls']};seen|=colors;confirmed-=colors
                self.publish('planning',scene=scene,observation_id=observation,observation_session_id=key,
                    initial_colors=sorted(seen),detected_total=len(seen),confirmed_colors=sorted(confirmed),
                    remaining=len(scene['balls'])+len(scene['unresolved']))
                with service.lock:raw=service.status();_ready(service.health())
                ordered=sorted(scene['balls'],key=lambda b:(tries.get(b['color'],0),math.dist(b['center_robot_mm'],[raw[k] for k in 'xyz'])))
                failures=[];choice=None
                for ball in ordered:
                    try:choice=(ball,build_plan(saved,raw,challenge_scene.single_scene(scene,ball),copy.deepcopy(pol['parameters'])));break
                    except ValueError as exc:failures.append({'label':ball['label'],'xyz_mm':ball['center_robot_mm'],'error':str(exc)})
                if not choice:
                    self.publish('blocked',failures=failures);raise RuntimeError('剩余球未找到完整抓放路径；未把它们计为已清空。')
                ball,plan=choice;tries[ball['color']]=tries.get(ball['color'],0)+1
                current=self._attempt(key,saved,scene,g,image,ball,plan,pol)
                observation=uuid.uuid4().hex
                cached=self._capture(saved,self.root(key)/'observations'/observation,source=current['result']['capture_trial_id'])
                post=cached[0];self.publish('reobserving',scene=post,observation_id=observation,observation_session_id=key,
                    remaining=len(post['balls'])+len(post['unresolved']))
                still_on_table=any(b['color']==ball['color'] for b in post['balls'])
                if not still_on_table and ball['color'] in post['bowl_ball_colors']:
                    verdict={'outcome':'placed','ball_state':'funnel','direction':'none','magnitude':'unknown',
                        'adjustment':'keep','note':'实际投放后RGB-D画面中，目标颜色位于容器开口内部，桌面未再见该球。'}
                    verdict=self._apply_review(saved,current,verdict,pol,source='rgbd')
                    if verdict['outcome']=='placed':confirmed.add(ball['color'])
                elif current['review_status']=='pending':
                    # At most one request runs, plus one latest deferred sample
                    # per colour. Do not build an unbounded model-call backlog.
                    old=deferred.get(ball['color'])
                    if old:
                        old.update(review_status='deferred',superseded_by=current['id']);self._archive(old,self._trial_folder(old));self._trial_row(old)
                    deferred[ball['color']]=current;drain()
        except Exception as exc:
            self.publish('stopped' if self.stop.is_set() else 'failed',status='stopped' if self.stop.is_set() else 'failed',error=str(exc)[:1500])
        finally:
            if pending and not pending[0].done():
                current=pending[1]
                self.events.model({'type':'stopped','error':'挑战已结束，未完成的后台分析保留为待复核。'},
                    attempt=current['task']['attempt'],plan_id=current['id'])
            review_stop.set();pool.shutdown(wait=False,cancel_futures=True)
            unfinished=list(deferred.values())+([pending[1]] if pending else [])
            try:
                for current in unfinished:
                    current['review_status']='deferred';self._archive(current,self._trial_folder(current));self._trial_row(current)
            finally:
                service.disarm()
                with service.lock:
                    if service.automation_owner==key:service.automation_owner=None
                with self.lock:_write(self.root(key)/'state.json',self.value)
