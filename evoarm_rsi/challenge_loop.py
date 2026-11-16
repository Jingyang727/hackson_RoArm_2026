"""Three-colour cleanup orchestration, reusing the main motion/recording path."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import threading
import time
import uuid

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

DECLARATION='现场已清空，允许逐个抓球入碗、自动回位和连续尝试'


def launch(target):threading.Thread(target=target,name='evoarm-challenge',daemon=True).start()


class Challenge:
    def __init__(self,service_provider):
        self.service_provider=service_provider;self.lock=threading.RLock();self.stop=threading.Event()
        self.value={'status':'idle','phase':'idle','confirmed_colors':[],'remaining':None,'attempt':0,'empty_checks':0,'trials':[]}
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
        if path.exists():return json.loads(path.read_text())
        source=Path.home()/'.evoarm/rsi/pick_place_policies'/saved['setup_id']/f'{fingerprint(saved)}.json'
        base=json.loads(source.read_text()) if source.exists() else {'parameters':copy.deepcopy(DEFAULT_PARAMETERS),'reviews':[],'version':1}
        base=copy.deepcopy(base);base['parameters']={**DEFAULT_PARAMETERS,**base['parameters']}
        base,_=repair_clipped_policy(base);base,_=upgrade_release_policy(base)
        return {'version':1,'parameters':copy.deepcopy(base['parameters']),'reviews':[],
                'source_single_ball_policy_id':policy_id(base['parameters'])}

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

    def _capture(self,saved,folder,*,model_review=None):
        self.guard(saved)
        health=client_request(Path.home(),'GET','/health')
        if health.get('recording') or health.get('camera_service')!='ready' or health.get('robot_control') is not False:
            raise RuntimeError('独立相机服务需空闲并处于ready状态')
        capture=client_request(Path.home(),'POST','/record',{'seconds':2})
        key=capture['trial_id']
        if len(key)!=32 or any(c not in '0123456789abcdef' for c in key):raise ValueError('相机返回无效录像ID')
        folder.mkdir(parents=True,exist_ok=True)
        value=challenge_scene.observe(bench_ui.experiment_root()/key,saved,folder,self.stop,
            lambda u:self.events.model(u,attempt=self.value['attempt'],plan_id=folder.name),model_review=model_review)
        _write(folder/'calibration-snapshot.json',saved)
        return value

    def observe(self):
        service=self.service_provider()
        with service.lock,self.lock:
            if self.value['status']=='running' or service.automation_owner or service.pending:raise ValueError('先等当前任务结束再单独识别')
            if self.value['status']=='observing':raise ValueError('正在识别')
            key=uuid.uuid4().hex;self.stop.clear();self.events.reset(key)
            self.value={'status':'observing','phase':'observing','attempt':0,'trials':[],'error':None,
                        'confirmed_colors':[],'remaining':None,'empty_checks':0}
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
        if declaration!=DECLARATION:raise ValueError('请确认现场已清空并允许逐球抓放和自动回位')
        config=grok_review.configuration()
        if not config['configured']:raise ValueError(config['reason'])
        saved=self.session();service=self.service_provider()
        with service.lock,self.lock:
            if self.value['status'] in ('running','observing') or service.automation_owner or service.pending:raise ValueError('已有任务占用控制器或相机')
            if not service.arm or not service.motion_enabled or service.teach_active:raise ValueError('请连接机械臂并开启实物执行，确认扭矩锁定')
            raw=service.status()
            if any(raw.get('torswitch'+k)!=1 for k in 'BSETRG'):raise ValueError('六关节锁定反馈尚未确认')
            key=uuid.uuid4().hex;self.stop.clear();self.events.reset(key)
            self.value={'session_id':key,'status':'running','phase':'starting','attempt':0,'remaining':None,
                'initial_colors':[],'confirmed_colors':[],'empty_checks':0,'trials':[],'error':None,
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

    def _attempt(self,key,saved,scene,g,image,ball,plan,pol):
        service=self.service_provider();trial=uuid.uuid4().hex;folder=self.root(key)/'attempts'/trial
        folder.mkdir(parents=True);count=self.value['attempt']+1
        current={'id':trial,'status':'prepared','plan':plan,'calibration_fingerprint':fingerprint(saved),
            'policy_version':pol['version'],'task':{'kind':'challenge_upgrade','session_id':key,'target_color':ball['color'],
                'target_label':ball['label'],'attempt':count}}
        _write(folder/'calibration-snapshot.json',saved);_write(folder/'state.json',current)
        cv2.imwrite(str(folder/'observation.jpg'),image)
        self.publish('executing',attempt=count,target=ball,plan_id=trial,transfer_height_mm=plan['route_policy']['transfer_height_mm'])
        self.events.emit('target',attempt=count,ball=ball,transfer_height_mm=plan['route_policy']['transfer_height_mm'])
        try:
            with service.lock:
                self.guard(saved);actual=service.status();_ready(service.health())
                if any(abs(actual[k]-plan['initial_status'][k])>math.radians(1) for k in 'bsetrg'):raise ValueError('规划时起点发生变化，尚未发送抓放')
                service.unlock_once('已检查设备并清空运动区域',automation_id=key)
                service.pending=reservation(trial)
            result=execute(service,plan,folder,self.stop,lambda phase,**v:self.publish(phase),
                           camera=client_request,automation_id=key,validate_plan=lambda:self.guard(saved))
            current.update(result=result,status='completed' if result.get('motion_completed') else 'failed')
            _write(folder/'state.json',current);self.guard(saved)
            self.publish('recovering')
            current['recovery']=recover(service,plan,result,{'ball_state':'unknown'},folder,self.stop,
                lambda phase,**v:self.publish(phase),automation_id=key)
            self.guard(saved);self.publish('reviewing')
            capture=result.get('capture_trial_id')
            if not capture or result.get('camera_error'):raise RuntimeError('本轮缺少完整录像证据，未判定成功')
            context={'plan_id':trial,'task_profile':'challenge_upgrade','result':result,
                'video_path':str(bench_ui.experiment_root()/capture/'rgb.mp4'),
                'visual_target':{'color':ball['color'],'label':ball['label'],'pixel_center':ball['pixel_center'],
                                 'bowl_boundary_px':scene['bowl']['mouth_boundary_px']},
                'comparison':{'available':False,'reference_plan_id':None,'reason':'逐球任务复核本轮选中的球'}}
            verdict=grok_review.review(context,folder,self.stop,
                on_update=lambda u:self.events.model(u,attempt=count,plan_id=trial))
            if verdict['outcome']=='placed' and (not result.get('placement_attempted') or not result.get('recording_covers_motion')):
                verdict={**verdict,'outcome':'unknown','ball_state':'unknown',
                         'reason':'录像或实际投放记录不完整，不能确认本轮进碗。'}
            self.guard(saved);diagnosis=diagnose(result,plan)
            evidence=basis(result,diagnosis,outcome=verdict['outcome'],ball_state=verdict.get('ball_state'))
            before=copy.deepcopy(pol['parameters'])
            if evidence['allow_visual']:
                after,changes,proposal=propose(before,plan['scene'],verdict['outcome'],verdict['direction'],
                    verdict.get('magnitude','unknown'),verdict.get('adjustment','unknown'),history=pol['reviews'])
            else:after,changes,proposal=execution_proposal(before,diagnosis)
            if after!=before:
                try:
                    with service.lock:build_plan(saved,service.status(),plan['scene'],after)
                except ValueError as exc:
                    after=before;changes=['候选参数未通过完整路径检查，保留原参数：'+str(exc)]
                    proposal={**proposal,'status':'unchanged','decision':'candidate_unreachable',
                              'parameters_changed':[],'before':{},'after':{},'reason':changes[0]}
            entry={'plan_id':trial,'before':before,'after':after,'proposal':proposal,'changes':changes,
                   **verdict,'ball_xyz_mm':ball['center_robot_mm'],'funnel_xyz_mm':scene['bowl']['mouth_center_robot_mm'],
                   'grasp_pitch_deg':plan['grasp_pitch_deg'],'target_color':ball['color'],'at_unix':time.time()}
            pol['parameters']=copy.deepcopy(after);pol['reviews'].append(entry);pol['version']+=1
            _write(self.policy_path(saved),pol);current['review']=entry
            self.events.emit('review',attempt=count,target_color=ball['color'],verdict=verdict,changes=changes)
            with self.lock:
                self.value['trials'].append({'plan_id':trial,'target_color':ball['color'],'outcome':verdict['outcome'],
                    'motion_completed':result.get('motion_completed'),'physical_elapsed_s':result.get('physical_elapsed_s')})
                self.value['trials']=self.value['trials'][-100:]
            self.publish('reobserving',last_review=verdict,changes=changes)
            if diagnosis['category'] in ('execution_error','camera_error','not_started'):
                raise RuntimeError('本轮执行未确认：'+str(diagnosis.get('error')))
            if diagnosis['category']!='complete' and after==before and not evidence['allow_visual']:
                raise RuntimeError('执行故障尚无可验证的新方案：'+str(diagnosis.get('error')))
            return verdict
        except Exception as exc:
            current['error']=str(exc)[:1500];current['status']='stopped' if self.stop.is_set() else 'failed'
            raise
        finally:
            if (folder/'attempt.json').exists() and not current.get('result'):
                current['result']=json.loads((folder/'attempt.json').read_text());current['status']='failed'
            self._archive(current,folder)
            with self.lock:
                if not any(r['plan_id']==trial for r in self.value['trials']):
                    self.value['trials'].append({'plan_id':trial,'target_color':ball['color'],'outcome':'unknown',
                        'error':current.get('error'),'motion_completed':current.get('result',{}).get('motion_completed')})

    def _worker(self,key,saved):
        service=self.service_provider();confirmed=set();initial=set();tries={};pending=None
        try:
            self.publish('recovering')
            recover(service,None,{}, {'ball_state':'unknown'},self.root(key)/'startup',self.stop,
                lambda phase,**v:self.publish(phase),automation_id=key)
            pol=self.policy(saved)
            while True:
                self.guard(saved);observation=uuid.uuid4().hex;self.publish('observing')
                scene,g,image=self._capture(saved,self.root(key)/'observations'/observation)
                self.guard(saved);colors={b['color'] for b in scene['balls']};inside=set(scene['bowl_ball_colors'])
                if not initial and scene['view_clear'] and not scene['unresolved'] and len(colors|inside)==3:
                    initial=colors|inside
                confirmed|=(inside-colors)&initial;confirmed-=colors
                if pending and pending['color'] not in colors and pending['outcome']=='placed' and not scene['unresolved']:
                    confirmed.add(pending['color'])
                pending=None
                empty=not scene['balls'] and not scene['unresolved'] and scene['view_clear']
                streak=min(2,self.value['empty_checks']+1) if empty else 0
                self.publish('observed',scene=scene,observation_id=observation,observation_session_id=key,
                    initial_colors=sorted(initial),confirmed_colors=sorted(confirmed),remaining=len(scene['balls'])+len(scene['unresolved']),empty_checks=streak)
                if len(initial)>=3 and len(confirmed)>=len(initial) and streak==2:
                    self.publish('completed',status='completed',finished_at_unix=time.time(),message='三个球已确认进碗，连续两次观察确认桌面清空。');return
                if not scene['view_clear'] or len(initial)!=3 or not colors<=initial:
                    self.publish('waiting_view',message='等待清晰画面识别全部三个不同颜色的球；尚未发送本轮抓放。')
                    self.stop.wait(.25);continue
                if not scene['balls']:
                    self.publish('confirming_empty',message='正在复核清空与入碗证据。' if not scene['unresolved'] else '仍有球候选未定位，不能判定桌面清空。')
                    self.stop.wait(.25);continue
                # Refresh RGB-D after the model call. Plan from these new measurements,
                # not from a snapshot which waited through model generation.
                fresh=uuid.uuid4().hex;self.publish('refreshing')
                scene,g,image=self._capture(saved,self.root(key)/'observations'/fresh,model_review=scene['inventory_review'])
                self.publish('planning',scene=scene,observation_id=fresh,remaining=len(scene['balls'])+len(scene['unresolved']))
                with service.lock:raw=service.status();_ready(service.health())
                ordered=sorted(scene['balls'],key=lambda b:(tries.get(b['color'],0),math.dist(b['center_robot_mm'],[raw[k] for k in 'xyz'])))
                failures=[];choice=None
                for ball in ordered:
                    try:choice=(ball,build_plan(saved,raw,challenge_scene.single_scene(scene,ball),copy.deepcopy(pol['parameters'])));break
                    except ValueError as exc:failures.append({'label':ball['label'],'xyz_mm':ball['center_robot_mm'],'error':str(exc)})
                if not choice:
                    if not ordered:continue
                    self.publish('blocked',failures=failures)
                    raise RuntimeError('剩余球未找到完整抓放路径；未把它们计为已清空。')
                ball,plan=choice;tries[ball['color']]=tries.get(ball['color'],0)+1
                result=self._attempt(key,saved,scene,g,image,ball,plan,pol)
                pending={'color':ball['color'],'outcome':result['outcome']}
        except Exception as exc:
            self.publish('stopped' if self.stop.is_set() else 'failed',status='stopped' if self.stop.is_set() else 'failed',error=str(exc)[:1500])
        finally:
            service.disarm()
            with service.lock:
                if service.automation_owner==key:service.automation_owner=None
            with self.lock:_write(self.root(key)/'state.json',self.value)
