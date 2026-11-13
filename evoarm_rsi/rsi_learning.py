"""Bounded RSI loop with a separate official reset for every new failure."""
from __future__ import annotations

import copy
from pathlib import Path
import threading
import time
import uuid

from . import grok_review
from .rsi_journal import append_event,archive_session
from .task_agent import _write
from .rsi_events import LearningEvents

LEARNING_DECLARATION='现场已清空，允许自动抓放、失败后恢复官方姿态并连续重试'


def _launch_learning(target):
    threading.Thread(target=target,name='evoarm-rsi-learning',daemon=True).start()


def error_info(exc):
    detail=getattr(exc,'detail',None)
    if isinstance(detail,dict):
        return {'message':str(detail.get('message','规划失败')),'image_url':detail.get('image_url')}
    return {'message':detail if isinstance(detail,str) else str(exc),'image_url':None}


class ResetHandledError(RuntimeError):
    def __init__(self,cause,report):
        super().__init__(error_info(cause)['message'])
        self.report=report


class LearningLoop:
    def __init__(self,service_provider,*,prepare,run,get_trial,context,apply_review,
                 recover,archive,trial_stop,reset_failure):
        self.service_provider=service_provider;self.prepare=prepare;self.run_trial=run
        self.get_trial=get_trial;self.context=context;self.apply_review=apply_review
        self.recover=recover;self.archive=archive;self.trial_stop=trial_stop;self.reset_failure=reset_failure
        self.lock=threading.RLock();self.stop_event=threading.Event();self.worker_ident=None
        self.value={'status':'idle','phase':'idle'}
        self.events=LearningEvents();self._event_phase=None;self._motion_phase=None

    def snapshot(self):
        with self.lock:return copy.deepcopy(self.value)

    def is_worker(self):return self.worker_ident==threading.get_ident()

    def assert_interactive_allowed(self):
        if self.snapshot()['status']=='running' and not self.is_worker():raise ValueError('自动学习进行中，请先停止任务')

    def publish(self,phase,**values):
        with self.lock:
            self.value.update(phase=phase,**values)
            signature=(phase,self.value.get('attempt'),self.value.get('plan_id'))
            if signature!=self._event_phase:
                self.events.emit('phase',phase=phase,attempt=self.value.get('attempt',0),plan_id=self.value.get('plan_id'),
                                 message=values.get('message'),error=values.get('error'))
                self._event_phase=signature
            if 'review' in values:
                self.events.emit('policy',attempt=self.value.get('attempt',0),plan_id=self.value.get('plan_id'),
                                 review=values['review'],changes=values.get('changes',[]))
            if self.value.get('session_id'):
                _write(Path.home()/'.evoarm/rsi/learning_sessions'/self.value['session_id']/'state.json',self.value)

    def start(self,declaration,max_attempts=5):
        if declaration!=LEARNING_DECLARATION:raise ValueError('请确认自动抓放、恢复和连续重试的范围')
        if type(max_attempts) is not int or not 1<=max_attempts<=20:raise ValueError('自动试次需为1～20次')
        config=grok_review.configuration()
        if not config['configured']:raise ValueError(config['reason'])
        service=self.service_provider()
        with service.lock,self.lock:
            if self.value['status']=='running' or service.automation_owner:raise ValueError('自动学习已经启动')
            if not service.arm or not service.motion_enabled or service.teach_active or service.pending:
                raise ValueError('机械臂需连接、锁定、运动开启且空闲')
            raw=service.status()
            if any(raw.get('torswitch'+k)!=1 for k in 'BSETRG'):raise ValueError('六关节锁定反馈尚未确认')
            key=uuid.uuid4().hex;self.stop_event.clear();self.trial_stop.clear()
            self.value={'session_id':key,'status':'running','phase':'starting','attempt':0,'max_attempts':max_attempts,
                        'trials':[],'recovery_events':[],'started_at_unix':time.time(),'error':None}
            self.events.reset(key);self._event_phase=None;self._motion_phase=None
            service.automation_owner=key
            try:self.publish('starting');_launch_learning(lambda:self._worker(key,max_attempts))
            except Exception:
                service.automation_owner=None;self.value.update(status='failed',phase='failed');raise
        return {'started':True,'session_id':key,'max_attempts':max_attempts}

    def stop(self):
        with self.lock:
            if self.value['status']!='running':return {'stop_requested':False}
            self.stop_event.set();self.trial_stop.set();self.value['phase']='stopping'
            self.events.emit('phase',phase='stopping',attempt=self.value.get('attempt',0))
        threading.Thread(target=self.service_provider().disarm,daemon=True).start()
        return {'stop_requested':True,'notice':'阻止后续动作、恢复和重试；已接受的动作可能继续'}

    def reset_to_official(self,declaration):
        """UI reset works even before the model API has been configured."""
        if declaration!=LEARNING_DECLARATION:raise ValueError('请确认现场已清空并允许官方复位')
        service=self.service_provider()
        with service.lock,self.lock:
            if self.value['status']=='running' or service.automation_owner:raise ValueError('请先停止当前自动任务')
            if not service.arm or not service.motion_enabled:raise ValueError('机械臂需连接且运动已开启')
            key=uuid.uuid4().hex;self.stop_event.clear();self.trial_stop.clear()
            self.value={'session_id':key,'status':'running','phase':'recovering','attempt':0,'max_attempts':0,
                        'trials':[],'recovery_events':[],'started_at_unix':time.time(),'error':None}
            self.events.reset(key);self._event_phase=None;self._motion_phase=None
            service.automation_owner=key
            def worker():
                self.worker_ident=threading.get_ident()
                try:
                    report=self._reset_error(key,None,RuntimeError('用户请求恢复官方初始姿态'),'user_reset')
                    if report['status']!='completed':raise RuntimeError(report.get('error','官方复位未确认'))
                    self.publish('reset_completed',status='completed',finished_at_unix=time.time())
                except Exception as exc:
                    self.publish('stopped' if self.stop_event.is_set() else 'failed',
                                 status='stopped' if self.stop_event.is_set() else 'failed',error=str(exc))
                finally:self._finish(key)
            try:self.publish('recovering');_launch_learning(worker)
            except Exception:service.automation_owner=None;self.value.update(status='failed',phase='failed');raise
        return {'started':True,'session_id':key}

    def _guard(self):
        if self.stop_event.is_set():raise RuntimeError('用户停止了自动学习')

    def record_motion_phase(self,phase):
        with self.lock:
            if self.value.get('status')!='running' or self.stop_event.is_set():return
            signature=(self.value.get('plan_id'),phase)
            if signature!=self._motion_phase:
                self.events.emit('motion',phase=phase,attempt=self.value.get('attempt',0),plan_id=self.value.get('plan_id'))
                self._motion_phase=signature

    def _review(self,key,*,preflight=False):
        def update(value):
            if self.stop_event.is_set() and value['type'] not in ('failed','stopped'):return
            self.events.model(value,attempt=self.value.get('attempt',0),plan_id=key)
        return grok_review.review(self.context(key),Path.home()/'.evoarm/rsi/pick_place_runs'/key,
                                  self.stop_event,preflight=preflight,on_update=update)

    def _reset_error(self,session_id,current,exc,phase):
        """A NEW event, never suppressed by a previous trial's recovery receipt."""
        self._guard();key=uuid.uuid4().hex;info=error_info(exc)
        self.publish('recovering',recovery_cause=info['message'])
        try:report=self.reset_failure(key,current,exc,phase,self.stop_event,session_id)
        except Exception as reset_exc:report=getattr(reset_exc,'report',{'status':'failed','error':str(reset_exc)})
        event={'id':key,'phase':phase,'cause':info['message'],'status':report['status'],
               't100_attempted':any(row.get('command',{}).get('T')==100 for row in report.get('commands',[])),
               't100_sent':any(row.get('command',{}).get('T')==100 and row.get('status') in ('sent_not_verified','feedback_confirmed') for row in report.get('commands',[])),
               'already_at_official_pose':report.get('already_at_official_pose',False)}
        with self.lock:self.value['recovery_events'].append(event)
        self.publish('recovering',last_recovery=event)
        return report

    def _prepare_from_home_if_needed(self,session_id,attempt):
        for retry in range(2):
            self._guard();self.publish('observing',attempt=attempt,plan_id=None)
            try:return self.prepare()
            except Exception as exc:
                report=self._reset_error(session_id,None,exc,'planning')
                if report['status']!='completed' or retry==1:raise ResetHandledError(exc,report) from None
                self.publish('replanning',message='已恢复官方初始姿态，重新定位并规划')

    def _recover_trial(self,current,verdict,session_id):
        self._guard();self.publish('recovering')
        try:report=self.recover(current,verdict,self.stop_event,session_id)
        except Exception as exc:raise ResetHandledError(exc,getattr(exc,'report',{'status':'failed','error':str(exc)})) from None
        current['recovery']=report
        self.publish('recovering',last_recovery={'status':report['status'],'id':current['id'],
                     'already_at_official_pose':report.get('already_at_official_pose',False)})
        self.events.emit('recovery',attempt=self.value.get('attempt',0),status=report['status'],
                         already_at_official_pose=report.get('already_at_official_pose',False))
        return report

    def _worker(self,session_id,max_attempts):
        self.worker_ident=threading.get_ident();service=self.service_provider();current=None;phase='starting'
        try:
            if service.fault:
                report=self._reset_error(session_id,None,RuntimeError(service.fault['message']),'startup')
                if report['status']!='completed':raise ResetHandledError(RuntimeError('起步复位未确认'),report)
            for attempt in range(1,max_attempts+1):
                # Forget the old trial BEFORE preparing a new one. Its receipt
                # cannot stand in for recovery of a later planning failure.
                current=None;phase='planning';key=self._prepare_from_home_if_needed(session_id,attempt)
                current=self.get_trial(key);current['learning_session_id']=session_id;phase='scene_check'
                self.publish('scene_check',plan_id=key)
                self.events.emit('plan',attempt=attempt,plan_id=key,policy_version=current['policy_version'],
                                 policy_application=current['plan'].get('policy_application'),
                                 depth_used=current['plan']['scene']['ball']['depth_evidence']['depth_used'])
                if attempt==1 or current['plan']['scene']['funnel'].get('ambiguous'):
                    assessment=self._review(key,preflight=True)
                    if not assessment['ready']:raise RuntimeError('Grok无法确认当前球与漏斗：'+assessment['note'])
                self._guard();phase='executing';self.publish('executing');self.run_trial(key)
                until=time.monotonic()+150
                while current['status'] in ('starting','active') or not current.get('worker_finished',True):
                    if time.monotonic()>until:
                        self.trial_stop.set();raise RuntimeError('当前试次未及时结束，停止连续重试')
                    time.sleep(.1)
                self._guard();result=current.get('result',{})
                # Physical/software motion failures return home immediately;
                # do not leave the arm down while waiting for model analysis.
                if result.get('motion_error'):
                    self._recover_trial(current,{'ball_state':'unknown'},session_id)
                if current.get('backup_error'):raise RuntimeError('本轮备份未完成：'+current['backup_error'])
                phase='analyzing';self.publish('analyzing');root=Path.home()/'.evoarm/rsi/pick_place_runs'/key
                append_event(root,'grok_review_started');verdict=self._review(key)
                append_event(root,'grok_review_completed',verdict=verdict)
                if not result.get('grasp_attempted') and verdict['outcome'] not in ('unknown','execution_failed','pushed_away'):
                    verdict.update(outcome='execution_failed',direction='none')
                decision=self.apply_review(key,verdict)
                self.publish('learning',review=decision,changes=decision['changes'],policy_version=decision['after_version'])
                self.value['trials'].append({'plan_id':key,'outcome':verdict['outcome'],'policy_version':decision['after_version']})
                if verdict['outcome']=='placed':
                    self.archive(current);self.publish('completed',status='completed',finished_at_unix=time.time());return
                if current.get('recovery',{}).get('status')!='completed':self._recover_trial(current,verdict,session_id)
                self.archive(current)
                if verdict['outcome']=='unknown':
                    raise ResetHandledError(RuntimeError('Grok无法可靠判断本轮，已保留证据'),current['recovery'])
                if decision.get('proposal',{}).get('allow_retry') is False:
                    raise ResetHandledError(RuntimeError('本轮没有可验证的新修正，停止重复原策略；请查看学习记录中的原因。'),current['recovery'])
            self.publish('exhausted',status='completed',finished_at_unix=time.time(),message='尝试次数已用完，机械臂已恢复官方姿态，策略与证据已保存')
        except Exception as exc:
            report=exc.report if isinstance(exc,ResetHandledError) else None
            if not self.stop_event.is_set() and report is None:
                try:report=self._reset_error(session_id,current,exc,phase)
                except Exception as reset_exc:report={'status':'failed','error':str(reset_exc)}
            if current is not None:
                try:self.archive(current)
                except Exception as backup_exc:self.value['backup_error']=str(backup_exc)
            message=error_info(exc)['message']
            if report and report.get('status')=='completed':message='已恢复官方初始姿态；'+message
            elif report:message+='；官方复位未确认：'+report.get('error','请检查控制器连接')
            self.publish('stopped' if self.stop_event.is_set() else 'failed',status='stopped' if self.stop_event.is_set() else 'failed',
                         error=message[:1200],finished_at_unix=time.time())
        finally:self._finish(session_id)

    def _finish(self,session_id):
        service=self.service_provider()
        service.disarm()
        try:
            backup=archive_session(session_id)
            self.publish(self.value['phase'],session_backup_file=backup)
        except (OSError,ValueError,KeyError):self.value['session_backup_error']='自动任务诊断备份未完成'
        finally:
            with service.lock:
                if service.automation_owner==session_id:service.automation_owner=None
            self.worker_ident=None
