"""Bounded observe/execute/Grok-review/recover/learn loop, one controller owner."""
from __future__ import annotations

import copy
from pathlib import Path
import threading
import time
import uuid

from . import grok_review
from .rsi_journal import append_event
from .rsi_recovery import SOFT_FAULTS, _stable_feedback
from .task_agent import _write

LEARNING_DECLARATION = '现场已清空，允许自动抓放、失败后恢复官方姿态并连续重试'


def _launch_learning(target):
    threading.Thread(target=target, name='evoarm-rsi-learning', daemon=True).start()


class LearningLoop:
    def __init__(self, service_provider, *, prepare, run, get_trial, context, apply_review,
                 recover, archive, trial_stop, verify_recovery=None):
        self.service_provider = service_provider
        self.prepare = prepare
        self.run_trial = run
        self.get_trial = get_trial
        self.context = context
        self.apply_review = apply_review
        self.recover = recover
        self.archive = archive
        self.trial_stop = trial_stop
        self.verify_recovery=verify_recovery
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker_ident = None
        self.value = {'status': 'idle', 'phase': 'idle'}

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.value)

    def is_worker(self):
        return self.worker_ident == threading.get_ident()

    def assert_interactive_allowed(self):
        if self.snapshot()['status'] == 'running' and not self.is_worker():
            raise ValueError('自动学习进行中，请先停止任务')

    def publish(self, phase, **values):
        with self.lock:
            self.value.update(phase=phase, **values)
            if self.value.get('session_id'):
                _write(Path.home() / '.evoarm/rsi/learning_sessions' / self.value['session_id'] / 'state.json', self.value)

    def start(self, declaration, max_attempts=5):
        if declaration != LEARNING_DECLARATION:
            raise ValueError('请确认自动抓放、恢复和连续重试的范围')
        if type(max_attempts) is not int or not 1 <= max_attempts <= 20:
            raise ValueError('自动试次需为1～20次')
        config = grok_review.configuration()
        if not config['configured']:
            raise ValueError(config['reason'])
        service = self.service_provider()
        with service.lock, self.lock:
            if self.value['status'] == 'running' or service.automation_owner:
                raise ValueError('自动学习已经启动')
            if not service.arm or not service.motion_enabled or service.teach_active or service.pending:
                raise ValueError('机械臂需连接、锁定、运动开启且空闲')
            raw = service.status()
            if any(raw.get('torswitch' + k) != 1 for k in 'BSETRG'):
                raise ValueError('六关节锁定反馈尚未确认')
            if service.fault and service.fault.get('code') not in SOFT_FAULTS:
                raise ValueError(service.fault['message'])
            key = uuid.uuid4().hex
            self.stop_event.clear(); self.trial_stop.clear()
            self.value = {'session_id': key, 'status': 'running', 'phase': 'starting',
                          'attempt': 0, 'max_attempts': max_attempts, 'trials': [],
                          'started_at_unix': time.time(), 'error': None}
            service.automation_owner = key
            try:
                self.publish('starting')
                _launch_learning(lambda: self._worker(key, max_attempts))
            except Exception:
                service.automation_owner = None
                self.value.update(status='failed', phase='failed')
                raise
        return {'started': True, 'session_id': key, 'max_attempts': max_attempts}

    def stop(self):
        with self.lock:
            if self.value['status'] != 'running':
                return {'stop_requested': False}
            self.stop_event.set(); self.trial_stop.set()
            self.value['phase'] = 'stopping'
        # Disarm immediately when the serial owner's lock becomes available.
        threading.Thread(target=self.service_provider().disarm, daemon=True).start()
        return {'stop_requested': True, 'notice': '阻止后续动作、恢复和重试；已接受的动作可能继续'}

    def _guard(self):
        if self.stop_event.is_set():
            raise RuntimeError('用户停止了自动学习')

    def _worker(self, session_id, max_attempts):
        self.worker_ident = threading.get_ident()
        service = self.service_provider()
        current = None
        try:
            with service.lock:
                if service.fault:
                    # An old numerical/settling fault can be cleared by fresh
                    # stationary evidence; no command is repeated.
                    prior = dict(service.fault)
                    _stable_feedback(service, self.stop_event)
                    service.fault = None; service.pending = None
                    self.publish('starting', cleared_start_fault=prior)
            for attempt in range(1, max_attempts + 1):
                self._guard(); self.publish('observing', attempt=attempt)
                key = self.prepare()
                current = self.get_trial(key)
                current['learning_session_id'] = session_id
                self.publish('scene_check', plan_id=key)
                if attempt == 1 or current['plan']['scene']['funnel'].get('ambiguous'):
                    assessment = grok_review.review(self.context(key), Path.home() / '.evoarm/rsi/pick_place_runs' / key,
                                                    self.stop_event, preflight=True)
                    if not assessment['ready']:
                        raise RuntimeError('Grok无法确认当前球与漏斗：' + assessment['note'])
                self._guard(); self.publish('executing')
                self.run_trial(key)
                until = time.monotonic() + 150
                while current['status'] in ('starting', 'active') or not current.get('worker_finished', True):
                    if time.monotonic() > until:
                        self.trial_stop.set()
                        raise RuntimeError('当前试次未及时结束，停止连续重试')
                    # Wait for the trial's own recording/finally block even
                    # after stop; never release ownership while it can write.
                    time.sleep(.1)
                self._guard()
                result = current.get('result', {})
                if current.get('backup_error'):
                    raise RuntimeError('本轮本地备份未完成，停止连续试验：' + current['backup_error'])
                self.publish('analyzing')
                root = Path.home() / '.evoarm/rsi/pick_place_runs' / key
                append_event(root, 'grok_review_started')
                verdict = grok_review.review(self.context(key), root, self.stop_event)
                append_event(root, 'grok_review_completed', verdict=verdict)
                if not result.get('grasp_attempted') and verdict['outcome'] not in ('unknown', 'execution_failed', 'pushed_away'):
                    verdict.update(outcome='execution_failed', direction='none')
                decision = self.apply_review(key, verdict)
                self.publish('learning', review=verdict, changes=decision['changes'], policy_version=decision['after_version'])
                self.value['trials'].append({'plan_id': key, 'outcome': verdict['outcome'],
                                             'policy_version': decision['after_version']})
                if verdict['outcome'] == 'placed':
                    self.archive(current)
                    self.publish('completed', status='completed', finished_at_unix=time.time())
                    return
                self._guard(); self.publish('recovering')
                current['recovery'] = self.recover(current, verdict, self.stop_event, session_id)
                if current['recovery'].get('possible_ball_released_to_funnel'):
                    self._guard();self.publish('checking_recovery')
                    checked=self.verify_recovery(current,self.stop_event) if self.verify_recovery else None
                    current['recovery']['visual_review']=checked
                    self.archive(current)
                    if checked and checked['outcome']=='placed':
                        self.publish('completed',status='completed',review=checked,
                                     message='恢复阶段完成投放；已保留原试次与恢复证据',finished_at_unix=time.time())
                        return
                    raise RuntimeError('已恢复并释放可能夹持的小球；Grok尚不能确认最终位置，未盲目再抓')
                self.archive(current)
                if verdict['outcome'] == 'unknown':
                    raise RuntimeError('Grok无法可靠判断本轮；已保存证据并恢复，未盲目改变参数')
                self._guard()
            self.publish('exhausted', status='completed', finished_at_unix=time.time(),
                         message='本次尝试次数已用完，参数与失败证据已保存')
        except Exception as exc:
            if current is not None:
                result = current.get('result', {})
                recovery_file = Path.home() / '.evoarm/rsi/pick_place_runs' / current['id'] / 'recovery.json'
                if (not self.stop_event.is_set() and current['status'] in ('completed', 'failed')
                        and (result.get('robot_moved') or result.get('dispatches'))
                        and not current.get('recovery') and not recovery_file.exists()):
                    try:
                        current['recovery'] = self.recover(current, {'ball_state': 'unknown'}, self.stop_event, session_id)
                    except Exception as recovery_exc:
                        current['recovery'] = {'status': 'failed', 'error': str(recovery_exc)}
                try: self.archive(current)
                except Exception as backup_exc:
                    self.value['backup_error'] = str(backup_exc)
            self.publish('stopped' if self.stop_event.is_set() else 'failed',
                         status='stopped' if self.stop_event.is_set() else 'failed',
                         error=f'{type(exc).__name__}: {str(exc)[:700]}', finished_at_unix=time.time())
        finally:
            with service.lock:
                service.disarm()
                if service.automation_owner == session_id:
                    service.automation_owner = None
            self.worker_ident = None
