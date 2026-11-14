"""Recover directly with the accepted T100 interface, independently of XYZ IK."""
from __future__ import annotations

import math
from pathlib import Path
import time

from control_ui.safety import FACTORY_DEGREES, JOINTS, current_degrees
from .rsi_journal import append_event
from .task_agent import _write

# Kept for old clients; fresh feedback, rather than a fault-name allowlist,
# determines whether the current connection can perform a NEW reset.
SOFT_FAULTS={'rsi_position_unverified','rsi_action_unverified','rsi_gripper_unverified',
             'coordinate_timeout','cartesian_stalled','rsi_recovery_unverified','rsi_execution_failed'}


class OfficialResetError(RuntimeError):
    def __init__(self,message,report):
        super().__init__(message);self.report=report


def _stable_feedback(service,stop):
    samples=[]
    for index in range(3):
        if stop.is_set():raise RuntimeError('用户停止任务，不自动回位')
        if index:time.sleep(.25)
        raw=service.arm.status(timeout=3)
        current_degrees(raw)
        if any(raw.get('torswitch'+k)!=1 for k in 'BSETRG'):
            raise RuntimeError('扭矩未确认，无法自动回位')
        if any(type(raw.get(k)) not in (int,float) or not math.isfinite(raw[k]) for k in 'xyz'):
            raise RuntimeError('实际坐标反馈无效，无法自动回位')
        samples.append(raw)
    if (max(math.dist([a[k] for k in 'xyz'],[b[k] for k in 'xyz']) for a in samples for b in samples)>2
            or max(abs(a[k]-b[k]) for a in samples for b in samples for k in 'bsetrg')>math.radians(.6)):
        raise RuntimeError('机械臂仍在运动，不能叠加回位动作')
    return samples


def recover(service,plan,result,verdict,root,stop,publish,*,automation_id,validate_funnel=None):
    """One reset request. No Cartesian lift, camera, calibration or model needed.

    The learning-session declaration authorizes the official reset in a cleared
    bench. Read-only checks establish fresh locked, stationary feedback first.
    A failed/uncertain T100 is recorded and is never automatically replayed.
    """
    root=Path(root);receipt=root/'recovery.json'
    if receipt.exists():raise RuntimeError('本次恢复请求已使用，不重放复位命令')
    report={'status':'checking','request_id':root.name,'mode':'direct_official_t100','commands':[],
            'started_at_unix':time.time(),'requires_cartesian_plan':False}
    _write(receipt,report)
    def persist():_write(receipt,report)
    def guard():
        if stop.is_set():raise RuntimeError('用户停止任务，不发送后续恢复动作')
        if service.automation_owner!=automation_id:raise RuntimeError('自动任务占用已撤销')
    def consume(command,stage):
        guard()
        report['commands'].append({'command':command,'stage':stage,'at_unix':time.time(),
                                   'host_monotonic_ns':time.monotonic_ns(),'status':'write_attempted'})
        persist();append_event(root,'recovery_dispatch',stage=stage,command=command)
    def space():
        while time.monotonic()-service.last_motion_at<.85:
            guard();time.sleep(.05)
    try:
        publish('recovering')
        with service.lock:
            guard()
            if not service.arm or service.teach_active or not service.motion_enabled:
                raise RuntimeError('当前控制器不允许自动回位')
            samples=_stable_feedback(service,stop)
            report.update(stationary_feedback=samples,previous_fault=service.fault)
            persist()
            # This is a separately recorded reset from a freshly observed pose,
            # never a replay of the command that caused an old software fault.
            service.fault=None;service.pending=None
            service.unlock_once('已检查设备并清空运动区域',automation_id=automation_id)
            raw=samples[-1]
            possibly_held=(verdict.get('ball_state')=='gripper'
                           or result.get('grasp_attempted') and verdict.get('ball_state') not in ('table','funnel'))
            degrees=current_degrees(raw)
            if not possibly_held and all(abs(degrees[j]-goal)<=2 for j,goal in FACTORY_DEGREES.items()):
                report.update(status='completed',already_at_official_pose=True,finished_at_unix=time.time(),final_status=raw)
                persist();append_event(root,'recovery_completed',already_at_official_pose=True);return report
        if possibly_held:
            # Clear a possibly held ball at the current cleared bench position.
            # Reset must not require finding a funnel or solving another IK path.
            space()
            with service.lock:
                guard();service._preflight(6,automation_id=automation_id)
                command={'T':106,'cmd':math.radians(110),'spd':1000,'acc':20}
                consume(command,'release_before_reset');service._send_motion(command)
                report['commands'][-1]['status']='sent_not_verified';persist()
                service.last_motion_at=time.monotonic()
            until=time.monotonic()+4
            while time.monotonic()<until:
                guard()
                with service.lock:
                    raw=service.status()
                    if abs(raw['g']-command['cmd'])<=math.radians(3):break
                time.sleep(.1)
            else:raise RuntimeError('回位前夹爪未张开，未叠加整臂复位')
            report['possible_ball_released_on_bench']=True
        space()
        with service.lock:
            guard();service.pending=None
            service.factory_reset(before_send=lambda raw:consume({'T':100},'official_reset'),automation_id=automation_id)
            report['commands'][-1]['status']='sent_not_verified';persist()
        until=time.monotonic()+50
        while time.monotonic()<until:
            guard()
            with service.lock:
                raw=service.status()
                if service.fault:raise RuntimeError(service.fault['message'])
                if service.pending is None:
                    report['commands'][-1]['status']='feedback_confirmed'
                    report.update(status='completed',finished_at_unix=time.time(),final_status=raw)
                    persist();append_event(root,'recovery_completed');return report
            time.sleep(.15)
        raise RuntimeError('官方复位未在反馈窗口内确认，不重发')
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        persist();append_event(root,'recovery_failed',error=report['error'])
        with service.lock:
            if service.pending and not service.fault:
                service._latch_fault('rsi_recovery_unverified','官方复位未确认；已停止后续动作')
        raise OfficialResetError(str(exc),report) from None
    finally:service.disarm()
