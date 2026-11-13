"""Authorized automatic retreat/reset through the existing serial owner."""
from __future__ import annotations

import math
from pathlib import Path
import time

import numpy as np

from control_ui.safety import FACTORY_DEGREES, JOINTS, current_degrees
from .ball_trial import check_segment
from .coordinate_test import gripper_hold
from .roarm_geometry import forward, height_above_plane
from .rsi_execution import reservation
from .rsi_journal import append_event
from .task_agent import _write

SOFT_FAULTS = {'rsi_position_unverified', 'rsi_action_unverified', 'rsi_gripper_unverified',
               'coordinate_timeout', 'cartesian_stalled'}


def _stable_feedback(service, stop):
    samples = []
    for index in range(3):
        if stop.is_set():
            raise RuntimeError('用户停止任务，不自动回位')
        if index:
            time.sleep(.25)
        raw = service.arm.status(timeout=3)
        current_degrees(raw)
        if any(raw.get('torswitch' + key) != 1 for key in 'BSETRG'):
            raise RuntimeError('扭矩未确认，无法自动回位')
        if any(type(raw.get(k)) not in (int, float) or not math.isfinite(raw[k]) for k in 'xyz'):
            raise RuntimeError('实际坐标反馈无效，无法自动回位')
        samples.append(raw)
    if (max(math.dist([a[k] for k in 'xyz'], [b[k] for k in 'xyz']) for a in samples for b in samples) > 2
            or max(abs(a[k] - b[k]) for a in samples for b in samples for k in 'bsetrg') > math.radians(.6)):
        raise RuntimeError('机械臂仍在运动，不能叠加回位动作')
    return samples


def check_factory_path(actual, plane):
    """Check the straight joint interpolation to the accepted stock T100 pose."""
    target = {JOINTS[j][1]: math.radians(angle) for j, angle in FACTORY_DEGREES.items()}
    for part in range(51):
        weight = part / 50
        pose = {k: actual[k] + (target[k] - actual[k]) * weight for k in target}
        pose['T']=1051
        model = forward(pose)
        if height_above_plane(model['xyz_mm'], plane) < 0:
            raise ValueError('官方回位路径可能穿过桌面，已保留现场状态')
        if any(height_above_plane(point, plane) < 30 for point in model['joints_xyz_mm'][1:-1]):
            raise ValueError('官方回位时上游连杆过低，已保留现场状态')


def recover(service, plan, result, verdict, root, stop, publish, *, automation_id,
            validate_funnel=None):
    """Retreat, dispose of any potentially held ball into the funnel, then T100.

    Automatic authorization comes from the user's learning-session start.
    A stop, transport/torque fault or unknown controller state never resets.
    """
    root = Path(root)
    receipt = root / 'recovery.json'
    if receipt.exists():
        raise RuntimeError('本轮已有恢复收据，不重放恢复动作')
    report = {'status': 'checking', 'commands': [], 'started_at_unix': time.time()}
    _write(receipt, report)
    plane = plan['table_plane']
    normal = np.asarray(plane['normal_up'])
    run_id = root.name
    carry_preload=result.get('contact_preload',{}).get('target_rad') if verdict.get('ball_state') not in ('table','funnel') else None

    def persist(): _write(receipt, report)
    def guard():
        if stop.is_set(): raise RuntimeError('用户停止任务，不发送后续恢复动作')
        if service.automation_owner != automation_id: raise RuntimeError('自动任务占用已撤销')
    def consume(command, stage):
        guard()
        report['commands'].append({'command': command, 'stage': stage, 'at_unix': time.time(),
                                   'host_monotonic_ns': time.monotonic_ns(), 'status': 'write_attempted'})
        persist()
        append_event(root, 'recovery_dispatch', stage=stage, command=command)
    def wait_for_step(index, deadline):
        until = time.monotonic() + deadline
        while time.monotonic() < until:
            guard()
            with service.lock:
                service.status()
                if service.fault: raise RuntimeError(service.fault['message'])
                pending = service.pending or {}
                if pending.get('run_id') == run_id and pending.get('stage_kind') == 'reserved' and pending.get('completed_stage') == index:
                    return
            time.sleep(.08)
        raise RuntimeError('恢复动作未确认，不重发')
    def move(stage, point, pitch):
        while time.monotonic() - service.last_motion_at < .14:
            guard(); time.sleep(.03)
        with service.lock:
            guard(); service.pending = None
            actual = service._preflight(1, minimum_gap_s=.12, automation_id=automation_id)
            target = {'kind':'cartesian','stage':stage,'xyz_mm': list(point), 'pitch_rad': pitch, 'roll_rad': actual['r'],
                      'gripper_rad': carry_preload if carry_preload is not None else gripper_hold(actual)['command_rad'], 'minimum_tip_height_mm': 0}
            check_segment(actual, target, plane)
            command = {'T': 104, **dict(zip('xyz', target['xyz_mm'])), 't': pitch,
                       'r': target['roll_rad'], 'g': target['gripper_rad'], 'spd': .4}
            index = len(report['commands']); consume(command, stage); service._send_motion(command)
            service.last_motion_at = time.monotonic()
            service.last_command = {'status': 'sent_not_verified', 'rsi_run_id': run_id, 'command': command}
            stage_report = {'plan': {'initial_status': actual, 'target_xyz_mm': target['xyz_mm'],
                            'pose_preserved_rad': {'pitch': pitch, 'roll': actual['r'], 'gripper': target['gripper_rad']}},
                            'feedback': [], 'status': 'moving', 'report_file': str(root / f'recovery-step-{index}.json')}
            service.pending = {'kind': 'rsi_trial', 'run_id': run_id, 'index': index, 'stage_kind': 'cartesian',
                               'report': stage_report, 'since': time.monotonic()}
        wait_for_step(index, 10)
    def retreat():
        with service.lock: raw = service.status()
        height = height_above_plane([raw[k] for k in 'xyz'], plane)
        if height < 110:
            point = np.asarray([raw[k] for k in 'xyz']) + normal * (110 - height)
            move('retreat_up', point.tolist(), raw['tit'])
        with service.lock: raw = service.status()
        if height_above_plane([raw[k] for k in 'xyz'], plane) < 90:
            raise RuntimeError('抬升后实际位置仍过低，不执行整臂回位')

    try:
        publish('recovering')
        with service.lock:
            guard()
            if not service.arm or service.teach_active or not service.motion_enabled:
                raise RuntimeError('当前控制器不允许自动回位')
            prior_fault = service.fault
            if prior_fault and prior_fault.get('code') not in SOFT_FAULTS:
                raise RuntimeError('通信、扭矩或传输异常需要检查；不自动回位')
            pending = service.pending or {}
            if pending and pending.get('run_id') != run_id:
                raise RuntimeError('其他动作仍占用控制器')
            samples = _stable_feedback(service, stop)
            report.update(stationary_feedback=samples, previous_fault=prior_fault)
            persist()
            service.fault = None; service.pending = None
            service.unlock_once('已检查设备并清空运动区域', automation_id=automation_id)
            service.pending = reservation(run_id)
        retreat()
        possibly_held = (verdict.get('ball_state') == 'gripper'
                         or result.get('grasp_attempted') and verdict.get('ball_state') not in ('table','funnel'))
        if possibly_held:
            # Empty gripper is harmless here; a held ball must be released over
            # the freshly checked funnel before T100 closes the empty fingers.
            release_point = validate_funnel() if validate_funnel else plan['release_tip_xyz_mm']
            move('recover_to_funnel', release_point, math.radians(plan['grasp_pitch_deg']))
            with service.lock:
                guard(); raw = service.status(); service.pending = None
                target = math.radians(plan['parameters']['open_deg'])
                command = {'T': 106, 'cmd': target, 'spd': plan['parameters']['gripper_speed'], 'acc': plan['parameters']['gripper_acc']}
                index = len(report['commands']); consume(command, 'recover_release'); service._send_motion(command)
                service.last_motion_at = time.monotonic(); service.last_command = {'status': 'sent_not_verified', 'rsi_run_id': run_id}
                service.pending = {'kind': 'rsi_trial', 'run_id': run_id, 'index': index, 'stage_kind': 'open',
                                   'start': raw['g'], 'target': target, 'since': time.monotonic()}
            wait_for_step(index, 4);carry_preload=None;retreat()
            report['possible_ball_released_to_funnel']=True
        while time.monotonic() - service.last_motion_at < .85:
            guard(); time.sleep(.05)
        with service.lock:
            guard(); actual = service.status(); check_factory_path(actual, plane); service.pending = None
            service.factory_reset(before_send=lambda raw: consume({'T': 100}, 'official_reset'), automation_id=automation_id)
        until = time.monotonic() + 50
        while time.monotonic() < until:
            guard()
            with service.lock:
                service.status()
                if service.fault: raise RuntimeError(service.fault['message'])
                if service.pending is None:
                    report.update(status='completed', finished_at_unix=time.time())
                    persist(); append_event(root, 'recovery_completed'); return report
            time.sleep(.15)
        raise RuntimeError('官方回位未在反馈窗口内确认，不重发')
    except Exception as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        persist(); append_event(root, 'recovery_failed', error=report['error'])
        with service.lock:
            if service.pending and not service.fault:
                service._latch_fault('rsi_recovery_unverified', '自动恢复未确认；已停止后续动作')
        raise
    finally:
        service.disarm()
