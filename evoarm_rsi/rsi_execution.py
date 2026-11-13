"""One authorized pick/place, through ArmService and the existing camera service."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time
import uuid

from .ball_trial import check_segment
from .coordinate_test import gripper_hold
from .camera_service import client_request
from .grasp_once import export_keyframes
from .task_agent import _write
MIN_VERIFIED_STEP_GAP_S=.12


def reservation(run_id,completed=-1):
    return {'kind':'rsi_trial','stage_kind':'reserved','run_id':run_id,
            'completed_stage':completed,'since':time.monotonic()}


def observe_pending(service,pending,status,now):
    """GET/status is strictly read-only; preserve exclusive occupancy between steps."""
    if any(status.get('torswitch'+k)!=1 for k in 'BSETRG'):
        service._latch_fault('rsi_torque_changed','RSI试验中锁定状态改变，停止后续动作');return
    kind=pending['stage_kind']
    if kind=='reserved':return
    def finish(**fields):
        service.last_command.update(status='feedback_confirmed',**fields)
        service.pending=reservation(pending['run_id'],pending['index'])
    if kind=='cartesian':
        from control_ui.coordinates import observe_pending as observe_xyz
        observe_xyz(service,pending,status,now,disarm_on_finish=False)
        if service.pending is None and not service.fault:
            if pending['report']['status'] not in ('feedback_within_tolerance','settled_with_offset') or pending['report']['position_error_mm']>12:
                service._latch_fault('rsi_position_unverified','实际末端偏离本段目标，停止后续动作');return
            finish(position_error_mm=pending['report']['position_error_mm'])
        return
    g=status.get('g')
    if type(g) not in (int,float) or not math.isfinite(g):
        service._latch_fault('rsi_gripper_feedback','夹爪反馈无效');return
    elapsed=now-pending['since'];error=abs(math.degrees(g-pending['target']))
    if error<=2 and elapsed>=.15:
        finish(gripper_error_deg=error,grip_evidence='angle_only_not_visual_success');return
    samples=pending.setdefault('samples',[])
    if not samples or now-samples[-1][0]>=.1:samples.append([now,g])
    samples[:]=[s for s in samples if now-s[0]<=.7]
    stable=len(samples)>=3 and samples[-1][0]-samples[0][0]>=.3 and math.degrees(max(s[1] for s in samples)-min(s[1] for s in samples))<.8
    if kind=='close' and elapsed>=.6 and stable and g>pending['start']+math.radians(2) and g<pending['target']-math.radians(2):
        pending['contact_candidate']=True
        pending['contact_evidence']={'g':g,'tG':status.get('tG'),'meaning':'stable stop while closing; ball contact or other obstruction remains unproven'}
    if elapsed>3:service._latch_fault('rsi_gripper_unverified','夹爪未及时到位或形成稳定接触候选')


def execute(service,plan,root,stop,publish,*,camera=client_request,validate_plan=None):
    root=Path(root);home=Path.home();run_id=root.name;request_id=uuid.uuid4().hex
    report={'kind':'rsi_pick_place_execution_v1','plan_id':run_id,'setup_id':plan['setup_id'],
            'camera_request_id':request_id,'events':[],'dispatches':[],'feedback':[],
            'parameters':plan['parameters'],'robot_moved':False,'grasp_attempted':False,
            'placement_attempted':False,'motion_completed':False,'visual_success':None,
            'motion_error':None,'camera_error':None,'created_at_unix':time.time()}
    receipt=root/'attempt.json'
    if receipt.exists():raise RuntimeError('本轮已有执行收据，不会重放')
    _write(receipt,report)
    def event(name):report['events'].append({'stage':name,'host_monotonic_ns':time.monotonic_ns()})
    def persist():_write(receipt,report)
    def guard(capture_start):
        if stop.is_set():raise RuntimeError('用户停止了后续动作')
        if time.monotonic_ns()-capture_start>=9_600_000_000:raise RuntimeError('本轮10秒录像窗口即将结束，停止发送后续动作')
    def send(command,index,kind):
        report['dispatches'].append({'index':index,'stage':kind,'command':command,'host_monotonic_ns':time.monotonic_ns()})
        if report['robot_moved'] is False:report['robot_moved']=None
        persist()  # BEFORE the one write; uncertain transmission never retries.
        service._send_motion(command);service.last_motion_at=time.monotonic()
        service.last_command={'status':'sent_not_verified','rsi_run_id':run_id,'rsi_index':index,'command':command}
    with ThreadPoolExecutor(max_workers=1) as pool:
        video=pool.submit(camera,home,'POST','/record',{'seconds':10,'request_id':request_id})
        capture_start=None
        try:
            until=time.monotonic()+15
            while capture_start is None:
                if stop.is_set():raise RuntimeError('用户取消本轮')
                if video.done():video.result();raise RuntimeError('录像在动作开始前结束')
                cam=camera(home,'GET','/health')
                stamp=cam.get('recording_started_monotonic_ns')
                if (cam.get('recording') is True and cam.get('phase')=='recording' and cam.get('recording_request_id')==request_id
                        and type(stamp) is int and 0<=(time.monotonic_ns()-stamp)/1e9<1.5):capture_start=stamp
                if time.monotonic()>until:raise RuntimeError('本轮录像未能开始，没有发送运动')
                if capture_start is None:time.sleep(.04)
            for index,stage in enumerate(plan['steps']):
                guard(capture_start);publish(stage['stage'])
                if validate_plan:validate_plan()
                while time.monotonic()-service.last_motion_at<MIN_VERIFIED_STEP_GAP_S+.01:
                    guard(capture_start);time.sleep(.03)
                with service.lock:
                    pending=service.pending or {}
                    if pending.get('run_id')!=run_id or pending.get('stage_kind')!='reserved':
                        raise RuntimeError('本轮执行占用或前一动作确认丢失')
                    service.pending=None
                    try:actual=service._preflight(6 if stage['kind']=='gripper' else 1,minimum_gap_s=MIN_VERIFIED_STEP_GAP_S)
                    except Exception:
                        if not service.fault:service.pending=reservation(run_id,index-1)
                        raise
                    service.pending=reservation(run_id,index-1)
                    if not report['events']:event('start')
                    event(stage['stage'])
                    if stage['kind']=='cartesian':
                        target={**stage,'gripper_rad':gripper_hold(actual)['command_rad']}
                        check_segment(actual,target,plan['table_plane'])
                        command={'T':104,**dict(zip('xyz',target['xyz_mm'])),'t':target['pitch_rad'],
                                 'r':target['roll_rad'],'g':target['gripper_rad'],'spd':target['spd']}
                        stage_report={'plan':{'initial_status':actual,'target_xyz_mm':target['xyz_mm'],
                            'pose_preserved_rad':{'pitch':target['pitch_rad'],'roll':target['roll_rad'],'gripper':target['gripper_rad']}},
                            'status':'moving','feedback':[],'report_file':str(root/f'stage-{index:02d}.json')}
                        next_pending={'stage_kind':'cartesian','report':stage_report}
                    else:
                        command={'T':106,'cmd':stage['gripper_rad'],'spd':stage['spd'],'acc':stage['acc']}
                        next_pending={'stage_kind':'close' if stage['stage']=='close' else 'open',
                                      'start':actual['g'],'target':stage['gripper_rad']}
                    guard(capture_start);send(command,index,stage['stage'])
                    service.pending={'kind':'rsi_trial','run_id':run_id,'index':index,'since':time.monotonic(),**next_pending}
                    if stage['stage']=='close':report['grasp_attempted']=True
                    if stage['stage']=='release':report['placement_attempted']=True
                held=False
                while True:
                    guard(capture_start)
                    with service.lock:
                        status=service.status();health=service.health()
                        report['feedback'].append({'stage':stage['stage'],'host_monotonic_ns':time.monotonic_ns(),'status':status})
                        publish(stage['stage'],health=health)
                        if health.get('fault'):raise RuntimeError(health['fault']['message'])
                        if not health.get('armed'):raise RuntimeError('本轮运动授权已撤销，停止后续动作')
                        pending=service.pending or {}
                        if pending.get('run_id')!=run_id:raise RuntimeError('本轮控制占用丢失')
                        if pending.get('stage_kind')=='reserved' and pending.get('completed_stage')==index:
                            report['robot_moved']=True;event(stage['stage']+'_completed');persist();break
                        if pending.get('contact_candidate') and not held:
                            held=True;target=gripper_hold(status)['command_rad']
                            report['contact_candidate']=pending['contact_evidence']
                            send({'T':106,'cmd':target,'spd':plan['parameters']['gripper_speed'],
                                  'acc':plan['parameters']['gripper_acc']},index,'hold_contact')
                            service.pending={'kind':'rsi_trial','run_id':run_id,'index':index,'stage_kind':'hold',
                                             'start':target,'target':target,'since':time.monotonic()}
                    if video.done():video.result();raise RuntimeError('录像已结束，动作尚未完成')
                    time.sleep(.08)
            event('finish');report['motion_completed']=True
            report['physical_elapsed_s']=(report['events'][-1]['host_monotonic_ns']-report['events'][0]['host_monotonic_ns'])/1e9
            report['within_10s']=report['physical_elapsed_s']<=10
        except Exception as exc:
            report['motion_error']=f'{type(exc).__name__}: {str(exc)[:600]}';event('fault')
            with service.lock:
                pending=service.pending or {}
                if pending.get('run_id')==run_id and pending.get('stage_kind')!='reserved' and not service.fault:
                    service._latch_fault('rsi_action_unverified','本轮动作未确认，保留录像并停止后续动作')
        finally:
            with service.lock:
                service.disarm()
            publish('saving_recording')
            try:
                capture=video.result(timeout=40);trial_id=capture['trial_id']
                if capture.get('request_id')!=request_id or len(trial_id)!=32 or any(c not in '0123456789abcdef' for c in trial_id):
                    raise ValueError('录像ID与本轮请求不一致')
                trial=home/'.evoarm/rsi/experiments'/trial_id;report['capture_trial_id']=trial_id
                stamps=[json.loads(row)['host_monotonic_ns'] for row in (trial/'frames.jsonl').read_text().splitlines() if row]
                if len(stamps)<3 or any(b<=a for a,b in zip(stamps,stamps[1:])) or (stamps[-1]-stamps[0])/1e9<9:
                    raise ValueError('录像未保留完整10秒观察证据')
                if not (trial/'depth_samples.npz').is_file():raise ValueError('本轮录像缺少深度样本')
                report['recording_covers_motion']=bool(report['dispatches'] and stamps[0]<=report['dispatches'][0]['host_monotonic_ns'] and report['events'][-1]['host_monotonic_ns']<=stamps[-1])
                report['keyframes_file']=export_keyframes(trial,report['events'],stages=('close_completed','lift_completed','transport_completed','release_completed','finish','fault'))
                _write(trial/'rsi-plan.json',plan);_write(trial/'rsi-trial.json',report)
                manifest=json.loads((trial/'manifest.json').read_text())
                if manifest.get('trial_id')!=trial_id or manifest.get('simulated') is not False or manifest.get('status')!='recorded':
                    raise ValueError('录像清单不是本次完整的实物采集证据')
                manifest.update(kind='rsi_pick_place',robot_motion_executed=report['robot_moved'],grasp_attempted=report['grasp_attempted'],
                                placement_attempted=report['placement_attempted'],motion_report_file='rsi-trial.json')
                _write(trial/'manifest.json',manifest)
            except Exception as exc:report['camera_error']=f'{type(exc).__name__}: {str(exc)[:400]}'
            try:persist()
            finally:
                with service.lock:
                    if (service.pending or {}).get('run_id')==run_id and service.pending.get('stage_kind')=='reserved':service.pending=None
    return report
