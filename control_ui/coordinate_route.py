"""Feedback-driven waypoint execution, using the existing serial owner only."""
import math
from pathlib import Path
import time

from evoarm_rsi.coordinate_test import errors,check_coordinate_segment
from evoarm_rsi.rsi_route import execution_target
from evoarm_rsi.task_agent import _write


def execute_route(service,report,validate):
    key=report['plan_id'];plan=report['plan'];folder=Path(report['report_file']).parent
    def persist():_write(Path(report['report_file']),report)
    def guard():
        if service.automation_owner!=key or time.monotonic()>=service.armed_until:
            raise RuntimeError('后续途径点授权已撤销')
    try:
        for index,stage in enumerate(plan['segments']):
            while time.monotonic()-service.last_motion_at<.85:guard();time.sleep(.04)
            with service.lock:
                guard();validate()
                raw=service._preflight(1,automation_id=key)
                target=execution_target(stage,raw,plan['table_plane'])
                check_coordinate_segment(raw,target,plan['table_plane'])
                command={'T':104,**dict(zip('xyz',target['xyz_mm'])),'t':target['pitch_rad'],
                         'r':target['roll_rad'],'g':target['gripper_rad'],'spd':target['spd']}
                item={'stage':stage['stage'],'index':index,'command':command,'status':'write_attempted',
                      'dispatch_status':raw,'nominal_xyz_mm':stage['xyz_mm'],'sent_at_unix':time.time()}
                report['commands'].append(item);report.update(route_step=index+1,status='moving');persist()
                service._send_motion(command);item['status']='sent_not_verified';service.last_motion_at=time.monotonic()
                child={'plan':{'initial_status':raw,'target_xyz_mm':target['xyz_mm'],
                    'pose_preserved_rad':{'pitch':target['pitch_rad'],'roll':target['roll_rad'],'gripper':target['gripper_rad']}},
                    'status':'moving','feedback':[],'report_file':str(folder/f'stage-{index:02d}.json')}
                service.pending={'kind':'coordinate_test','route_owner':key,'since':time.monotonic(),'report':child}
                service.last_command={'status':'sent_not_verified','coordinate_plan_id':key,'command':command,'route_step':index+1}
                persist()
            until=time.monotonic()+35
            while True:
                guard()
                with service.lock:
                    raw=service.status()
                    if service.fault:raise RuntimeError(service.fault['message'])
                    if service.pending is None:break
                if time.monotonic()>until:raise RuntimeError('途径点未确认，不发送下一段')
                time.sleep(.15)
            item['status']=child['status'];persist()
        final=errors(raw,plan['target_xyz_mm'],plan['pose_preserved_rad'])
        report.update(**final,status='feedback_within_tolerance' if final['position_error_mm']<=6 else 'settled_with_offset',
                      finished_at_unix=time.time())
    except Exception as exc:
        report.update(status='fault' if service.fault else 'stopped',error=str(exc),finished_at_unix=time.time())
        if report.get('commands') and report['commands'][-1]['status']=='write_attempted':report['status']='transmission_uncertain'
    finally:
        with service.lock:
            service.disarm()
            if service.automation_owner==key:service.automation_owner=None
            persist()
