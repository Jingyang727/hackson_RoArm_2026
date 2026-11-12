"""Operator-driven XYZ validation; the existing ArmService owns all motor I/O."""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
from fastapi import APIRouter, HTTPException
from fastapi.responses import Response
from pydantic import BaseModel, ConfigDict, Field

from . import bench_ui
from evoarm_rsi.bench_calibration import load_session
from evoarm_rsi.coordinate_test import calibrated_table, fingerprint, plan_point, errors
from evoarm_rsi.roarm_geometry import forward, height_above_plane
from evoarm_rsi.ball_trial import camera_geometry, check_segment
from evoarm_rsi.task_scene import load_rgbd
from evoarm_rsi.task_agent import _write
from evoarm_rsi.bench_start import _ready

DECLARATION='已核对XYZ目标和路径，现场有人看护且双手离开运动范围'


class Target(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)
    xyz_mm:list[float]=Field(min_length=3,max_length=3)


class PlanID(BaseModel):
    model_config=ConfigDict(extra='forbid')
    plan_id:str=Field(pattern=r'^[a-f0-9]{32}$')


class Move(PlanID):
    declaration:str


class Verify(PlanID):
    result:str=Field(pattern=r'^(matches|offset|uncertain)$')
    note:str=Field(default='',max_length=1000)


class DepthPoint(BaseModel):
    model_config=ConfigDict(extra='forbid')
    snapshot_id:str=Field(pattern=r'^[a-f0-9]{32}$')
    u:int=Field(ge=0,strict=True)
    v:int=Field(ge=0,strict=True)


def observe_pending(service,pending,status,now):
    """Classify one move, preserving per-axis evidence; never actuate here."""
    report=pending['report'];plan=report['plan']
    values=[status.get(k) for k in ('x','y','z','tit','b','s','e','t','r','g')]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        service._latch_fault('coordinate_feedback_invalid','坐标验证反馈缺失或无效');return
    if any(status.get('torswitch'+k)!=1 for k in 'BSETRG'):
        service._latch_fault('coordinate_torque_changed','坐标验证期间扭矩状态变化');return
    measured=errors(status,plan['target_xyz_mm'],plan['pose_preserved_rad'])
    report.update(**measured,elapsed_s=now-pending['since'])
    report['feedback'].append({'host_monotonic_ns':time.monotonic_ns(),'status':dict(status),**measured})
    samples=pending.setdefault('samples',[]);samples.append((now,[status[k] for k in 'xyz']))
    samples[:]=[s for s in samples if now-s[0]<=1.5]
    # This is XYZ validation, not a grasp sequence. Report attitude/gripper
    # residuals without turning them into another all-axis arrival gate.
    stable=(len(samples)>=3 and samples[-1][0]-samples[0][0]>=.8 and
            max(math.dist(a[1],b[1]) for a in samples for b in samples)<=2)
    progress=math.dist(measured['actual_xyz_mm'],[plan['initial_status'][k] for k in 'xyz'])
    report.update(progress_mm=progress,movement_observed=progress>=.8)
    if measured['position_error_mm']<=6:pending.setdefault('near_since',now)
    outcome=None
    if stable and measured['position_error_mm']<=6:
        outcome='feedback_within_tolerance'
    elif stable and now-pending['since']>=8 and progress>=.8:
        outcome='settled_with_offset'
    elif now-pending['since']>=20 and now-pending.get('near_since',-100)<1.5:
        pass  # A blocking T104 may yield its first near-target feedback late.
    elif now-pending['since']>=20:
        outcome='timeout_unverified'
    if outcome:
        report.update(status=outcome,finished_at_unix=time.time())
        service.pending=None;service.armed_until=0
        service.last_command.update(status=outcome,**measured)
        if outcome=='timeout_unverified':
            service._latch_fault('coordinate_timeout','坐标验证超时；目标未确认，请核对实际状态',**measured)
    _write(Path(report['report_file']),report)


def install(app, service_provider):
    router=APIRouter(prefix='/api/coordinates');plans={};snapshots={};lock=threading.RLock();latest=None
    def session():
        _,saved=load_session(bench_ui.session_path())
        if saved['setup_id']!=bench_ui.setup_id():raise ValueError('当前布置与保存标定不一致')
        return saved
    def known(key):
        if key not in plans:raise ValueError('目标计划不存在或控制服务已重启，请重新预览')
        return plans[key]
    def persist(report):_write(Path(report['report_file']),report)
    def fail(exc):raise HTTPException(status_code=409,detail=str(exc)) from exc
    def result_view(report):
        return {k:v for k,v in report.items() if k not in ('feedback','plan','created_monotonic')}

    @router.get('/state')
    def state():
        service=service_provider();raw=None;error=None;plane=None;table_error=None
        try:
            with service.lock:
                if service.arm is not None:raw=service.status()
                health=service.health()
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
            error=str(exc);health=service.health()
        try:plane=calibrated_table(session())
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:table_error=str(exc)
        model=None
        if raw:
            try:model=forward(raw)
            except ValueError as exc:error=str(exc)
        with lock:report=plans.get(latest)
        # Transport faults can clear pending before the coordinate observer.
        if report and report['status']=='moving' and health.get('fault'):
            report.update(status='fault',fault=health['fault']);persist(report)
        return {'health':health,'robot':raw,'model':model,'table_plane':plane,'setup_id':bench_ui.setup_id(),
                'feedback_error':error,'calibration_error':table_error,'frame':'robot_firmware_mm',
                'static_layout_assumed':True,'board_required':False,'camera_required':False,
                'result':result_view(report) if report else None,
                'height_mm':height_above_plane([raw[k] for k in 'xyz'],plane) if raw and plane else None}

    @router.post('/prepare')
    def prepare(req:Target):
        nonlocal latest
        service=service_provider()
        try:
            with service.lock,lock:
                if service.pending:raise ValueError('当前动作尚未结束，请先核对反馈')
                raw=service.status();saved=session();plan=plan_point(saved,raw,req.xyz_mm)
                for key in list(plans):
                    if key!=latest and time.monotonic()-plans[key]['created_monotonic']>180:plans.pop(key)
                if len(plans)>=128:raise ValueError('本次服务的点位记录已满，请保留记录后重启网页服务')
                key=uuid.uuid4().hex
                report={'plan_id':key,'status':'prepared','created_at_unix':time.time(),'created_monotonic':time.monotonic(),
                        'plan':plan,'target_xyz_mm':req.xyz_mm,'feedback':[],
                        'physical_accuracy':'not_checked','report_file':str(Path.home()/'.evoarm/rsi/coordinate_runs'/key/'report.json')}
                old=plans.get(latest)
                if old and old['status']=='prepared':old['status']='superseded';persist(old)
                plans[key]=report;latest=key;persist(report)
                return {'plan_id':key,'plan':plan,'expires_in_s':180}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.post('/move')
    def move(req:Move):
        service=service_provider()
        unlocked=False
        try:
            with service.lock,lock:
                report=known(req.plan_id);plan=report['plan']
                if report['status']!='prepared':raise ValueError('该计划已执行或消耗，不会重发')
                if time.monotonic()-report['created_monotonic']>180:raise ValueError('预览已过期，请重新预览')
                if req.declaration!=DECLARATION:raise ValueError('请核对目标、路径和现场运动范围')
                raw=service.status();_ready(service.health())
                if fingerprint(session())!=plan['calibration_fingerprint']:raise ValueError('标定发生变化，请重新预览')
                if any(abs(raw[k]-plan['initial_status'][k])>math.radians(1) for k in 'bsetrg'):
                    raise ValueError('机械臂姿态已改变，请重新预览目标')
                if service.allowed_joints!=set(range(1,7)):raise ValueError('本次XYZ移动需要现有六关节授权')
                check_segment(raw,plan['stage'],plan['table_plane'])
                if math.dist([raw[k] for k in 'xyz'],plan['target_xyz_mm'])<.8:
                    report.update(status='already_at_target',**errors(raw,plan['target_xyz_mm'],plan['pose_preserved_rad']))
                    persist(report);return {'sent':False,**result_view(report)}
                service.unlock_once('已检查设备并清空运动区域');unlocked=True;raw=service._preflight(1)
                if any(abs(raw[k]-plan['initial_status'][k])>math.radians(1) for k in 'bsetrg'):
                    service.disarm();raise ValueError('执行前姿态变化，请重新预览')
                check_segment(raw,plan['stage'],plan['table_plane'])
                # Consume before the only serial write, including uncertain writes.
                report.update(status='dispatching',command=plan['command'],sent_at_unix=time.time());persist(report)
                service._send_motion(plan['command'])
                service.last_motion_at=time.monotonic()
                report['status']='moving'
                service.pending={'kind':'coordinate_test','since':time.monotonic(),'report':report}
                service.last_command={'status':'sent_not_verified','coordinate_plan_id':req.plan_id,'command':plan['command']}
                persist(report)
                return {'sent':True,**result_view(report)}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
            if unlocked:service.disarm()
            if 'report' in locals() and report['status']=='dispatching':
                report.update(status='transmission_uncertain',error=str(exc));persist(report)
            fail(exc)

    @router.post('/verify')
    def verify(req:Verify):
        try:
            with lock:
                report=known(req.plan_id)
                if report['status'] in ('prepared','dispatching','moving'):raise ValueError('动作尚未结束')
                report.update(physical_accuracy=req.result,operator_note=req.note,checked_at_unix=time.time());persist(report)
                return result_view(report)
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/depth-snapshot')
    def depth_snapshot():
        try:
            saved=session();candidates=sorted(bench_ui.experiment_root().glob('*/manifest.json'),key=lambda p:p.stat().st_mtime,reverse=True)
            frame=None
            for manifest in candidates[:50]:
                try:
                    frame=load_rgbd(manifest.parent)
                    intr=json.loads((manifest.parent/'gemini_color_intrinsics.json').read_text())
                    geometry=camera_geometry(saved,intr)
                    break
                except (OSError,RuntimeError,ValueError,KeyError,TypeError):frame=None
            if frame is None:raise ValueError('没有可读取的已录制RGB-D；坐标移动功能不依赖相机')
            depth=frame['depth_mm'];valid=np.isfinite(depth)&(depth>0)
            low,high=(float(v) for v in np.percentile(depth[valid],[2,98])) if valid.any() else (0.,1.)
            if valid.any() and high-low<1:
                low=float(depth[valid].min());high=max(float(depth[valid].max()),low+1)
            normalized=np.uint8(np.clip((np.nan_to_num(depth.astype(float))-low)/max(high-low,1)*255,0,255))
            colored=cv2.applyColorMap(normalized,cv2.COLORMAP_TURBO);colored[~valid]=0
            images={}
            for kind,array in [('rgb',frame['bgr']),('depth',colored)]:
                ok,encoded=cv2.imencode('.jpg',array)
                if not ok:raise ValueError('深度预览编码失败')
                images[kind]=encoded.tobytes()
            key=uuid.uuid4().hex
            with lock:
                if len(snapshots)>=8:snapshots.pop(next(iter(snapshots)))
                snapshots[key]={'frame':frame,'geometry':geometry,'images':images}
            return {'snapshot_id':key,'trial_id':frame['trial_id'],'frame_index':frame['frame_index'],
                    'source':'recorded_rgbd_not_live','image_size':list(depth.shape[::-1]),'depth_unit':'mm',
                    'valid_fraction':float(valid.mean()),'color_range_mm':[low,high],'pairing_ms':frame['pairing_ms'],
                    'rgb_url':f'/api/coordinates/depth/{key}/rgb','depth_url':f'/api/coordinates/depth/{key}/depth'}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:fail(exc)

    @router.get('/depth/{key}/{kind}')
    def depth_image(key:str,kind:str):
        with lock:entry=snapshots.get(key)
        if not entry or kind not in ('rgb','depth'):raise HTTPException(status_code=404,detail='预览不存在，请重新读取')
        return Response(entry['images'][kind],media_type='image/jpeg',headers={'Cache-Control':'no-store'})

    @router.post('/depth-point')
    def depth_point(req:DepthPoint):
        with lock:entry=snapshots.get(req.snapshot_id)
        if not entry:raise HTTPException(status_code=409,detail='深度快照已失效，请重新读取')
        frame=entry['frame'];depth=frame['depth_mm'];geometry=entry['geometry']
        if req.u>=depth.shape[1] or req.v>=depth.shape[0]:raise HTTPException(status_code=422,detail='像素超出图像范围')
        z=float(depth[req.v,req.u])
        if not math.isfinite(z) or z<=0:return {'valid':False,'reason':'该像素没有有效深度，不推算空间点'}
        camera=(geometry['inverse_K']@np.array([req.u,req.v,1.]))*z
        robot=geometry['R']@camera+geometry['t']
        return {'valid':True,'pixel':[req.u,req.v],'depth_mm':z,'camera_xyz_mm':camera.tolist(),
                'robot_xyz_mm':robot.tolist(),'height_mm':height_above_plane(robot,geometry['plane']),
                'meaning':'measured_visible_surface_not_ball_center','source':'recorded_aligned_depth',
                'trial_id':frame['trial_id'],'motion_sent':False}

    app.include_router(router)
