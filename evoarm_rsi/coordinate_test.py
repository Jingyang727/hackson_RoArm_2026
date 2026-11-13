"""Single XYZ validation in the firmware frame. No serial or camera access."""
from __future__ import annotations

import math

from .ball_trial import check_segment, SPEED
from .bench_calibration import fit_points, _digest
from .roarm_geometry import forward, inverse, height_above_plane
from control_ui.safety import JOINTS, validate_angle_range


def gripper_hold(status):
    """Use the measured aperture, bounded by legal COMMAND limits.

    Firmware feedback at a mechanical stop may be slightly over the command
    boundary. The existing manual-control policy already permits a 2-degree
    feedback band for an inward recovery. Reuse that policy, never extend
    the command range or rewrite the actual feedback.
    """
    measured=status['g']
    low=math.radians(JOINTS[6][2]);high=min(math.radians(JOINTS[6][3]),3.14)
    target=min(max(measured,low),high)
    # Preview is pure geometry; torque is checked by the serial owner when
    # executing. Never manufacture locked feedback to make preview work.
    validate_angle_range(status,6,math.degrees(target))
    return {'feedback_rad':measured,'command_rad':target,'feedback_deg':math.degrees(measured),
            'command_deg':math.degrees(target),'normalized':abs(measured-target)>1e-9}


def check_coordinate_segment(status,stage,plane):
    # Recheck the real gripper feedback band at preview AND each preflight.
    # Only the gripper hold command can be normalized; all other joints and
    # the entire Cartesian path retain the existing strict checks.
    validate_angle_range(status,6,math.degrees(stage['gripper_rad']))
    check_segment(status,stage,plane)


def calibrated_table(session):
    fit=fit_points(session['records'])
    if not fit['planar_fit_validated']:raise ValueError('已有五点标定未通过验证')
    return fit['table_plane_robot']


def fingerprint(session):
    return _digest({k:session[k] for k in ('session_id','reference_sha256','records')})


def plan_point(session, status, target):
    if len(target)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1000 for v in target):
        raise ValueError('请输入有限的机械臂 XYZ，单位mm')
    current=[float(status[k]) for k in 'xyz'];model=forward(status)
    if math.dist(current,model['xyz_mm'])>3 or abs(status['tit']-model['pitch_rad'])>math.radians(1):
        raise ValueError('当前固件XYZ与原装夹爪几何不一致，需要先核对实际反馈')
    plane=calibrated_table(session)
    start_height=height_above_plane(current,plane);target_height=height_above_plane(target,plane)
    if target_height < -1e-6:raise ValueError(f'目标低于已标定桌面{-target_height:.1f}mm，不能生成穿过桌面的运动')
    minimum=0.
    if start_height<0:
        if start_height < -10 or math.dist(current[:2],target[:2])>1 or target_height<=start_height:
            raise ValueError('当前末端反馈低于保存平面；仅允许预览同XY向上的竖直抬升')
        minimum=start_height-1
    hold=gripper_hold(status)
    stage={'stage':'coordinate_test','kind':'cartesian','xyz_mm':list(target),'pitch_rad':status['tit'],
           'roll_rad':status['r'],'gripper_rad':hold['command_rad'],'minimum_tip_height_mm':minimum}
    check_coordinate_segment(status,stage,plane)
    predicted=inverse(target,status['tit'],status['r'],hold['command_rad'])
    count=max(2,math.ceil(math.dist(current,target)/8))
    path=[[a+(b-a)*i/count for a,b in zip(current,target)] for i in range(count+1)]
    return {'kind':'single_xyz_validation_v1','setup_id':session['setup_id'],'frame':'robot_firmware_mm',
            'initial_status':status,'target_xyz_mm':list(target),'table_plane':plane,'path_xyz_mm':path,
            'target_height_mm':target_height,'distance_mm':math.dist(current,target),
            'clearance_policy':'no_below_table_target_or_path',
            'pose_preserved_rad':{'pitch':status['tit'],'roll':status['r'],'gripper':hold['command_rad']},
            'gripper_hold':hold,
            'predicted_joints_xyz_mm':forward(predicted)['joints_xyz_mm'],
            'command':{'T':104,'x':target[0],'y':target[1],'z':target[2],'t':status['tit'],'r':status['r'],
                       'g':hold['command_rad'],'spd':SPEED},'stage':stage,'calibration_fingerprint':fingerprint(session),
            'physical_accuracy':'unverified_until_operator_checks','camera_required':False,'board_required':False}


def errors(status, target, pose):
    residual=[float(status[k])-v for k,v in zip('xyz',target)]
    return {'actual_xyz_mm':[status[k] for k in 'xyz'],'actual_minus_target_mm':residual,
            'position_error_mm':math.sqrt(sum(v*v for v in residual)),
            'pose_error_deg':{'pitch':math.degrees(status['tit']-pose['pitch']),
                              'roll':math.degrees(math.atan2(math.sin(status['r']-pose['roll']),math.cos(status['r']-pose['roll']))),
                              'gripper':math.degrees(status['g']-pose['gripper'])}}
