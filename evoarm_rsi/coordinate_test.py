"""Single XYZ validation in the firmware frame. No serial or camera access."""
from __future__ import annotations

import math

from .ball_trial import check_segment, SPEED
from .bench_calibration import fit_points, _digest
from .roarm_geometry import forward, inverse, height_above_plane, MotionConstraintError
from control_ui.safety import JOINTS, validate_angle_range
from evoarm.limits import GRIPPER_MIN_RAD, GRIPPER_MAX_RAD, joint_limits_info


def gripper_hold(status):
    """Use the measured aperture, bounded by legal COMMAND limits.

    Firmware feedback at a mechanical stop may be slightly over the command
    boundary. The existing manual-control policy already permits a 2-degree
    feedback band for an inward recovery. Reuse that policy, never extend
    the command range or rewrite the actual feedback.
    """
    measured=status['g']
    low=GRIPPER_MIN_RAD;high=GRIPPER_MAX_RAD
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


def plan_point(session, status, target, pitch_mode='auto'):
    if pitch_mode not in ('auto','hold'):raise ValueError('未知的末端俯仰规划方式')
    if len(target)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in target):
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
    candidates=[status['tit']]
    if pitch_mode=='auto':
        # Pure preview search, never physical trial-and-error. Prefer the
        # current pitch; otherwise test the nearest 5-degree grid samples.
        # Global pitch = shoulder + elbow + wrist - 90 degrees; derive its
        # search span from the shared joint ranges, not an extra pitch cap.
        pitch_low=sum(JOINTS[j][2] for j in (2,3,4))-90
        pitch_high=sum(JOINTS[j][3] for j in (2,3,4))-90
        candidates+=sorted((math.radians(d) for d in range(math.ceil(pitch_low/5)*5,math.floor(pitch_high/5)*5+1,5)
                            if abs(math.radians(d)-status['tit'])>1e-8),key=lambda p:abs(p-status['tit']))
    rejected=[]
    for pitch in candidates:
        stage={'stage':'coordinate_test','kind':'cartesian','xyz_mm':list(target),'pitch_rad':pitch,
               'roll_rad':status['r'],'gripper_rad':hold['command_rad'],'minimum_tip_height_mm':minimum}
        try:check_coordinate_segment(status,stage,plane)
        except ValueError as exc:
            rejected.append(getattr(exc,'detail',{'message':str(exc),'pitch_deg':math.degrees(pitch)}))
            if pitch_mode=='hold':raise
            continue
        break
    else:
        raise MotionConstraintError(f'未找到通过检查的直线路径（已检查{len(candidates)}个末端俯仰）。当前俯仰的原因：{rejected[0]["message"]}',
                                    code='no_cartesian_path',pitch_mode=pitch_mode,attempts=rejected,
                                    target_xyz_mm=list(target))
    predicted=inverse(target,pitch,status['r'],hold['command_rad'])
    count=max(2,math.ceil(math.dist(current,target)/8))
    path=[[a+(b-a)*i/count for a,b in zip(current,target)] for i in range(count+1)]
    return {'kind':'single_xyz_validation_v2','setup_id':session['setup_id'],'frame':'robot_firmware_mm',
            'initial_status':status,'target_xyz_mm':list(target),'table_plane':plane,'path_xyz_mm':path,
            'target_height_mm':target_height,'distance_mm':math.dist(current,target),
            'clearance_policy':'no_below_table_target_or_path',
            'pose_preserved_rad':{'pitch':pitch,'roll':status['r'],'gripper':hold['command_rad']},
            'orientation':{'mode':pitch_mode,'initial_pitch_deg':math.degrees(status['tit']),
                           'target_pitch_deg':math.degrees(pitch),'changed':abs(pitch-status['tit'])>1e-8,
                           'candidates_checked':len(rejected)+1,'rejected_candidates':rejected},
            'joint_limits':joint_limits_info(),
            'gripper_hold':hold,
            'predicted_joints_xyz_mm':forward(predicted)['joints_xyz_mm'],
            'command':{'T':104,'x':target[0],'y':target[1],'z':target[2],'t':pitch,'r':status['r'],
                       'g':hold['command_rad'],'spd':SPEED},'stage':stage,'calibration_fingerprint':fingerprint(session),
            'physical_accuracy':'unverified_until_operator_checks','camera_required':False,'board_required':False}


def errors(status, target, pose):
    residual=[float(status[k])-v for k,v in zip('xyz',target)]
    return {'actual_xyz_mm':[status[k] for k in 'xyz'],'actual_minus_target_mm':residual,
            'position_error_mm':math.sqrt(sum(v*v for v in residual)),
            'pose_error_deg':{'pitch':math.degrees(status['tit']-pose['pitch']),
                              'roll':math.degrees(math.atan2(math.sin(status['r']-pose['roll']),math.cos(status['r']-pose['roll']))),
                              'gripper':math.degrees(status['g']-pose['gripper'])}}
