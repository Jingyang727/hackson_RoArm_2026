"""Explicit camera-frame corrections, effect comparison and policy application."""
from __future__ import annotations

import copy
import hashlib
import json
import math

import numpy as np

from .pick_place import adapt,BALL_RADIUS,DEFAULT_PARAMETERS,XY_STEPS_MM,MAX_XY_STEP_MM
from .rsi_comparison import validate_assessment
from .image_directions import OPPOSITE,PIXEL_VECTOR

LABELS={'image_left':'画面左侧','image_right':'画面右侧','image_up':'画面上方','image_down':'画面下方',
        'lower':'降低','higher':'抬高','none':'方向未知'}


def policy_id(parameters):
    return hashlib.sha256(json.dumps(parameters,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()[:12]


def project(scene,point):
    camera=scene['camera_geometry'];R=np.asarray(camera['R']);t=np.asarray(camera['t']);K=np.asarray(camera['K'])
    value=R.T@(np.asarray(point)-t)
    if value[2]<=0:raise ValueError('目标投影在相机后方')
    uv=K@value
    return (uv[:2]/uv[2]).tolist()


def center_target(scene,parameters,target='ball'):
    normal=np.asarray(scene['table_plane']['normal_up'])
    if target=='ball':
        value=np.asarray(scene['ball']['center_robot_mm'])+normal*parameters['grasp_z_offset_mm']
        value[:2]+=parameters['pickup_xy_mm']
    else:
        value=np.asarray(scene['funnel']['mouth_center_robot_mm'])+normal*(BALL_RADIUS+parameters['release_clearance_mm'])
        value[:2]+=parameters['release_xy_mm']
    return value.tolist()


def target_audit(scene,parameters):
    ball=scene['ball']['pixel_center'];funnel=scene['funnel']['pixel_center']
    side='left' if funnel[0]<ball[0] else 'right'
    return {'frame':'unmirrored_rgb','ball_pixel':ball,'funnel_pixel':funnel,'funnel_side_of_ball':side,
            'grasp_center_target_px':project(scene,center_target(scene,parameters)),
            'release_center_target_px':project(scene,center_target(scene,parameters,'funnel')),
            'note':'空中投放点因透视可能不与漏斗像素中心重合；原始目标圈标明实际识别区域'}


def propose(parameters,scene,outcome,direction,magnitude='medium',adjustment='unknown',*,history=(),comparison=None,assessment=None,requested_step_mm=None):
    before=copy.deepcopy(parameters);step=XY_STEPS_MM[magnitude]
    target='funnel' if outcome=='placement_missed' else 'ball'
    key='release_xy_mm' if target=='funnel' else 'pickup_xy_mm'
    location=scene[target]['mouth_center_robot_mm' if target=='funnel' else 'center_robot_mm']
    context=comparison or {};previous=history[-1] if history else None
    if assessment is not None:assessment=validate_assessment(assessment)
    verified=bool(context.get('available') and previous and assessment and
                  previous['plan_id']==context.get('reference_plan_id')==assessment.get('reference_plan_id') and
                  previous.get('after')==before)
    if assessment and assessment['reference_plan_id'] is not None and not verified:
        raise ValueError('前后对照未对应本轮实际使用的参数与画面，不能据此学习')
    effect=assessment['effect'] if verified else 'not_compared'
    movement=assessment['movement'] if verified else 'unknown'
    comparison_status='first_observation' if not context.get('available') else 'unconfirmed'
    why=[];allow_retry=True;rollback=False;decision='directional_trial'
    previous_step=None
    changed_before=[]
    if verified:
        changed_before=[k for k in before if previous['before'][k]!=previous['after'][k]]
        previous_step=math.dist(previous['before'][key],before[key]) if key in changed_before else None
        feedback=context.get('feedback',{})
        if outcome=='placed':
            comparison_status=effect;decision='keep';why.append('本轮已成功，保留本轮实际使用的参数。')
        elif movement=='opposite' or feedback.get('status')=='opposite' or effect=='worsened':
            comparison_status='direction_disproved' if movement=='opposite' or feedback.get('status')=='opposite' else 'error_worsened'
            rollback=True;allow_retry=False;decision='undo'
            why.append('前后实帧或反馈否定了上一轮修正，撤销该项参数；回位后停止重复同向叠加，保留对照证据。')
        elif movement=='no_visible_motion' and effect=='unchanged' and previous_step and previous_step<8:
            requested=previous.get('proposal',{}).get('step_mm') or previous_step
            clipped=previous_step<requested*.9
            step=min(MAX_XY_STEP_MM,max(8.,step,previous_step+4))
            comparison_status='invalid_previous_vector' if clipped else 'probe_too_small'
            decision='repair_step' if clipped else 'increase_step'
            why.append('上一轮申请与保存向量不一致，重新生成完整修正，不再把裁剪当成小步探测。' if clipped else
                       '上一轮试探较小，画面无法确认变化；扩大到可见试验尺度，不把看不出变化当成方向已错。')
        elif movement=='no_visible_motion' and effect=='unchanged':
            comparison_status='no_effective_motion';rollback=True;allow_retry=False;decision='undo'
            why.append('画面未显示相应变化，不把已修改的命令当成有效移动；撤销上一轮试验参数并回位。')
        elif previous_step and effect=='overshot':
            step=max(2.,previous_step/2);comparison_status='overshot';decision='shrink_step'
            why.append('前后对照显示越过目标，按本轮观察方向反向修正，步长减半。')
        elif previous_step and effect=='improved':
            step=max(2.,min(4.,step,previous_step/2));comparison_status='improved';decision='shrink_step'
            why.append('前后实帧显示偏差缩小，保留有效方向并缩小步长收敛。')
        elif previous_step and effect=='unchanged' and movement=='as_expected' and feedback.get('status','missing') in ('consistent','missing'):
            step=min(MAX_XY_STEP_MM,max(step,previous_step+4));comparison_status='same_error_repeated';decision='increase_step'
            why.append('已观察到向预期方向移动，但偏差仍未明显改善，增大下一轮试验步长。')
        else:
            comparison_status=effect
            why.append('前轮改善尚未确认，不凭重复的方向或幅度标签自动加码；仅按本轮可见偏差进行试验。')
    if requested_step_mm is not None and not rollback:
        if type(requested_step_mm) not in (int,float) or not math.isfinite(requested_step_mm) or requested_step_mm<=0:
            raise ValueError('Agent试验步长需为正的有限数值')
        step=float(requested_step_mm);decision='agent_step'
        why.append(f'本轮试验步长由执行Agent选择：{step:g}mm。')
    after,changes=adapt(before,scene,outcome,direction,magnitude,adjustment,step_override_mm=step)
    if rollback:
        after=copy.deepcopy(before)
        for name in changed_before:after[name]=copy.deepcopy(previous['before'][name])
        changes=[f'已将{name}从{before[name]}恢复为{after[name]}。' for name in changed_before]
    changed=[name for name in before if before[name]!=after[name]]
    correction=OPPOSITE.get(direction,'none')
    proposal={'observed_direction':direction,'correction_direction':correction,'frame':'unmirrored_rgb',
              'step_mm':step if key in changed and not rollback else None,'parameters_changed':changed,'before':{k:before[k] for k in changed},
              'after':{k:after[k] for k in changed},'comparison':comparison_status,'allow_retry':allow_retry,'decision':decision,
              'comparison_evidence_bound':verified,'effect_assessed':verified and effect not in ('unknown','not_comparable','not_compared'),
              'effect_verified':verified and effect=='improved','improvement_confirmed':verified and effect=='improved',
              'previous_motion_visual_check':movement,
              'before_policy_id':policy_id(before),'after_policy_id':policy_id(after),'source_trial_id':previous.get('plan_id') if verified else None}
    if rollback:proposal['rejected_policy_id']=policy_id(before)
    if direction in PIXEL_VECTOR and outcome in ('grasp_missed','placement_missed') and not rollback:
        a=project(scene,center_target(scene,before,target));b=project(scene,center_target(scene,after,target))
        shift=[b[0]-a[0],b[1]-a[1]]
        delta=[after[key][i]-before[key][i] for i in range(2)];length=math.hypot(*shift)
        alignment=sum(v*w for v,w in zip(shift,PIXEL_VECTOR[correction]))/max(length,1e-12)
        whole_step=math.isclose(math.hypot(*delta),step,rel_tol=.02,abs_tol=.02)
        proposal.update(robot_delta_mm=delta,predicted_pixel_delta=shift,projection_direction_alignment=alignment,
                        requested_step_preserved=whole_step,projection_direction_verified=length>1e-6 and alignment>=math.cos(math.radians(15)) and whole_step)
        proposal['saved_xy_step_mm']=math.hypot(*proposal['robot_delta_mm'])
        if not proposal['projection_direction_verified']:
            after=before;changed=[];allow_retry=False
            proposal.update(after={},before={},parameters_changed=[],after_policy_id=policy_id(before),allow_retry=False)
            proposal.update(candidate_robot_delta_mm=proposal['robot_delta_mm'],candidate_pixel_delta=shift,
                            robot_delta_mm=[0.,0.],predicted_pixel_delta=[0.,0.],saved_xy_step_mm=0.)
            changes=[]
            why.append('候选补偿未在当前相机投影中朝正确方向移动，未应用，也不重复原策略。')
        else:
            actual_step=proposal['saved_xy_step_mm']
            why.insert(0,f"观察到夹爪在目标的{LABELS[direction]}，下一轮向{LABELS[correction]}修正，保存的补偿变化{actual_step:.1f}mm。")
    for record in history:
        old_location=record.get('funnel_xyz_mm' if target=='funnel' else 'ball_xyz_mm')
        old_pitch=record.get('grasp_pitch_deg');current_pitch=context.get('current_pitch_deg')
        same_pose=old_pitch is None or current_pitch is None or abs(old_pitch-current_pitch)<=5
        rejected=record.get('before',{});rejected_id=record.get('proposal',{}).get('rejected_policy_id')
        same_trial=bool(rejected_id and set(rejected)==set(after) and rejected_id==policy_id(rejected) and
                        all(math.dist(after[k],rejected[k])<=1. if k in ('pickup_xy_mm','release_xy_mm') else after[k]==rejected[k] for k in after))
        if (same_pose and old_location and math.dist(old_location[:2],location[:2])<=BALL_RADIUS and
                same_trial and changed and not rollback):
            after=copy.deepcopy(before);changed=[];changes=[]
            proposal.update(after={},before={},parameters_changed=[],after_policy_id=policy_id(before),allow_retry=False,decision='avoid_rejected_policy')
            if 'robot_delta_mm' in proposal:proposal.update(robot_delta_mm=[0.,0.],predicted_pixel_delta=[0.,0.],saved_xy_step_mm=0.)
            why=['该位置和姿态的同一策略已被前后录像否定，本轮未重复应用。'];break
    actionable=outcome in ('grasp_missed','placement_missed','pushed_away','slipped','bounced_out')
    if actionable and not changed:
        proposal['allow_retry']=False
        why.append('本轮没有产生新的有效参数，停止重复相同策略。')
    proposal['status']='rolled_back' if rollback else 'applied' if changed else 'unchanged'
    proposal['reason']='\n'.join(why+changes)
    return after,why+changes,proposal


def application(parameters,plan_parameters,reviews,baseline_resets=()):
    """Check the persisted decision actually reached this newly constructed plan."""
    if parameters!=plan_parameters:raise ValueError('本轮规划没有使用当前RSI参数')
    previous=reviews[-1] if reviews else None
    prior_after={**{'squeeze_deg':DEFAULT_PARAMETERS['squeeze_deg']},**previous.get('after',{})} if previous else None
    reset=baseline_resets[-1] if baseline_resets else None
    repaired=bool(previous and reset and reset.get('source_trial_id')==previous['plan_id'] and
                  reset.get('before')==prior_after and reset.get('after')==parameters)
    if previous and prior_after!=parameters and not repaired:
        raise ValueError('当前策略与上一轮已保存的修正不一致')
    return {'policy_id':policy_id(parameters),'parameters_verified':True,
            'source_trial_id':previous['plan_id'] if previous else None,
            'baseline_reset_id':reset['id'] if repaired else None,
            'changed_parameters':reset['parameters_reset'] if repaired else previous.get('proposal',{}).get('parameters_changed',[]) if previous else [],
            'note':'已核对新计划实际参数；方向投影检查只验证坐标计算，实际效果仍由下一轮录像判断。'}
