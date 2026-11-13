"""Explicit camera-frame corrections, effect comparison and policy application."""
from __future__ import annotations

import copy
import hashlib
import json
import math

import numpy as np

from .pick_place import adapt,BALL_RADIUS,DEFAULT_PARAMETERS

OPPOSITE={'image_left':'image_right','image_right':'image_left','image_up':'image_down','image_down':'image_up',
          'too_high':'lower','too_low':'higher'}
LABELS={'image_left':'画面左侧','image_right':'画面右侧','image_up':'画面上方','image_down':'画面下方',
        'lower':'降低','higher':'抬高','none':'方向未知'}
PIXEL_VECTOR={'image_left':[-1,0],'image_right':[1,0],'image_up':[0,-1],'image_down':[0,1]}


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


def propose(parameters,scene,outcome,direction,magnitude='medium',adjustment='unknown',*,history=(),visual=None):
    before=copy.deepcopy(parameters);step={'small':2.,'medium':4.,'large':6.,'unknown':4.}[magnitude]
    target='funnel' if outcome=='placement_missed' else 'ball'
    key='release_xy_mm' if target=='funnel' else 'pickup_xy_mm'
    location=scene[target]['mouth_center_robot_mm' if target=='funnel' else 'center_robot_mm']
    previous=next((r for r in reversed(history) if r.get('outcome')==outcome),None)
    comparison='first_observation';why=[];allow_retry=True
    related=False
    if previous:
        prior=previous.get('funnel_xyz_mm' if target=='funnel' else 'ball_xyz_mm')
        related=bool(prior and math.dist(prior[:2],location[:2])<=BALL_RADIUS)
    if related and outcome in ('grasp_missed','placement_missed') and direction in PIXEL_VECTOR:
        previous_step=previous.get('proposal',{}).get('step_mm')
        if previous_step is None:
            previous_step=math.dist(previous.get('before',before)[key],previous.get('after',before)[key])
        old_error=(previous.get('visual_evidence') or {}).get('normalized_error')
        error=(visual or {}).get('normalized_error')
        ranks={'small':0,'medium':1,'large':2,'unknown':None}
        old_rank=ranks.get(previous.get('magnitude'));new_rank=ranks.get(magnitude)
        worsened=(old_error is not None and error is not None and error>old_error*1.25+.05)
        if worsened or (old_rank is not None and new_rank is not None and new_rank>old_rank):
            comparison='error_worsened';allow_retry=False
            why.append('同位置补偿后视觉偏差变大，撤销上一轮该项补偿，停止盲目叠加并核对目标和映射。')
        elif previous.get('direction')==OPPOSITE[direction]:
            step=max(1.,float(previous_step or step)/2);comparison='direction_reversed'
            why.append('偏差方向反转，可能越过目标，下一轮缩小步长。')
        elif previous.get('direction')==direction:
            improved=(old_error is not None and error is not None and error<old_error*.8)
            improved=improved or (old_rank is not None and new_rank is not None and new_rank<old_rank)
            if improved:comparison='improved';why.append('偏差缩小，保留小步修正。')
            elif previous_step and previous_step<=8:
                step=min(8.,max(step,float(previous_step)+2));comparison='same_error_repeated'
                why.append('同位置连续报告同向偏差，小步未确认改善；本轮增大试验步长，不把它当作已测得的误差。')
    after,changes=adapt(before,scene,outcome,direction,magnitude,adjustment,step_override_mm=step)
    if comparison=='error_worsened':
        after[key]=copy.deepcopy(previous['before'][key])
        changes=[f'已将{key}恢复为上一轮调整前的值{after[key]}。']
    changed=[name for name in before if before[name]!=after[name]]
    correction=OPPOSITE.get(direction,'none')
    proposal={'observed_direction':direction,'correction_direction':correction,'frame':'unmirrored_rgb',
              'step_mm':step,'parameters_changed':changed,'before':{k:before[k] for k in changed},
              'after':{k:after[k] for k in changed},'comparison':comparison,'allow_retry':allow_retry,
              'before_policy_id':policy_id(before),'after_policy_id':policy_id(after),'source_trial_id':previous.get('plan_id') if related else None}
    if direction in PIXEL_VECTOR and outcome in ('grasp_missed','placement_missed') and comparison!='error_worsened':
        a=project(scene,center_target(scene,before,target));b=project(scene,center_target(scene,after,target))
        shift=[b[0]-a[0],b[1]-a[1]]
        proposal.update(robot_delta_mm=[after[key][i]-before[key][i] for i in range(2)],predicted_pixel_delta=shift,
                        projection_direction_verified=sum(v*w for v,w in zip(shift,PIXEL_VECTOR[correction]))>.05)
        if not proposal['projection_direction_verified']:
            after=before;changed=[];allow_retry=False
            proposal.update(after={},before={},parameters_changed=[],after_policy_id=policy_id(before),allow_retry=False)
            changes=[]
            why.append('候选补偿未在当前相机投影中朝正确方向移动，未应用，也不重复原策略。')
        else:why.insert(0,f"观察到夹爪在目标的{LABELS[direction]}，下一轮向{LABELS[correction]}修正，试验步长{step:g}mm。")
    actionable=outcome in ('grasp_missed','placement_missed','pushed_away','slipped','bounced_out')
    if actionable and not changed:
        proposal['allow_retry']=False
        why.append('本轮没有产生新的有效参数，停止重复相同策略。')
    proposal['status']='rolled_back' if comparison=='error_worsened' else 'applied' if changed else 'unchanged'
    proposal['reason']='\n'.join(why+changes)
    return after,why+changes,proposal


def application(parameters,plan_parameters,reviews):
    """Check the persisted decision actually reached this newly constructed plan."""
    if parameters!=plan_parameters:raise ValueError('本轮规划没有使用当前RSI参数')
    previous=reviews[-1] if reviews else None
    prior_after={**{'squeeze_deg':DEFAULT_PARAMETERS['squeeze_deg']},**previous.get('after',{})} if previous else None
    if previous and prior_after!=parameters:
        raise ValueError('当前策略与上一轮已保存的修正不一致')
    return {'policy_id':policy_id(parameters),'parameters_verified':True,
            'source_trial_id':previous['plan_id'] if previous else None,
            'changed_parameters':previous.get('proposal',{}).get('parameters_changed',[]) if previous else [],
            'note':'已核对新计划实际参数；方向投影检查只验证坐标计算，实际效果仍由下一轮录像判断。'}
