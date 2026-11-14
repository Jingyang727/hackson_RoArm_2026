"""Pair actual trials and separate commanded, encoder and visual evidence."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np

from .pick_place import BALL_RADIUS

EFFECTS={'improved','worsened','overshot','unchanged','unknown','not_comparable','not_compared'}
MOVEMENTS={'as_expected','opposite','no_visible_motion','unknown'}


def validate_assessment(value):
    if not isinstance(value,dict) or set(value)!={'reference_plan_id','effect','movement','note'}:
        raise ValueError('Grok前后对照格式无效')
    ref=value['reference_plan_id']
    if ref is not None and (not isinstance(ref,str) or not re.fullmatch('[a-f0-9]{32}',ref)):
        raise ValueError('Grok前后对照试次ID无效')
    if value['effect'] not in EFFECTS or value['movement'] not in MOVEMENTS:
        raise ValueError('Grok前后对照类别无效')
    if not isinstance(value['note'],str) or len(value['note'])>2000:raise ValueError('Grok前后对照依据无效')
    if ref is None and (value['effect']!='not_compared' or value['movement']!='unknown'):
        raise ValueError('没有前轮证据时不能宣称已比较')
    if ref is not None and value['effect']=='not_compared':raise ValueError('已引用前轮时需明确对照结果或说明无法比较')
    return copy.deepcopy(value)


def evidence_frame(state,experiments,*,target='ball',lift=False):
    trial=state.get('result',{}).get('capture_trial_id')
    if not isinstance(trial,str) or not re.fullmatch('[a-f0-9]{32}',trial):return None
    folder=Path(experiments)/trial;file=folder/'keyframes.json'
    if not file.is_file() or file.is_symlink():return None
    try:rows=json.loads(file.read_text()).get('frames',[])
    except (OSError,ValueError,AttributeError):return None
    labels=('lift_review','lift') if lift else ('release_review','release') if target=='funnel' else ('close_review','close')
    for label in labels:
        for row in rows:
            if label not in row.get('labels',row.get('label','').split('/')):continue
            path=folder/row['file']
            if not path.is_file() or path.is_symlink() or path.resolve().parent!=folder.resolve():continue
            later=('transport','transit_arc') if lift else ('finish',) if target=='funnel' else ('lift',)
            verified=label.endswith('_review') or any(e.get('stage') in later for e in state.get('result',{}).get('events',[]))
            return {'path':str(path),'label':row['label'],'frame_index':row.get('frame_index'),
                    'elapsed_s':row['elapsed_s'],'plan_id':state['id'],'role':'current','stage_verified':verified}
    return None


def feedback_pose(state,target='ball'):
    stage='release' if target=='funnel' else 'close'
    for row in reversed(state.get('result',{}).get('feedback',[])):
        status=row.get('status',{})
        if row.get('stage')!=stage:continue
        if all(type(status.get(k)) in (int,float) and math.isfinite(status[k]) for k in 'xyz'):
            return [status[k] for k in 'xyz']
    return None


def build_comparison(current,previous,review,experiments):
    context={'available':False,'reference_plan_id':review.get('plan_id') if review else None,
             'current_pitch_deg':current['plan']['grasp_pitch_deg'],
             'reason':'没有上一轮已执行修正的证据','frame':'unmirrored_rgb','parameter_changes':[],
             'feedback':{'available':False,'status':'missing','note':'固件反馈并非图像实测位移'}}
    if not previous or not review:return context
    a=previous['plan'];b=current['plan'];before=a['parameters'];after=b['parameters']
    if a.get('route_policy',{}).get('version')!=b.get('route_policy',{}).get('version'):
        context['reason']='执行路径版本已变化，先验证新流程，不把变化归因于单项参数';return context
    context['parameter_changes']=[{'name':k,'before':before[k],'after':after[k]} for k in before if before[k]!=after.get(k)]
    if current['id']==previous['id'] or previous['id']!=review['plan_id']:
        context['reason']='前轮ID不对应本次已应用的修正';return context
    if previous.get('calibration_fingerprint')!=current.get('calibration_fingerprint'):
        context['reason']='前后标定不同，不比较方向';return context
    if review.get('after')!=after or review.get('before')!=before:
        context['reason']='当前执行参数不是该前轮决策的直接结果';return context
    if len(context['parameter_changes'])!=1:
        context['reason']='前后不是单项参数试验，不能归因修正效果';return context
    changed=context['parameter_changes'][0]['name']
    target='funnel' if changed.startswith('release_') else 'ball'
    context['target']=target
    if previous.get('result',{}).get('capture_trial_id')==current.get('result',{}).get('capture_trial_id'):
        context['reason']='两轮引用了同一段录像，不能当成前后证据';return context
    field='mouth_center_robot_mm' if target=='funnel' else 'center_robot_mm'
    if math.dist(a['scene'][target][field][:2],b['scene'][target][field][:2])>BALL_RADIUS:
        context['reason']='目标位置变化较大，本轮按新位置试验';return context
    if abs(a['grasp_pitch_deg']-b['grasp_pitch_deg'])>5:
        context['reason']='前后抓取俯仰差异较大，不把变化归因于单项补偿';return context
    for state in (current,previous):
        if not state.get('result',{}).get('placement_attempted' if target=='funnel' else 'grasp_attempted'):
            context['reason']='某轮未执行对应的闭爪或投放动作';return context
    key='release_xy_mm' if target=='funnel' else 'pickup_xy_mm'
    tip_key='release_tip_xyz_mm' if target=='funnel' else 'grasp_tip_xyz_mm'
    context.update(previous_policy_version=previous['policy_version'],current_policy_version=current['policy_version'],
                   previous_outcome=review['outcome'],previous_direction=review.get('direction'),
                   previous_note=review.get('note'),expected_image_delta_px=review.get('proposal',{}).get('predicted_pixel_delta'))
    if changed==key:
        requested=np.asarray(after[key])-before[key]
        context['requested_xy_delta_mm']=requested.tolist()
        geometry=b['scene']['camera_geometry'];normal=np.asarray(b['scene']['table_plane']['normal_up'])
        height=after['grasp_z_offset_mm'] if target=='ball' else BALL_RADIUS+after['release_clearance_mm']
        center=np.asarray(b['scene'][target][field])+normal*height;center[:2]+=after[key]
        def project(point):
            camera=np.asarray(geometry['R']).T@(point-np.asarray(geometry['t']))
            uv=np.asarray(geometry['K'])@camera
            return uv[:2]/uv[2]
        context['expected_image_delta_px']=(project(center)-project(center-np.r_[requested,0.])).tolist()
        old=feedback_pose(previous,target);new=feedback_pose(current,target)
        if old is not None and new is not None and np.linalg.norm(requested)>0:
            # Remove target relocation / nominal pose changes before attributing
            # a difference to the learned XY parameter. Never call this video truth.
            baseline=(np.asarray(b[tip_key])[:2]-after[key])-(np.asarray(a[tip_key])[:2]-before[key])
            reported=np.asarray(new)[:2]-np.asarray(old)[:2]-baseline
            gain=float(reported@requested/(requested@requested))
            alignment=float(reported@requested/max(1e-8,np.linalg.norm(reported)*np.linalg.norm(requested)))
            status=('inconclusive' if np.linalg.norm(requested)<4 else 'opposite' if gain<-.25 and alignment<-.5
                    else 'too_small' if np.linalg.norm(reported)<np.linalg.norm(requested)*.25
                    else 'consistent' if gain>.25 and alignment>.5 else 'inconclusive')
            context['feedback'].update(available=True,status=status,requested_xy_delta_mm=requested.tolist(),
                                       reported_xy_delta_mm=reported.tolist(),gain_along_command=gain,
                                       previous_tip_xyz_mm=old,current_tip_xyz_mm=new)
    prior_frame=evidence_frame(previous,experiments,target=target)
    current_frame=evidence_frame(current,experiments,target=target)
    if not prior_frame or not current_frame:
        context['reason']='缺少前轮或本轮对应动作的实际关键帧';return context
    if not prior_frame['stage_verified'] or not current_frame['stage_verified']:
        context['reason']='只有动作开始帧，缺少对应阶段已完成的证据';return context
    if ('_review' in prior_frame['label'])!=('_review' in current_frame['label']):
        context['reason']='前后帧处于动作开始与完成的不同时刻，不比较夹爪偏移';return context
    prior_frame['role']='previous'
    context.update(available=True,reason='相同标定、相近位置、单项修正，提供对应动作的前后实帧',
                   previous_frame=prior_frame,current_frame=current_frame,
                   previous_target_pixel=a['scene'][target]['pixel_center'],current_target_pixel=b['scene'][target]['pixel_center'])
    for role,plan in (('previous',a),('current',b)):
        obj=plan['scene'][target]
        context[role+'_target_radius_px']=obj.get('radius_px',obj.get('expected_diameter_px',40.)/2)
    lift_frame=evidence_frame(previous,experiments,lift=True)
    if lift_frame:lift_frame['role']='previous';context['previous_lift_frame']=lift_frame
    return context


def public_comparison(context,assessment=None,*,plan_id=None,panel=False):
    keys=('available','reference_plan_id','reason','target','parameter_changes','requested_xy_delta_mm',
          'expected_image_delta_px','feedback','previous_policy_version','current_policy_version','current_pitch_deg')
    value={k:copy.deepcopy(context[k]) for k in keys if k in context}
    value['plan_id']=plan_id
    value['assessment']=copy.deepcopy(assessment)
    value['panel_url']=f'/api/bench/rsi/plans/{plan_id}/comparison' if panel and plan_id else None
    return value


def camera_view(image,title,expected=None):
    """Label camera axes without mirroring, warping or hiding the actual frame."""
    result=cv2.copyMakeBorder(image,58,60,0,0,cv2.BORDER_CONSTANT,value=(245,248,245))
    cv2.putText(result,title,(12,23),cv2.FONT_HERSHEY_SIMPLEX,.55,(20,55,40),1)
    cv2.putText(result,'CAMERA VIEW: LEFT <--- ---> RIGHT | UP ^ / DOWN v | NOT MIRRORED',
                (12,47),cv2.FONT_HERSHEY_SIMPLEX,.48,(20,55,40),1)
    y=result.shape[0]-24
    if expected is not None and len(expected)==2 and np.linalg.norm(expected)>0:
        vector=np.asarray(expected,float)/np.linalg.norm(expected)*25
        center=np.array([max(30,result.shape[1]-55),y],float)
        cv2.arrowedLine(result,tuple((center-vector).astype(int)),tuple((center+vector).astype(int)),(40,120,210),2,tipLength=.3)
        text='PLANNED SHIFT: direction only, NOT measured motion'
    else:text='OBSERVED CAMERA IMAGE - no mirroring'
    cv2.putText(result,text,(12,y-18),cv2.FONT_HERSHEY_SIMPLEX,.42,(20,55,40),1)
    return result


def target_crop(image,pixel,radius=20.):
    x,y=map(int,pixel);margin=max(80,min(200,int(radius*4)))
    roi=[max(0,x-margin),max(0,y-margin),min(image.shape[1],x+margin),min(image.shape[0],y+margin)]
    crop=image[roi[1]:roi[3],roi[0]:roi[2]]
    if not crop.size:raise ValueError('目标局部实帧范围无效')
    scale=min(560/crop.shape[1],420/crop.shape[0]);crop=cv2.resize(crop,(round(crop.shape[1]*scale),round(crop.shape[0]*scale)))
    canvas=np.full((420,560,3),245,np.uint8);y0=(420-crop.shape[0])//2;x0=(560-crop.shape[1])//2
    canvas[y0:y0+crop.shape[0],x0:x0+crop.shape[1]]=crop
    return canvas,roi


def comparison_panel(context,root):
    if not context.get('available'):return None
    tiles=[]
    for role in ('previous','current'):
        frame=context[role+'_frame'];image=cv2.imread(frame['path'])
        if image is None:return None
        canvas,_=target_crop(image,context[role+'_target_pixel'],context.get(role+'_target_radius_px',20.))
        tiles.append(camera_view(canvas,f"{role.upper()} | {frame['plan_id'][:10]} | frame {frame['frame_index']} | {frame['elapsed_s']:.2f}s",
                                 context.get('expected_image_delta_px') if role=='current' else None))
    path=Path(root)/'comparison.jpg'
    if not cv2.imwrite(str(path),np.hstack(tiles)):raise ValueError('前后实帧对照图保存失败')
    return str(path)
