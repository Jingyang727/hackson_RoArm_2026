"""Lift, transfer, then descend. Pure planning and measured-pose anchoring."""
import math

import numpy as np

from .ball_trial import check_segment
from .roarm_geometry import height_above_plane,inverse,MotionConstraintError


def at_height(point,height,plane):
    """Change robot Z only; a tilted plane must not turn a lift into XY drag."""
    x,y,_=point;nx,ny,nz=plane['normal_up']
    return [float(x),float(y),float((height-plane['d_mm']-nx*x-ny*y)/nz)]


def execution_target(stage,actual,plane):
    """Preserve vertical motion even when the prior segment settled off-target."""
    target=dict(stage)
    if stage.get('vertical_only'):
        height=height_above_plane(stage['xyz_mm'],plane)
        target['xyz_mm']=at_height([actual[k] for k in 'xyz'],height,plane)
    if stage.get('hold_orientation'):
        target.update(pitch_rad=actual['tit'],roll_rad=actual['r'])
    if stage['stage']=='raise_current':
        target['minimum_tip_height_mm']=min(0.,height_above_plane([actual[k] for k in 'xyz'],plane))
        if target['xyz_mm'][2]<actual['z']-.01:raise ValueError('起步抬升目标已低于实际位置，请重新规划')
    return target


def build_route(initial,grasp,release,pitch,plane,p,*,ball_top_height,funnel_height,ball_radius,release_candidates=None):
    # An object-derived transit clearance, not a restriction on selectable XYZ.
    floor=max(funnel_height,2*ball_radius)+ball_radius+10.
    releases=release_candidates or [{'xyz_mm':list(release),'pitch_rad':pitch}]
    travel=max(p['lift_height_mm'],floor+10.,max(height_above_plane(r['xyz_mm'],plane) for r in releases)+10.)
    hover=max(p['hover_height_mm'],ball_top_height+p['finger_offset_mm']+5.)
    start_height=height_above_plane([initial[k] for k in 'xyz'],plane)
    corridors=list(dict.fromkeys([max(start_height,travel),travel]))
    angles=list(dict.fromkeys([initial['tit'],pitch,math.radians(40),math.radians(60),math.radians(30)]))
    entry_height=max(hover,floor+10.)
    variants=[(max(start_height,entry_height) if start_height<floor else start_height,pitch,True)]
    variants.extend((corridor,angle,False) for corridor in corridors for angle in angles)
    failures=[]
    def rejected(exc,stage):
        detail={**getattr(exc,'detail',{'message':str(exc)}),'stage':stage['stage'],
                'xyz_mm':stage['xyz_mm'],'pitch_deg':math.degrees(stage['pitch_rad'])}
        if detail not in failures:failures.append(detail)
    for corridor,transfer_pitch,merged in variants:
        current=dict(initial);steps=[]
        def move(stage,point,angle,speed,minimum,*,vertical=False,hold=False):
            nonlocal current
            point=np.asarray(point,float).tolist()
            if vertical and math.dist(point[:2],[current['x'],current['y']])>1e-6:
                raise ValueError('内部规划错误：竖直段改变了XY')
            if math.dist([current[k] for k in 'xyz'],point)<.8 and abs(current['tit']-angle)<math.radians(.5):return
            step={'stage':stage,'kind':'cartesian','xyz_mm':point,'pitch_rad':angle,'roll_rad':initial['r'],
                  'gripper_rad':current['g'],'spd':speed,'minimum_tip_height_mm':minimum,
                  'vertical_only':vertical,'hold_orientation':hold}
            try:check_segment(current,step,plane)
            except ValueError as exc:rejected(exc,step);raise
            steps.append(step)
            current=inverse(point,angle,initial['r'],current['g'])
        def hand(stage,angle):
            if abs(current['g']-angle)<math.radians(1):return
            steps.append({'stage':stage,'kind':'gripper','gripper_rad':angle,
                          'spd':p['gripper_speed'],'acc':p['gripper_acc']});current['g']=angle
        try:
            start=[current[k] for k in 'xyz']
            move('raise_current' if corridor>=start_height else 'clearance_adjust',at_height(start,corridor,plane),
                 current['tit'],p['travel_speed'],min(0.,start_height) if corridor>=start_height else floor,
                 vertical=True,hold=True)
            if not merged:move('orient_clear',[current[k] for k in 'xyz'],transfer_pitch,p['travel_speed'],floor,vertical=True)
            hand('open',math.radians(p['open_deg']))
            move('approach_ball',at_height(grasp,entry_height if merged else corridor,plane),transfer_pitch,p['travel_speed'],floor)
            if not merged:move('align_above_ball',at_height(grasp,hover,plane),pitch,p['approach_speed'],min(hover,floor),vertical=True)
            move('descend',grasp,pitch,p['approach_speed'],0.,vertical=True)
            from evoarm.limits import GRIPPER_MAX_RAD
            hand('close',min(math.radians(p['close_deg']),GRIPPER_MAX_RAD))
            # Grasping and transporting have different reachable pose sets.
            # Recompute the stock-finger target for EACH release pitch. A
            # changed pitch with the old fingertip XYZ would miss the mouth.
            picked=current.copy();prefix=list(steps);selected=None
            for candidate in releases:
                drop=candidate['xyz_mm'];drop_pitch=candidate['pitch_rad']
                for mode in ('direct','orient','arc'):
                    current=picked.copy();steps=list(prefix)
                    try:
                        # Change to the transport pitch WHILE lifting, at
                        # measured XY. Keeping a steep grasp pitch until
                        # after lift needlessly loads the extended wrist.
                        move('lift',at_height(grasp,travel,plane),drop_pitch,p['travel_speed'],0.,vertical=True)
                        if mode!='direct':
                            move('orient_transport',[current[k] for k in 'xyz'],drop_pitch,p['travel_speed'],floor,vertical=True)
                        if mode=='arc':
                            a=math.atan2(current['y'],current['x']);b=math.atan2(drop[1],drop[0])
                            delta=math.atan2(math.sin(b-a),math.cos(b-a));count=max(2,math.ceil(abs(delta)/math.radians(25)))
                            for i in range(count+1):
                                angle=a+delta*i/count
                                move('transit_arc',at_height([260.*math.cos(angle),260.*math.sin(angle),0.],travel,plane),
                                     drop_pitch,p['travel_speed'],floor)
                        move('transport',at_height(drop,travel,plane),drop_pitch,p['travel_speed'],floor)
                        move('lower_to_release',drop,drop_pitch,p['approach_speed'],funnel_height,vertical=True)
                        selected=candidate;break
                    except ValueError:continue
                if selected is not None:break
            if selected is None:continue
            # The completed Cartesian stage already requires stationary feedback;
            # leave a short additional pause over the mouth before opening.
            steps[-1]['settle_s']=.2
            hand('release',math.radians(p['open_deg']))
            for index,step in enumerate(steps):
                if step['stage']=='open' and index+1<len(steps) and steps[index+1]['stage']=='approach_ball':
                    steps[index+1].update(gripper_rad=step['gripper_rad'],open_while_moving=True)
                    del steps[index];break
            return steps,{'version':4,'mode':'lift_transfer_descend','vertical_axis':'robot_z_keep_xy',
                'minimum_transfer_height_mm':floor,'transfer_height_mm':travel,'entry_transfer_height_mm':corridor,
                'pickup_hover_height_mm':hover,'release_settle_s':.2,
                'release_pitch_rad':selected['pitch_rad'],'release_tip_xyz_mm':list(selected['xyz_mm']),
                'merged_high_approach':merged,'lift_pitch_rad':selected['pitch_rad'],
                'feedback_anchoring':'vertical segments keep measured XY; high transfers recheck actual clearance'}
        except ValueError:continue
    labels={'raise_current':'起步抬升','clearance_adjust':'竖直调整通行高度','orient_clear':'起步高处转腕',
        'approach_ball':'接近球上方','align_above_ball':'球上方调整','descend':'抓取下探','lift':'抓后抬升',
        'transport':'运输到漏斗上方','orient_transport':'运输前高处转腕','transit_arc':'高处绕行','lower_to_release':'投放下降'}
    summary='；'.join(f"{labels.get(f['stage'],f['stage'])} XYZ={[round(v,1) for v in f['xyz_mm']]}、俯仰{f['pitch_deg']:.1f}°：{f['message']}" for f in failures[:3])
    raise MotionConstraintError('未找到通过检查的候选路径（不代表目标物体不可达）：'+summary,
        code='route_search_exhausted',attempts=failures[:64],attempt_count=len(failures))
