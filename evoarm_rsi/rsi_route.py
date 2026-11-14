"""Lift, transfer, then descend. Pure planning and measured-pose anchoring."""
import math

import numpy as np

from .ball_trial import check_segment
from .roarm_geometry import height_above_plane,inverse


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


def build_route(initial,grasp,release,pitch,plane,p,*,ball_top_height,funnel_height,ball_radius):
    # An object-derived transit clearance, not a restriction on selectable XYZ.
    floor=max(funnel_height,2*ball_radius)+ball_radius+10.
    travel=max(p['lift_height_mm'],floor+10.,height_above_plane(release,plane)+10.)
    hover=max(p['hover_height_mm'],ball_top_height+p['finger_offset_mm']+5.)
    start_height=height_above_plane([initial[k] for k in 'xyz'],plane)
    corridors=list(dict.fromkeys([max(start_height,travel),travel]))
    angles=list(dict.fromkeys([initial['tit'],pitch,math.radians(40),math.radians(60),math.radians(30)]))
    failures=[]
    for corridor in corridors:
        for transfer_pitch in angles:
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
                check_segment(current,step,plane);steps.append(step)
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
                move('orient_clear',[current[k] for k in 'xyz'],transfer_pitch,p['travel_speed'],floor,vertical=True)
                hand('open',math.radians(p['open_deg']))
                move('approach_ball',at_height(grasp,corridor,plane),transfer_pitch,p['travel_speed'],floor)
                move('align_above_ball',at_height(grasp,hover,plane),pitch,p['approach_speed'],min(hover,floor),vertical=True)
                move('descend',grasp,pitch,p['approach_speed'],0.,vertical=True)
                from evoarm.limits import GRIPPER_MAX_RAD
                hand('close',min(math.radians(p['close_deg']),GRIPPER_MAX_RAD))
                move('lift',at_height(grasp,travel,plane),pitch,p['travel_speed'],0.,vertical=True,hold=True)
                try:move('transport',at_height(release,travel,plane),pitch,p['travel_speed'],floor)
                except ValueError:
                    a=math.atan2(current['y'],current['x']);b=math.atan2(release[1],release[0])
                    delta=math.atan2(math.sin(b-a),math.cos(b-a));count=max(2,math.ceil(abs(delta)/math.radians(25)))
                    for i in range(count+1):
                        angle=a+delta*i/count
                        move('transit_arc',at_height([260.*math.cos(angle),260.*math.sin(angle),0.],travel,plane),
                             pitch,p['travel_speed'],floor)
                    move('transport',at_height(release,travel,plane),pitch,p['travel_speed'],floor)
                move('lower_to_release',release,pitch,p['approach_speed'],funnel_height,vertical=True)
                # The completed Cartesian stage already requires stationary feedback;
                # leave a short additional pause over the mouth before opening.
                steps[-1]['settle_s']=.2
                hand('release',math.radians(p['open_deg']))
                for index,step in enumerate(steps):
                    if step['stage']=='open' and index+1<len(steps) and steps[index+1]['stage']=='approach_ball':
                        steps[index+1].update(gripper_rad=step['gripper_rad'],open_while_moving=True)
                        del steps[index];break
                return steps,{'version':2,'mode':'lift_transfer_descend','vertical_axis':'robot_z_keep_xy',
                    'minimum_transfer_height_mm':floor,'transfer_height_mm':travel,'entry_transfer_height_mm':corridor,
                    'pickup_hover_height_mm':hover,'release_settle_s':.2,
                    'feedback_anchoring':'vertical segments keep measured XY; high transfers recheck actual clearance'}
            except ValueError as exc:
                if str(exc) not in failures:failures.append(str(exc))
    raise ValueError('抬升/高处转移/竖直下探路径不可达：'+'；'.join(failures[:3]))
