"""Sample the current planner on a tabletop; never own or command a device."""
from __future__ import annotations

import copy
import math
import time

import numpy as np

from evoarm.limits import GRIPPER_MAX_RAD,JOINTS
from .ball_trial import check_segment
from .pick_place import BALL_RADIUS,build_plan,_opening_direction
from .planning_diagnostics import explain
from .roarm_geometry import inverse,MotionConstraintError,height_above_plane
from .rsi_route import ROUTE_VERSION


class PreviewBudgetReached(RuntimeError):pass


def _contact_possible(center,status,scene,p):
    normal=np.asarray(scene['table_plane']['normal_up'])
    for degrees in (80.,75.,85.,70.,90.,65.,60.):
        pitch=math.radians(degrees)
        tip=center+normal*p['grasp_z_offset_mm']-_opening_direction(center,pitch,status['r'])*p['finger_offset_mm']
        tip[:2]+=p['pickup_xy_mm']
        if height_above_plane(tip,scene['table_plane'])<-.5:continue
        try:raw=inverse(tip.tolist(),pitch,status['r'],min(math.radians(p['close_deg']),GRIPPER_MAX_RAD))
        except ValueError:continue
        if all(low<=math.degrees(raw[field])<=high for _,field,low,high in JOINTS.values()):return True
    return False


def compute_workspace(session,status,scene,parameters,*,columns=32,rows=18,budget_s=12.,progress=None):
    """Green means a full route passed at a sampled ball center, not a new gate.

    Cells are estimates between samples. Unknown/uncomputed cells stay '?';
    neither the grid nor the clock changes the actual execution constraints.
    """
    if not (2<=columns<=48 and 2<=rows<=32 and 0<budget_s<=30):raise ValueError('工作区采样设置无效')
    started=time.monotonic();deadline=started+budget_s;cache={};grid=[['?']*columns for _ in range(rows)]
    width,height=scene['image_size'];geometry=scene['camera_geometry']
    R=np.asarray(geometry['R']);t=np.asarray(geometry['t']);inverse_K=np.asarray(geometry['inverse_K'])
    normal=np.asarray(scene['table_plane']['normal_up']);normal_camera=R.T@normal
    d_camera=float(normal@t+scene['table_plane']['d_mm'])
    completed=0;last_update=0.;budget_hit=False
    def checker(initial,step,plane):
        if time.monotonic()>=deadline:raise PreviewBudgetReached()
        key=(*[initial[k] for k in 'xyz'],initial['tit'],step['stage'],*step['xyz_mm'],
             step['pitch_rad'],step['roll_rad'],step['gripper_rad'],step['minimum_tip_height_mm'])
        if key in cache:
            failure=cache[key]
            if failure:raise MotionConstraintError(failure['message'],**{k:v for k,v in failure.items() if k!='message'})
            return
        try:check_segment(initial,step,plane)
        except ValueError as exc:
            cache[key]=getattr(exc,'detail',{'message':str(exc)});raise
        cache[key]=None
    def plan_at(center):
        candidate={**scene,'ball':{**scene['ball'],'center_robot_mm':np.asarray(center).tolist()}}
        return build_plan(session,status,candidate,parameters,segment_checker=checker)
    selected={'status':'unknown'}
    try:
        exact=plan_at(scene['ball']['center_robot_mm'])
        selected={'status':'planned','grasp_pitch_deg':exact['grasp_pitch_deg'],
                  'release_pitch_deg':exact['release_pitch_deg'],'transfer_height_mm':exact['route_policy']['transfer_height_mm']}
    except MotionConstraintError as exc:
        selected={'status':'not_planned','diagnosis':explain(exc.detail)}
    except PreviewBudgetReached:budget_hit=True
    result={'schema_version':1,'status':'computing','robot_moved':False,'grid':[],
            'image_size':[width,height],'columns':columns,'rows':rows,'total':columns*rows,'completed':0,
            'selected_ball':selected,'initial_xyz_mm':[status[k] for k in 'xyz'],
            'initial_pitch_deg':math.degrees(status['tit']),'initial_roll_rad':status['r'],'parameters':copy.deepcopy(parameters),
            'funnel_xyz_mm':scene['funnel']['mouth_center_robot_mm'],'source_trial_id':scene.get('source_trial_id'),
            'sampled_ball_center_height_mm':BALL_RADIUS,'route_policy_version':ROUTE_VERSION,
            'note':'绿色为采样球心找到完整抓放路径；灰色为当前候选搜索未通过；空白为未完成采样。边界是估计，不是机械臂的固定限制。'}
    def update(force=False):
        nonlocal last_update
        if force or time.monotonic()-last_update>.25:
            result.update(grid=[''.join(row) for row in grid],completed=completed,
                          elapsed_s=round(time.monotonic()-started,3))
            if progress:progress(copy.deepcopy(result))
            last_update=time.monotonic()
    update(True)
    origin=scene['ball'].get('pixel_center',[width/2,height/2])
    cells=[(row,col) for row in range(rows) for col in range(columns)]
    cells.sort(key=lambda rc:((rc[1]+.5)*width/columns-origin[0])**2+((rc[0]+.5)*height/rows-origin[1])**2)
    for row,col in cells:
        if budget_hit or time.monotonic()>=deadline:budget_hit=True;break
        u=(col+.5)*width/columns;v=(row+.5)*height/rows;ray=inverse_K@np.array([u,v,1.])
        denominator=float(ray@normal_camera)
        if abs(denominator)<1e-8:completed+=1;continue
        distance=(BALL_RADIUS-d_camera)/denominator
        if distance<=0:completed+=1;continue
        center=R@(ray*distance)+t
        if not _contact_possible(center,status,scene,parameters):
            grid[row][col]='-'
        else:
            try:plan_at(center);grid[row][col]='+'
            except MotionConstraintError:grid[row][col]='-'
            except PreviewBudgetReached:budget_hit=True;break
        completed+=1;update()
    result.update(status='partial' if budget_hit else 'completed',
                  planned_samples=sum(row.count('+') for row in grid),
                  rejected_samples=sum(row.count('-') for row in grid),
                  unknown_samples=sum(row.count('?') for row in grid))
    update(True)
    return result
