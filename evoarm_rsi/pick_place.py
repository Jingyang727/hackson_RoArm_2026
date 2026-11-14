"""RGB-D scene, stock-finger pick/place plan and qualitative RSI updates.

Pure calculation: no device owner, model API or motor write is created here.
"""
from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np

from .ball_trial import observe_ball, pixel_at_height, check_segment
from .coordinate_test import gripper_hold
from .roarm_geometry import forward, inverse, height_above_plane,MotionConstraintError
from .task_scene import load_rgbd
from control_ui.vision import yellow_ball_mask
from evoarm.limits import GRIPPER_MAX_RAD
from .image_directions import OPPOSITE,PIXEL_VECTOR

BALL_RADIUS = 17.
FUNNEL_RADIUS = 38.
FUNNEL_HEIGHT = 45.4
XY_STEPS_MM={'small':4.,'medium':8.,'large':12.,'unknown':8.}
MAX_XY_STEP_MM=16.
DEFAULT_PARAMETERS = {
    'pickup_xy_mm':[0.,0.], 'release_xy_mm':[0.,0.], 'grasp_z_offset_mm':3.,
    'finger_offset_mm':19., 'hover_height_mm':70., 'lift_height_mm':100.,
    'release_clearance_mm':18., 'open_deg':110., 'close_deg':180.,
    'travel_speed':.65, 'approach_speed':.30, 'gripper_speed':1000, 'gripper_acc':20,'squeeze_deg':2.,
}


def sphere_center(ball, frame, geometry):
    """Fit a known-radius sphere to actual yellow surface samples, with a table prior."""
    center=np.asarray(ball['center_robot_mm'],float)
    mask=yellow_ball_mask(frame['bgr'])>0
    yy,xx=np.indices(mask.shape);u,v=ball['pixel_center']
    mask &= (xx-u)**2+(yy-v)**2 < (.85*ball['radius_px'])**2
    depth=frame['depth_mm'];mask &= np.isfinite(depth)&(depth>0)
    ys,xs=np.nonzero(mask)
    evidence={'method':'rgb_saved_table_and_34mm_ball','valid_samples':len(xs),'fit_residual_mm':None,
              'center_robot_mm':center.tolist(),'depth_used':False}
    if len(xs)<12:return center,evidence
    stride=max(1,len(xs)//1500);xs=xs[::stride];ys=ys[::stride]
    rays=np.c_[xs,ys,np.ones(len(xs))]@geometry['inverse_K'].T
    points=(rays*depth[ys,xs,None])@geometry['R'].T+geometry['t']
    normal=np.asarray(geometry['plane']['normal_up']);d=geometry['plane']['d_mm']
    heights=points@normal+d
    points=points[(heights>-8)&(heights<2*BALL_RADIUS+12)]
    if len(points)<12:return center,evidence
    estimate=center.copy()
    for _ in range(12):
        delta=estimate-points;distance=np.linalg.norm(delta,axis=1)
        valid=distance>1e-6;residual=distance[valid]-BALL_RADIUS
        jacobian=delta[valid]/distance[valid,None]
        weights=np.minimum(1.,4./np.maximum(np.abs(residual),1e-6))
        A=jacobian.T@(weights[:,None]*jacobian)+.5*np.outer(normal,normal)+.001*np.eye(3)
        b=jacobian.T@(weights*residual)+.5*normal*(estimate@normal+d-BALL_RADIUS)
        step=np.linalg.solve(A,b);length=np.linalg.norm(step)
        estimate-=step*min(1.,5./max(length,1e-6))
        if length<.01:break
    residual=float(np.median(np.abs(np.linalg.norm(points-estimate,axis=1)-BALL_RADIUS)))
    evidence.update(fit_samples=len(points),fit_residual_mm=residual,
                    fitted_center_robot_mm=estimate.tolist())
    if residual<=7 and abs(height_above_plane(estimate,geometry['plane'])-BALL_RADIUS)<=10:
        center=estimate;evidence.update(method='rgbd_known_radius_sphere_fit',depth_used=True,center_robot_mm=center.tolist())
    return center,evidence


def detect_funnel(image, geometry):
    """Find a dark, approximately circular opening; never use hole depth as rim Z."""
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    mask=cv2.inRange(gray,0,85)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    candidates=[]
    for contour in contours:
        area=cv2.contourArea(contour);perimeter=cv2.arcLength(contour,True)
        if area<80 or perimeter<=0:continue
        x,y,w,h=cv2.boundingRect(contour)
        if min(x,y)<2 or x+w>=image.shape[1]-2 or y+h>=image.shape[0]-2 or not .4<w/max(h,1)<2.5:continue
        circularity=4*math.pi*area/perimeter**2
        if circularity<.38:continue
        moments=cv2.moments(contour);pixel=[moments['m10']/moments['m00'],moments['m01']/moments['m00']]
        center,camera=pixel_at_height(pixel,FUNNEL_HEIGHT,geometry)
        expected=2*FUNNEL_RADIUS*math.sqrt(geometry['K'][0,0]*geometry['K'][1,1])/camera[2]
        diameter=2*math.sqrt(area/math.pi);ratio=diameter/expected
        if not .4<ratio<1.8:continue
        score=circularity*math.exp(-abs(math.log(ratio)))
        candidates.append({'pixel_center':pixel,'mouth_center_robot_mm':center.tolist(),
                           'contour':contour,'bbox_px':[x,y,w,h],'score':score,'observed_diameter_px':diameter,
                           'expected_diameter_px':expected,'radius_mm':FUNNEL_RADIUS,'height_mm':FUNNEL_HEIGHT})
    if not candidates:raise ValueError('当前没有找到黑色漏斗开口；请让开口完整出现在画面中后重新观察。')
    candidates.sort(key=lambda c:c['score'],reverse=True)
    selected=candidates[0];selected['candidate_count']=len(candidates)
    selected['ambiguous']=len(candidates)>1 and candidates[1]['score']>.8*selected['score']
    return selected


def observe_scene(folder, session):
    ball,geometry,image=observe_ball(folder,session)
    frame=load_rgbd(folder,frame_index=ball['frame_index'])
    center,depth=sphere_center(ball,frame,geometry)
    ball.update(center_robot_mm=center.tolist(),depth_evidence=depth)
    funnel=detect_funnel(image,geometry)
    contour=funnel.pop('contour')
    ring=np.zeros(image.shape[:2],np.uint8);cv2.drawContours(ring,[contour],-1,255,5)
    ys,xs=np.nonzero((ring>0)&(frame['depth_mm']>0))
    rim={'valid_samples':len(xs),'method':'rgb_opening_center_and_measured_45.4mm_height',
         'hole_depth_used_as_rim':False,'median_rim_height_mm':None}
    if len(xs):
        rays=np.c_[xs,ys,np.ones(len(xs))]@geometry['inverse_K'].T
        points=(rays*frame['depth_mm'][ys,xs,None])@geometry['R'].T+geometry['t']
        heights=points@np.asarray(geometry['plane']['normal_up'])+geometry['plane']['d_mm']
        plausible=heights[np.abs(heights-FUNNEL_HEIGHT)<12]
        if len(plausible)>=12:
            rim['median_rim_height_mm']=float(np.median(plausible));rim['rim_samples']=len(plausible)
    funnel['depth_evidence']=rim
    warnings=[w for w in ball.get('warnings',[]) if not (depth['depth_used'] and w.startswith('黄球表面深度稀疏'))]
    if not depth['depth_used']:warnings.append('球面有效深度不足或拟合不可靠；本轮明确使用保存桌面与34mm球径估计球心。')
    if funnel['ambiguous']:warnings.append('画面有多个相近的黑色圆形候选；请核对预览中标出的漏斗。')
    scene={'ball':ball,'funnel':funnel,'warnings':warnings,'source_trial_id':frame['trial_id'],
           'frame_index':frame['frame_index'],'host_monotonic_ns':frame['host_monotonic_ns'],
           'image_size':list(image.shape[1::-1]),'camera_geometry':{
               k:geometry[k].tolist() for k in ('K','inverse_K','R','t','normal_camera')},
           'd_camera':geometry['d_camera'],'table_plane':geometry['plane']}
    return scene,geometry,image,frame['depth_mm']


def _opening_direction(point,pitch,roll):
    base=math.atan2(point[1],point[0])
    return np.array([math.sin(pitch)*math.cos(base),math.sin(pitch)*math.sin(base),math.cos(pitch)])*math.cos(roll)+np.array([-math.sin(base),math.cos(base),0.])*math.sin(roll)


def build_plan(session,status,scene,parameters):
    """Plan the complete attempt using stock-finger offsets and existing path checks."""
    p=parameters;plane=scene['table_plane'];normal=np.asarray(plane['normal_up'])
    if set(p)!=set(DEFAULT_PARAMETERS):raise ValueError('RSI参数字段不完整')
    for key,value in p.items():
        values=value if isinstance(value,list) else [value]
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):raise ValueError(f'RSI参数{key}不是有限数值')
    if any(len(p[k])!=2 for k in ('pickup_xy_mm','release_xy_mm')):raise ValueError('XY补偿需为二维向量')
    if not .05<=p['approach_speed']<=1 or not .05<=p['travel_speed']<=1:raise ValueError('RSI速度参数需在0.05～1内')
    if not 1<=p['gripper_speed']<=4095 or not 1<=p['gripper_acc']<=254:raise ValueError('夹爪速度或加速度参数无效')
    if not 0<=p['squeeze_deg']<=5:raise ValueError('接触后夹角预紧参数需在0～5度内')
    if not math.degrees(1.08)<=p['open_deg']<p['close_deg']<=180:raise ValueError('夹爪开合参数超出官方范围或顺序无效')
    if not 0<=p['finger_offset_mm']<=50 or not -10<=p['grasp_z_offset_mm']<=15 or not 5<=p['release_clearance_mm']<=45:
        raise ValueError('本轮高度或指尖偏移超出实验参数搜索域')
    if not 40<=p['hover_height_mm']<=150 or not 80<=p['lift_height_mm']<=180:raise ValueError('抓放过渡高度参数无效')
    if math.dist(forward(status)['xyz_mm'],[status[k] for k in 'xyz'])>3:
        raise ValueError('当前固件XYZ与原装夹爪FK不一致')
    hold=gripper_hold(status);initial={**status,'g':hold['command_rad']}
    opening=math.radians(p['open_deg']);closing=min(math.radians(p['close_deg']),GRIPPER_MAX_RAD)
    ball=np.asarray(scene['ball']['center_robot_mm']);mouth=np.asarray(scene['funnel']['mouth_center_robot_mm'])
    from .rsi_route import build_route
    failures=[]
    for pitch_deg in (80.,75.,85.,70.,90.,65.,60.):
        pitch=math.radians(pitch_deg);roll=status['r']
        grasp_center=ball+normal*p['grasp_z_offset_mm']
        release_center=mouth+normal*(BALL_RADIUS+p['release_clearance_mm'])
        grasp=grasp_center-_opening_direction(ball,pitch,roll)*p['finger_offset_mm']
        release_candidates=[]
        for angle in dict.fromkeys((pitch_deg,80.,75.,70.,65.,60.,50.,40.,30.,20.,10.,0.)):
            drop_pitch=math.radians(angle)
            tip=release_center-_opening_direction(mouth,drop_pitch,roll)*p['finger_offset_mm']
            tip[:2]+=p['release_xy_mm']
            release_candidates.append({'pitch_rad':drop_pitch,'xyz_mm':tip.tolist()})
        release=release_candidates[0]['xyz_mm']
        grasp[:2]+=p['pickup_xy_mm']
        try:
            steps,route=build_route(initial,grasp,release,pitch,plane,p,
                ball_top_height=height_above_plane(ball,plane)+BALL_RADIUS,funnel_height=FUNNEL_HEIGHT,ball_radius=BALL_RADIUS,
                release_candidates=release_candidates)
            release=route['release_tip_xyz_mm']
            return {'kind':'rsi_pick_place_v1','setup_id':session['setup_id'],'initial_status':status,
                    'scene':scene,'table_plane':plane,'steps':steps,'parameters':p,'route_policy':route,
                    'object_geometry':{'ball_diameter_mm':2*BALL_RADIUS,'ball_top_xyz_mm':(ball+normal*BALL_RADIUS).tolist(),
                        'grasp_center_xyz_mm':(grasp_center+np.array([*p['pickup_xy_mm'],0.])).tolist(),
                        'release_ball_center_xyz_mm':(release_center+np.array([*p['release_xy_mm'],0.])).tolist(),
                        'finger_offset_mm':p['finger_offset_mm'],'reference':'stock_fixed_fingertip_is_not_ball_center'},
                    'gripper_hold':hold,'grasp_tip_xyz_mm':grasp.tolist(),'release_tip_xyz_mm':list(release),
                    'grasp_pitch_deg':pitch_deg,'release_pitch_deg':math.degrees(route['release_pitch_rad']),
                    'target_duration_s':10,'board_required':False,
                    'camera_frame':'unmirrored_rgb','warnings':scene['warnings'],
                    'timing_verified':False,'pose_reference':'stock_firmware_fixed_fingertip'}
        except ValueError as exc:
            detail=getattr(exc,'detail',{'message':str(exc)})
            if detail not in failures:failures.append(detail)
    raise MotionConstraintError('完整抓放候选路径检查未通过：'+'；'.join(f['message'] for f in failures[:1]),
        code='pick_place_search_exhausted',attempts=[a for f in failures for a in f.get('attempts',[f])][:64],
        grasp_pitch_candidates_deg=[80.,75.,85.,70.,90.,65.,60.])


def preview(image,plan,geometry):
    result=image.copy()
    def pixel(point):
        camera=geometry['R'].T@(np.asarray(point)-geometry['t']);uv=geometry['K']@camera
        return tuple(np.round(uv[:2]/uv[2]).astype(int))
    ball=tuple(np.round(plan['scene']['ball']['pixel_center']).astype(int))
    funnel=tuple(np.round(plan['scene']['funnel']['pixel_center']).astype(int))
    cv2.circle(result,ball,22,(0,210,255),2);cv2.circle(result,funnel,40,(220,100,30),2)
    cv2.putText(result,'BALL',ball,cv2.FONT_HERSHEY_SIMPLEX,.5,(0,180,220),2)
    cv2.putText(result,'FUNNEL',funnel,cv2.FONT_HERSHEY_SIMPLEX,.5,(220,100,30),2)
    previous=pixel([plan['initial_status'][k] for k in 'xyz'])
    for index,step in enumerate(plan['steps']):
        if step['kind']!='cartesian':continue
        point=pixel(step['xyz_mm']);cv2.line(result,previous,point,(40,160,40),2)
        cv2.circle(result,point,4,(30,80,230),-1)
        cv2.putText(result,str(index+1),point,cv2.FONT_HERSHEY_SIMPLEX,.4,(30,30,220),1);previous=point
    return result


def image_correction(scene,direction,step_mm=8.,*,target='ball',center_robot_mm=None):
    if direction not in PIXEL_VECTOR:return [0.,0.]
    geometry=scene['camera_geometry'];R=np.asarray(geometry['R'],float);K=np.asarray(geometry['K'],float)
    field='center_robot_mm' if target=='ball' else 'mouth_center_robot_mm'
    center=np.asarray(center_robot_mm if center_robot_mm is not None else scene[target][field],float)
    camera=R.T@(center-np.asarray(geometry['t'],float));uv=K@camera
    if uv[2]<=0:raise ValueError('当前补偿参考点位于相机后方')
    derivative=np.vstack([(K[i]*uv[2]-uv[i]*K[2])/uv[2]**2 for i in (0,1)])
    jacobian=derivative@R.T[:,:2]
    if not np.all(np.isfinite(jacobian)) or np.linalg.cond(jacobian)>1e8:raise ValueError('当前相机到机器人XY方向映射退化')
    delta=np.linalg.solve(jacobian,np.asarray(PIXEL_VECTOR[OPPOSITE[direction]],float))
    return (delta/np.linalg.norm(delta)*step_mm).tolist()


def adapt(parameters,scene,outcome,direction,magnitude='medium',adjustment='unknown',*,step_override_mm=None):
    result={k:list(v) if isinstance(v,list) else v for k,v in parameters.items()};change=[]
    step_mm=XY_STEPS_MM.get(magnitude,8.)
    if step_override_mm is not None:
        if type(step_override_mm) not in (int,float) or not math.isfinite(step_override_mm) or step_override_mm<=0:
            raise ValueError('学习试验步长无效')
        step_mm=step_override_mm
    if outcome=='placed':return result,['本轮成功，保留参数；下一轮换一个球位置验证。']
    if outcome in ('grasp_missed','placement_missed') and direction.startswith('image_'):
        key='pickup_xy_mm' if outcome=='grasp_missed' else 'release_xy_mm'
        target='ball' if outcome=='grasp_missed' else 'funnel'
        center=np.asarray(scene[target]['center_robot_mm' if target=='ball' else 'mouth_center_robot_mm'],float).copy()
        height=result['grasp_z_offset_mm'] if target=='ball' else BALL_RADIUS+result['release_clearance_mm']
        center+=np.asarray(scene['table_plane']['normal_up'])*height;center[:2]+=result[key]
        delta=image_correction(scene,direction,step_mm,target=target,center_robot_mm=center)
        result[key]=[v+d for v,d in zip(result[key],delta)]
        change.append(f'{key}按Grok判断的方向和幅度调整{step_mm:g}mm；数值是试验步长，不是Grok测得的绝对误差。')
    elif outcome=='grasp_missed' and direction in ('too_high','too_low'):
        delta=-2 if direction=='too_high' else 2
        result['grasp_z_offset_mm']=max(-10.,min(15.,result['grasp_z_offset_mm']+delta))
        change.append(f'下一轮抓取高度调整{delta:+d}mm。')
    elif outcome=='grasp_missed' and direction=='none':
        result['grasp_z_offset_mm']=max(-10.,result['grasp_z_offset_mm']-2)
        change.append('Grok确认抓空但方向不清，下一轮只探索降低抓取高度2mm；不把探索步长当作测量误差。')
    elif outcome=='pushed_away':
        result['grasp_z_offset_mm']=min(15.,result['grasp_z_offset_mm']+2)
        change.append('下降时推走了球，下一轮先提高抓取高度2mm，再用录像验证。')
    elif outcome=='slipped':
        if adjustment=='grip_tighten' and result['squeeze_deg']<5:
            result['squeeze_deg']=min(5.,result['squeeze_deg']+1)
            change.append('Grok建议加强夹持，下一轮接触后角度预紧增加1°；这是位置控制试验，不是测得的夹持力。')
        else:
            result['travel_speed']=max(.2,round(result['travel_speed']*.85,3))
            change.append('发生滑落，下一轮快速移动速度降低15%，先隔离运动造成的滑落。')
    elif outcome=='bounced_out':
        result['release_clearance_mm']=max(5.,result['release_clearance_mm']-4)
        change.append('下一轮降低投放高度4mm，保留漏斗口上方间隙。')
    elif outcome=='placement_missed' and direction in ('too_high','too_low'):
        delta=-4 if direction=='too_high' else 4
        result['release_clearance_mm']=max(5.,min(45.,result['release_clearance_mm']+delta))
        change.append(f'下一轮投放高度调整{delta:+d}mm。')
    else:change.append('证据不足或方向不明确，保留参数，重新观察后再试。')
    return result,change
