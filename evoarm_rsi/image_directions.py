"""Camera-image semantics shared by visual reviewers, agents and controllers."""
import numpy as np

OPPOSITE={'image_left':'image_right','image_right':'image_left','image_up':'image_down','image_down':'image_up',
          'too_high':'lower','too_low':'higher'}
PIXEL_VECTOR={'image_left':[-1,0],'image_right':[1,0],'image_up':[0,-1],'image_down':[0,1]}
CONTRACT={'frame':'unmirrored_rgb','u_positive':'image_right','v_positive':'image_down',
          'view_transform_owner':'opencode_primary_agent_with_calibrated_tools','execution_frame':'robot_firmware_mm',
          'observed_side_to_correction':{k:OPPOSITE[k] for k in PIXEL_VECTOR},
          'note':'Grok只按画面观察；主Agent负责相机视角到机械臂视角的转换，并用标定工具落实XYZ。原图纠偏效果必须符合视觉建议，已转换的XYZ原值用于执行。'}


def robot_xy_for_image_motion(scene,point,direction,distance_mm):
    """Desired image motion, as distinct from the observer's error direction."""
    geometry=scene['camera_geometry'];R=np.asarray(geometry['R'],float);K=np.asarray(geometry['K'],float)
    camera=R.T@(np.asarray(point,float)-np.asarray(geometry['t'],float));uv=K@camera
    if uv[2]<=0:raise ValueError('当前补偿参考点位于相机后方')
    derivative=np.vstack([(K[i]*uv[2]-uv[i]*K[2])/uv[2]**2 for i in (0,1)])
    jacobian=derivative@R.T[:,:2]
    if not np.all(np.isfinite(jacobian)) or np.linalg.cond(jacobian)>1e8:raise ValueError('当前相机到机器人XY方向映射退化')
    delta=np.linalg.solve(jacobian,np.asarray(PIXEL_VECTOR[direction],float))
    return (delta/np.linalg.norm(delta)*distance_mm).tolist()
