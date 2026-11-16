"""Measured-table release clearance, independent of learned pickup offsets."""
import copy
import time
import uuid

from .roarm_geometry import height_above_plane,MotionConstraintError
from .funnel_geometry import HEIGHT_MM,RADIUS_MM

DEFAULT_RELEASE_CLEARANCE_MM=100.  # Ball bottom above lip, not robot Z.
MIN_RELEASE_CLEARANCE_MM=80.
MAX_RELEASE_CLEARANCE_MM=160.
MIN_VERIFIED_TIP_CLEARANCE_MM=60.
RELEASE_REFERENCE_VERSION=2
RELEASE_SIDE_OFFSET_MM=RADIUS_MM/2.  # User's half-mouth-radius reference: 19 mm.


def release_reference(scene):
    plane=scene['table_plane'];mouth=scene['funnel']['mouth_center_robot_mm']
    measured=height_above_plane(mouth,plane);height=max(HEIGHT_MM,measured)
    return [v+n*(height-measured) for v,n in zip(mouth,plane['normal_up'])],height


def verify_release(status,plan):
    policy=plan.get('release_policy')
    if not policy:raise MotionConstraintError('缺少高处投放规则，请重新规划',code='release_policy_missing')
    actual=height_above_plane([status[k] for k in 'xyz'],plan['table_plane'])
    minimum=policy['minimum_tip_height_mm']
    evidence={'actual_tip_xyz_mm':[status[k] for k in 'xyz'],'actual_tip_height_mm':actual,
              'minimum_tip_height_mm':minimum,'lip_height_mm':policy['lip_height_mm'],
              'actual_tip_clearance_mm':actual-policy['lip_height_mm'],'verified':actual>=minimum}
    if not evidence['verified']:
        raise MotionConstraintError('投放前实测指尖离口沿不足，未发送开爪；下一轮提高投放高度',
                                    code='release_clearance',**evidence)
    return evidence


def upgrade_release_policy(policy):
    old=policy['parameters']['release_clearance_mm']
    if old>=MIN_RELEASE_CLEARANCE_MM:return policy,None
    updated=copy.deepcopy(policy);before=copy.deepcopy(policy['parameters'])
    updated['parameters']['release_clearance_mm']=DEFAULT_RELEASE_CLEARANCE_MM
    record={'id':uuid.uuid4().hex,'kind':'high_release_clearance','at_unix':time.time(),
            'source_trial_id':policy.get('reviews',[])[-1]['plan_id'] if policy.get('reviews') else None,
            'before_version':policy['version'],'after_version':policy['version']+1,
            'before':before,'after':copy.deepcopy(updated['parameters']),
            'parameters_reset':['release_clearance_mm'],
            'reason':'旧投放间隙过低；球底投放高度改为口沿上方100mm，并在开爪前核对实测指尖间隙。'}
    updated['version']=record['after_version'];updated.setdefault('baseline_resets',[]).append(record)
    return updated,record


def upgrade_release_reference(policy):
    """Do not stack learned offsets from the old radial endpoint reference."""
    if policy.get('release_reference_version')==RELEASE_REFERENCE_VERSION:return policy,None
    updated=copy.deepcopy(policy);before=copy.deepcopy(policy['parameters'])
    updated['release_reference_version']=RELEASE_REFERENCE_VERSION
    updated['parameters']['release_xy_mm']=[0.,0.]
    if 'successful_policy' in updated:
        updated['successful_policy_before_release_reference']=updated.pop('successful_policy')
    record={'id':uuid.uuid4().hex,'kind':'one_sided_release_reference','at_unix':time.time(),
            'source_trial_id':policy.get('reviews',[])[-1]['plan_id'] if policy.get('reviews') else None,
            'before_version':policy['version'],'after_version':policy['version']+1,
            'before':before,'after':copy.deepcopy(updated['parameters']),
            'parameters_reset':['release_xy_mm'],
            'reason':'投放端点改为原始相机画面向右偏19mm；旧径向参考的投放XY补偿清零，抓取参数和原始历史保留。'}
    updated['version']=record['after_version'];updated.setdefault('baseline_resets',[]).append(record)
    return updated,record
