"""Raw recorded targets and feedback; no sampling, motion or re-calibration."""
import math

from .rsi_comparison import feedback_pose


def xyz(raw):
    if raw and all(type(raw.get(k)) in (int,float) and math.isfinite(raw[k]) for k in 'xyz'):
        return [raw[k] for k in 'xyz']


def feedback(state,*stages):
    for stage in stages:
        for row in reversed(state.get('result',{}).get('feedback',[])):
            if row.get('stage')==stage:
                value=xyz(row.get('status'))
                if value is not None:return value


def command(state,stage):
    for row in reversed(state.get('result',{}).get('dispatches',[])):
        if row.get('stage')==stage and row.get('command',{}).get('T')==104:return xyz(row['command'])


def delta(current,previous):
    return [a-b for a,b in zip(current,previous)] if current is not None and previous is not None else None


def audit(states):
    rows=[]
    for state in states:
        plan=state['plan'];review=state.get('review') or {};used=plan['parameters']['pickup_xy_mm']
        row={'plan_id':state['id'],'policy_version':state.get('policy_version'),'status':state.get('status'),
            'used_pickup_xy_mm':used,'saved_for_next_pickup_xy_mm':review.get('after',{}).get('pickup_xy_mm'),
            'ball_xyz_mm':plan['scene']['ball']['center_robot_mm'],'grasp_pitch_deg':plan['grasp_pitch_deg'],
            'nominal_grasp_xyz_mm':plan['grasp_tip_xyz_mm'],
            'approach_feedback_xyz_mm':feedback(state,'align_above_ball','approach_ball'),
            'descend_command_xyz_mm':command(state,'descend'),'descend_feedback_xyz_mm':feedback(state,'descend'),
            'close_feedback_xyz_mm':feedback_pose(state),
            'direction':review.get('direction'),'decision':review.get('proposal',{}).get('decision')}
        row['close_minus_descend_command_mm']=delta(row['close_feedback_xyz_mm'],row['descend_command_xyz_mm'])
        row['descend_minus_nominal_grasp_mm']=delta(row['descend_command_xyz_mm'],row['nominal_grasp_xyz_mm'])
        if rows:
            previous=rows[-1]
            applied=delta(used,previous['used_pickup_xy_mm'])
            planned=delta(row['nominal_grasp_xyz_mm'],previous['nominal_grasp_xyz_mm'])
            raw=delta(row['close_feedback_xyz_mm'],previous['close_feedback_xyz_mm'])
            baseline=[planned[i]-applied[i] for i in range(2)]
            row['versus_previous']={'plan_id':previous['plan_id'],'used_compensation_delta_mm':applied,
                'nominal_grasp_delta_xyz_mm':planned,'raw_close_delta_xyz_mm':raw,
                'localization_and_pose_baseline_delta_mm':baseline,
                'baseline_adjusted_feedback_delta_mm':[raw[i]-baseline[i] for i in range(2)] if raw is not None else None}
        rows.append(row)
    return rows
