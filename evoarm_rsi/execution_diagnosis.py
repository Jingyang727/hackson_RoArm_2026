"""Measured execution failures, separate from qualitative grasp judgement."""
import math

from .roarm_geometry import height_above_plane


def stage_timings(result):
    events=result.get('events') or []
    start=next((e['host_monotonic_ns'] for e in events if e.get('stage')=='start'),None)
    if start is None:return []
    rows=[];opened={}
    for event in events:
        name=event['stage'];elapsed=round((event['host_monotonic_ns']-start)/1e9,3)
        if name.endswith('_completed'):
            row=opened.get(name[:-10])
            if row is not None:row.update(completed_s=elapsed,duration_s=round(elapsed-row['started_s'],3))
        elif name not in ('start','finish','fault'):
            row={'stage':name,'started_s':elapsed};rows.append(row);opened[name]=row
    return rows


def diagnose(result,plan):
    error=result.get('motion_error') or result.get('camera_error');failure=result.get('failure') or {}
    plane=plan.get('table_plane') or plan.get('scene',{}).get('table_plane')
    lift=next((f.get('status') for f in reversed(result.get('feedback',[])) if f.get('stage')=='lift'),None)
    preflight=result.get('last_preflight',{})
    if preflight.get('stage')=='transport':lift=preflight.get('status') or lift
    command=next((d['command'] for d in reversed(result.get('dispatches',[])) if d.get('stage')=='lift' and d.get('command',{}).get('T')==104),None)
    def height(raw):
        if not plane or not raw or any(type(raw.get(k)) not in (int,float) or not math.isfinite(raw[k]) for k in 'xyz'):return None
        return height_above_plane([raw[k] for k in 'xyz'],plane)
    actual=height(lift);target=height(command)
    floor=plan.get('route_policy',{}).get('minimum_transfer_height_mm')
    category=('complete' if result.get('motion_completed') and not error else
        'not_started' if not error and not result.get('events') and not result.get('dispatches') else
        'time_budget' if error and ('10秒录像窗口' in error or '录像已结束' in error) else
        'clearance_shortfall' if (failure.get('code')=='tip_clearance' and failure.get('progress')==0 or error and '路径0.0%' in error and '指尖低于' in error) and actual is not None and floor is not None and actual<floor else
        'camera_error' if result.get('camera_error') and not result.get('motion_error') else 'execution_error')
    options={'time_budget':['travel_speed','approach_speed'],'clearance_shortfall':['lift_height_mm']}.get(category,[])
    value={'category':category,'error':error,'failed_stage':failure.get('stage'),
        'grasp_attempted':bool(result.get('grasp_attempted')),'placement_attempted':bool(result.get('placement_attempted')),
        'stage_timings':stage_timings(result),'execution_trial_parameters':options,
        'lift':{'target_xyz_mm':[command[k] for k in 'xyz'] if command else None,
                'actual_xyz_mm':[lift[k] for k in 'xyz'] if actual is not None else None,
                'target_height_mm':target,'actual_height_mm':actual,'required_transfer_height_mm':floor,
                'height_reference':'distance_above_calibrated_table_not_robot_z'}}
    events=result.get('events') or []
    started=next((e['host_monotonic_ns'] for e in events if e.get('stage')=='start'),None)
    value['elapsed_s']=round((events[-1]['host_monotonic_ns']-started)/1e9,3) if started is not None else None
    value['target_duration_s']=10
    if target is not None and actual is not None and floor is not None:
        value['lift']['undershoot_mm']=max(0.,target-actual)
        if category=='clearance_shortfall':
            value['suggested_trial']={'parameter':'lift_height_mm','value':round(max(target,floor+max(0.,target-actual)+10),1),
                'meaning':'single measured execution trial; full parameter bounds and IK still required'}
    value['retry_instruction']=('本轮尚未执行，计划不是执行结果。' if category=='not_started' else
        '使用新的合并接近路径；仍超时时只选择一个速度试验并保存，不重复同策略。' if category=='time_budget' else
        '不降低横移间隙；按实测抬升不足选择一次抬升高度试验，先检查完整IK。' if category=='clearance_shortfall' else
        '先保存视觉复核与决策；未解决的执行故障不原样重试。')
    return value
