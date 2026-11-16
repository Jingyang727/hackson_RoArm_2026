"""Bounded explanations of candidate search failures, not physical limits."""
from collections import Counter

STAGES={
    'raise_current':('起步抬升',0),'clearance_adjust':('起步高度调整',0),'orient_clear':('起步转腕',1),
    'approach_ball':('接近球上方',2),'align_above_ball':('球上方对齐',3),'descend':('抓取下探',4),
    'lift':('抓后抬升',5),'orient_transport':('运输转腕',6),'transit_arc':('高处绕行',7),
    'transport':('运输到漏斗',8),'lower_to_release':('到漏斗投放点',9),
}
CODES={'ik_workspace','joint_limit','tip_clearance','link_clearance','base_branch_jump',
       'route_search_exhausted','pick_place_search_exhausted'}


def search_summary(failures):
    counts=Counter();examples={}
    for row in failures:
        counts.update(row.get('stage_failures') or {row.get('stage','unknown'):1})
        for stage,example in row.get('stage_examples',{}).items():examples.setdefault(stage,example)
        if row.get('stage') in STAGES:examples.setdefault(row['stage'],{k:row[k] for k in
            ('stage','message','xyz_mm','pitch_deg','joint','required_deg','min_deg','max_deg','progress','code') if k in row})
    known=[s for s in counts if s in STAGES]
    return {'stage_failures':dict(counts),
            'furthest_stage':max(known,key=lambda s:STAGES[s][1]) if known else None,'stage_examples':examples}


def explain(detail):
    if not isinstance(detail,dict) or detail.get('code') not in CODES:return None
    attempts=detail.get('attempts') or ([detail] if detail.get('stage') else [])
    summary=search_summary(attempts)
    stage=detail.get('furthest_stage') or summary['furthest_stage']
    matching=[row for row in attempts if row.get('stage')==stage]
    evidence=(detail.get('stage_examples') or summary['stage_examples']).get(stage) or (matching[0] if matching else {})
    label=STAGES.get(stage,('完整抓放路径',0))[0]
    target=('ball' if stage in ('approach_ball','align_above_ball','descend','lift') else
            'funnel' if stage=='lower_to_release' else 'start' if stage in ('raise_current','clearance_adjust','orient_clear') else 'route')
    suggestion={
        'ball':'可在工作区预览中选择另一处放球；抓取高度、俯仰与抬升高度都会影响本轮可规划区域。',
        'funnel':'检查漏斗投放点是否过远；仅移动黄球未必能解决投放点的姿态或关节约束。',
        'start':'检查起步姿态；从已确认的官方初始姿态重新规划后，仍需整条路径通过检查。',
        'route':'检查高处运输通道和漏斗位置；抓球点可达仍需运输及投放路径可规划。',
    }[target]
    return {'code':detail['code'],'stage':stage,'stage_label':label,'target':target,
            'summary':f'当前抓放候选路径未通过：{label}',
            'reason':evidence.get('message') or detail.get('message','未找到通过检查的候选路径'),
            'suggestion':suggestion,'stage_failures':detail.get('stage_failures') or summary['stage_failures'],
            'image_url':detail.get('image_url'),'draft_id':detail.get('draft_id'),
            'evidence':{k:evidence[k] for k in ('xyz_mm','pitch_deg','joint','required_deg','min_deg','max_deg','progress','code') if k in evidence}}
