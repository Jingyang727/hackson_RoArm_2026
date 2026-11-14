"""Quarantine legacy per-axis-clipped XY learning without rewriting trials."""
from __future__ import annotations

import copy
import math
import time
import uuid

from .pick_place import DEFAULT_PARAMETERS


def repair_clipped_policy(policy):
    """Return a repaired copy plus provenance; leave unrelated policies untouched."""
    reviews=policy.get('reviews',[])
    if not reviews:return policy,None
    last=reviews[-1];proposal=last.get('proposal',{});step=proposal.get('step_mm')
    if last.get('outcome')=='placed' or not isinstance(step,(int,float)) or step<=0:return policy,None
    # A repair is applicable only to the exact policy produced by this review.
    expected={**DEFAULT_PARAMETERS,**last.get('after',{})}
    if policy['parameters']!=expected:return policy,None
    affected=[]
    for key in ('pickup_xy_mm','release_xy_mm'):
        before=last.get('before',{}).get(key);after=last.get('after',{}).get(key)
        if not before or not after or len(before)!=2 or len(after)!=2:continue
        saturated=any(abs(b)==30 and a==b for a,b in zip(before,after))
        if saturated and math.dist(before,after)<step*.9:affected.append(key)
    if not affected:return policy,None
    repaired=copy.deepcopy(policy);before=copy.deepcopy(policy['parameters'])
    # Only a success actually reviewed as placed can provide a trusted baseline.
    success=policy.get('successful_policy',{})
    successful=next((r for r in reviews if r['plan_id']==success.get('plan_id') and r.get('outcome')=='placed'
                     and r.get('after')==success.get('parameters')),None)
    baseline=success['parameters'] if successful else DEFAULT_PARAMETERS
    for key in affected:repaired['parameters'][key]=copy.deepcopy(baseline[key])
    record={'id':uuid.uuid4().hex,'kind':'legacy_xy_vector_repair','at_unix':time.time(),
            'source_trial_id':last['plan_id'],'before_version':policy['version'],'after_version':policy['version']+1,
            'before':before,'after':copy.deepcopy(repaired['parameters']),'parameters_reset':affected,
            'baseline_success_trial_id':success.get('plan_id') if successful else None,
            'reason':'旧版±30mm逐轴裁剪破坏了申请步长和方向；隔离该XY补偿，保留原始试次与其他参数。'}
    repaired['version']=record['after_version'];repaired.setdefault('baseline_resets',[]).append(record)
    return repaired,record
