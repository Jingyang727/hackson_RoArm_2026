"""Keep completed pickup evidence usable when a later stage runs out of time."""
import copy


def basis(result,diagnosis,*,outcome,ball_state):
    category=diagnosis['category']
    completed={e['stage'][:-10] for e in result.get('events',[]) if e.get('stage','').endswith('_completed')}
    late=diagnosis.get('failed_stage') in {'transport','transit_arc','orient_transport','lower_to_release','release','finish'}
    pickup_visible=(result.get('grasp_attempted') and {'close','lift'}<=completed
        and result.get('recording_covers_motion') and not result.get('camera_error'))
    if category=='time_budget' and late and pickup_visible and outcome=='grasp_missed' and ball_state=='table':
        return {'scope':'visual_pickup','allow_visual':True,'execution_issue_pending':True,
            'reason':'闭爪和抬升已完成，录像确认抓空；后段超时不撤销前段证据，本轮先修正抓取偏差，10秒执行要求仍保留。'}
    if category=='time_budget' and late and pickup_visible and outcome=='slipped' and ball_state=='table':
        return {'scope':'visual_grip','allow_visual':True,'execution_issue_pending':True,
            'reason':'闭爪和抬升已完成，录像确认球滑落；先修正夹持或运输策略，不因为后段超时而盲目加速。'}
    if category in ('time_budget','clearance_shortfall','release_clearance_shortfall'):
        return {'scope':'execution','allow_visual':False,'execution_issue_pending':True,
            'reason':'本轮先按实际阶段耗时选择单项速度试验。' if category=='time_budget' else
                     '投放前实际指尖过低，下一轮提高投放高度，不降低开爪间隙。' if category=='release_clearance_shortfall' else
                     '实际运输间隙不足，本轮先按反馈选择抬升高度试验，不降低间隙要求。'}
    return {'scope':'visual','allow_visual':True,'execution_issue_pending':False,
            'reason':'按本轮实际画面选择单项抓取或投放修正；没有可比前轮不妨碍首次试验。'}


def execution_proposal(parameters,diagnosis):
    """Web controller trials are measured experiments, never Grok measurements."""
    category=diagnosis['category'];name=None;value=None
    if category=='time_budget':
        for candidate,step in (('travel_speed',.2),('approach_speed',.1)):
            if parameters[candidate]<1:
                name=candidate;value=min(1.,round(parameters[candidate]+step,3));break
    elif category=='clearance_shortfall':
        trial=diagnosis.get('suggested_trial') or {}
        if trial.get('parameter')=='lift_height_mm' and 80<=trial['value']<=180:
            name='lift_height_mm';value=trial['value']
    elif category=='release_clearance_shortfall':
        from .release_geometry import MIN_RELEASE_CLEARANCE_MM,MAX_RELEASE_CLEARANCE_MM
        trial=diagnosis.get('suggested_trial') or {}
        if trial.get('parameter')=='release_clearance_mm' and MIN_RELEASE_CLEARANCE_MM<=trial['value']<=MAX_RELEASE_CLEARANCE_MM:
            name='release_clearance_mm';value=trial['value']
    after=copy.deepcopy(parameters)
    if name is not None:after[name]=value
    changed=name is not None and value!=parameters[name]
    reason=(f'网页控制器依据执行诊断选择单项试验：{name}从{parameters[name]:g}改为{value:g}；这是待验证的试验值。' if changed else
            '当前速度已到参数上限，仍未在10秒内完成；需要进一步优化路径，未重复原策略。' if category=='time_budget' else
            '缺少有效反馈高度，或建议抬升高度超出当前试验范围；未猜测高度或降低运输间隙。')
    proposal={'status':'applied' if changed else 'unchanged','decision':'execution_parameter_trial',
        'parameters_changed':[name] if changed else [],'before':{name:parameters[name]} if changed else {},
        'after':{name:value} if changed else {},'step_mm':None,'allow_retry':changed,'reason':reason}
    return after,[reason],proposal
