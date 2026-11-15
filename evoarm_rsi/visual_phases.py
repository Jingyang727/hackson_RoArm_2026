"""Independent visual phase facts. Motor acknowledgements are not evidence."""

RESULTS={
    'pickup':{'held','missed','unknown'},
    'transport':{'held','dropped','unknown'},
    'release':{'entered','missed','bounced_out','unknown'},
}


def validate_phases(value):
    if not isinstance(value,dict) or set(value)!=set(RESULTS):
        raise ValueError('视觉阶段证据需分别包含抓起、运输和投放')
    result={}
    for phase,allowed in RESULTS.items():
        fact=value[phase]
        if (not isinstance(fact,dict) or set(fact)!={'result','frames','note'}
                or not isinstance(fact['result'],str) or fact['result'] not in allowed
                or not isinstance(fact['frames'],list)
                or any(type(i) is not int or i<0 for i in fact['frames'])
                or len(fact['frames'])>12 or len(set(fact['frames']))!=len(fact['frames'])
                or not isinstance(fact['note'],str) or len(fact['note'])>1000
                or fact['result']!='unknown' and not fact['frames']):
            raise ValueError('视觉阶段结论缺少有效实帧依据')
        result[phase]=dict(fact)
    if result['pickup']['result']=='missed' and (
            result['transport']['result'] in ('held','dropped') or result['release']['result'] in ('entered','bounced_out')):
        raise ValueError('视觉阶段证据互相矛盾：未抓起与持球/入漏斗不能同时成立')
    return result


def resolve_outcome(value):
    """A later table ball cannot overwrite an earlier visibly held ball."""
    result=dict(value);facts=value['phases']
    pickup=facts['pickup']['result'];transport=facts['transport']['result'];release=facts['release']['result']
    reported=value['outcome'];outcome=reported
    held=pickup=='held' or transport=='held'
    if release=='entered':
        if value['ball_state']!='funnel':raise ValueError('入漏斗阶段与最终球状态矛盾')
        outcome='placed'
    elif release=='bounced_out':outcome='bounced_out'
    elif transport=='dropped':outcome='slipped'
    elif held and release=='missed':outcome='placement_missed'
    elif pickup=='missed':outcome='pushed_away' if reported=='pushed_away' else 'grasp_missed'
    elif held and reported in ('grasp_missed','pushed_away'):outcome='unknown'
    elif reported=='placed':outcome='unknown'  # Requires an independent entry fact.
    elif reported=='grasp_missed' and pickup!='missed':outcome='unknown'
    elif reported=='slipped' and transport!='dropped':outcome='unknown'
    elif reported=='placement_missed' and not held:outcome='unknown'
    if outcome!=reported:
        result['reported_outcome']=reported
        result['outcome']=outcome
        # A pickup alignment direction must never become a release correction.
        result.update(direction='none',magnitude='unknown',adjustment='unknown')
    return result


def bind_phase_frames(value,frames):
    if 'phases' not in value:return  # Legacy saved/Agent verdicts remain readable.
    current={f['frame_index'] for f in frames if f.get('role','current')=='current' and type(f.get('frame_index')) is int}
    if any(set(fact['frames'])-current for fact in value['phases'].values()):
        raise ValueError('视觉阶段引用了本次未提供的实帧，未采用判断')
