"""Explain task readiness without confusing torque, user scope and arming."""
import math


AUTHORIZATION_CONTRACT={
    'user_task_request_required':True,
    'temporary_arm_requested_by':'POST /api/bench/rsi/run',
    'armed_false_blocks_preparation':False,
    'manual_unlock_button_required':False,
}


def describe(health,raw,error=None):
    reasons=[]
    if not health.get('connected'):reasons.append('控制服务当前未连接机械臂')
    if error:reasons.append('本次反馈读取失败：'+error)
    valid=bool(raw and raw.get('T')==1051 and all(type(raw.get(k)) in (int,float) and math.isfinite(raw[k]) for k in ('x','y','z','tit','b','s','e','t','r','g')))
    if not valid:
        reasons.append('本次缺少有效的新鲜机器人反馈')
    if not raw or any(raw.get('torswitch'+k)!=1 for k in 'BSETRG'):reasons.append('本次六关节锁定反馈未全部确认')
    if health.get('teach_active'):reasons.append('当前仍处于释放扭矩的示教状态')
    if health.get('fault'):reasons.append('控制器有待处理故障：'+str(health['fault'].get('message','')))
    if health.get('pending') or health.get('automation_owner'):reasons.append('当前动作或自动任务尚未结束')
    execution=list(reasons)
    if not health.get('motion_enabled'):execution.append('网页服务处于只读模式，motion_enabled=false')
    if set(health.get('allowed_joints',[]))!=set(range(1,7)):execution.append('执行抓放所需的六关节尚未全部启用')
    return {'ready_to_prepare':not reasons,'execution_enabled':not execution,
        'blocking_reasons':reasons,'execution_blocking_reasons':execution,
        'feedback_verified':valid and error is None,
        'authorization_contract':dict(AUTHORIZATION_CONTRACT),
        'instruction':'armed=false是等待执行时的正常状态。已有用户任务授权时，prepare和run会完成本轮检查与临时授权；不要要求用户寻找网页解锁按钮。'}
