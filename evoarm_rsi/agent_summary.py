"""Bounded CLI views; raw receipts stay in archives and behind --full."""
from .execution_diagnosis import diagnose


def pick(value,keys):return {k:value.get(k) for k in keys if k in value}


def review(value):
    if not value:return None
    out=pick(value,('plan_id','outcome','source','note','changes','before_version','after_version','proposal','visual_review','execution_diagnosis'))
    if out.get('note'):out['note']=out['note'][:600]
    return out


def health(value):
    out=pick(value,('connected','motion_enabled','teach_active','armed','arm_seconds_left','torque_status','fault','automation_owner','port'))
    pending=value.get('pending');out['pending']=pick(pending,('kind','stage_kind','stage','run_id','index')) if pending else None
    last=value.get('last_command');out['last_command']=pick(last,('status','rsi_index','rsi_run_id','command','error_deg')) if last else None
    return out


def scene(value):
    return {name:pick(value.get(name,{}),('pixel_center','center_robot_mm','mouth_center_robot_mm','depth_evidence')) for name in ('ball','funnel')}


def plan(value):
    if not value:return None
    return {**pick(value,('kind','plan_id','status','seconds_left','setup_id','parameters','policy_version','route_policy',
        'grasp_tip_xyz_mm','release_tip_xyz_mm','grasp_pitch_deg','release_pitch_deg','policy_application','warnings','backup_url')),
        'steps':[s.get('stage') if isinstance(s,dict) else s for s in value.get('steps',[])],
        'scene':scene(value.get('scene',{})),'review':review(value.get('review'))}


def result(value,current_plan):
    if not value:return None
    return {**pick(value,('motion_completed','grasp_attempted','placement_attempted','physical_elapsed_s','within_10s',
        'recording_covers_motion','motion_error','camera_error','capture_trial_id','keyframes_file')),
        'execution_diagnosis':value.get('execution_diagnosis') or diagnose(value,current_plan or {})}


def summarize(command,value):
    if not isinstance(value,dict):return value
    if command in ('state','wait'):
        current=value.get('plan') or {};job=value.get('job',{});learning=value.get('learning',{});h=health(value.get('health',{}))
        waiting=[]
        if job.get('status') in ('running','preparing'):waiting.append('current_job')
        if learning.get('status')=='running':waiting.append('learning_or_recovery')
        if h.get('pending'):waiting.append('controller_pending')
        return {**pick(value,('api_version','setup_id','code_revision')),'health':h,'waiting_for':waiting,
            'task_finished':not waiting,'job':{**pick(job,('status','phase','plan_id','error')),'result':result(job.get('result'),current)},
            'learning':pick(learning,('status','phase','attempt','max_attempts','session_id','error','last_recovery')),'plan':plan(current),
            'instruction':'job/learning/pending是当前状态；result内是历史记录，不能据此声称仍在运动。task_finished只表示软件任务结束，不证明实物静止；复位仍须新鲜稳定反馈检查。'}
    if command=='context':
        if 'plan' in value:
            return {**pick(value,('plan_id','direction_contract','video_path','current_policy')),'plan':plan(value['plan']),
                'result':result(value.get('result'),value['plan']),'review':review(value.get('review')),
                'comparison':pick(value.get('comparison',{}),('available','reference_plan_id','reason','target','expected_image_delta_px')),
                'recent_reviews':[review(r) for r in value.get('recent_reviews',[])[-3:]]}
        pol=value.get('policy',{});memory=value.get('memory',{})
        return {**pick(value,('api_version','setup_id','calibration_fingerprint','authorization_contract','readiness_endpoint',
            'direction_contract','latest_plan_id','coordinates_source','instruction','recent_executions')),
            'policy':{**pick(pol,('version','parameters','statistics','successful_policy')),'reviews':[review(r) for r in pol.get('reviews',[])[-3:]]},
            'memory':{'notes':[{**pick(r,('plan_id','at_unix')),'note':r.get('note','')[:600]} for r in memory.get('notes',[])[-4:]]}}
    if command=='readiness':return {**{k:v for k,v in value.items() if k not in ('health','robot')},'health':health(value.get('health',{})),
        'robot':pick(value.get('robot') or {},('T','x','y','z','tit','r','g'))}
    if command=='vision':return pick(value,('plan_id','vision_packet_id','phase','vision_task'))
    if command=='decision':return review(value)
    return value
