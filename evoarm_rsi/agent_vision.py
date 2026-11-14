"""Image-only packets for the Grok visual subagent; no robot frame or recipe."""
import hashlib
import json
from pathlib import Path

import cv2

from .grok_review import select_frames
from .rsi_comparison import camera_view
from .task_agent import _write


INSTRUCTIONS=('只读取本包图像，按原始画面左右上下判断。不要猜机器人坐标、摆放关系、毫米、角度、力度或速度。'
    'direction描述夹爪偏在哪边；建议移动方向也用画面方向。球确实离桌才算抓起，确实进漏斗才算placed。'
    '先读图片；遮挡用unknown。comparison可用时对照前后同阶段图片，说明improved/worsened/overshot/unchanged。'
    '返回JSON：outcome,direction,ball_state,magnitude,adjustment,confidence,note,comparison。'
    'outcome=placed|grasp_missed|pushed_away|slipped|placement_missed|bounced_out|execution_failed|unknown；'
    'direction=none|image_left|image_right|image_up|image_down|too_high|too_low；'
    'ball_state=table|gripper|funnel|unknown；magnitude=small|medium|large|unknown；'
    'adjustment=keep|pickup_xy|grasp_height|grip_tighten|transport_slower|release_xy|release_height|unknown。'
    'comparison={reference_plan_id,effect,movement,note}，effect=improved|worsened|overshot|unchanged|unknown|not_comparable|not_compared；'
    'movement=as_expected|opposite|no_visible_motion|unknown；遮挡或不可比时不得猜改善。'
    '没有可比前轮时reference_plan_id=null,effect=not_compared,movement=unknown。')


def packet(current,comparison,root,capture_root):
    root=Path(root);result=current.get('result',{});scene=current['status']=='prepared'
    frames=[]
    if scene:
        for name in ('observation.jpg','preview.jpg'):
            if (root/name).is_file():frames.append({'path':root/name,'role':'current','label':name})
    else:
        trial=result.get('capture_trial_id')
        if not isinstance(trial,str) or len(trial)!=32 or any(c not in '0123456789abcdef' for c in trial):
            raise ValueError('本轮没有可复核的实际录像')
        folder=Path(capture_root)/trial;index=json.loads((folder/'keyframes.json').read_text())['frames']
        for row in select_frames(index,limit=6):
            name=row['file']
            if Path(name).name!=name:raise ValueError('关键帧文件名无效')
            frames.append({'path':folder/name,'role':'current','label':row['label'],'elapsed_s':row['elapsed_s']})
        if comparison.get('available'):
            references=[]
            for key in ('previous_frame','previous_lift_frame'):
                row=comparison.get(key)
                if row and not any(r['path']==Path(row['path']) for r in references):
                    references.append({'path':Path(row['path']),'role':'previous','label':row.get('label',key),'elapsed_s':row.get('elapsed_s')})
            frames=sorted(references,key=lambda r:r['elapsed_s'] if r['elapsed_s'] is not None else -1)+frames
    if not frames:raise ValueError('缺少真实图片，不能进行视觉复核')
    source=[]
    for row in frames:
        path=row['path']
        if not path.is_file() or path.is_symlink():raise ValueError('视觉复核图片缺失')
        source.append({**{k:v for k,v in row.items() if k!='path'},'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    comparison_info={'available':bool(not scene and comparison.get('available')),
                     'reference_plan_id':comparison.get('reference_plan_id') if not scene and comparison.get('available') else None}
    delta=comparison.get('expected_image_delta_px')
    if comparison_info['available'] and delta and max(map(abs,delta))>1e-6:
        axis=0 if abs(delta[0])>=abs(delta[1]) else 1
        comparison_info['expected_image_motion']=(('image_left' if delta[0]<0 else 'image_right') if axis==0 else
                                                   ('image_up' if delta[1]<0 else 'image_down'))
    identity={'version':2,'plan_id':current['id'],'phase':'scene' if scene else 'result','frames':source,'comparison':comparison_info}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:16]
    folder=root/'vision-evidence'/key;folder.mkdir(parents=True,exist_ok=True)
    images=[]
    for i,row in enumerate(frames):
        image=cv2.imread(str(row['path']))
        if image is None:raise ValueError('视觉图片不能解码')
        path=folder/f'frame-{i:02d}.jpg'
        elapsed=f" / t={row['elapsed_s']:.3f}s" if row.get('elapsed_s') is not None else ''
        if not cv2.imwrite(str(path),camera_view(image,f"FRAME {i+1:02d} / {row['role'].upper()} / {row['label']}{elapsed}")):
            raise ValueError('视觉图片保存失败')
        images.append({**source[i],'url':f"/api/bench/rsi/plans/{current['id']}/vision-{key}-{i}",
                       'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    instructions=('只读取本包实际图片，确认标记的是黄球与黑色漏斗开口、夹爪为空。路径叠加只是计划。'
                  '不猜机器人摆放或数值坐标，返回JSON {"ready":true或false,"note":"中文可见依据"}。') if scene else INSTRUCTIONS
    value={'vision_packet_version':2,'vision_packet_id':key,'plan_id':current['id'],'phase':identity['phase'],'frame':'unmirrored_rgb',
           'instructions':instructions,'comparison':comparison_info,'images':images,
           'execution':{'grasp_attempted':bool(result.get('grasp_attempted')),'placement_attempted':bool(result.get('placement_attempted')),
                        'motion_completed':bool(result.get('motion_completed'))}}
    _write(folder/'request.json',value)
    _write(folder/'binding.json',{'calibration_fingerprint':current['calibration_fingerprint'],
                                'capture_trial_id':result.get('capture_trial_id'),'phase':identity['phase']})
    return value


def validate_binding(current,root,key,phase='result'):
    folder=Path(root)/'vision-evidence'/key
    binding=json.loads((folder/'binding.json').read_text())
    if (binding['phase']!=phase or binding['calibration_fingerprint']!=current['calibration_fingerprint'] or
            binding['capture_trial_id']!=current.get('result',{}).get('capture_trial_id')):
        raise ValueError('视觉结论不属于本轮实际录像和标定')
