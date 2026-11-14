"""Local OpenCode tools. HTTP only: reuse the single existing robot owner."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import time
from urllib.error import HTTPError,URLError
from urllib.parse import urlsplit
from urllib.request import Request,build_opener,ProxyHandler

PREFIX='/api/bench/rsi'
RUN_DECLARATION='已核对黄球、漏斗和完整抓放路径，现场有人看护且双手离开运动范围'
RESET_DECLARATION='现场已清空，允许自动抓放、失败后恢复官方姿态并连续重试'


class Controller:
    def __init__(self,url='http://127.0.0.1:8765'):
        parsed=urlsplit(url)
        if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost','::1') or parsed.username or parsed.password or parsed.path not in ('','/') or parsed.query or parsed.fragment:
            raise ValueError('Agent工具只访问本机已有控制服务')
        self.url=url.rstrip('/');self.opener=build_opener(ProxyHandler({}))

    def request(self,path,payload=None,*,binary=False):
        req=Request(self.url+PREFIX+path,data=None if payload is None else json.dumps(payload,allow_nan=False).encode(),
                    headers={'Content-Type':'application/json'},method='GET' if payload is None else 'POST')
        try:
            with self.opener.open(req,timeout=120) as response:body=response.read()
        except HTTPError as exc:
            body=exc.read().decode(errors='replace')
            raise ValueError(f'HTTP {exc.code}: {body[:4000]}') from exc
        if binary:return body
        try:return json.loads(body)
        except (ValueError,UnicodeError) as exc:raise ValueError('控制服务没有返回JSON，请检查服务日志') from exc


def plan_id(value):
    if not re.fullmatch('[a-f0-9]{32}',value):raise argparse.ArgumentTypeError('需32位试次ID')
    return value


def objects(state):
    plan=state.get('plan')
    if not plan:return {'available':False,'reason':'尚无本轮观察；先prepare，只录制和规划，不移动机械臂'}
    scene=plan['scene'];ball=scene['ball'];funnel=scene['funnel']
    return {'available':True,'plan_id':plan['plan_id'],'status':plan['status'],'seconds_left':plan['seconds_left'],
            'frame':'robot_firmware_mm','units':'mm','live_tracking':False,
            'ball':{'pixel':ball['pixel_center'],'center_xyz_mm':ball['center_robot_mm'],
                    'source':'OpenCV + RGB-D球面拟合' if ball.get('depth_evidence',{}).get('depth_used') else 'OpenCV + 已标定桌面 + 球半径17mm',
                    'depth_evidence':ball.get('depth_evidence')},
            'funnel':{'pixel':funnel['pixel_center'],'mouth_center_xyz_mm':funnel['mouth_center_robot_mm'],
                      'source':'OpenCV黑色开口 + 标定桌面 + 实测漏斗高度45.4mm'},
            'manual_test_note':'这是物体中心；固件末端为固定指尖参考点。先在坐标页预览，不直接把球心当作夹爪中心指令。'}


def invoke(args,controller):
    command=args.command
    if command=='readiness':
        value=controller.request('/agent/readiness')
        value.update(controller_url=controller.url,client_project_root=str(Path(__file__).resolve().parent.parent),client_cwd=str(Path.cwd()))
        return value
    if command=='state':return controller.request('/state')
    if command=='context':return controller.request('/context/'+args.plan_id if args.plan_id else '/agent/context')
    if command=='coordinate-evidence':return controller.request('/agent/coordinate-evidence')
    if command=='objects':return objects(controller.request('/state'))
    if command=='history':return controller.request('/history')
    if command=='prepare':return controller.request('/prepare',{})
    if command=='observe':return controller.request('/observe',{})
    if command=='remember':return controller.request('/agent/memory',{'note':args.note,'plan_id':args.plan_id})
    if command=='review':
        payload=json.loads(args.json if args.json is not None else Path(args.file).read_text())
        if not isinstance(payload,dict):raise ValueError('复核文件需为JSON对象')
        payload['source']='opencode_agent'
        return controller.request('/review',payload)
    if command=='decision':
        from .grok_review import validate_verdict
        value=json.loads(args.json)
        if not isinstance(value,dict) or set(value)-{'plan_id','vision_packet_id','visual_review','requested_step_mm','parameter_trial','reason'}:
            raise ValueError('decision需提交视觉结论、图片包编号和Agent决策理由')
        required={'plan_id','vision_packet_id','visual_review','reason'}
        if not required<=set(value) or not isinstance(value['reason'],str) or not value['reason'].strip():raise ValueError('缺少视觉来源或决策理由')
        verdict=validate_verdict(value['visual_review'])
        payload={**verdict,'plan_id':value['plan_id'],'source':'opencode_agent','note':value['reason'],
                 'vision_packet_id':value['vision_packet_id'],'visual_review':verdict}
        for key in ('requested_step_mm','parameter_trial'):
            if key in value:payload[key]=value[key]
        return controller.request('/review',payload)
    if command in ('run','reset'):
        if not args.execute:raise ValueError('该工具会移动机械臂；执行Agent须在用户已授权的现场任务中带--execute调用')
        if command=='run':
            payload={'plan_id':args.plan_id,'declaration':RUN_DECLARATION}
            if args.scene_json is not None or args.vision_packet_id:
                if args.scene_json is None or not args.vision_packet_id:raise ValueError('场景复核需同时提供图片包编号与JSON')
                payload.update(vision_packet_id=args.vision_packet_id,scene_review=json.loads(args.scene_json))
            return controller.request('/run',payload)
        return controller.request('/learning/reset',{'declaration':RESET_DECLARATION})
    if command=='stop':
        state=controller.request('/state')
        if state['learning']['status']=='running':return controller.request('/learning/stop',{})
        if state['job'].get('status')=='running':return controller.request('/stop',{'plan_id':state['job']['plan_id']})
        return {'stopped':True,'reason':'没有正在执行的任务'}
    if command=='wait':
        end=time.monotonic()+args.seconds
        while True:
            state=controller.request('/state')
            if state['learning']['status']!='running' and state['job'].get('status') not in ('running','preparing'):
                return state
            if time.monotonic()>=end:raise ValueError('等待超时；未自动重发动作。先读取state确认状态')
            time.sleep(.5)
    if command=='evidence':
        context=controller.request('/context/'+args.plan_id)
        folder=Path.cwd()/'.evoarm-agent-evidence'/args.plan_id;folder.mkdir(parents=True,exist_ok=True)
        paths={};missing={}
        for kind in ('preview','keyframes','comparison'):
            try:data=controller.request(f'/plans/{args.plan_id}/{kind}',binary=True)
            except ValueError as exc:missing[kind]=str(exc);continue
            path=folder/(kind+'.jpg');path.write_bytes(data);paths[kind]=str(path)
        return {'context':context,'images':paths,'missing':missing,
                'instruction':'用图片读取工具打开这些实际图片，再做复核；没有看到图像时返回unknown，不凭坐标宣称成功。'}
    if command=='vision':
        packet=controller.request('/vision-context/'+args.plan_id)
        folder=Path.cwd()/'.evoarm-agent-evidence'/args.plan_id/'vision'/packet['vision_packet_id'];folder.mkdir(parents=True,exist_ok=True)
        for i,row in enumerate(packet['images']):
            if not row['url'].startswith(PREFIX+'/plans/'+args.plan_id+'/vision-'):raise ValueError('视觉图片地址不属于本轮')
            data=controller.request(row['url'][len(PREFIX):],binary=True)
            if hashlib.sha256(data).hexdigest()!=row['sha256']:raise ValueError('视觉图片校验失败')
            path=folder/f'frame-{i:02d}.jpg';path.write_bytes(data);row['local_path']=str(path)
        # Only this packet is passed to the visual task: no main-agent context,
        # robot coordinates, extrinsics, policies or remembered view relationship.
        packet['vision_task']=packet['instructions']+'\n'+json.dumps({k:v for k,v in packet.items() if k!='instructions'},ensure_ascii=False)
        path=folder/'vision-task.json';path.write_text(json.dumps(packet,ensure_ascii=False,indent=2))
        return packet
    raise ValueError('未知工具')


def parser():
    result=argparse.ArgumentParser(description=__doc__);result.add_argument('--url',default='http://127.0.0.1:8765')
    sub=result.add_subparsers(dest='command',required=True)
    for name in ('state','objects','history','coordinate-evidence','observe','prepare','stop','readiness'):sub.add_parser(name)
    context=sub.add_parser('context');context.add_argument('plan_id',type=plan_id,nargs='?')
    memory=sub.add_parser('remember');memory.add_argument('--note',required=True);memory.add_argument('--plan-id',type=plan_id)
    review=sub.add_parser('review');source=review.add_mutually_exclusive_group(required=True)
    source.add_argument('--file');source.add_argument('--json')
    decision=sub.add_parser('decision');decision.add_argument('--json',required=True)
    run=sub.add_parser('run');run.add_argument('plan_id',type=plan_id);run.add_argument('--execute',action='store_true')
    run.add_argument('--vision-packet-id');run.add_argument('--scene-json')
    reset=sub.add_parser('reset');reset.add_argument('--execute',action='store_true')
    wait=sub.add_parser('wait');wait.add_argument('--seconds',type=float,default=60)
    evidence=sub.add_parser('evidence');evidence.add_argument('plan_id',type=plan_id)
    vision=sub.add_parser('vision');vision.add_argument('plan_id',type=plan_id)
    return result


def main():
    args=parser().parse_args()
    try:result=invoke(args,Controller(args.url));print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    except (OSError,ValueError,URLError) as exc:
        print(json.dumps({'error':str(exc),'action_reissued':False},ensure_ascii=False));raise SystemExit(1)


if __name__=='__main__':main()
