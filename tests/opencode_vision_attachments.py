"""Exercise real OpenCode Task image expansion against a local mock model.

No company model, credentials, serial port, camera service or robot is used.
Run: python -m tests.opencode_vision_attachments --opencode /path/to/opencode --output /tmp/check
"""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import threading
from unittest.mock import Mock,patch

import cv2
import numpy as np

from evoarm_rsi.agent_tools import invoke,parser


def check(binary,output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    project=output/'project with spaces';project.mkdir(exist_ok=True)
    subprocess.run(['git','init','-q',str(project)],check=True,capture_output=True)
    repo=Path(__file__).resolve().parents[1]
    agents=project/'.opencode/agents';agents.mkdir(parents=True,exist_ok=True)
    for name in ('roarm-rsi','roarm-vision'):
        text=(repo/'.opencode/agents'/f'{name}.md').read_text()
        if name=='roarm-rsi':text=text.replace('bottlerocket/grok-4.7-build','local/main')
        (agents/f'{name}.md').write_text(text)
    key='a'*32;packet_id='b'*16;images=[];payload=[]
    for i,seconds in enumerate((0.,4.004,5.606,7.474,9.743,10.011)):
        image=np.full((120,200,3),60+i*20,np.uint8)
        cv2.putText(image,f'FRAME {i+1} / {seconds:.3f}s',(4,60),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
        ok,encoded=cv2.imencode('.jpg',image);assert ok
        data=encoded.tobytes();payload.append(data)
        images.append({'url':f'/api/bench/rsi/plans/{key}/vision-{packet_id}-{i}',
            'sha256':hashlib.sha256(data).hexdigest(),'role':'current','label':f'frame_{i}', 'elapsed_s':seconds})
    packet={'plan_id':key,'vision_packet_id':packet_id,'phase':'result','vision_packet_version':2,
        'instructions':'只根据附图返回完整JSON，不使用工具。','images':images,'comparison':{'available':False},
        'execution':{'grasp_attempted':True,'placement_attempted':False,'motion_completed':False}}
    api=Mock();api.request.side_effect=[packet,*payload]
    with patch.object(Path,'cwd',return_value=project):task=invoke(parser().parse_args(['vision',key]),api)['vision_task']
    calls=[];state={'task_issued':False}
    visual={'outcome':'grasp_missed','direction':'none','ball_state':'table','magnitude':'unknown',
        'adjustment':'unknown','confidence':.5,'note':'离线传输测试，不代表实际视觉判断',
        'comparison':{'reference_plan_id':None,'effect':'not_compared','movement':'unknown','note':'首轮'}}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_POST(self):
            request=json.loads(self.rfile.read(int(self.headers['Content-Length'])));calls.append(request)
            name=request['model'];delta={'role':'assistant'};reason='stop'
            if name=='vision':delta['content']=json.dumps(visual,ensure_ascii=False)
            elif any(t.get('function',{}).get('name')=='task' for t in request.get('tools',[])) and not state['task_issued']:
                state['task_issued']=True
                delta['tool_calls']=[{'index':0,'id':'call_vision','type':'function','function':{
                    'name':'task','arguments':json.dumps({'subagent_type':'roarm-vision','description':'Read ordered actual frames','prompt':task})}}]
                reason='tool_calls'
            else:delta['content']='Offline transport check completed.'
            self.send_response(200);self.send_header('Content-Type','text/event-stream');self.end_headers()
            for change,finish in ((delta,None),({},reason)):
                data={'id':'local-test','object':'chat.completion.chunk','created':1,'model':name,
                    'choices':[{'index':0,'delta':change,'finish_reason':finish}]}
                self.wfile.write(('data: '+json.dumps(data)+'\n\n').encode())
            self.wfile.write(b'data: [DONE]\n\n');self.wfile.flush()

    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    config=json.loads((repo/'opencode.json').read_text());config['agent']['roarm-vision']['model']='local/vision'
    config['provider']={'local':{'npm':'@ai-sdk/openai-compatible','options':{
        'baseURL':f'http://127.0.0.1:{server.server_port}/v1','apiKey':'local-mock-only'},
        'models':{name:{'name':name,'attachment':True,'modalities':{'input':['text','image'],'output':['text']},
                       'limit':{'context':32768,'output':2048}} for name in ('main','vision')}}}
    (project/'opencode.json').write_text(json.dumps(config))
    env={**os.environ,'OPENCODE_DISABLE_MODELS_FETCH':'1','OPENCODE_DISABLE_SHARE':'1',
         'npm_config_cache':str(output/'npm-cache')}
    for name in ('DATA','CONFIG','CACHE','STATE'):env[f'XDG_{name}_HOME']=str(output/name.lower())
    try:
        result=subprocess.run([str(Path(binary).resolve()),'run','--pure','--dir',str(project),'--agent','roarm-rsi','--format','json',
            'Transport test only. Call the visual Task with the exact supplied prompt, then finish.'],
            cwd=project,env=env,text=True,capture_output=True,timeout=90)
        (output/'stdout.jsonl').write_text(result.stdout);(output/'stderr.log').write_text(result.stderr)
        (output/'requests.json').write_text(json.dumps(calls,ensure_ascii=False,indent=2))
        assert result.returncode==0,(result.stderr+result.stdout)[-3000:]
        reviews=[row for row in calls if row['model']=='vision']
        assert len(reviews)==1,f'Expected one visual model request, got {len(reviews)}'
        parts=[part for msg in reviews[0]['messages'] if isinstance(msg.get('content'),list) for part in msg['content']]
        attached=[part for part in parts if part.get('type')=='image_url']
        assert len(attached)==6,f'Expected six images in one request, got {len(attached)}'
        texts='\n'.join(part.get('text','') for part in parts if part.get('type')=='text')
        assert all(f'frame-{i:02d}.jpg' in texts for i in range(6))
        assert '"elapsed_s": 10.011' in texts
        assert not any(msg['role']=='tool' for msg in reviews[0]['messages'])
        assert 'grasp_missed' in result.stdout,'Complete subagent JSON did not reach parent'
        report={'opencode_version':subprocess.check_output([binary,'--version'],env=env,text=True).strip(),
            'visual_model_requests':1,'images_in_first_request':6,'visual_tool_calls':0,'path_with_spaces':True,
            'complete_json_returned_to_parent':True,'company_model_called':False,'robot_moved':False}
        (output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    finally:server.shutdown();server.server_close()


if __name__=='__main__':
    args=argparse.ArgumentParser(description=__doc__);args.add_argument('--opencode',required=True);args.add_argument('--output',required=True)
    options=args.parse_args();check(options.opencode,options.output)
