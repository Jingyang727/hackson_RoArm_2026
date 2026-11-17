"""Desktop challenge acceptance. All hardware/model endpoints are intercepted."""
import argparse
import copy
import json
from pathlib import Path

import cv2
import numpy as np
from playwright.sync_api import sync_playwright


def check(url,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    image=np.full((432,768,3),225,np.uint8)
    for x,color in ((240,(0,0,220)),(350,(0,200,0)),(460,(220,0,0))):cv2.circle(image,(x,225),21,color,-1)
    cv2.ellipse(image,(600,190),(45,35),0,0,360,(20,20,20),-1)
    cv2.putText(image,'SIMULATED / NO HARDWARE',(20,38),cv2.FONT_HERSHEY_SIMPLEX,.6,(50,80,50),2)
    _,jpeg=cv2.imencode('.jpg',image);session='a'*32;observation='b'*32
    scene={'balls':[{'id':f'ball-{i+1}','label':label,'color':color,'center_robot_mm':[330,i*50-40,-98],
                     'depth_evidence':{'depth_used':True}} for i,(color,label) in enumerate((('red','1号 红色'),('green','2号 绿色'),('blue','3号 蓝色')))],
           'bowl':{'mouth_center_robot_mm':[290,-110,-69.6]},'unresolved':[]}
    state={'challenge_api_version':2,'health':{'connected':True,'motion_enabled':True,'teach_active':False,'armed':False,
           'torque_status':[1]*6,'pending':None,'fault':None,'automation_owner':None},'calibrated':True,
           'model':{'configured':True,'model':'DeepSeek-V4.1-Flash'},'process':{'reset':True,'events':[]},
           'challenge':{'status':'idle','phase':'idle','remaining':None,'confirmed_colors':[],'trials':[]}}
    calls=[];errors=[];fail_observe=False
    def route(handler):
        req=handler.request;path=req.url.split('/api',1)[1].split('?',1)[0];body=req.post_data_json if req.method=='POST' else None
        calls.append([req.method,path,body])
        if path=='/bench/live-frame':return handler.fulfill(content_type='image/jpeg',body=jpeg.tobytes(),headers={'X-Frame-Sequence':'1'})
        if path.startswith('/challenge/sessions/') and '/observations/' in path:return handler.fulfill(content_type='image/jpeg',body=jpeg.tobytes())
        if path=='/challenge/events':return handler.fulfill(content_type='text/event-stream',body=': test\n\n')
        if path=='/challenge/state':data=state
        elif path=='/challenge/observe':
            if fail_observe:return handler.fulfill(status=409,content_type='application/json',body=json.dumps({'detail':'只读定位失败：相机暂不可用'}))
            state['challenge'].update(scene=scene,phase='ready',remaining=3,observation_session_id=session,observation_id=observation);data={'scene':scene,'robot_moved':False}
        elif path=='/challenge/start':
            assert 'declaration' in body
            state['challenge'].update(status='running',phase='executing',session_id=session,attempt=1,target=scene['balls'][0],
                error=None,failure_phase=None,container_reference={'bowl':scene['bowl']})
            state['process']={'reset':True,'events':[]};data={'started':True,'session_id':session}
        elif path=='/challenge/stop':state['challenge'].update(status='stopped',phase='stopped');data={'stop_requested':True}
        else:raise AssertionError('unexpected API '+req.method+' '+path)
        handler.fulfill(content_type='application/json',body=json.dumps(data))
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1540,'height':1100});page.route('**/api/**',route);page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(url+'/challenge');page.locator('#modelStatus').filter(has_text='DeepSeek').wait_for()
        assert page.locator('nav a[aria-current=page]').inner_text()=='挑战升级'
        assert page.locator('input[type=number]').count()==0;assert not any(c[0]=='POST' for c in calls)
        assert page.locator('#remaining').inner_text()=='';assert page.locator('#placed').inner_text()=='0'
        assert page.locator('#empty').count()==0;assert '0 / 3' not in page.content()
        assert page.locator('#start').is_disabled();page.locator('#observe').click();page.locator('#remaining').filter(has_text='3').wait_for()
        page.locator('#observation summary').click();assert page.locator('#balls li').count()==3
        page.locator('#confirmed').check();assert page.locator('#start').is_enabled() # armed=false is valid.
        page.locator('#start').click();page.locator('#stop').wait_for(state='visible');assert page.locator('#start').is_disabled()
        assert '漏斗位置已锁定' in page.locator('#containerStatus').inner_text()
        assert page.locator('#pauseReason').is_hidden()
        page.screenshot(path=str(output/'challenge-desktop.png'),full_page=True)
        page.reload();page.locator('#stop').wait_for(state='visible');assert len([c for c in calls if c[1]=='/challenge/start'])==1
        c=state['challenge'];c.update(attempt=2,remaining=2,confirmed_colors=['red'],target=scene['balls'][1])
        page.locator('#placed').filter(has_text='1').wait_for();assert page.locator('#stop').is_visible();assert not page.locator('#success').is_visible()
        c.update(attempt=3,remaining=1,confirmed_colors=['red','green'],target=scene['balls'][2]);page.locator('#placed').filter(has_text='2').wait_for()
        c.update(status='completed',phase='completed',remaining=0,confirmed_colors=['red','green','blue'],placement_confirmed=True,
                 trials=[{'plan_id':'c'*32,'target_color':'blue','outcome':'placed','physical_elapsed_s':8.2}])
        page.locator('#success').wait_for(state='visible');assert not page.locator('#stop').is_visible()
        assert page.locator('#placed').inner_text()=='3';assert page.locator('#trials a').count()==3
        page.screenshot(path=str(output/'challenge-completed.png'),full_page=True)
        reason='当前没有找到完整的大漏斗口沿；请让大开口出现在画面中后重新观察。'
        c.update(status='failed',phase='failed',error=reason,failure_phase='observing')
        state['process']={'reset':True,'events':[{'kind':'phase','phase':'failed','message':reason,'at_unix':1794862800}]}
        page.locator('#pauseReason').wait_for(state='visible')
        assert reason in page.locator('#pauseText').inner_text()
        assert '发生阶段：观察桌面上的球' in page.locator('#pauseText').inner_text()
        assert '已暂停' in page.locator('#readiness').inner_text()
        assert reason in page.locator('#events').inner_text()
        page.screenshot(path=str(output/'challenge-pause-reason.png'),full_page=True)
        # Explicit restart and stop work; page reload never starts anything.
        page.locator('#confirmed').check();page.locator('#start').click();page.locator('#stop').wait_for(state='visible');page.locator('#stop').click()
        page.locator('#phase').filter(has_text='已停止').wait_for()
        assert page.locator('#pauseReason').is_hidden();assert page.locator('#error').is_hidden();assert not errors,errors
        fail_observe=True;page.locator('#observe').click();page.locator('#error').filter(has_text='只读定位失败').wait_for()
        page.wait_for_timeout(1700)  # State polling must not erase a failed action's explanation.
        assert page.locator('#error').is_visible()
        fail_observe=False;page.locator('#observe').click();page.locator('#error').wait_for(state='hidden')
        assert [c[1] for c in calls if c[0]=='POST']==['/challenge/observe','/challenge/start','/challenge/start','/challenge/stop','/challenge/observe','/challenge/observe']
        browser.close()
    result={'hardware_called':False,'model_called':False,'browser_errors':errors,'checks':[
        'separate navigation','no size inputs','read-only preview','actual RGB-D XYZ labels','armed=false permits explicit start',
        'continue after first and second ball','locked container explanation','pause reason and stage shown in controls and timeline',
        'restart clears stale errors','polling preserves failed action explanations',
        'dynamic counts with no denominator or empty-check counter','reload does not replay','explicit stop']}
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--url',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(json.dumps(check(args.url,args.output),ensure_ascii=False,indent=2))
