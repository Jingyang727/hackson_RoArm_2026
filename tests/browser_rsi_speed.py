"""Desktop speed settings acceptance. Every hardware/model API is intercepted."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from playwright.sync_api import sync_playwright


def check(url,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    image=np.full((360,640,3),220,np.uint8)
    cv2.circle(image,(280,160),15,(0,210,230),-1);cv2.circle(image,(450,170),35,(15,15,15),-1)
    cv2.putText(image,'SIMULATED / NO HARDWARE',(20,330),cv2.FONT_HERSHEY_SIMPLEX,.6,(70,80,70),2)
    _,jpeg=cv2.imencode('.jpg',image)
    settings={'policy_version':4,'calibration_fingerprint':'fixture','travel_speed':.65,'approach_speed':.3}
    state={'api_version':10,'web_learning_version':3,'continuous_learning_version':1,'route_policy_version':7,'visual_pipeline_version':3,'speed_settings_version':1,
        'speed_settings':settings,'workspace_preview_version':1,'setup_id':'bench-layout-v2',
        'health':{'connected':True,'motion_enabled':True,'torque_status':[1]*6,'fault':None,'pending':None},
        'job':{'status':'idle'},'learning':{'status':'idle'},'grok':{'configured':True,'model':'test-vlm'},'plan':None}
    calls=[];errors=[]
    def route(handler):
        request=handler.request;path=request.url.split('/api',1)[1].split('?',1)[0]
        body=request.post_data_json if request.method=='POST' else None;calls.append([request.method,path,body])
        if path=='/bench/live-frame':
            sequence=int(request.url.split('after=')[1])+1
            return handler.fulfill(content_type='image/jpeg',body=jpeg.tobytes(),headers={
                'X-Frame-Sequence':str(sequence),'X-Frame-Age-Ms':'5','X-Frame-Width':'640','X-Frame-Height':'360'})
        if path=='/bench/rsi/state':data=state
        elif path=='/bench/rsi/history':data={'trials':[]}
        elif path=='/bench/rsi/learning/events':return handler.fulfill(content_type='text/event-stream',body=': idle\n\n')
        elif path=='/bench/rsi/speed':
            assert body=={**settings,'travel_speed':1.,'approach_speed':.5}
            settings.update(travel_speed=1.,approach_speed=.5,policy_version=settings['policy_version']+1)
            data={**settings,'changed':True,'robot_moved':False}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(content_type='application/json',body=json.dumps(data))
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1100});page.route('**/api/**',route)
        page.on('pageerror',lambda exc:errors.append(str(exc)))
        page.goto(url+'/experiments');page.locator('#travelSpeed').wait_for(state='attached')
        page.wait_for_function("!document.getElementById('speedFast').disabled")
        assert not any(c[0]=='POST' for c in calls)
        page.locator('#speedSettings summary').click()
        assert page.locator('#travelSpeed').input_value()=='0.65';assert page.locator('#approachSpeed').input_value()=='0.3'
        page.locator('#confirmed').check();assert page.locator('#start').is_enabled()
        page.locator('#speedFast').click();assert page.locator('#start').is_disabled()
        assert page.locator('#travelSpeed').input_value()=='1';assert page.locator('#approachSpeed').input_value()=='0.5'
        page.wait_for_timeout(1000);assert page.locator('#travelSpeed').input_value()=='1'
        page.locator('#saveSpeed').click();page.locator('#speedStatus').filter(has_text='速度已保存').wait_for()
        assert page.locator('#start').is_enabled();assert page.locator('#saveSpeed').is_disabled()
        assert [c[1] for c in calls if c[0]=='POST']==['/bench/rsi/speed']
        page.screenshot(path=str(output/'rsi-speed-desktop.png'),full_page=True)
        page.reload();page.locator('#speedSettings summary').click()
        page.wait_for_function("document.getElementById('travelSpeed').value==='1'")
        assert page.locator('#approachSpeed').input_value()=='0.5'
        state['learning']={'status':'running','phase':'executing'}
        page.wait_for_function("document.getElementById('travelSpeed').disabled")
        assert page.locator('#speedFast').is_disabled();assert page.locator('#saveSpeed').is_disabled()
        assert not errors,errors
        browser.close()
    report={'real_hardware_access':False,'model_api_called':False,'checks':[
        'load does not write','current saved speeds shown','fast preset only edits until save',
        'unsaved edits block start and survive polling','save calls only speed API',
        'saved values survive reload','running task disables editing'],'errors':errors}
    (output/'report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--url',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(json.dumps(check(args.url,args.output),indent=2))
