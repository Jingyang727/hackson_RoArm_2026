"""Desktop continuous/finite start and stop. No physical device API is called."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from playwright.sync_api import sync_playwright


def check(url,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    image=np.full((360,640,3),220,np.uint8)
    cv2.putText(image,'SIMULATED / NO HARDWARE',(20,170),cv2.FONT_HERSHEY_SIMPLEX,.7,(50,80,50),2)
    _,jpeg=cv2.imencode('.jpg',image)
    state={'api_version':10,'web_learning_version':3,'visual_pipeline_version':3,'continuous_learning_version':1,
        'workspace_preview_version':1,'speed_settings_version':1,
        'speed_settings':{'policy_version':1,'calibration_fingerprint':'fixture','travel_speed':1.,'approach_speed':.5},
        'setup_id':'bench-layout-v2','job':{'status':'idle'},'learning':{'status':'idle'},'plan':None,
        'health':{'connected':True,'motion_enabled':True,'torque_status':[1]*6,'pending':None,'fault':None},
        'grok':{'configured':True,'model':'test-vlm'}}
    calls=[];errors=[]
    def route(handler):
        req=handler.request;path=req.url.split('/api',1)[1].split('?',1)[0]
        body=req.post_data_json if req.method=='POST' else None;calls.append([req.method,path,body])
        if path=='/bench/live-frame':
            return handler.fulfill(content_type='image/jpeg',body=jpeg.tobytes(),headers={
                'X-Frame-Sequence':str(int(req.url.split('after=')[1])+1),'X-Frame-Age-Ms':'5','X-Frame-Width':'640','X-Frame-Height':'360'})
        if path=='/bench/rsi/state':data=state
        elif path=='/bench/rsi/history':data={'trials':[]}
        elif path=='/bench/rsi/learning/events':return handler.fulfill(content_type='text/event-stream',body=': idle\n\n')
        elif path=='/bench/rsi/learning/start':
            limit=body['max_attempts'];assert limit is None or limit==1234
            state['learning']={'status':'running','phase':'retrying','max_attempts':limit,'attempt':6 if limit is None else 1,'continuous':limit is None}
            data={'started':True,'max_attempts':limit}
        elif path=='/bench/rsi/learning/stop':
            state['learning'].update(status='stopped',phase='stopped');data={'stop_requested':True}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(content_type='application/json',body=json.dumps(data))
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1100});page.route('**/api/**',route)
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(url+'/experiments');page.wait_for_function("document.getElementById('grokStatus').textContent.includes('test-vlm')")
        assert not any(c[0]=='POST' for c in calls)
        page.locator('#attemptSettings summary').click();assert page.locator('#maxAttempts').input_value()=='continuous'
        page.locator('#maxAttempts').select_option('custom');page.locator('#confirmed').check()
        for invalid in ('0','-1','1.5'):
            page.locator('#customAttempts').fill(invalid);assert page.locator('#start').is_disabled()
        page.locator('#customAttempts').fill('1234');assert page.locator('#start').is_enabled()
        page.locator('#start').click();page.locator('#attemptBadge').filter(has_text='1 / 1234').wait_for()
        assert page.locator('#maxAttempts').is_disabled()
        page.locator('#stop').click();page.wait_for_function("!document.getElementById('maxAttempts').disabled")
        page.locator('#maxAttempts').select_option('continuous');page.locator('#confirmed').check();page.locator('#start').click()
        page.locator('#attemptBadge').filter(has_text='第 6 轮 · 持续尝试').wait_for()
        assert 'null' not in page.locator('#attemptBadge').inner_text()
        page.screenshot(path=str(output/'continuous-desktop.png'),full_page=True)
        page.reload();page.locator('#stop').wait_for(state='visible')
        assert len([c for c in calls if c[1].endswith('/learning/start')])==2
        page.locator('#stop').click();page.wait_for_function("!document.getElementById('maxAttempts').disabled")
        state['continuous_learning_version']=0;page.locator('#confirmed').check()
        page.locator('#grokStatus').filter(has_text='重启服务').wait_for();assert page.locator('#start').is_disabled()
        assert not errors,errors
        posts=[c for c in calls if c[0]=='POST']
        assert [c[1] for c in posts]==['/bench/rsi/learning/start','/bench/rsi/learning/stop']*2
        assert posts[0][2]['max_attempts']==1234;assert posts[2][2]['max_attempts'] is None
        browser.close()
    report={'real_hardware_access':False,'model_api_called':False,'errors':errors,'checks':[
        'continuous is the default','custom finite counts above twenty','invalid counts block start',
        'continuous badge after round five','start and stop only on explicit clicks',
        'reload never resumes or replays','old backend blocks unsupported start']}
    (output/'report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--url',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(json.dumps(check(args.url,args.output),indent=2))
