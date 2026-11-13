"""Complete RSI workspace acceptance; every hardware API is intercepted."""
import argparse
import copy
import json
from pathlib import Path

import cv2
import numpy as np
from playwright.sync_api import sync_playwright

from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.browser_video import browser_video


def check(url,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    image=np.full((720,1280,3),220,np.uint8);cv2.circle(image,(600,350),20,(0,210,230),-1);cv2.circle(image,(850,400),48,(15,15,15),-1)
    _,jpeg=cv2.imencode('.jpg',image)
    video_folder=output/'video-fixture';video_folder.mkdir(exist_ok=True)
    writer=cv2.VideoWriter(str(video_folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),8,(160,90))
    for _ in range(8):writer.write(cv2.resize(image,(160,90)))
    writer.release()
    playback=browser_video(video_folder).read_bytes()
    health={'connected':True,'motion_enabled':True,'torque_status':[1]*6,'fault':None,'pending':None,'armed':False}
    model={'api_version':1,'setup_id':'bench-layout-v2','health':health,'job':{'status':'idle','phase':'idle'},'plan':None}
    calls=[];errors=[];generation=0;history=[]
    scene={'image_size':[1280,720],'ball':{'pixel_center':[600,350],'center_robot_mm':[330,0,-98],
            'depth_evidence':{'depth_used':True,'valid_samples':350,'fit_residual_mm':1.2}},
           'funnel':{'pixel_center':[850,400],'mouth_center_robot_mm':[290,-110,-69.6]}}
    def route(handler):
        nonlocal generation
        request=handler.request;path=request.url.split('/api',1)[1].split('?',1)[0]
        body=request.post_data_json if request.method=='POST' else None;calls.append((request.method,path,body))
        if path=='/bench/live-frame':
            after=int(request.url.split('after=')[1]);return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes(),
                headers={'X-Frame-Sequence':str(after+1),'X-Frame-Age-Ms':'10','X-Frame-Width':'1280','X-Frame-Height':'720'})
        if path.startswith('/bench/rsi/plans/'):
            if path.endswith('/video'):return handler.fulfill(status=200,content_type='video/mp4',body=playback)
            return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes())
        if path=='/bench/rsi/state':data=model
        elif path=='/bench/rsi/history':data={'trials':history}
        elif path=='/bench/rsi/prepare':
            generation+=1;key=('%032x'%generation)
            model['plan']={'plan_id':key,'status':'prepared','seconds_left':180,'scene':scene,'policy_version':generation,
                'parameters':copy.deepcopy(DEFAULT_PARAMETERS),'steps':['open','approach_ball','descend','close','lift','transport','release'],
                'pitch_deg':80,'warnings':[],'preview_url':f'/api/bench/rsi/plans/{key}/preview','depth_url':f'/api/bench/rsi/plans/{key}/depth'}
            model['job']={'status':'ready','phase':'ready','plan_id':key};data={'plan_id':key,'preview_url':model['plan']['preview_url']}
        elif path=='/bench/rsi/run':
            assert body['plan_id']==model['plan']['plan_id']
            assert body['declaration']=='已核对黄球、漏斗和完整抓放路径，现场有人看护且双手离开运动范围'
            model['plan']['status']='active';model['job'].update(status='running',phase='open')
            health.update(armed=True,pending={'kind':'rsi_trial'});data={'started':True,'plan_id':body['plan_id']}
        elif path=='/bench/rsi/stop':model['job']['phase']='stopping';data={'stop_requested':True}
        elif path=='/bench/rsi/review':
            assert body['source']=='human';assert body['outcome']=='grasp_missed';assert body['direction']=='image_left'
            data={'plan_id':body['plan_id'],'outcome':body['outcome'],'source':'human','after_version':2,
                  'changes':['下一轮按相机变换修正抓取偏移4mm。']}
            model['plan']['review']=data;history[0]['review']=data
        elif path.startswith('/bench/rsi/context/'):
            data={'instructions':'只给定性结果，不估计毫米值','video_path':'/local/trial/rgb.mp4','allowed_outcomes':['placed','grasp_missed','unknown']}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(status=202 if path.endswith('/run') else 200,content_type='application/json',body=json.dumps(data))
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1100});page.route('**/api/**',route);page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(url+'/experiments');page.locator('#readiness').filter(has_text='已有标定').wait_for()
        assert not any(c[0]=='POST' for c in calls);assert page.locator('#run').is_disabled()
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert 'RGB-D球面拟合' in page.locator('#sceneFacts').inner_text();assert '漏斗口 XYZ' in page.locator('#sceneFacts').inner_text()
        assert 'release' in page.locator('#planFacts').inner_text();assert page.locator('#run').is_disabled()
        page.locator('#depthPanel summary').click();assert page.locator('#depthFrame').is_visible()
        page.locator('#confirmed').check();page.screenshot(path=str(output/'rsi-plan.png'),full_page=True)
        page.locator('#run').click();page.evaluate("document.getElementById('run').click()")
        page.locator('#phase').filter(has_text='张爪').wait_for();assert page.locator('#prepare').is_disabled()
        page.locator('#imageCaption').filter(has_text='实时执行画面').wait_for()
        assert len([c for c in calls if c[1]=='/bench/rsi/run'])==1
        count=len([c for c in calls if c[0]=='POST']);page.reload()
        page.locator('#phase').filter(has_text='张爪').wait_for();assert len([c for c in calls if c[0]=='POST'])==count
        result={'motion_completed':True,'grasp_attempted':True,'placement_attempted':True,'physical_elapsed_s':8.1,
                'within_10s':True,'recording_covers_motion':True,'capture_trial_id':'c'*32,'keyframes_file':'/local/keyframes.jpg'}
        model['job'].update(status='completed',phase='finished',result=result);model['plan']['status']='completed';health.update(pending=None,armed=False)
        history.append({'id':model['plan']['plan_id'],'status':'completed','policy_version':1,'result':result,'review':None})
        page.locator('#reviewPanel').wait_for(state='visible');page.locator('#resultText').filter(has_text='松爪投放：已执行').wait_for()
        assert page.locator('#trialVideo').is_visible();assert page.locator('#run').is_disabled()
        page.locator('#trialVideo').evaluate('(video)=>video.load()')
        page.wait_for_function("document.getElementById('trialVideo').readyState>=1")
        assert page.locator('#rawVideoLink').is_visible()
        assert page.locator('#saveReview').is_enabled()
        page.locator('#grokContext').click();page.locator('#grokPrompt').wait_for(state='visible')
        assert 'video_path' in page.locator('#grokPrompt').input_value()
        page.locator('#outcome').select_option('grasp_missed');page.locator('#direction').select_option('image_left')
        page.locator('#reviewNote').fill('夹爪偏左，没有夹起')
        before=len([c for c in calls if c[1]=='/bench/rsi/run']);page.locator('#saveReview').click()
        page.locator('#reviewResult').filter(has_text='版本 v2').wait_for()
        assert page.locator('#saveReview').is_disabled();assert len([c for c in calls if c[1]=='/bench/rsi/run'])==before
        page.screenshot(path=str(output/'rsi-review.png'),full_page=True)
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert page.locator('#outcome').input_value()=='unknown';assert '版本 2' in page.locator('#planFacts').inner_text()
        page.locator('#confirmed').check();page.locator('#run').click();page.locator('#stop').wait_for(state='visible')
        page.locator('#stop').click();page.locator('#phase').filter(has_text='当前已发动作可能继续').wait_for()
        assert len([c for c in calls if c[1]=='/bench/rsi/stop'])==1
        model['job'].update(status='failed',phase='finished',error='用户停止',result={**result,'motion_completed':False,'placement_attempted':False})
        health.update(pending=None,armed=False,fault={'message':'请检查卡滞并复核'})
        page.locator('#readiness').filter(has_text='卡滞').wait_for();assert page.locator('#prepare').is_disabled()
        assert page.locator('#recoveryLink').is_visible()
        page.set_viewport_size({'width':390,'height':844});assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'rsi-mobile.png'),full_page=True)
        assert not errors,errors;browser.close()
    report={'real_hardware_access':False,'browser':'Chromium','checks':['no motion on load or prepare','RGB-D ball and movable funnel shown',
        'full pick/place and speed profile visible','explicit confirmation and one run under double click','live image during capture',
        'reload resumes without replay','H.264 video metadata loads in Chromium; raw download and keyframes available','qualitative feedback updates next policy without motion',
        'next observation resets feedback inputs','stop blocks following commands','fault recovery link','390px layout','no page errors']}
    (output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8891');parser.add_argument('--output',required=True)
    args=parser.parse_args();check(args.url,args.output)
