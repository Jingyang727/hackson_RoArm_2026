"""Automatic RSI workspace acceptance; all device/model APIs are intercepted."""
import argparse
import copy
import json
from pathlib import Path

import cv2
import numpy as np
from playwright.sync_api import sync_playwright

from evoarm_rsi.browser_video import browser_video
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.rsi_learning import LEARNING_DECLARATION


def check(url, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    image=np.full((720,1280,3),220,np.uint8)
    cv2.circle(image,(600,350),20,(0,210,230),-1);cv2.circle(image,(850,400),48,(15,15,15),-1)
    cv2.putText(image,'SIMULATED CAMERA / NO HARDWARE',(30,680),cv2.FONT_HERSHEY_SIMPLEX,.7,(90,100,90),2)
    _,jpeg=cv2.imencode('.jpg',image)
    video_folder=output/'video-fixture';video_folder.mkdir(exist_ok=True)
    writer=cv2.VideoWriter(str(video_folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),8,(160,90))
    for _ in range(8):writer.write(cv2.resize(image,(160,90)))
    writer.release();playback=browser_video(video_folder).read_bytes()
    health={'connected':True,'motion_enabled':True,'torque_status':[1]*6,'fault':None,'pending':None,'armed':False}
    model={'api_version':2,'setup_id':'bench-layout-v2','health':health,'job':{'status':'idle','phase':'idle'},
           'learning':{'status':'idle','phase':'idle'},'grok':{'configured':True,'model':'bottlerocket/grok-4.7-build'},'plan':None}
    scene={'image_size':[1280,720],'depth_scale':{'near_mm':420,'far_mm':550},
           'ball':{'pixel_center':[600,350],'center_robot_mm':[330,0,-98],
                   'depth_evidence':{'depth_used':True,'valid_samples':350,'fit_residual_mm':1.2}},
           'funnel':{'pixel_center':[850,400],'mouth_center_robot_mm':[290,-110,-69.6]}}
    calls=[];errors=[];history=[]
    def plan(key,version):
        return {'plan_id':key,'status':'active','seconds_left':180,'scene':scene,'policy_version':version,
                'parameters':copy.deepcopy(DEFAULT_PARAMETERS),'steps':['approach_ball','descend','close','lift','transport','release'],
                'pitch_deg':80,'warnings':[],'preview_url':f'/api/bench/rsi/plans/{key}/preview',
                'depth_url':f'/api/bench/rsi/plans/{key}/depth','backup_url':None}
    def route(handler):
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
        elif path=='/bench/rsi/learning/start':
            assert body=={'declaration':LEARNING_DECLARATION,'max_attempts':5}
            model['learning']={'status':'running','phase':'observing','session_id':'d'*32,'attempt':1,'max_attempts':5,'trials':[]}
            data={'started':True,'session_id':'d'*32}
        elif path=='/bench/rsi/learning/stop':
            model['learning']['phase']='stopping';data={'stop_requested':True}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(status=202 if path.endswith('/start') else 200,content_type='application/json',body=json.dumps(data))
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1100});page.route('**/api/**',route)
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(url+'/experiments');page.locator('#grokStatus').filter(has_text='公司Grok已配置').wait_for()
        page.locator('#workspaceFrame').wait_for(state='visible')
        assert not any(call[0]=='POST' for call in calls)
        assert page.locator('#saveReview').count()==0;assert page.locator('#start').is_disabled()
        page.locator('#confirmed').check();page.locator('#start').click();page.evaluate("document.getElementById('start').click()")
        page.locator('#phase').filter(has_text='观察黄球').wait_for()
        assert len([call for call in calls if call[1]=='/bench/rsi/learning/start'])==1
        key='1'*32;model['plan']=plan(key,1);model['learning']['phase']='executing'
        model['job']={'status':'running','phase':'descend','plan_id':key}
        health.update(armed=True,pending={'kind':'rsi_trial'})
        page.locator('#phase').filter(has_text='下降').wait_for();assert page.locator('#start').is_disabled()
        page.screenshot(path=str(output/'rsi-plan.png'),full_page=True)
        result={'motion_completed':True,'grasp_attempted':True,'placement_attempted':True,'physical_elapsed_s':7.8,
                'within_10s':True,'recording_covers_motion':True,'capture_trial_id':'a'*32,'keyframes_file':'/local/keyframes.jpg'}
        model['job'].update(status='completed',phase='finished',result=result);model['learning']['phase']='analyzing'
        health.update(pending=None,armed=False)
        page.locator('#phase').filter(has_text='Grok正在分析').wait_for()
        posts=len([call for call in calls if call[0]=='POST']);page.reload()
        page.locator('#phase').filter(has_text='Grok正在分析').wait_for()
        assert len([call for call in calls if call[0]=='POST'])==posts
        review={'outcome':'grasp_missed','direction':'image_left','magnitude':'large','source':'company_grok',
                'note':'夹爪偏左，球未离桌','changes':['下一轮修正抓取偏移6mm。'],'after_version':2}
        model['plan']['review']=review;model['plan']['backup_url']=f'/api/bench/rsi/plans/{key}/archive'
        model['learning'].update(phase='recovering',review=review,changes=review['changes'])
        page.locator('#phase').filter(has_text='恢复官方姿态').wait_for()
        page.locator('#grokVerdict').filter(has_text='夹爪偏左').wait_for()
        assert page.locator('#start').is_disabled()
        # No user review or second execution request: backend starts round two.
        key='2'*32;model['plan']=plan(key,2);model['learning'].update(phase='executing',attempt=2)
        model['job']={'status':'running','phase':'close','plan_id':key}
        health.update(armed=True,pending={'kind':'rsi_trial'})
        page.locator('#attemptBadge').filter(has_text='2 / 5').wait_for();page.locator('#phase').filter(has_text='合抓').wait_for()
        success={'outcome':'placed','source':'company_grok','note':'球随夹爪离桌，释放后留在漏斗内','changes':['保留成功策略。'],'after_version':3}
        model['plan'].update(status='completed',review=success,backup_url=f'/api/bench/rsi/plans/{key}/archive')
        model['job'].update(status='completed',phase='finished',result=result)
        model['learning'].update(status='completed',phase='completed',review=success,changes=success['changes'])
        health.update(armed=False,pending=None)
        history.append({'id':key,'status':'completed','policy_version':2,'review':success,'result':result,'backup_url':model['plan']['backup_url']})
        page.locator('#phase').filter(has_text='策略已保存').wait_for()
        page.locator('#grokVerdict').filter(has_text='进入漏斗').wait_for()
        page.locator('#trialVideo').evaluate('(video)=>video.load()');page.wait_for_function("document.getElementById('trialVideo').readyState>=1")
        assert page.locator('#archiveLink').is_visible();assert not page.locator('#diagnostics').get_attribute('open')
        page.screenshot(path=str(output/'rsi-review.png'),full_page=True)
        page.locator('#diagnostics summary').click();page.locator('#depthLegend').filter(has_text='420.0').wait_for()
        assert page.locator('#depthFrame').is_visible();page.locator('#diagnostics summary').click()
        page.set_viewport_size({'width':390,'height':844});assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'rsi-mobile.png'),full_page=True)
        model['learning'].update(status='running',phase='analyzing');page.locator('#stop').wait_for(state='visible')
        page.locator('#stop').click();page.locator('#phase').filter(has_text='正在停止').wait_for()
        assert len([call for call in calls if call[1]=='/bench/rsi/learning/stop'])==1
        assert not any(call[1].endswith('/review') for call in calls)
        model['learning'].update(status='stopped',phase='stopped');model['grok'].update(configured=False,reason='公司Grok未配置')
        page.locator('#grokStatus').filter(has_text='未配置').wait_for();page.locator('#confirmed').check()
        assert page.locator('#start').is_disabled();assert not errors,errors
        browser.close()
    report={'real_hardware_access':False,'browser':'Chromium','checks':[
        'single learning start with explicit scope; no double click replay','live workspace without motion on load',
        'no mandatory human result form','Grok analysis progress survives reload','automatic recovery and round two without browser action',
        'successful Grok verdict and persistent policy shown','native H.264 playback','backup and raw download links',
        'depth diagnostics folded with distance legend','stop cancels further learning','missing Grok blocks unattended execution',
        '390px layout','no page errors']}
    (output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8893')
    parser.add_argument('--output',required=True);args=parser.parse_args();check(args.url,args.output)
