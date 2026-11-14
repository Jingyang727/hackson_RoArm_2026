"""Automatic RSI workspace acceptance; all device/model APIs are intercepted."""
import argparse
import copy
import json
import time
from pathlib import Path

import cv2
import numpy as np
from playwright.sync_api import sync_playwright

from evoarm_rsi.browser_video import browser_video
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.rsi_learning import LEARNING_DECLARATION
from evoarm_rsi.rsi_comparison import comparison_panel


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
    process={'reset':True,'cursor':'idle:0','session_id':None,'started_at_unix':time.time(),'events':[],'model_output':None}
    model={'api_version':10,'setup_id':'bench-layout-v2','health':health,'job':{'status':'idle','phase':'idle'},'process':process,
           'learning':{'status':'idle','phase':'idle'},'grok':{'configured':True,'model':'grok-4.7-build','backend':'company_api'},'plan':None}
    scene={'image_size':[1280,720],'depth_scale':{'near_mm':420,'far_mm':550},
           'ball':{'pixel_center':[600,350],'center_robot_mm':[330,0,-98],
                   'depth_evidence':{'depth_used':True,'valid_samples':350,'fit_residual_mm':1.2}},
           'funnel':{'pixel_center':[850,400],'mouth_center_robot_mm':[290,-110,-69.6]}}
    calls=[];errors=[];history=[]
    pair_context={'available':True,'expected_image_delta_px':[14.,0.],
                  'previous_target_pixel':[600,350],'current_target_pixel':[600,350]}
    for role,key,shift in (('previous','1'*32,0),('current','2'*32,14)):
        view=image.copy();cv2.rectangle(view,(550+shift,330),(558+shift,375),(10,10,10),-1)
        path=output/f'{role}-close.jpg';cv2.imwrite(str(path),view)
        pair_context[role+'_frame']={'path':str(path),'plan_id':key,'frame_index':4,'elapsed_s':3.2}
    pair_jpeg=Path(comparison_panel(pair_context,output)).read_bytes()
    def emit(kind,**data):
        sequence=int(process['cursor'].split(':')[-1])+1
        row={'id':f"{process['session_id']}:{sequence}",'sequence':sequence,'session_id':process['session_id'],
             'kind':kind,'at_unix':time.time(),**data}
        process['cursor']=row['id'];process['events'].append(row)
    def plan(key,version):
        parameters=copy.deepcopy(DEFAULT_PARAMETERS)
        if version>1:parameters['pickup_xy_mm']=[0,-12]
        return {'plan_id':key,'status':'active','seconds_left':180,'scene':scene,'policy_version':version,
                'parameters':parameters,'policy_application':{'parameters_verified':True,'source_trial_id':'1'*32 if version>1 else None},
                'steps':['raise_current','approach_ball','align_above_ball','descend','close','lift','transport','lower_to_release','release'],
                'route_policy':{'version':4,'mode':'lift_transfer_descend'},
                'pitch_deg':80,'release_pitch_deg':50,'warnings':[],'preview_url':f'/api/bench/rsi/plans/{key}/preview',
                'depth_url':f'/api/bench/rsi/plans/{key}/depth','backup_url':None}
    def route(handler):
        request=handler.request;path=request.url.split('/api',1)[1].split('?',1)[0]
        body=request.post_data_json if request.method=='POST' else None;calls.append((request.method,path,body))
        if path=='/bench/live-frame':
            after=int(request.url.split('after=')[1]);return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes(),
                headers={'X-Frame-Sequence':str(after+1),'X-Frame-Age-Ms':'10','X-Frame-Width':'1280','X-Frame-Height':'720'})
        if path=='/bench/rsi/learning/events':
            body='retry: 250\n'+f"id: {process['cursor']}\nevent: snapshot\ndata: {json.dumps(process)}\n\n"
            return handler.fulfill(status=200,content_type='text/event-stream',body=body)
        if path.startswith('/bench/rsi/observations/'):
            return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes())
        if path.startswith('/bench/rsi/plans/'):
            if path.endswith('/video'):return handler.fulfill(status=200,content_type='video/mp4',body=playback)
            if path.endswith('/comparison'):return handler.fulfill(status=200,content_type='image/jpeg',body=pair_jpeg)
            return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes())
        if path=='/bench/rsi/state':data=model
        elif path=='/bench/rsi/history':data={'trials':history}
        elif path=='/bench/rsi/observe':
            assert body=={}
            data={'observation_id':'f'*32,'observed_at_unix':time.time(),'scene':scene,
                  'image_url':'/api/bench/rsi/observations/'+('f'*32)+'/preview',
                  'note':'本次快照；球或漏斗移动后重新采集，保持物体原位再验证。'}
        elif path=='/bench/rsi/learning/start':
            assert body=={'declaration':LEARNING_DECLARATION,'max_attempts':5}
            model['learning']={'status':'running','phase':'observing','session_id':'d'*32,'attempt':1,'max_attempts':5,'trials':[]}
            key='d'*32 if not process['session_id'] else 'e'*32
            process.update(cursor=key+':0',session_id=key,started_at_unix=time.time(),events=[],model_output=None)
            emit('phase',phase='observing',attempt=1)
            data={'started':True,'session_id':'d'*32}
        elif path=='/bench/rsi/learning/stop':
            model['learning']['phase']='stopping';data={'stop_requested':True}
        elif path=='/bench/rsi/learning/reset':
            assert body=={'declaration':LEARNING_DECLARATION}
            model['learning'].update(status='running',phase='recovering');data={'started':True,'session_id':'d'*32}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(status=202 if path.endswith('/start') else 200,content_type='application/json',body=json.dumps(data))
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1100});page.route('**/api/**',route)
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(url+'/experiments');page.locator('#grokStatus').filter(has_text='公司Grok API已配置').wait_for()
        page.locator('#workspaceFrame').wait_for(state='visible')
        assert not any(call[0]=='POST' for call in calls)
        assert page.locator('#saveReview').count()==0;assert page.locator('#start').is_disabled()
        page.get_by_text('采集当前物体坐标（不运动）',exact=True).click()
        page.locator('#observeObjects').click();page.locator('#observationFacts').filter(has_text='330.0').wait_for()
        assert page.locator('#observationImage').is_visible()
        assert all(call[1]=='/bench/rsi/observe' for call in calls if call[0]=='POST')
        scene['ball']['center_robot_mm'][0]=345.
        page.locator('#observeObjects').click();page.locator('#observationFacts').filter(has_text='345.0').wait_for()
        assert '330.0' not in page.locator('#observationFacts').inner_text()
        scene['ball']['center_robot_mm'][0]=330.
        page.get_by_text('采集当前物体坐标（不运动）',exact=True).click()
        page.locator('#confirmed').check();page.locator('#start').click();page.evaluate("document.getElementById('start').click()")
        page.locator('#phase').filter(has_text='观察黄球').wait_for()
        assert len([call for call in calls if call[1]=='/bench/rsi/learning/start'])==1
        key='1'*32;model['plan']=plan(key,1);model['learning']['phase']='executing'
        model['job']={'status':'running','phase':'descend','plan_id':key}
        health.update(armed=True,pending={'kind':'rsi_trial'})
        page.locator('#phase').filter(has_text='下降').wait_for();assert page.locator('#start').is_disabled()
        assert '抓取俯仰 80.0°；运输/投放俯仰 50.0°' in page.locator('#planFacts').text_content(), page.locator('#planFacts').text_content()
        assert page.locator('#targetOverlay circle').count()==2
        assert page.locator('#targetOverlay').is_visible()
        assert '画面右侧' in page.locator('#targetCaption').inner_text()
        assert '正在执行' in page.locator('#resultText').inner_text()
        page.screenshot(path=str(output/'rsi-plan.png'),full_page=True)
        result={'motion_completed':True,'grasp_attempted':True,'placement_attempted':True,'physical_elapsed_s':7.8,
                'within_10s':True,'recording_covers_motion':True,'capture_trial_id':'a'*32,'keyframes_file':'/local/keyframes.jpg'}
        model['job'].update(status='completed',phase='finished',result=result);model['learning']['phase']='analyzing'
        health.update(pending=None,armed=False)
        emit('motion',phase='close',attempt=1,plan_id=key)
        emit('motion',phase='lift',attempt=1,plan_id=key)
        emit('phase',phase='analyzing',attempt=1,plan_id=key)
        emit('grok_started',preflight=False,model='grok-4.7-build',frame_count=8,attempt=1,plan_id=key)
        process['model_output']={'plan_id':key,'attempt':1,'preflight':False,'status':'generating','mode':'streaming','text':'球仍在桌面，'}
        page.locator('#phase').filter(has_text='Grok正在分析').wait_for()
        page.locator('#grokLiveText').filter(has_text='球仍在桌面').wait_for()
        assert '尚未确认' in page.locator('#grokReplyStatus').inner_text()
        assert model['plan'].get('review') is None
        process['model_output']['text']+='夹爪在球左侧合拢，建议向画面右侧修正。'
        emit('grok_mode',mode='streaming',attempt=1,plan_id=key)
        page.locator('#grokLiveText').filter(has_text='建议向画面右侧修正').wait_for()
        page.screenshot(path=str(output/'rsi-streaming.png'),full_page=True)
        posts=len([call for call in calls if call[0]=='POST']);page.reload()
        page.locator('#phase').filter(has_text='Grok正在分析').wait_for()
        page.locator('#grokLiveText').filter(has_text='建议向画面右侧修正').wait_for()
        assert len([call for call in calls if call[0]=='POST'])==posts
        review={'outcome':'grasp_missed','direction':'image_left','magnitude':'large','source':'opencode_agent',
                'note':'主Agent选择12mm可见试验','visual_review':{'note':'夹爪偏左，球未离桌'},'changes':['下一轮修正抓取偏移12mm。'],'after_version':2,
                'ball_state':'table','adjustment':'pickup_xy','confidence':.92}
        review['proposal']={'parameters_changed':['pickup_xy_mm'],'before':{'pickup_xy_mm':[0,0]},
                            'after':{'pickup_xy_mm':[0,-12]},'robot_delta_mm':[0,-12],
                            'predicted_pixel_delta':[14,0],'allow_retry':True}
        model['plan']['review']=review;model['plan']['backup_url']=f'/api/bench/rsi/plans/{key}/archive'
        model['learning'].update(phase='recovering',review=review,changes=review['changes'])
        process['model_output'].update(status='validated',verdict=review)
        emit('grok_validated',verdict=review,attempt=1,plan_id=key)
        emit('policy',review=review,changes=review['changes'],attempt=1,plan_id=key)
        emit('recovery',status='completed',attempt=1)
        page.locator('#phase').filter(has_text='恢复官方初始姿态').wait_for()
        page.locator('#grokVerdict').filter(has_text='夹爪偏左').wait_for()
        assert '调整抓取平面偏移' in page.locator('#grokVerdict').inner_text()
        assert 'Grok画面依据' in page.locator('#grokVerdict').inner_text()
        assert '执行Agent决策' in page.locator('#grokVerdict').inner_text()
        page.locator('#processEvents').filter(has_text='程序采用的修正').wait_for()
        count=page.locator('#processEvents li').count()
        page.evaluate('window.RSIProcess.close();window.RSIProcess.connect()')
        page.wait_for_timeout(450);assert page.locator('#processEvents li').count()==count
        page.locator('#changeDetails summary').click();assert '→' in page.locator('#changeFacts').inner_text()
        assert page.locator('#start').is_disabled()
        # No user review or second execution request: backend starts round two.
        key='2'*32;model['plan']=plan(key,2);model['learning'].update(phase='executing',attempt=2)
        model['job']={'status':'running','phase':'close','plan_id':key}
        health.update(armed=True,pending={'kind':'rsi_trial'})
        page.locator('#attemptBadge').filter(has_text='2 / 5').wait_for();page.locator('#phase').filter(has_text='合抓').wait_for()
        paired={'available':True,'plan_id':key,'reference_plan_id':'1'*32,'panel_url':f'/api/bench/rsi/plans/{key}/comparison',
                'parameter_changes':[{'name':'pickup_xy_mm','before':[0,0],'after':[0,-12]}],
                'requested_xy_delta_mm':[0,-12],'expected_image_delta_px':[14,0],
                'feedback':{'available':True,'status':'consistent','reported_xy_delta_mm':[0,-11.8]},'assessment':None}
        emit('comparison',comparison=paired,attempt=2,plan_id=key)
        model['learning']['phase']='analyzing';model['job'].update(status='completed',result=result)
        process['model_output']={'plan_id':key,'attempt':2,'preflight':False,'status':'generating','mode':'streaming','text':'第二轮夹爪更接近小球。'}
        page.locator('#comparisonFrame').wait_for(state='visible')
        page.locator('#comparisonFacts').filter(has_text='11.80').wait_for()
        assert '不是图像实测' in page.locator('#comparisonFacts').inner_text()
        page.screenshot(path=str(output/'rsi-effect-comparison.png'),full_page=True)
        success={'outcome':'placed','source':'company_grok','note':'球随夹爪离桌，释放后留在漏斗内','changes':['保留成功策略。'],'after_version':3}
        success['comparison']={**paired,'assessment':{'reference_plan_id':'1'*32,'effect':'improved','movement':'as_expected','note':'前后实帧显示夹爪更接近并夹住了球'}}
        model['plan'].update(status='completed',review=success,backup_url=f'/api/bench/rsi/plans/{key}/archive')
        model['job'].update(status='completed',phase='finished',result=result)
        model['learning'].update(status='completed',phase='completed',review=success,changes=success['changes'])
        process['model_output']={'plan_id':key,'attempt':2,'preflight':False,'status':'validated','mode':'streaming',
                                 'text':'球随夹爪离桌，释放后留在漏斗内。','verdict':success}
        emit('motion',phase='close',attempt=2,plan_id=key)
        emit('grok_validated',verdict=success,attempt=2,plan_id=key)
        emit('policy',review=success,changes=success['changes'],attempt=2,plan_id=key)
        emit('phase',phase='completed',attempt=2,plan_id=key)
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
        assert page.locator('#start').is_disabled();assert page.locator('#reset').is_enabled()
        page.locator('#reset').click();page.locator('#phase').filter(has_text='恢复官方初始姿态').wait_for()
        model['learning'].update(status='completed',phase='reset_completed')
        page.locator('#phase').filter(has_text='已恢复官方初始姿态').wait_for()
        assert len([call for call in calls if call[1]=='/bench/rsi/learning/reset'])==1
        model['job']['result']={**result,'motion_completed':False,'motion_error':'transport clearance failure',
            'execution_diagnosis':{'category':'clearance_shortfall','lift':{
                'actual_height_mm':56.,'required_transfer_height_mm':72.4,'actual_xyz_mm':[340,-10,-59]}}}
        page.locator('#resultText').filter(has_text='实测离桌 56.0 mm，运输要求 72.4 mm').wait_for()
        assert '这里的高度不是机器人Z' in page.locator('#resultText').inner_text()
        model['api_version']=5
        page.locator('#grokStatus').filter(has_text='网页服务仍是旧版').wait_for()
        assert page.locator('#start').is_disabled()
        assert not errors,errors
        browser.close()
    report={'real_hardware_access':False,'browser':'Chromium','checks':[
        'single learning start with explicit scope; no double click replay','live workspace without motion on load',
        'no mandatory human result form','Grok analysis progress survives reload','automatic recovery and round two without browser action',
        'successful Grok verdict and persistent policy shown','native H.264 playback','backup and raw download links',
        'depth diagnostics folded with distance legend','stop cancels further learning','missing Grok blocks unattended execution but not official reset',
        'observation target overlays and camera-relative side labels','before/after policy changes shown',
        'stale backend blocks new learning with restart explanation','public Grok response before final validation',
        'stream reconnect and page reload preserve process without duplicate events or execution',
        'structured outcome, direction, suggestion and confidence shown','paired actual frames and commanded vs reported displacement',
        '390px layout','no page errors']}
    report['checks'].append('fresh object coordinates and snapshot without a grasp plan or motion; recapture replaces prior coordinates')
    report['checks'].append('execution clearance diagnosis uses plane-relative height rather than robot Z')
    (output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8893')
    parser.add_argument('--output',required=True);args=parser.parse_args();check(args.url,args.output)
