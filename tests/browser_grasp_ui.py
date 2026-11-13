"""Chromium acceptance for user-driven grasp controls; all device APIs mocked."""
import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit, parse_qs

import cv2
from playwright.sync_api import sync_playwright
from tests.test_ball_trial import scene


def check(url, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    image,_=scene();cv2.ellipse(image,(965,405),(48,30),0,0,360,(18,18,18),-1)
    cv2.arrowedLine(image,(690,260),(690,390),(50,130,60),3)
    cv2.putText(image,'SIMULATED UI / NO HARDWARE',(30,680),cv2.FONT_HERSHEY_SIMPLEX,.7,(80,80,80),2)
    _,jpeg=cv2.imencode('.jpg',image)
    calls=[];errors=[]
    health={'connected':True,'motion_enabled':True,'torque_status':[1]*6,'fault':None,'pending':None}
    model={'health':health,'job':{'status':'idle'},'plan':None,'counter':0,'uncertain':False,'conflict':False,'sequence':0,'trials':[]}
    size={'observed_area_diameter_px':43.,'observed_enclosing_diameter_px':48.,'expected_diameter_px':42.,
          'relative_error_percent':2.38,'allowed_relative_error_percent':22.}
    def route(handler):
        request=handler.request;path=urlsplit(request.url).path;body=request.post_data_json if request.method=='POST' else None
        calls.append((request.method,path,body))
        if path.endswith('/preview') or path.endswith('/image') or path.endswith('/mask') or path.endswith('/keyframes') or path.endswith('/observation'):
            return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes())
        if path=='/api/bench/live-frame':
            after=int(parse_qs(urlsplit(request.url).query).get('after',['0'])[0]);model['sequence']=max(after+1,model['sequence']+1)
            return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes(),headers={
                'X-Frame-Sequence':str(model['sequence']),'X-Frame-Age-Ms':'20','X-Frame-Width':'1280','X-Frame-Height':'720'})
        status=200
        if path=='/api/bench/grasp/ui-state':data={k:model[k] for k in ('health','job','plan')};data['setup_id']='bench-layout-v2'
        elif path=='/api/task/trials':data={'setup_id':'bench-layout-v2','trials':model['trials']}
        elif path=='/api/auto-connect':health['connected']=True;data={'connected':True}
        elif path=='/api/bench/grasp/prepare':
            model['job']={'status':'idle'};model['plan']=None
            if model['conflict']:
                detail={'code':'target_coordinate_conflict','message':'当前球面深度与保存平面模型相差80.0mm，存在明确坐标冲突。',
                        'image_url':'/api/bench/grasp/diagnostics/'+'b'*32+'/observation'}
                model['job']['last_error']=detail;data={'detail':detail};status=409
            else:
                model['counter']+=1;key=f'{model["counter"]:032x}'
                warnings=['黄球表面深度稀疏；沿用已有五点标定和34mm球径估计目标，允许本轮试抓。','黄色轮廓尺寸偏差+42.9%；作为估计偏差记录，允许试抓。'] if model['uncertain'] else []
                measured={**size,'observed_area_diameter_px':60.,'relative_error_percent':42.9} if model['uncertain'] else size
                model['plan']={'plan_id':key,'status':'prepared','seconds_left':179,'expired':False,'warnings':warnings,
                               'image_size':[1280,720],'size':measured,'preview_url':f'/api/bench/grasp/plans/{key}/preview',
                               'mask_url':'/api/bench/grasp/diagnostics/'+'b'*32+'/mask'}
                data=model['plan']
        elif path=='/api/bench/grasp/run':
            assert body['plan_id']==model['plan']['plan_id']
            assert body['declaration']=='已核对黄球和路径，现场有人看护且双手离开运动范围'
            model['plan']['status']='active';model['job']={'status':'running','phase':'hover','plan_id':body['plan_id'],'stop_requested':False}
            data={'started':True};status=202
        elif path=='/api/bench/grasp/stop':
            model['job'].update(stop_requested=True,phase='stopping');data={'stop_requested':True}
        else:raise AssertionError('Unexpected API: '+path)
        handler.fulfill(status=status,content_type='application/json',body=json.dumps(data))
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1080})
        page.on('pageerror',lambda error:errors.append(str(error)));page.route('**/api/**',route)
        page.goto(url+'/experiments');page.wait_for_function("!document.getElementById('prepare').disabled")
        assert not any(c[0]=='POST' for c in calls)
        assert page.locator('#run').is_disabled()
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert page.locator('#run').is_disabled()
        assert page.locator('#observedSize').inner_text()=='43.0 px'
        page.locator('#confirmed').check();assert page.locator('#run').is_enabled()
        page.screenshot(path=str(output/'grasp-ready-desktop.png'),full_page=True)
        count=len([c for c in calls if c[0]=='POST'])
        page.reload();page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert not page.locator('#confirmed').is_checked();assert page.locator('#run').is_disabled()
        assert len([c for c in calls if c[0]=='POST'])==count
        model['plan']['expired']=True
        page.locator('#planExpiry').filter(has_text='已过期').wait_for()
        assert page.locator('#run').is_disabled()
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        page.locator('#confirmed').check();page.locator('#run').click()
        page.evaluate("document.getElementById('run').click()")
        page.locator('#phase').filter(has_text='球上方悬停').wait_for()
        page.wait_for_function("document.getElementById('imageCaption').textContent.includes('实时画面 ·')")
        assert len([c for c in calls if c[1]=='/api/bench/grasp/run'])==1
        assert page.locator('#prepare').is_disabled()
        page.screenshot(path=str(output/'grasp-running-desktop.png'),full_page=True)
        count=len([c for c in calls if c[0]=='POST'])
        page.reload();page.locator('#stop').wait_for(state='visible')
        assert len([c for c in calls if c[0]=='POST'])==count
        page.locator('#stop').click();page.locator('#phase').filter(has_text='已请求停止').wait_for()
        assert page.locator('#stop').is_disabled()
        model['job'].update(status='stopped');model['plan']['status']='aborted'
        page.locator('#phase').filter(has_text='后续动作已停止').wait_for()
        assert page.locator('#run').is_disabled()
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        page.locator('#confirmed').check();page.locator('#run').click()
        page.locator('#stop').wait_for(state='visible')
        model['job'].update(status='completed',result={'motion_completed':True,'grasp_attempted':True,'physical_elapsed_s':5.23,
                              'keyframes_url':'/api/bench/grasp/evidence/'+'c'*32+'/keyframes'})
        model['plan']['status']='completed_motion'
        model['trials']=[{'trial_id':'c'*32,'phase':'grasp_lift','pickup_observed':None,'ball_entered_funnel':None,
                          'within_10s':True,'evidence':['模拟抓取动作完成；视觉结果待核对']}]
        page.locator('#resultTitle').filter(has_text='抓取与提升动作已执行').wait_for()
        page.get_by_text('抓取试次 cccccccccc',exact=True).wait_for()
        assert '视觉抓取结果待核对' in page.locator('#resultText').inner_text()
        assert '抓起：无法确认' in page.locator('#trials').inner_text()
        page.set_viewport_size({'width':390,'height':844})
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'grasp-result-mobile.png'),full_page=True)
        model['uncertain']=True;page.locator('#prepare').click()
        page.locator('#observationWarnings').filter(has_text='允许带估计偏差试抓').wait_for()
        page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert page.locator('#observedSize').inner_text()=='60.0 px'
        assert page.locator('#maskLink').is_visible();assert not page.locator('#error').is_visible()
        page.locator('#confirmed').check();assert page.locator('#run').is_enabled()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'grasp-uncertain-allowed-mobile.png'),full_page=True)
        count=len([c for c in calls if c[0]=='POST'])
        page.reload();page.locator('#observationWarnings').filter(has_text='允许带估计偏差试抓').wait_for()
        assert len([c for c in calls if c[0]=='POST'])==count
        page.locator('#confirmed').check();assert page.locator('#run').is_enabled()
        page.locator('#run').click();page.locator('#stop').wait_for(state='visible')
        assert len([c for c in calls if c[1]=='/api/bench/grasp/run'])==3
        model['job'].update(status='completed',result={'motion_completed':True,'grasp_attempted':True})
        model['plan']['status']='completed_motion';page.locator('#stop').wait_for(state='hidden')
        model['conflict']=True;page.locator('#prepare').click()
        page.locator('#error').filter(has_text='80.0mm').wait_for()
        assert page.locator('#run').is_disabled();assert page.locator('#confirmed').is_disabled()
        assert not page.locator('#observationWarnings').is_visible()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'grasp-conflict-rejected-mobile.png'),full_page=True)
        assert not errors,errors
        browser.close()
    result={'browser':'Chromium','real_hardware_access':False,'api_calls':len(calls),
            'checks':['no actuation on load/reload','explicit image review before run','expired plan blocks run',
                      'single run under double click','live preview during execution','stop blocks following actions',
                      'reload reconnects to running job without resubmission','completion keeps visual success unknown',
                      'sparse ball depth and size warnings allow an explicit trial','coordinate conflict shows evidence and blocks run','390px responsive layout','no page errors']}
    (output/'report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8877')
    parser.add_argument('--output',required=True);args=parser.parse_args();check(args.url,args.output)
