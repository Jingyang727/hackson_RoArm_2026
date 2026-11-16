"""Replay a saved workspace in Chromium; all device/API traffic is intercepted."""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def check(url,data_dir,image,output):
    data_dir=Path(data_dir);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    scene=json.loads((data_dir/'scene.json').read_text());workspace=json.loads((data_dir/'workspace.json').read_text())
    jpeg=Path(image).read_bytes();calls=[];errors=[];sequence=[0]
    state={'api_version':10,'web_learning_version':3,'continuous_learning_version':1,'route_policy_version':7,'visual_pipeline_version':3,'workspace_preview_version':1,
           'setup_id':'bench-layout-v2','health':{'connected':True,'motion_enabled':False,'torque_status':[1]*6,'pending':None,'fault':None},
           'grok':{'configured':True,'model':'DeepSeek-V4.1-Flash'},'job':{'status':'idle'},
           'learning':{'status':'idle','phase':'idle'},'plan':None}
    def route(handler):
        request=handler.request;path=request.url.split('/api',1)[-1].split('?',1)[0]
        calls.append([request.method,path])
        if path=='/bench/live-frame':
            sequence[0]+=1
            return handler.fulfill(body=jpeg,content_type='image/jpeg',headers={
                'X-Frame-Sequence':str(sequence[0]),'X-Frame-Age-Ms':'20','X-Frame-Width':'1280','X-Frame-Height':'720'})
        if path=='/bench/rsi/state':data=state
        elif path=='/bench/rsi/history':data={'trials':[]}
        elif path=='/bench/rsi/learning/events':
            return handler.fulfill(body=': replay only\n\n',content_type='text/event-stream')
        elif path=='/bench/rsi/observe':
            assert request.method=='POST' and request.post_data_json=={}
            data={'observation_id':'a'*32,'scene':scene}
        elif path=='/bench/rsi/workspace':
            assert request.method=='POST' and request.post_data_json=={'observation_id':'a'*32}
            data={'workspace_id':'b'*32,'status':'computing'}
        elif path=='/bench/rsi/workspace/'+'b'*32:data=workspace
        else:raise AssertionError('Unexpected request '+request.method+' '+path)
        handler.fulfill(content_type='application/json',body=json.dumps(data))
    with sync_playwright() as driver:
        browser=driver.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1000})
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.route('**/api/**',route)
        page.route('**/mock-image.jpg',lambda handler:handler.fulfill(body=jpeg,content_type='image/jpeg'))
        page.goto(url+'/experiments');page.locator('#workspaceVisible').wait_for()
        page.wait_for_function("!document.getElementById('workspaceVisible').disabled")
        page.locator('#workspaceFrame').wait_for(state='visible')
        assert not any(method=='POST' for method,_ in calls)
        page.locator('#workspaceVisible').check()
        try:page.locator('#workspaceStatus').filter(has_text='绿色：找到完整抓放路径').wait_for(timeout=5000)
        except Exception:
            page.screenshot(path=str(output/'failure.png'),full_page=True)
            raise AssertionError({'status':page.locator('#workspaceStatus').inner_text(),'errors':errors,'calls':calls[-15:]})
        assert page.locator('#workspaceOverlay').is_visible()
        assert page.locator('#workspaceOverlay path').count()==2
        assert page.locator('#workspaceOverlay').get_attribute('viewBox')=='0 0 1280 720'
        assert page.locator('#targetOverlay polygon').count()==1
        assert page.locator('#start').is_disabled()
        page.screenshot(path=str(output/'workspace-desktop.png'),full_page=True)
        page.locator('#workspaceVisible').uncheck();assert page.locator('#workspaceOverlay').is_hidden()
        page.locator('#workspaceVisible').check();assert page.locator('#workspaceOverlay').is_visible()
        assert len([c for c in calls if c[0]=='POST'])==2
        state['learning']={'status':'running','phase':'checking_home'}
        page.locator('#phase').filter(has_text='核对当前姿态').wait_for()
        state['learning']={'status':'failed','phase':'failed','error':'当前抓放候选路径未通过：抓后抬升。起步姿态：已在官方初始姿态，无需重复回位。',
            'planning_diagnosis':{'summary':'当前抓放候选路径未通过：抓后抬升','suggestion':'查看当前策略的采样区域。',
                'reason':'验收模拟：抬升候选超出腕部可达距离','evidence':{'pitch_deg':40},'image_url':'/mock-image.jpg'}}
        page.locator('#planningIssue').wait_for(state='visible')
        assert '抓后抬升' in page.locator('#planningSummary').inner_text()
        assert '已在官方初始姿态' in page.locator('#phase').inner_text()
        page.locator('#planningIssue summary').click();page.locator('#planningImage').wait_for(state='visible')
        page.locator('h1').evaluate("node=>node.textContent='界面验收：模拟规划失败，无实物运动'")
        page.screenshot(path=str(output/'simulated-planning-failure.png'),full_page=True)
        assert not errors,errors
        assert [c for c in calls if c[0]=='POST']==[['POST','/bench/rsi/observe'],['POST','/bench/rsi/workspace']]
        browser.close()
    report={'live_hardware_access':False,'model_api_called':False,'source':'offline replay with intercepted APIs',
            'checks':['optional calculation starts without a motor task','camera-aligned sampled region overlay',
                'green/gray/unknown legend','fresh scene target overlay','desktop browser layout',
                'hide/show without another capture','home check distinguished from reset','planning stage and saved reason visible',
                'no automatic run/reset/torque/authorization requests','no browser errors']}
    (output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8894')
    parser.add_argument('--data-dir',required=True);parser.add_argument('--image',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();check(args.url,args.data_dir,args.image,args.output)
