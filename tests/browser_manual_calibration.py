"""Real Chromium UI acceptance against simulated API responses only.

Run separately from hardware-free unit tests with a local UI server:
python tests/browser_manual_calibration.py --url http://127.0.0.1:8876 --output /tmp/evoarm-ui
Never connects to real serial or camera: every /api request is intercepted.
"""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def check(url, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    points=[{"id":key,"grid":grid,"captured":False,"pose_warnings":[]} for key,grid in
            {"P1":[1,1],"P2":[7,1],"P3":[7,4],"P4":[1,4],"V1":[4,2]}.items()]
    model={"mode":"locked","known":True,"error":False}
    calls=[]
    def health():
        return {"connected":True,"teach_enabled":True,"pending":None,"fault":None,
                "teach_active":model["mode"]=="manual",
                "torque_status":[0 if model["mode"]=="manual" else 1]*6 if model["known"] else None}
    def state():
        return {"setup_id":"bench-layout-v2","session_exists":True,"points":points,"health":health(),
                "next_point":next((p["id"] for p in points if not p["captured"]),None),
                "current_image_available":False,"reference_trial_id":"simulated-browser-fixture",
                "fit":model.get('fit')}
    squares=''.join(f'<rect x="{280+c*35}" y="{90+r*35}" width="35" height="35" fill="{("#24302b" if (r+c)%2 else "#fafbf8")}"/>' for r in range(7) for c in range(10))
    labels=''.join(f'<circle cx="{315+p["grid"][0]*35}" cy="{125+p["grid"][1]*35}" r="8" stroke="#c84b35" stroke-width="2" fill="none"/><text x="{326+p["grid"][0]*35}" y="{117+p["grid"][1]*35}" fill="#a94432" font-size="14">{p["id"]}</text>' for p in points)
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500"><rect width="900" height="500" fill="#e9ece5"/><rect x="250" y="60" width="410" height="310" fill="white"/>{squares}{labels}<text x="24" y="475" fill="#89928a" font-size="13">界面预览 · 模拟棋盘与反馈</text></svg>'
    def route(handler):
        request=handler.request;path=request.url.split('/api',1)[-1].split('?',1)[0]
        calls.append((request.method,path,request.post_data_json if request.method=='POST' else None))
        if path=='/bench/frame':
            return handler.fulfill(status=200,content_type='image/svg+xml',body=svg)
        if path=='/bench/status':data=state()
        elif path=='/bench/sources':data={"sources":[]}
        elif path=='/ports':data=[]
        elif path=='/bench/robot':data={"health":health(),"xyz_mm":[300.,20.,-90.],"pitch_rad":.06,"roll_rad":0.,"gripper_rad":3.}
        elif path=='/teach/torque':
            if model['error']:return handler.fulfill(status=500,content_type='text/plain',body='Internal Server Error')
            model['mode']='locked' if request.post_data_json['enabled'] else 'manual';data={"verified":True}
        elif path=='/bench/capture':
            assert model['mode']=='manual'
            assert request.post_data_json['aligned_and_supported'] is True
            next(p for p in points if p['id']==request.post_data_json['point_id'])['captured']=True
            data=state()
            if data['next_point'] is None:
                model['fit']={"planar_fit_validated":True,"train_rms_mm":2.,"validation_error_mm":3.2}
                data['fit']=model['fit']
        elif path=='/task/trials':data={"setup_id":"bench-layout-v2","trials":[]}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(status=200,content_type='application/json',body=json.dumps(data))
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={"width":1440,"height":1080},device_scale_factor=1)
        errors=[];page.on('pageerror',lambda error:errors.append(str(error)))
        page.route('**/api/**',route)
        page.goto(url+'/calibration');page.get_by_text('全部已锁定',exact=True).wait_for()
        page.locator('#frame').wait_for(state='visible')
        page.screenshot(path=str(output/'calibration-desktop.png'),full_page=True)
        assert not any(c[0]=='POST' for c in calls)
        page.locator('#supported').check();page.get_by_role('button',name='释放全部关节',exact=True).click()
        page.get_by_text('六关节已释放',exact=True).wait_for()
        for key in ('P1','P2','P3','P4','V1'):
            assert page.locator('#capture').is_disabled()
            page.locator('#aligned').check();page.get_by_role('button',name='记录 '+key,exact=True).click()
            page.wait_for_function("!document.getElementById('aligned').checked")
        assert len([c for c in calls if c[1]=='/teach/torque'])==1
        page.get_by_text('五点标定通过',exact=True).wait_for()
        assert page.locator('#progress').inner_text()=='5 / 5'
        page.screenshot(path=str(output/'calibration-complete.png'),full_page=True)
        page.locator('#supported').check();page.get_by_role('button',name='恢复全部扭矩',exact=True).click()
        page.get_by_text('全部已锁定',exact=True).wait_for()
        assert len([c for c in calls if c[1]=='/teach/torque'])==2
        # Reload must retain all points and must never issue another action.
        count=len([c for c in calls if c[0]=='POST'])
        page.reload();page.wait_for_function("document.getElementById('progress').textContent==='5 / 5'")
        assert len([c for c in calls if c[0]=='POST'])==count
        # Unknown torque cannot actuate even if the support
        # checkbox is clicked. HTTP500 exposes status without retrying writes.
        model['known']=False;page.reload();page.locator('#capabilityNote').filter(has_text='未核实').wait_for()
        page.locator('#supported').check();assert page.locator('#release').is_disabled()
        model['known']=True;model['error']=True;page.reload();page.locator('#supported').check()
        page.locator('#release').click();page.locator('#error').filter(has_text='HTTP 500').wait_for()
        assert 'Unexpected token' not in page.locator('#error').inner_text()
        model['error']=False
        page.set_viewport_size({"width":390,"height":844});page.reload()
        page.locator('#frame').wait_for(state='visible')
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'calibration-mobile.png'),full_page=True)
        page.goto(url+'/experiments');page.get_by_text('还没有抓放试次',exact=True).wait_for()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'experiments-mobile.png'),full_page=True)
        for source,target in (('/teach','/calibration'),('/pick','/experiments'),('/','/calibration')):
            page.goto(url+source);assert page.url.endswith(target)
        assert not errors,errors
        browser.close()
    result={"browser":"Chromium","real_hardware_access":False,"api_calls":len(calls),
            "physical_actions_simulated":len([c for c in calls if c[0]=='POST']),
            "checks":["one release, five supported captures, one final relock","retained progress on reload", "unknown torque block",
                      "HTTP500 no JSON syntax error","390px responsive layout","retired-page redirects","no page errors"]}
    (output/'report.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8876');parser.add_argument('--output',required=True)
    args=parser.parse_args();check(args.url,args.output)
