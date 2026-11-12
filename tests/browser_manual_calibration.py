"""Real Chromium UI acceptance against simulated API responses only.

Run separately from hardware-free unit tests with a local UI server:
python tests/browser_manual_calibration.py --url http://127.0.0.1:8876 --output /tmp/evoarm-ui
Never connects to real serial or camera: every /api request is intercepted.
"""
import argparse
import json
from urllib.parse import parse_qs, urlsplit
from pathlib import Path

import cv2
import numpy as np

from playwright.sync_api import sync_playwright


def check(url, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    points=[{"id":key,"grid":grid,"captured":False,"pose_warnings":[]} for key,grid in
            {"P1":[1,1],"P2":[7,1],"P3":[7,4],"P4":[1,4],"V1":[4,2]}.items()]
    model={"mode":"locked","known":True,"error":False,"sequence":0,"hold":False}
    held=[]
    calls=[]
    def health():
        return {"connected":True,"teach_enabled":True,"pending":None,"fault":None,
                "teach_active":model["mode"]=="manual",
                "torque_status":[0 if model["mode"]=="manual" else 1]*6 if model["known"] else None}
    def state():
        return {"setup_id":"bench-layout-v2","session_exists":True,"points":points,"health":health(),
                "next_point":next((p["id"] for p in points if not p["captured"]),None),
                "current_image_available":False,"reference_trial_id":"simulated-browser-fixture",
                "image_size":[900,500],"overlay_points":[{"id":p["id"],"grid":p["grid"],
                    "pixel_xy":[315+p["grid"][0]*35,125+p["grid"][1]*35]} for p in points],
                "fit":model.get('fit')}
    squares=''.join(f'<rect x="{280+c*35}" y="{90+r*35}" width="35" height="35" fill="{("#24302b" if (r+c)%2 else "#fafbf8")}"/>' for r in range(7) for c in range(10))
    labels=''.join(f'<circle cx="{315+p["grid"][0]*35}" cy="{125+p["grid"][1]*35}" r="8" stroke="#c84b35" stroke-width="2" fill="none"/><text x="{326+p["grid"][0]*35}" y="{117+p["grid"][1]*35}" fill="#a94432" font-size="14">{p["id"]}</text>' for p in points)
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500"><rect width="900" height="500" fill="#e9ece5"/><rect x="250" y="60" width="410" height="310" fill="white"/>{squares}{labels}<text x="24" y="475" fill="#89928a" font-size="13">界面预览 · 模拟棋盘与反馈</text></svg>'
    raw=np.full((500,900,3),235,np.uint8)
    for r in range(7):
        for c in range(10):
            cv2.rectangle(raw,(280+c*35,90+r*35),(314+c*35,124+r*35),
                          (43,49,45) if (r+c)%2 else (250,250,250),-1)
    # Deliberate synthetic occlusion: markers must survive a hidden corner.
    cv2.rectangle(raw,(324,15),(350,190),(70,70,70),-1)
    cv2.putText(raw,"SIMULATED CAMERA / NO HARDWARE",(24,475),cv2.FONT_HERSHEY_SIMPLEX,.6,(100,110,100),1)
    ok,jpeg=cv2.imencode('.jpg',raw);assert ok
    def live_response(handler):
        after=int(parse_qs(urlsplit(handler.request.url).query).get('after',['0'])[0])
        model['sequence']=max(model['sequence']+1,after+1)
        handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes(),headers={
            'X-Frame-Sequence':str(model['sequence']),'X-Frame-Age-Ms':'0',
            'X-Frame-Width':'900','X-Frame-Height':'500'})
    def route(handler):
        request=handler.request;path=request.url.split('/api',1)[-1].split('?',1)[0]
        calls.append((request.method,path,request.post_data_json if request.method=='POST' else None))
        if path=='/bench/frame':
            return handler.fulfill(status=200,content_type='image/svg+xml',body=svg)
        if path=='/bench/live-frame':
            if model['hold']:held.append(handler);return
            return live_response(handler)
        if path=='/bench/status':data=state()
        elif path=='/bench/live/start':data={'started':True,'robot_control':False}
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
        page.locator('#liveToggle').click()
        page.get_by_text('实时 · 五点位置叠加',exact=True).wait_for()
        assert page.locator('#pointOverlay g').count()==5
        assert page.locator('#pointOverlay').is_visible()
        assert page.locator('#pointOverlay g.selected').get_attribute('data-point')=='P1'
        page.screenshot(path=str(output/'calibration-live-desktop.png'),full_page=True)
        page.set_viewport_size({'width':390,'height':844})
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        # Image and SVG share object-fit/preserveAspectRatio even with letterboxing.
        assert page.evaluate("""() => {
          const image=document.getElementById('frame').getBoundingClientRect();
          const overlay=document.getElementById('pointOverlay').getBoundingClientRect();
          return Math.abs(image.x-overlay.x)<1 && Math.abs(image.width-overlay.width)<1;
        }""")
        page.screenshot(path=str(output/'calibration-live-mobile.png'),full_page=True)
        page.set_viewport_size({'width':1440,'height':1080})
        model['hold']=True
        page.locator('#aligned').check()
        page.wait_for_function("document.getElementById('pointOverlay').hidden && document.getElementById('capture').disabled")
        assert not page.locator('#aligned').is_checked()
        page.locator('#supported').check()
        assert page.locator('#lock').is_enabled()  # camera cannot block torque restore
        page.locator('#liveToggle').click()  # abort stalled request; freeze is labelled
        assert page.locator('#liveNotice').is_visible()
        model['hold']=False
        for handler in held:
            try:handler.abort()
            except Exception:pass
        held.clear()
        page.locator('#liveToggle').click()
        page.get_by_text('实时 · 五点位置叠加',exact=True).wait_for()
        for key in ('P1','P2','P3','P4','V1'):
            assert page.locator('#pointOverlay g.selected').get_attribute('data-point')==key
            assert page.locator('#capture').is_disabled()
            page.locator('#aligned').check();page.get_by_role('button',name='记录 '+key,exact=True).click()
            page.wait_for_function("!document.getElementById('aligned').checked")
        assert page.locator('#pointOverlay g.captured').count()==5
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
                      "HTTP500 no JSON syntax error","live five-point overlay with selected/captured states",
                      "stale preview blocks alignment but allows torque restore","390px live overlay letterboxing",
                      "390px responsive layout","retired-page redirects","no page errors"]}
    (output/'report.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8876');parser.add_argument('--output',required=True)
    args=parser.parse_args();check(args.url,args.output)
