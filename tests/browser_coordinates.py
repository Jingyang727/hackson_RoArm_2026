"""Chromium XYZ/depth acceptance with every device API intercepted."""
import argparse
import json
from pathlib import Path

import cv2
from playwright.sync_api import sync_playwright
from tests.test_ball_trial import SimulatedArm, scene
from evoarm_rsi.roarm_geometry import forward


def check(url,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    raw=SimulatedArm().status();raw['g']=3.155555287605748  # Real device reports ~180.8 at its closed stop.
    raw.update({'torswitch'+j:True for j in 'BSETRG'})
    plane={'normal_up':[0,0,1.],'d_mm':115.}
    h={'connected':True,'motion_enabled':True,'torque_status':[True]*6,'pending':None,'fault':None}
    model={'robot':raw,'model':forward(raw),'health':h,'table_plane':plane,'setup_id':'bench-layout-v2','height_mm':260.,'result':None}
    calls=[];errors=[];key='a'*32
    image,_=scene();_,jpeg=cv2.imencode('.jpg',image)
    def route(handler):
        request=handler.request;path=request.url.split('/api',1)[1].split('?',1)[0]
        body=request.post_data_json if request.method=='POST' else None;calls.append((request.method,path,body))
        if path.startswith('/coordinates/depth/'):
            return handler.fulfill(status=200,content_type='image/jpeg',body=jpeg.tobytes())
        if path=='/coordinates/state':
            data=dict(model)
            if model.get('canonical_torque'):
                data['torque_feedback']={'source':'current_T1051_reply','fresh':True,
                    'joints':[{'joint':i+1,'field':'torswitch'+j,'raw':model['robot'].get('torswitch'+j)} for i,j in enumerate('BSETRG')]}
        elif path=='/coordinates/prepare':
            target=body['xyz_mm'];model['result']={'plan_id':key,'status':'prepared','target_xyz_mm':target,'physical_accuracy':'not_checked'}
            data={'plan_id':key,'expires_in_s':180,'plan':{'target_xyz_mm':target,'distance_mm':20.,
                  'gripper_hold':{'normalized':True,'feedback_deg':180.8,'command_deg':179.90874767},
                  'predicted_joints_xyz_mm':model['model']['joints_xyz_mm']}}
        elif path=='/coordinates/move':
            assert body['plan_id']==key
            assert body['declaration']=='已核对XYZ目标和路径，现场有人看护且双手离开运动范围'
            h['pending']={'kind':'coordinate_test'};model['result']['status']='moving';data={'sent':True,**model['result']}
        elif path=='/coordinates/verify':
            model['result']['physical_accuracy']=body['result'];data=model['result']
        elif path=='/disarm':data={'locked':True}
        elif path=='/coordinates/depth-snapshot':
            data={'snapshot_id':'b'*32,'trial_id':'c'*32,'frame_index':50,'image_size':[1280,720],
                  'color_range_mm':[460,520],'valid_fraction':.9,'rgb_url':'/api/coordinates/depth/'+'b'*32+'/rgb',
                  'depth_url':'/api/coordinates/depth/'+'b'*32+'/depth','source':'recorded_rgbd_not_live'}
        elif path=='/coordinates/depth-point':
            data={'valid':True,'depth_mm':480.,'camera_xyz_mm':[30.,15.,480.],'robot_xyz_mm':[315.,-16.,-95.],
                  'height_mm':20.,'motion_sent':False}
        else:raise AssertionError('unexpected API '+path)
        handler.fulfill(status=200,content_type='application/json',body=json.dumps(data))
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport={'width':1440,'height':1080});page.route('**/api/**',route)
        page.on('pageerror',lambda exc:errors.append(str(exc)))
        page.goto(url+'/coordinates');page.locator('#actualX').filter(has_text='340.0').wait_for()
        assert not any(c[0]=='POST' for c in calls)
        assert page.locator('#move').is_disabled()
        assert page.locator('#torqueFeedback .locked').count()==6
        assert '请先预览当前目标' in page.locator('#moveReason').inner_text()
        assert '扭矩' not in page.locator('#moveReason').inner_text()
        assert page.locator('#x').get_attribute('readonly') is not None
        page.locator('#viewFront').click();page.locator('#scene').click(position={'x':200,'y':200})
        page.locator('#selectionHint').filter(has_text='无法确定桌面位置').wait_for()
        page.locator('#viewTop').click()
        box=page.locator('#scene').bounding_box();scale=min(box['width']/700,box['height']/620)
        page.locator('#scene').click(position={'x':box['width']*.48+180*scale,'y':box['height']*.48+30*scale})
        assert abs(float(page.locator('#x').input_value())-350)<.2
        assert abs(float(page.locator('#y').input_value())-30)<.2
        assert page.locator('#heightRange').get_attribute('min')=='0'
        page.locator('#heightRange').evaluate("e=>{e.value='10';e.dispatchEvent(new Event('input',{bubbles:true}));}")
        assert abs(float(page.locator('#z').input_value())+105)<.02
        assert page.locator('#prepare').is_enabled()
        assert '至少25' not in page.locator('body').inner_text()
        page.locator('#heightRange').evaluate("e=>{e.value='150';e.dispatchEvent(new Event('input',{bubbles:true}));}")
        assert abs(float(page.locator('#z').input_value())-35)<.02
        assert not any(c[0]=='POST' for c in calls)  # Picking and slider movement cannot actuate.
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert '尚未勾选现场确认' in page.locator('#moveReason').inner_text()
        page.locator('#confirmed').check();assert page.locator('#move').is_enabled()
        model['canonical_torque']=True  # Also exercise the new backend snapshot, not only the legacy fallback.
        # The health cache stays 'locked'; only this current reply changes.
        # A released or unreported joint cannot be silently treated as locked.
        raw['torswitchS']=False
        page.locator('#torqueFeedback [data-joint="2"]').filter(has_text='释放').wait_for()
        assert page.locator('#prepare').is_enabled();assert page.locator('#move').is_disabled()
        assert '肩释放' in page.locator('#moveReason').inner_text()
        page.screenshot(path=str(output/'coordinates-released-torque.png'),full_page=True)
        raw['torswitchS']=None
        page.locator('#torqueFeedback [data-joint="2"]').filter(has_text='未知').wait_for()
        assert page.locator('#prepare').is_enabled();assert page.locator('#move').is_disabled()
        assert '肩未知' in page.locator('#moveReason').inner_text()
        raw['torswitchS']=True
        page.wait_for_function("!document.getElementById('move').disabled")
        selected=[page.locator('#'+axis).input_value() for axis in ('x','y','z')]
        page.locator('#view3d').click();box=page.locator('#scene').bounding_box()
        page.mouse.move(box['x']+180,box['y']+180);page.mouse.down()
        page.mouse.move(box['x']+215,box['y']+200,steps=5);page.mouse.up()
        assert [page.locator('#'+axis).input_value() for axis in ('x','y','z')]==selected
        assert page.locator('#move').is_enabled()  # Camera orbit changes no robot target.
        page.locator('#view3d').click()
        page.screenshot(path=str(output/'coordinates-point-selection.png'),full_page=True)
        page.locator('#heightRange').evaluate("e=>{e.value='160';e.dispatchEvent(new Event('input',{bubbles:true}));}")
        assert page.locator('#move').is_disabled();assert not page.locator('#confirmed').is_checked()
        assert not any(c[1]=='/coordinates/move' for c in calls)
        page.locator('#numericMode').click()
        page.locator('#useCurrent').click();page.locator('#x').fill('360')
        assert page.locator('#move').is_disabled()
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        assert '180.8°' in page.locator('#planNote').inner_text()
        assert '179.9°' in page.locator('#planNote').inner_text()
        assert not any(c[1]=='/coordinates/move' for c in calls)
        page.locator('#confirmed').check();assert page.locator('#move').is_enabled()
        page.locator('#viewTop').click();page.locator('#view3d').click()
        page.screenshot(path=str(output/'coordinates-desktop.png'),full_page=True)
        page.locator('#y').fill('10');assert page.locator('#move').is_disabled();assert not page.locator('#confirmed').is_checked()
        page.locator('#prepare').click();page.wait_for_function("!document.getElementById('confirmed').disabled")
        page.locator('#confirmed').check();page.locator('#move').click();page.evaluate("document.getElementById('move').click()")
        page.locator('#summary').filter(has_text='等待实际反馈').wait_for()
        assert len([c for c in calls if c[1]=='/coordinates/move'])==1
        assert page.locator('#x').is_disabled()
        count=len([c for c in calls if c[0]=='POST']);page.reload()
        page.locator('#summary').filter(has_text='等待实际反馈').wait_for()
        assert len([c for c in calls if c[0]=='POST'])==count
        assert float(page.locator('#x').input_value())==360  # Restore the running target, not current XYZ.
        h['pending']=None;model['result'].update(status='settled_with_offset',actual_xyz_mm=[350.,10.,145.],
            actual_minus_target_mm=[-10.,0.,0.],position_error_mm=10.,pose_error_deg={'pitch':.4,'roll':.1,'gripper':.2},movement_observed=True)
        model['robot']={**raw,'x':350.,'y':10.};model['model']=forward(raw)
        page.locator('#summary').filter(has_text='反馈已稳定，但与目标有偏差').wait_for()
        assert '反馈－目标：-10.0, 0.0, 0.0' in page.locator('#summary').inner_text()
        page.locator('input[name=verified][value=offset]').check();page.locator('#note').fill('实物偏右')
        page.locator('#saveCheck').click();page.locator('#saved').filter(has_text='有偏差').wait_for()
        page.locator('#depthPanel summary').click();page.locator('#readDepth').click()
        page.locator('#depthRGB').wait_for(state='visible');page.locator('#depthRGB').click(position={'x':100,'y':100})
        page.locator('#depthPoint').filter(has_text='机械臂 XYZ').wait_for()
        assert '不是球心' in page.locator('#depthPoint').inner_text()
        assert len([c for c in calls if c[1]=='/coordinates/move'])==1
        page.set_viewport_size({'width':390,'height':844})
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'coordinates-mobile.png'),full_page=True)
        assert not errors,errors;browser.close()
    report={'real_hardware_access':False,'browser':'Chromium','api_calls':len(calls),'checks':[
        '3D robot/table/frame and target drawn','raw 180.8 feedback and legal 179.9 hold both visible',
        'boolean torque flags display locked','released or unknown current torque blocks movement but allows preview',
        'disabled button reasons visible','height below 25mm selectable','no motion on load, preview or depth clicks',
        'table click and height slider generate XYZ without motion','changed target invalidates plan','one move under double click','reload restores target and never resends',
        'per-axis feedback offset visible','physical verification stored separately','recorded depth surface projected',
        '390px layout','no page errors']}
    (output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8879')
    parser.add_argument('--output',required=True);args=parser.parse_args();check(args.url,args.output)
