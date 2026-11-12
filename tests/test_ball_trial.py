"""Synthetic stock-arm, checkerboard and RGB-D evidence; no real hardware."""
import asyncio
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
import uuid
from unittest.mock import patch

import cv2
import numpy as np

from control_ui import main
from control_ui.service import ArmService
from evoarm_rsi import ball_trial
from evoarm_rsi.bench_calibration import POINTS,_digest,SESSION_KIND
from evoarm_rsi.camera import CameraFrame
from evoarm_rsi.camera_hub import CameraHub
from evoarm_rsi.camera_service import CameraServer
from evoarm_rsi.grasp_once import execute
from evoarm_rsi.roarm_geometry import forward,inverse
from tests.test_serial_transport_errors import api_request


K={"fx":600.,"fy":600.,"cx":640.,"cy":360.,"width":1280,"height":720,"aligned_to":"color","verified":False}
RB=np.array([[0.,-1,0],[-1,0,0],[0,0,-1.]])
TB=np.array([395.,110.,-115.])
TC=np.array([-96.,-60.,500.])


def scene(ball_height=17.):
    image=np.full((720,1280,3),215,np.uint8)
    depth=np.full((720,1280),500,np.uint16)
    for r in range(-1,6):
        for c in range(-1,9):
            u0=round(640+600*(24*c-96)/500);v0=round(360+600*(24*r-60)/500)
            u1=round(640+600*(24*(c+1)-96)/500);v1=round(360+600*(24*(r+1)-60)/500)
            cv2.rectangle(image,(u0,v0),(u1,v1),(25,25,25) if (r+c)%2 else (245,245,245),-1)
    z=500-ball_height;u=round(640+600*60/z);v=round(360+600*16.8/z);radius=round(600*17/z)
    cv2.circle(image,(u,v),radius,(0,210,230),-1)
    yy,xx=np.ogrid[:720,:1280];rho=((xx-u)*z/600)**2+((yy-v)*z/600)**2
    mask=rho<=17**2;depth[mask]=np.round(z-np.sqrt(np.maximum(0,17**2-rho[mask]))).astype(np.uint16)
    return image,depth


def layout(home):
    corners=[[640+600*(c*24-96)/500,360+600*(r*24-60)/500] for r in range(6) for c in range(9)]
    ref={"image_size":[1280,720],"square_mm":24.,"corners_px":corners}
    records={}
    for key,grid in POINTS.items():
        board=np.array([grid[0]*24.,grid[1]*24.,0.]);robot=RB@board+TB
        records[key]={"id":key,"board_mm":board[:2].tolist(),"robot_xyz_mm":robot.tolist(),
                      "tool_pose_rad":{"pitch":math.radians(80),"roll":0.,"gripper":math.radians(179)}}
    session={"kind":SESSION_KIND,"setup_id":"bench-layout-v2","session_id":"synthetic-trial",
             "reference":ref,"reference_sha256":_digest(ref),"points":POINTS,"records":records,"history":[]}
    path=home/".evoarm/rsi/calibrations/bench-layout-v2/session.json";path.parent.mkdir(parents=True)
    path.write_text(json.dumps(session));return session


def observation(home):
    trial_id=uuid.uuid4().hex;folder=home/".evoarm/rsi/experiments"/trial_id;folder.mkdir(parents=True)
    image,depth=scene();writer=cv2.VideoWriter(str(folder/"rgb.mp4"),cv2.VideoWriter_fourcc(*"mp4v"),10,(1280,720))
    stamps=[time.monotonic_ns()-1_000_000_000+i*200_000_000 for i in range(6)]
    for _ in stamps:writer.write(image)
    writer.release()
    (folder/"frames.jsonl").write_text("\n".join(json.dumps({"host_monotonic_ns":t}) for t in stamps))
    np.savez_compressed(folder/"depth_samples.npz",depth_mm=np.stack([depth]*6),monotonic_ns=stamps)
    (folder/"gemini_color_intrinsics.json").write_text(json.dumps(K))
    (folder/"manifest.json").write_text(json.dumps({"trial_id":trial_id,"simulated":False,"status":"recorded"}))
    return folder


class SimulatedArm:
    def __init__(self,contact=False):
        self.raw=inverse([340.,0.,145.],math.radians(22),0.,math.radians(179))
        self.commands=[];self.contact=contact;self.ball_height=17.;self.failed=False
    def status(self,timeout=None):
        return {**self.raw,**{"torswitch"+k:1 for k in "BSETRG"},"tG":20 if self.contact and math.degrees(self.raw["g"])>=158 and self.commands else 0}
    def send(self,payload):
        self.commands.append(payload)
        if self.failed:raise OSError("synthetic uncertain transmission")
        if payload["T"]==104:
            self.raw=inverse([payload[k] for k in "xyz"],payload["t"],payload["r"],payload["g"])
            if len([c for c in self.commands if c["T"]==104])==4:self.ball_height=67.
        elif payload["T"]==106:
            self.raw["g"]=min(payload["cmd"],math.radians(158)) if self.contact else payload["cmd"]
        else:raise AssertionError("unexpected command")


class BallGeometry(unittest.TestCase):
    def test_forward_inverse_roundtrip_for_nonsingular_stock_branch(self):
        for shoulder in (-20,10,35):
            for elbow in (45,80,120):
                s={"T":1051,"b":.1,"s":math.radians(shoulder),"e":math.radians(elbow),"t":.3,"r":0.,"g":2.8}
                pose=forward(s)
                if pose["xyz_mm"][0]<=50:continue
                recovered=inverse(pose["xyz_mm"],pose["pitch_rad"],0.,2.8)
                for key in "bset":self.assertAlmostEqual(recovered[key],s[key],places=7)

    def test_current_rgbd_and_real_dimensions_produce_grasp_not_a_start_only_plan(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp);session=layout(home);folder=observation(home)
            ball,geometry,image=ball_trial.observe_ball(folder,session)
            plan=ball_trial.build_plan(session,SimulatedArm().status(),ball,geometry)
            self.assertEqual([s["stage"] for s in plan["steps"]],["align_open","hover","descend","close","lift"])
            self.assertGreater(ball["board_check"]["visible_corners"],12)
            self.assertFalse(plan["taught_positions_used"])
            self.assertFalse(plan["experimental_policy"]["grasp_tcp_globally_verified"])
            self.assertFalse(K["verified"])
            self.assertEqual(plan["steps"][3]["gripper_rad"],math.radians(162))

    def test_bad_depth_or_camera_profile_does_not_become_verified(self):
        with tempfile.TemporaryDirectory() as temp:
            session=layout(Path(temp));geometry=ball_trial.camera_geometry(session,K);image,depth=scene()
            with self.assertRaises(ValueError):ball_trial.visible_board_check(image,depth+80,session,geometry)
            with self.assertRaises(ValueError):ball_trial.camera_geometry(session,{**K,"width":640})


class BallAPITest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.home=Path(self.temp.name)
        layout(self.home);self.folder=observation(self.home)
        self.arm=SimulatedArm();self.service=ArmService();self.service.arm=self.arm;self.service.armed_until=time.monotonic()+60
        self.nonce=uuid.uuid4().hex;self.camera_start=time.monotonic_ns()
        self.recording=False
        image,_=scene();_,jpeg=cv2.imencode('.jpg',image);self.jpeg=jpeg.tobytes()
        def camera(home,method,path,data=None):
            if path=="/record":return {"trial_id":self.folder.name}
            return {"recording":self.recording,"robot_control":False,"phase":"recording","recording_request_id":self.nonce,
                    "recording_started_monotonic_ns":self.camera_start}
        for p in (patch.object(main,"service",self.service),patch.object(Path,"home",return_value=self.home),
                  patch("control_ui.ball_trial.client_request",side_effect=camera),
                  patch("control_ui.ball_trial.preview_frame",return_value={"jpeg":self.jpeg}),
                  patch.dict(os.environ,{"EVOARM_ENABLE_MOTION":"1","EVOARM_SETUP_ID":"bench-layout-v2"})):
            p.start();self.addCleanup(p.stop)
    def request(self,path,body):return asyncio.run(api_request("/api/bench/grasp/"+path,body))
    def prepare(self):
        response,data=self.request("prepare",{});self.assertEqual(response["status"],200,data);return data
    def begin(self,key):
        self.recording=True;self.camera_start=time.monotonic_ns()
        response,data=self.request("begin",{"plan_id":key,"camera_request_id":self.nonce})
        self.assertEqual(response["status"],200,data)
    def confirm(self):
        self.service.status();self.assertIsNone(self.service.pending)
        self.service.last_motion_at-=1

    def test_prepare_is_readonly_and_begin_requires_a_real_lease(self):
        data=self.prepare();self.assertEqual(self.arm.commands,[])
        self.service.armed_until=0
        response,_=self.request("begin",{"plan_id":data["plan_id"],"camera_request_id":self.nonce})
        self.assertEqual(response["status"],409);self.assertEqual(self.arm.commands,[])
    def test_consumed_step_and_out_of_order_requests_cannot_move_twice(self):
        data=self.prepare();key=data["plan_id"];self.begin(key)
        response,_=self.request("step",{"plan_id":key,"step_index":1});self.assertEqual(response["status"],409)
        response,body=self.request("step",{"plan_id":key,"step_index":0});self.assertEqual(response["status"],200,body)
        response,_=self.request("step",{"plan_id":key,"step_index":0});self.assertEqual(response["status"],409)
        self.assertEqual(len(self.arm.commands),1)
    def test_fresh_ball_motion_blocks_descent(self):
        data=self.prepare();key=data["plan_id"];self.begin(key)
        self.request("step",{"plan_id":key,"step_index":0});self.confirm()
        image,_=scene();image=np.roll(image,30,axis=1);_,jpeg=cv2.imencode('.jpg',image)
        with patch("control_ui.ball_trial.preview_frame",return_value={"jpeg":jpeg.tobytes()}):
            response,_=self.request("step",{"plan_id":key,"step_index":1})
        self.assertEqual(response["status"],409);self.assertEqual(len(self.arm.commands),1)
    def test_full_angle_reached_trial_closes_and_lifts_without_old_teaching(self):
        data=self.prepare();key=data["plan_id"];self.begin(key)
        for i in range(5):
            response,body=self.request("step",{"plan_id":key,"step_index":i});self.assertEqual(response["status"],200,body)
            self.confirm()
        response,body=self.request("end",{"plan_id":key});self.assertTrue(body["completed_motion"])
        self.assertIsNone(body["visual_grasp_success"])
        self.assertEqual([p["T"] for p in self.arm.commands],[104,104,104,106,104])
        self.assertEqual(self.service.armed_until,0)
    def test_already_aligned_and_open_is_a_readback_noop_not_an_infinite_pending(self):
        first=self.prepare();align=first["plan"]["steps"][0]
        self.arm.raw=inverse(align["xyz_mm"],align["pitch_rad"],align["roll_rad"],align["gripper_rad"])
        data=self.prepare();key=data["plan_id"];self.begin(key)
        response,body=self.request("step",{"plan_id":key,"step_index":0})
        self.assertEqual(response["status"],200,body)
        self.assertFalse(body["sent"])
        self.assertEqual(self.arm.commands,[])
        self.assertIsNone(self.service.pending)
        self.assertTrue(self.service.last_command["verified_current_pose"])
    def test_uncertain_serial_write_is_consumed_and_fault_is_not_cleared(self):
        data=self.prepare();key=data["plan_id"];self.begin(key);self.arm.failed=True
        response,_=self.request("step",{"plan_id":key,"step_index":0});self.assertEqual(response["status"],409)
        self.assertEqual(self.service.fault["code"],"motion_transmission_unverified")
        response,_=self.request("step",{"plan_id":key,"step_index":0});self.assertEqual(response["status"],409)
        self.assertEqual(len(self.arm.commands),1)


class BallEndToEnd(unittest.TestCase):
    def test_grasp_runner_with_real_asgi_camera_http_and_contact_hold(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp);layout(home);arm=SimulatedArm(contact=True)
            service=ArmService();service.arm=arm;service.armed_until=time.monotonic()+60
            sources=[]
            class Source:
                color_intrinsics=K
                def __init__(self):self.frame_provider=self;sources.append(self)
                def start(self):pass
                def stop(self):pass
                def read(self):
                    time.sleep(.02);image,depth=scene(arm.ball_height)
                    return CameraFrame(image,time.monotonic_ns(),depth)
            server=CameraServer(home/"camera.sock",os.getuid(),os.getgid(),home,hub=CameraHub(Source))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def get(base,path):
                response,body=asyncio.run(api_request(path,{},method="GET"))
                if response["status"]!=200:raise RuntimeError(body)
                return body
            def post(base,path,body):
                response,data=asyncio.run(api_request(path,body))
                if response["status"]!=200:raise RuntimeError(data)
                return data
            try:
                with (patch.object(main,"service",service),patch.object(Path,"home",return_value=home),
                      patch.dict(os.environ,{"EVOARM_ENABLE_MOTION":"1","EVOARM_SETUP_ID":"bench-layout-v2"}),
                      patch("evoarm_rsi.camera_service.socket_path",return_value=home/"camera.sock"),
                      patch("evoarm_rsi.grasp_once._controller_get",side_effect=get),
                      patch("evoarm_rsi.grasp_once._controller_post",side_effect=post)):
                    prepared=post(None,"/api/bench/grasp/prepare",{})
                    self.assertEqual(arm.commands,[])
                    report=execute(prepared["plan_id"],home=home)
                self.assertTrue(report["motion_completed"],report)
                self.assertTrue(report["grasp_attempted"],report)
                self.assertTrue(report["recording_covers_motion"],report)
                self.assertIn("contact_hold",report)
                self.assertIsNone(report["visual_grasp_success"])
                self.assertEqual([c["T"] for c in arm.commands],[104,104,104,106,106,104])
                self.assertAlmostEqual(arm.commands[-1]["g"],math.radians(158),places=5)
                self.assertEqual(service.armed_until,0)
                self.assertEqual(len(sources),1)
                self.assertTrue(Path(report["keyframes_file"]).is_file())
            finally:
                server.shutdown();server.server_close();thread.join(timeout=2)
