"""Measured, bounded FIRST grasp/lift trial for the stock RoArm-M3-S tool.

No saved task poses and no model API. OpenCode's approved Grok reviews the
preview and the actual video. Tool offsets below are experimental first-
trial policy, never a claim of a globally verified grasp TCP.
"""
from __future__ import annotations

import math
from pathlib import Path
import json

import cv2
import numpy as np

from control_ui.safety import JOINTS, current_degrees
from control_ui.vision import find_yellow_ball, yellow_ball_mask
from .bench_calibration import fit_points
from .roarm_geometry import forward, inverse, height_above_plane
from .task_scene import load_rgbd

OPEN_DEG = 150.
CLOSE_DEG = 162.
BALL_RADIUS_MM = 17.
FIXED_FINGER_MARGIN_MM = 2.
GRASP_EXTRA_HEIGHT_MM = 3.
LIFT_MM = 50.
SPEED = .12  # unchanged conservative Cartesian speed used by existing execution


class ObservationRejected(ValueError):
    def __init__(self,message,evidence):
        super().__init__(message)
        self.evidence=evidence


def camera_geometry(session, intrinsics):
    ref=session["reference"]
    if intrinsics.get("aligned_to")!="color" or [intrinsics.get("width"),intrinsics.get("height")]!=ref["image_size"]:
        raise ValueError("RGB-D profile differs from the five-point reference")
    numbers=[intrinsics.get(k) for k in ("fx","fy","cx","cy")]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in numbers) or min(numbers[:2])<=0:
        raise ValueError("finite active-profile color intrinsics required")
    fx,fy,cx,cy=numbers
    K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    board=np.array([[c*ref["square_mm"],r*ref["square_mm"],0.] for r in range(6) for c in range(9)])
    corners=np.asarray(ref["corners_px"],dtype=float)
    ok,rv,tv=cv2.solvePnP(board,corners,K,None)
    if not ok:raise ValueError("reference camera pose could not be fitted")
    projected=cv2.projectPoints(board,rv,tv,K,None)[0][:,0,:]
    reprojection=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))))
    if reprojection>1.:raise ValueError("reference reprojection error exceeds one pixel")
    fit=fit_points(session["records"])
    if not fit["planar_fit_validated"]:raise ValueError("five-point fit failed held-out validation")
    Rc=cv2.Rodrigues(rv)[0];tc=tv[:,0]
    Rb=np.asarray(fit["board_to_robot_rotation_3x3"]);tb=np.asarray(fit["board_to_robot_translation_mm"])
    R=Rb@Rc.T;t=tb-R@tc
    normal=np.asarray(fit["table_plane_robot"]["normal_up"])
    nc=R.T@normal;dc=-float(nc@tc)
    return {"K":K,"inverse_K":np.linalg.inv(K),"R":R,"t":t,"plane":fit["table_plane_robot"],
            "normal_camera":nc,"d_camera":dc,"reprojection_rms_px":reprojection,
            "board_rotation":Rb,"board_translation":tb}


def pixel_at_height(pixel, height, geometry):
    ray=geometry["inverse_K"]@np.array([*pixel,1.])
    denominator=float(geometry["normal_camera"]@ray)
    if abs(denominator)<.1:raise ValueError("view ray is too parallel to the table")
    camera=ray*((height-geometry["d_camera"])/denominator)
    if camera[2]<=0:raise ValueError("target would be behind the camera")
    return geometry["R"]@camera+geometry["t"],camera


def visible_board_check(image, depth, session, geometry):
    """Assess optional current evidence; occlusion does not erase calibration.

    Keep raw residuals before classifying them. Missing corner/depth evidence
    is uncertainty; many visible corners with contradictory depth are a
    coordinate conflict. The old code discarded the latter as 'not enough'.
    """
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    visible=[];sampled=[];matched=[];residuals=[]
    height,width=gray.shape
    for uv in session["reference"]["corners_px"]:
        u,v=np.round(uv).astype(int)
        if not 9<=u<width-9 or not 9<=v<height-9:continue
        quadrants=[np.mean(gray[v+dy-1:v+dy+2,u+dx-1:u+dx+2]) for dx,dy in ((-4,-4),(4,4),(-4,4),(4,-4))]
        if abs(sum(quadrants[:2])-sum(quadrants[2:]))<60:continue
        corner=np.array([[uv]],np.float32)
        cv2.cornerSubPix(gray,corner,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,20,.01))
        if float(np.linalg.norm(corner[0,0]-uv))>3:continue
        visible.append(uv)
        patch=depth[v-3:v+4,u-3:u+4];values=patch[np.isfinite(patch)&(patch>0)]
        if len(values)<25:continue
        z=float(np.median(values));ray=geometry["inverse_K"]@np.array([u,v,1.])
        residual=float((ray*z)@geometry["normal_camera"]+geometry["d_camera"])
        sampled.append(uv);residuals.append(residual)
        if abs(residual)<=15:matched.append(uv)
    distributed=len(sampled)>=12 and float(np.ptp(np.asarray(sampled),axis=0).min())>=50
    median=float(np.median(np.abs(residuals))) if residuals else None
    conflict=bool(distributed and median>25)
    warnings=[]
    if not distributed:
        warnings.append(f"当前角点/深度证据不足（可见角点{len(visible)}个，含有效深度{len(sampled)}个）；沿用已有五点标定，允许本轮试抓。")
    elif median>8 and not conflict:
        warnings.append(f"当前桌面深度与保存平面相差约{median:.1f}mm；作为估计不确定性记录，沿用已标定平面试抓。")
    return {"status":"conflict" if conflict else "consistent" if distributed and median<=8 else "reference_reused",
            "visible_corners":len(matched),"rgb_corners":len(visible),"depth_corners":len(sampled),
            "distributed_depth_evidence":bool(distributed),"median_absolute_plane_residual_mm":median,
            "warnings":warnings,"calibration_source":"saved_five_point_session",
            "scope":"optional current evidence; no global intrinsic/TCP verification flag changed"}


def observe_ball(folder, session):
    folder=Path(folder)
    intr=json.loads((folder/"gemini_color_intrinsics.json").read_text())
    geometry=camera_geometry(session,intr)
    latest=load_rgbd(folder)
    if list(latest["bgr"].shape[1::-1])!=session["reference"]["image_size"]:
        raise ValueError("current image dimensions changed")
    if latest["trial_id"]!=folder.name:raise ValueError("observation manifest identifier does not match its folder")
    board=visible_board_check(latest["bgr"],latest["depth_mm"],session,geometry)
    warnings=list(board["warnings"])
    evidence={"kind":"supervised_rsi_observation_v1","trial_id":folder.name,"frame_index":latest["frame_index"],
              "calibration_session_id":session["session_id"],"board_check":board,"warnings":warnings,
              "target_estimated":False,"policy":"reuse saved planar calibration; perception quality is recorded, not a success prerequisite",
              "assumptions":["camera and robot base have not moved since calibration", "34mm ball rests on the calibrated table plane"]}
    def save_evidence():
        (folder/"grasp-observation.json").write_text(json.dumps(evidence,ensure_ascii=False,indent=2,allow_nan=False))
        out=latest["bgr"].copy()
        cv2.putText(out,f"SAVED CALIBRATION / {board['rgb_corners']} RGB CORNERS / {board['depth_corners']} DEPTH CORNERS",
                    (15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(30,30,230),2)
        if not cv2.imwrite(str(folder/"grasp-observation.jpg"),out):raise OSError("cannot save observation image")
    def reject(message,code):
        evidence.update(rejection_code=code,rejection_message=message);save_evidence()
        raise ObservationRejected(message,evidence)
    save_evidence()
    if board["status"]=="conflict":
        reject(f"当前{board['depth_corners']}个可见角点的桌面深度与保存平面相差约{board['median_absolute_plane_residual_mm']:.1f}mm，"
               "存在明确坐标冲突；已保存当前画面和数值，请检查相机位置或深度数据。","table_coordinate_conflict")
    rows=[json.loads(row) for row in (folder/"frames.jsonl").read_text().splitlines() if row]
    indices=sorted(set([0,len(rows)//2,latest["frame_index"]]))
    detections=[]
    cap=cv2.VideoCapture(str(folder/"rgb.mp4"))
    try:
        for index in indices:
            cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,image=cap.read()
            ball=find_yellow_ball(image) if ok else None
            if not ball or ball["candidate_count"]!=1 or ball["near_edge"]:
                reject("当前未找到唯一、完整的黄色球目标；已保存当前画面。","target_unavailable")
            detections.append(ball)
    finally:cap.release()
    pixels=np.asarray([b["center_px"] for b in detections])
    if np.max(np.linalg.norm(pixels-pixels.mean(axis=0),axis=1))>4:
        reject("黄球在观察期间明显移动；本次目标位置未稳定，已保存画面。","target_moved")
    ball=detections[-1];center,camera=pixel_at_height(ball["center_px"],BALL_RADIUS_MM,geometry)
    expected_radius=float(intr["fx"]*BALL_RADIUS_MM/camera[2])
    # An enclosing circle is disproportionately enlarged by a small yellow
    # reflection/spur. Use the visible contour area for this size check;
    # retain the old enclosing radius for ROI/stability consumers.
    size_radius=ball["area_radius_px"]
    relative=size_radius/expected_radius-1
    size={"source_trial_id":latest["trial_id"],"frame_index":latest["frame_index"],
          "measured_ball_diameter_mm":34.,"observed_area_diameter_px":round(2*size_radius,2),
          "observed_enclosing_diameter_px":round(2*ball["enclosing_radius_px"],2),
          "expected_diameter_px":round(2*expected_radius,2),"relative_error_percent":round(relative*100,2),
          "inferred_diameter_mm":round(34*(1+relative),2),"reference_relative_error_percent":22.,
          "predicted_center_camera_z_mm":round(float(camera[2]),2),"pixel_center":ball["center_px"],
          "method":"equal-area contour radius; enclosing circle retained as a diagnostic", "passed":abs(relative)<=.22,
          "blocks_execution":False}
    (folder/"ball-size-diagnostic.json").write_text(json.dumps(size,ensure_ascii=False,indent=2,allow_nan=False))
    mask=yellow_ball_mask(latest["bgr"]);cv2.imwrite(str(folder/"ball-size-mask.png"),mask)
    overlay=latest["bgr"].copy();pixel=tuple(np.round(ball["center_px"]).astype(int))
    cv2.circle(overlay,pixel,round(expected_radius),(255,180,20),2)
    cv2.circle(overlay,pixel,round(size_radius),(20,180,255),2)
    cv2.putText(overlay,f"area D={2*size_radius:.1f}px / expected D={2*expected_radius:.1f}px / error={relative*100:+.1f}%",
                (15,overlay.shape[0]-20),cv2.FONT_HERSHEY_SIMPLEX,.6,(30,30,230),2)
    cv2.imwrite(str(folder/"ball-size-overlay.jpg"),overlay)
    if abs(relative)>.22:
        warnings.append(f"黄色轮廓尺寸偏差{relative*100:+.1f}%，超出±22%参考带；保留实测34mm球径进行一次估计抓取，请核对预览。")
    local=geometry["board_rotation"].T@(center-geometry["board_translation"])
    square=session["reference"]["square_mm"]
    if not BALL_RADIUS_MM<=local[0]<=8*square-BALL_RADIUS_MM or not BALL_RADIUS_MM<=local[1]<=5*square-BALL_RADIUS_MM:
        warnings.append("黄球超出棋盘内框；沿用同一桌面平面外推目标，运动仍需通过可达性、限位和离桌检查。")
    u,v=np.round(ball["center_px"]).astype(int)
    patch=latest["depth_mm"][v-6:v+7,u-6:u+7];values=patch[np.isfinite(patch)&(patch>0)]
    error=float(np.median(values)-(camera[2]-BALL_RADIUS_MM)) if len(values) else None
    depth_evidence={"valid_samples":len(values),"expected_surface_camera_z_mm":float(camera[2]-BALL_RADIUS_MM),
                    "surface_residual_mm":error,"target_source":"saved_table_plane_plus_measured_ball_radius"}
    evidence.update(ball_size=size,ball_depth=depth_evidence)
    if len(values)>=80 and abs(error)>50:
        reject(f"黄球表面深度与桌面球模型相差约{abs(error):.1f}mm，存在明确目标坐标冲突；已保存画面和测量。","target_coordinate_conflict")
    if len(values)<80 or abs(error)>12:
        warnings.append("黄球表面深度稀疏或偏差较大；本轮用已有平面标定和34mm球径估计目标，将不确定性与动作一起记录。")
    evidence.update(target_estimated=True,pixel_center=ball["center_px"],center_robot_mm=center.tolist())
    save_evidence()
    return {"pixel_center":ball["center_px"],"radius_px":ball["radius_px"],"center_robot_mm":center.tolist(),
            "frame_index":latest["frame_index"],"host_monotonic_ns":latest["host_monotonic_ns"],
            "source_trial_id":latest["trial_id"],"board_check":board,
            "size_evidence":size,
            "depth_evidence":depth_evidence,"warnings":warnings,"observation_file":"grasp-observation.json",
            "reprojection_rms_px":geometry["reprojection_rms_px"]},geometry,latest["bgr"]


def check_segment(initial, step, plane):
    """Validate the actual T104 XYZ/Pitch interpolation and stock link endpoints."""
    if step["kind"]!="cartesian":return
    start=[initial[k] for k in "xyz"];goal=step["xyz_mm"]
    count=max(40,math.ceil(math.dist(start,goal)/3),math.ceil(abs(step["pitch_rad"]-initial["tit"])/math.radians(1)))
    for i in range(count+1):
        a=i/count;point=[x+(y-x)*a for x,y in zip(start,goal)]
        pitch=initial["tit"]+(step["pitch_rad"]-initial["tit"])*a
        pose=inverse(point,pitch,step["roll_rad"],step["gripper_rad"])
        for j,angle in current_degrees(pose).items():
            if not JOINTS[j][2]<=angle<=JOINTS[j][3]:
                raise ValueError(f"{step['stage']} exceeds joint {j} training range")
        model=forward(pose)
        if height_above_plane(point,plane)<step["minimum_tip_height_mm"]-.5:
            raise ValueError("tip path violates measured table clearance")
        if any(height_above_plane(p,plane)<30 for p in model["joints_xyz_mm"][1:3]):
            raise ValueError("an upstream link is too close to the table")


def build_plan(session,status,ball,geometry):
    model=forward(status)
    if math.dist(model["xyz_mm"],[status[k] for k in "xyz"])>3 or abs(model["pitch_rad"]-status["tit"])>math.radians(1):
        raise ValueError("live firmware pose does not match stock-tool geometry")
    if height_above_plane(model["xyz_mm"],geometry["plane"])<100:
        raise ValueError("first grasp needs an already clear start; inspect measured start plan")
    if not math.radians(65)<=status["g"]<=math.pi+.02:
        raise ValueError("stock gripper feedback is outside its training range")
    reference=session["records"]["P1"]["tool_pose_rad"]
    if abs(status["r"]-reference["roll"])>math.radians(5):
        raise ValueError("stock gripper roll differs from the calibrated contact configuration")
    for j,angle in current_degrees(status).items():
        if not JOINTS[j][2]<=angle<=JOINTS[j][3]:raise ValueError(f"start joint {j} outside training range")
    center=np.asarray(ball["center_robot_mm"])
    normal=np.asarray(geometry["plane"]["normal_up"])
    base=math.atan2(center[1],center[0]);roll=status["r"]
    failures=[]
    for pitch_deg in (80.,75.):
        pitch=math.radians(pitch_deg)
        radial=np.array([math.sin(pitch)*math.cos(base),math.sin(pitch)*math.sin(base),math.cos(pitch)])
        tangent=np.array([-math.sin(base),math.cos(base),0.])
        opening_direction=radial*math.cos(roll)+tangent*math.sin(roll)
        grasp=center+normal*GRASP_EXTRA_HEIGHT_MM-(BALL_RADIUS_MM+FIXED_FINGER_MARGIN_MM)*opening_direction
        if height_above_plane(grasp,geometry["plane"])<12:continue
        align=[float(grasp[0]),float(grasp[1]),float(status["z"])]
        # Align along a high corridor, allowing an unnecessarily tall start
        # to descend at the same time instead of making an extra XY detour.
        current_height=height_above_plane(model["xyz_mm"],geometry["plane"])
        align_height=min(current_height,150.)
        align[2]=float((align_height-geometry["plane"]["d_mm"]-normal[0]*align[0]-normal[1]*align[1])/normal[2])
        align_pitch=min(max(status["tit"],math.radians(30)),math.radians(60))
        lift=(grasp+normal*LIFT_MM).tolist()
        hover=(grasp+normal*(50.-height_above_plane(grasp,geometry["plane"]))).tolist()
        steps=[{"stage":"align_open","kind":"cartesian","xyz_mm":align,"pitch_rad":align_pitch,
                "roll_rad":roll,"gripper_rad":math.radians(OPEN_DEG),"minimum_tip_height_mm":100},
               {"stage":"hover","kind":"cartesian","xyz_mm":hover,"pitch_rad":pitch,
                "roll_rad":roll,"gripper_rad":math.radians(OPEN_DEG),"minimum_tip_height_mm":50},
               {"stage":"descend","kind":"cartesian","xyz_mm":grasp.tolist(),"pitch_rad":pitch,
                "roll_rad":roll,"gripper_rad":math.radians(OPEN_DEG),"minimum_tip_height_mm":12},
               {"stage":"close","kind":"gripper","gripper_rad":math.radians(CLOSE_DEG)},
               {"stage":"lift","kind":"cartesian","xyz_mm":lift,"pitch_rad":pitch,
                "roll_rad":roll,"gripper_rad":math.radians(CLOSE_DEG),"minimum_tip_height_mm":8}]
        current=dict(status)
        try:
            for step in steps:
                check_segment(current,step,geometry["plane"])
                if step["kind"]=="cartesian":
                    current=inverse(step["xyz_mm"],step["pitch_rad"],roll,step["gripper_rad"])
                else:current["g"]=step["gripper_rad"]
        except ValueError as exc:
            failures.append(str(exc));continue
        return {"kind":"bounded_first_grasp_lift_v1","setup_id":session["setup_id"],
                "image_size":session["reference"]["image_size"],
                "calibration_session_id":session["session_id"],"reference_sha256":session["reference_sha256"],
                "initial_status":status,"ball":ball,"table_plane":geometry["plane"],"steps":steps,
                "observation_warnings":ball.get("warnings",[]),"perception_policy":"supervised_rsi_saved_plane_v1",
                "experimental_policy":{"open_deg":OPEN_DEG,"close_deg":CLOSE_DEG,
                    "fixed_finger_margin_mm":FIXED_FINGER_MARGIN_MM,"grasp_extra_height_mm":GRASP_EXTRA_HEIGHT_MM,
                    "stock_fixed_finger_side":"inward radial side","grasp_tcp_globally_verified":False},
                "taught_positions_used":False,"placement_attempt_included":False,
                "physical_budget_s":10.,"speed":SPEED,"visual_success":None,
                "review_required":"current scene and stock fixed-finger side must match the preview; supervised first trial"}
    raise ValueError("no bounded default-gripper path: "+"; ".join(failures))


def preview(image,plan,geometry):
    out=image.copy();K=geometry["K"]
    def project(point):
        camera=geometry["R"].T@(np.asarray(point)-geometry["t"])
        if camera[2]<=0:return None
        pixel=K@camera;return tuple(np.round(pixel[:2]/pixel[2]).astype(int))
    ball=tuple(np.round(plan["ball"]["pixel_center"]).astype(int))
    cv2.circle(out,ball,round(plan["ball"]["radius_px"]),(0,220,255),2)
    for step in plan["steps"]:
        if step["kind"]!="cartesian":continue
        pixel=project(step["xyz_mm"])
        if pixel:
            cv2.drawMarker(out,pixel,(50,70,255),cv2.MARKER_CROSS,18,2)
            cv2.putText(out,step["stage"],(pixel[0]+10,pixel[1]+15),cv2.FONT_HERSHEY_SIMPLEX,.55,(50,70,255),2)
    cv2.putText(out,f"RSI TRIAL / SAVED CALIBRATION / {len(plan.get('observation_warnings',[]))} OBSERVATION WARNINGS / NO MOTION YET",(15,out.shape[0]-20),
                cv2.FONT_HERSHEY_SIMPLEX,.6,(40,40,230),2)
    return out
