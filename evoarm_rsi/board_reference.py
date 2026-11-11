"""One-time persistent Gemini 9x6 checkerboard reference from Trial 003.1.

The checkerboard does NOT need to be empty on subsequent runs. Save a
frame's measured full 54-corner geometry once. Future RGB-D ball trials
reuse that same camera-to-board homography only while physical placement
is unchanged; later live detections can cross-check alignment.

Does not connect to camera or robot, and NEVER sends motor commands.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np


def create_reference(trial, *, output=None, min_board_frames=3, square_mm=24.0):
    import cv2
    from control_ui.vision import find_chessboard
    if (type(square_mm) not in (int, float) or not math.isfinite(square_mm)
            or not 0.1 <= square_mm <= 200):
        raise ValueError("measured square size must be between 0.1 and 200 mm")
    trial=Path(trial).expanduser()
    manifest=json.loads((trial/"manifest.json").read_text(encoding="utf-8"))
    if manifest.get("simulated") is not False:
        raise ValueError("real Gemini RGB-D trial required")
    if not (trial/"gemini_color_intrinsics.json").is_file():
        raise ValueError("source lacks Gemini color intrinsics")
    intr=json.loads((trial/"gemini_color_intrinsics.json").read_text(encoding="utf-8"))
    width,height=int(intr.get("width",0)),int(intr.get("height",0))
    if width<=0 or height<=0:
        raise ValueError("Gemini intrinsics image size missing")
    cap=cv2.VideoCapture(str(trial/"rgb.mp4"))
    if not cap.isOpened():
        raise ValueError("unable to read reference trial video")
    board=[]
    try:
        frame=0
        while True:
            ok,img=cap.read()
            if not ok:
                break
            if img.shape[:2]!=(height,width):
                raise ValueError("video resolution != Gemini intrinsics")
            corners,h=find_chessboard(img)
            if corners is not None:
                board.append((frame,corners.reshape(54,2)))
            frame+=1
    finally:
        cap.release()
    if len(board)<min_board_frames:
        raise ValueError("not enough real full-board frames for persistent reference")
    points=np.asarray([row[1] for row in board],dtype=float)
    middle=np.median(points,axis=0)
    distances=np.linalg.norm(points-middle,axis=2)
    error=np.median(distances,axis=1)
    stable=[i for i,e in enumerate(error) if e<=1]
    if len(stable)<min_board_frames:
        raise ValueError("checkerboard was not stable: cannot persist reference")
    corners=np.median(points[stable],axis=0).astype("float32")
    target=np.asarray([(c,r) for r in range(6) for c in range(9)],dtype="float32")
    H,_=cv2.findHomography(corners,target,0)
    if H is None or not np.isfinite(H).all():
        raise ValueError("no valid checkerboard homography")
    reference={
        "kind":"gemini_checkerboard_reference_v1",
        "source_trial_id":manifest["trial_id"],
        "source_frame_indices":[int(board[i][0]) for i in stable],
        "image_size":[width,height],
        "square_mm":float(square_mm),
        "corners_px":corners.tolist(),
        "pixel_to_grid_h":H.tolist(),
        "frames_verified":len(stable),
        "max_median_corner_motion_px":round(float(max(error[i] for i in stable)),3),
        "physical_placement_independently_verified":False,
        "notes":"Reusable while camera, chessboard and robot base stay fixed; not proof of current pose stability",
    }
    path=Path(output).expanduser() if output else (
        Path.home()/".evoarm"/"gemini_board_reference.json"
    )
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(reference,ensure_ascii=False,indent=2),encoding="utf-8")
    return {**reference,"output_file":str(path)}


def main(argv=None):
    p=argparse.ArgumentParser(description="Extract and save Gemini board reference once")
    p.add_argument("reference_trial")
    p.add_argument("--output")
    p.add_argument("--square-mm", type=float, default=24.0,
                   help="actual measured grid pitch in mm; confirm before a new layout calibration")
    args=p.parse_args(argv)
    result=create_reference(args.reference_trial,output=args.output,square_mm=args.square_mm)
    print(json.dumps({k:v for k,v in result.items() if k not in
        ("corners_px","pixel_to_grid_h")},ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
