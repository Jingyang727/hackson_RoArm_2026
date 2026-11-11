"""Camera interfaces and timestamped video/depth evidence recording.

The Gemini source is intentionally an injectable interface until the actual
device/firmware/pyorbbecsdk v2 combination is verified on the user's Mac.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time


@dataclass
class CameraFrame:
    rgb_bgr: object
    monotonic_ns: int
    depth_mm: object = None
    device_timestamp_ms: float | None = None
    camera_name: str = "unknown"


class USBCameraAdapter:
    def __init__(self, index=0):
        self.index = index
        self.capture = None

    def start(self):
        import cv2
        self.capture = cv2.VideoCapture(self.index)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = None
            raise RuntimeError("USB camera not available")

    def read(self):
        if self.capture is None:
            raise RuntimeError("camera has not been started")
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError("USB camera returned no frame")
        return CameraFrame(rgb_bgr=frame, monotonic_ns=time.monotonic_ns(),
                           camera_name="usb")

    def stop(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None


class OrbbecRGBDAdapter:
    """Receives aligned RGB/depth from a verified device driver.

    Frame provider MUST return a CameraFrame with depth in millimeters
    aligned to the color image; the SDK integration is a separate P1 task.
    """
    def __init__(self, frame_provider=None):
        self.frame_provider = frame_provider

    def start(self):
        if self.frame_provider is None:
            raise RuntimeError(
                "Orbbec Gemini 335L driver is not configured: "
                "verify pyorbbecsdk v2 and RGB/depth alignment on this host first"
            )
        self.frame_provider.start()

    def read(self):
        if self.frame_provider is None:
            raise RuntimeError("Orbbec provider not configured")
        result = self.frame_provider.read()
        if not isinstance(result, CameraFrame):
            raise TypeError("Orbbec provider must return CameraFrame")
        if result.depth_mm is None:
            raise ValueError("RGB-D frame missing aligned depth")
        return result

    def stop(self):
        if self.frame_provider is not None:
            self.frame_provider.stop()


class TrialVideoRecorder:
    """Write RGB video, sampled depth and frame-to-clock alignment metadata.

    Call in a capture thread parallel to robot execution, never in a robot
    motion control thread. Caller owns source.start/stop.
    """
    def __init__(self, source, fps=20, max_depth_frames=32):
        if not 1 <= fps <= 60:
            raise ValueError("fps must be 1..60")
        self.source = source
        self.fps = fps
        self.max_depth_frames = max_depth_frames

    def capture(self, folder, duration_s, *, clock=time.monotonic):
        if not 0 < duration_s <= 20:
            raise ValueError("recording duration must be 0..20 seconds")
        import cv2
        import numpy as np

        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        writer = None
        frames = []
        depth_samples = []
        depth_stamps = []
        # Obtain the first *valid* RGB-D frame before starting the trial
        # recording clock. Initial Orbbec SDK/aligner warmup can take seconds.
        # Keep that initialization outside the requested 10s evidence window.
        first_frame = self.source.read()
        start = clock()
        end = start + duration_s
        next_frame = start
        try:
            while clock() < end:
                if first_frame is not None:
                    frame, first_frame = first_frame, None
                else:
                    frame = self.source.read()
                at = clock()
                if at < next_frame:
                    continue
                next_frame = at + 1 / self.fps
                img = frame.rgb_bgr
                if img is None or getattr(img, "ndim", 0) != 3 or img.shape[2] != 3:
                    raise ValueError("camera frame must be 3-channel BGR")
                if writer is None:
                    h, w = img.shape[:2]
                    writer = cv2.VideoWriter(
                        str(folder/"rgb.mp4"),
                        cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h)
                    )
                    if not writer.isOpened():
                        raise RuntimeError("MP4 encoder could not be opened")
                writer.write(img)
                frames.append({
                    "frame_index": len(frames),
                    "host_monotonic_ns": frame.monotonic_ns,
                    "device_timestamp_ms": frame.device_timestamp_ms,
                    "camera_name": frame.camera_name,
                })
                if frame.depth_mm is not None and len(depth_samples) < self.max_depth_frames:
                    depth = np.asarray(frame.depth_mm)
                    if depth.ndim != 2 or depth.shape != img.shape[:2]:
                        raise ValueError("depth not aligned to RGB frame")
                    depth_samples.append(depth.astype(np.uint16).copy())
                    depth_stamps.append(frame.monotonic_ns)
            if not frames:
                raise RuntimeError("no camera frames recorded")
            with (folder/"frames.jsonl").open("w", encoding="utf-8") as out:
                for record in frames:
                    out.write(json.dumps(record) + "\n")
            if depth_samples:
                np.savez_compressed(folder/"depth_samples.npz",
                                    depth_mm=np.stack(depth_samples),
                                    monotonic_ns=np.asarray(depth_stamps, dtype=np.int64))
            return {"rgb_file": "rgb.mp4", "frames": len(frames),
                    "depth_samples": len(depth_samples), "camera": frames[0]["camera_name"]}
        finally:
            if writer is not None:
                writer.release()
