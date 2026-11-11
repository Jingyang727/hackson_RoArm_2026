"""Gemini 335L capture provider for the Orbbec SDK v2 Python bindings.

Runs only when explicitly started; it never touches the robot. Prefer
software depth-to-color alignment until camera firmware is verified.
"""
from __future__ import annotations

import time
import numpy as np

from .camera import CameraFrame


class Gemini335LProvider:
    def __init__(self, *, align_depth=True, timeout_ms=1000):
        self.align_depth = align_depth
        self.timeout_ms = timeout_ms
        self.pipeline = None
        self.aligner = None

    def start(self):
        from pyorbbecsdk import Pipeline, Config, OBSensorType, OBStreamType, AlignFilter
        if self.pipeline is not None:
            raise RuntimeError("Gemini pipeline already started")
        pipeline = Pipeline()
        cfg = Config()
        for sensor in (OBSensorType.COLOR_SENSOR, OBSensorType.DEPTH_SENSOR):
            profile = pipeline.get_stream_profile_list(sensor).get_default_video_stream_profile()
            cfg.enable_stream(profile)
        pipeline.start(cfg)
        self.pipeline = pipeline
        self.aligner = AlignFilter(OBStreamType.COLOR_STREAM) if self.align_depth else None

    @staticmethod
    def _bgr(color):
        import cv2
        from pyorbbecsdk import OBFormat
        width, height = color.get_width(), color.get_height()
        fmt = color.get_format()
        raw = np.frombuffer(color.get_data(), dtype=np.uint8)
        if fmt == OBFormat.RGB:
            rgb = raw.reshape((height, width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if fmt == OBFormat.BGR:
            return raw.reshape((height, width, 3)).copy()
        if fmt == OBFormat.MJPG:
            image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (height, width):
                raise RuntimeError("invalid MJPG color frame")
            return image
        if fmt == OBFormat.YUYV:
            yuyv = raw.reshape((height, width, 2))
            return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        raise RuntimeError(f"unsupported Gemini color format: {fmt}")

    def read(self):
        if self.pipeline is None:
            raise RuntimeError("Gemini pipeline not started")
        frames = self.pipeline.wait_for_frames(self.timeout_ms)
        if frames is None:
            raise RuntimeError("Gemini timed out waiting for RGB-D frames")
        if self.aligner is not None:
            aligned = self.aligner.process(frames)
            if aligned is None:
                raise RuntimeError("Gemini depth alignment failed")
            frames = aligned.as_frame_set()
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            raise RuntimeError("Gemini frame set missing RGB or depth")
        bgr = self._bgr(color)
        dw, dh = depth.get_width(), depth.get_height()
        raw = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape((dh, dw))
        # The raw integer depth is scaled to millimeters by SDK calibration.
        depth_mm = raw.astype(np.float32) * float(depth.get_depth_scale())
        if depth_mm.shape != bgr.shape[:2]:
            raise RuntimeError(
                f"RGB {bgr.shape[:2]} and depth {depth_mm.shape} mismatch; "
                "enable software depth-to-color alignment"
            )
        depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
        return CameraFrame(
            rgb_bgr=bgr, depth_mm=depth_mm,
            monotonic_ns=time.monotonic_ns(),
            camera_name="orbbec_gemini_335l",
        )

    def stop(self):
        pipeline, self.pipeline = self.pipeline, None
        self.aligner = None
        if pipeline is not None:
            pipeline.stop()
