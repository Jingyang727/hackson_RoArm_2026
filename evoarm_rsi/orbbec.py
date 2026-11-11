"""Gemini 335L capture provider for the Orbbec SDK v2 Python bindings.

Runs only when explicitly started; it never touches the robot. Prefer
software depth-to-color alignment until camera firmware is verified.
"""
from __future__ import annotations

import time
import numpy as np

from .camera import CameraFrame


class Gemini335LProvider:
    def __init__(self, *, align_depth=True, timeout_ms=1000,
                 warmup_timeout_s=8.0):
        if timeout_ms <= 0 or timeout_ms > 5000:
            raise ValueError("Gemini frame timeout must be between 1 and 5000ms")
        if not 0 < warmup_timeout_s <= 30:
            raise ValueError("Gemini warmup timeout must be in (0, 30] seconds")
        self.align_depth = align_depth
        self.timeout_ms = timeout_ms
        self.warmup_timeout_s = warmup_timeout_s
        self.pipeline = None
        self.aligner = None
        self.dropped_frames = 0
        self.color_intrinsics = None
        self.intrinsics_error = None

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
        """Discard transient incomplete frames; fail after bounded warmup.

        The SDK may yield RGB-only/depth-only frames or None from AlignFilter
        directly after Pipeline.start(). A single such frame is not a fatal
        capture error. Only valid, depth-aligned RGB-D sets reach the recorder.
        """
        if self.pipeline is None:
            raise RuntimeError("Gemini pipeline not started")
        deadline = time.monotonic() + self.warmup_timeout_s
        discarded = 0
        last_reason = "no complete frame"
        # The count also bounds this loop when an SDK immediately returns
        # invalid frames without blocking (e.g. a broken mock/driver).
        for _ in range(240):
            frames = self.pipeline.wait_for_frames(self.timeout_ms)
            if frames is None:
                last_reason = "no frames"
            else:
                # Test the raw set first: aligners can reject RGB-only sets.
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if color is None or depth is None:
                    last_reason = "incomplete RGB/depth pair"
                else:
                    if self.aligner is not None:
                        aligned = self.aligner.process(frames)
                        if aligned is None:
                            last_reason = "depth aligner not ready"
                            frames = None
                        else:
                            frames = aligned.as_frame_set()
                            if frames is None:
                                last_reason = "aligned set not ready"
                    if frames is not None:
                        color = frames.get_color_frame()
                        depth = frames.get_depth_frame()
                        if color is not None and depth is not None:
                            bgr = self._bgr(color)
                            dw, dh = depth.get_width(), depth.get_height()
                            raw = np.frombuffer(
                                depth.get_data(), dtype=np.uint16
                            ).reshape((dh, dw))
                            # SDK raw depth * scale is millimeters.
                            depth_mm = raw.astype(np.float32) * float(
                                depth.get_depth_scale()
                            )
                            if depth_mm.shape != bgr.shape[:2]:
                                raise RuntimeError(
                                    f"RGB {bgr.shape[:2]} and depth {depth_mm.shape} mismatch; "
                                    "enable software depth-to-color alignment"
                                )
                            depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
                            self.dropped_frames += discarded
                            # Get active color profile intrinsics only after a
                            # valid stream frame; SDK may report zeros earlier.
                            # Missing calibration is not a reason to discard
                            # otherwise valid camera evidence.
                            if self.color_intrinsics is None:
                                try:
                                    param = self.pipeline.get_camera_param()
                                    c = param.rgb_intrinsic
                                    fx,fy = float(c.fx),float(c.fy)
                                    cx,cy = float(c.cx),float(c.cy)
                                    if (not np.isfinite([fx,fy,cx,cy]).all() or
                                            fx <= 10 or fy <= 10 or
                                            int(c.width) != bgr.shape[1] or
                                            int(c.height) != bgr.shape[0]):
                                        raise ValueError("invalid SDK RGB intrinsics or stream dimensions")
                                    self.color_intrinsics = {
                                        "fx":fx, "fy":fy, "cx":cx, "cy":cy,
                                        "width": int(c.width), "height": int(c.height),
                                        "aligned_to": "color",
                                        "verified": False,
                                        "source": "orbbec_sdk_active_color_profile",
                                        "note": "SDK-provided candidate; verify alignment before 3D use",
                                    }
                                except Exception as exc:
                                    self.intrinsics_error = type(exc).__name__
                            return CameraFrame(
                                rgb_bgr=bgr, depth_mm=depth_mm,
                                monotonic_ns=time.monotonic_ns(),
                                camera_name="orbbec_gemini_335l",
                            )
                        last_reason = "aligned set missing RGB/depth"
            discarded += 1
            if time.monotonic() >= deadline:
                break
        self.dropped_frames += discarded
        raise RuntimeError(
            f"Gemini RGB-D warmup timed out after {discarded} incomplete frames "
            f"(last: {last_reason}, limit {self.warmup_timeout_s:g}s)"
        )

    def stop(self):
        pipeline, self.pipeline = self.pipeline, None
        self.aligner = None
        if pipeline is not None:
            pipeline.stop()
