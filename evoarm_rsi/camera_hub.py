"""One camera owner shared by live preview and timestamped RGB-D recordings.

Only the worker touches the SDK. HTTP clients and recorders read immutable
frames. No robot imports, serial access, filesystem paths or motor commands.
"""
from __future__ import annotations

import threading
import time

from .camera import CameraFrame, OrbbecRGBDAdapter


def gemini_source():
    from .orbbec import Gemini335LProvider
    return OrbbecRGBDAdapter(Gemini335LProvider())


class CameraHub:
    def __init__(self, source_factory=gemini_source, *, idle_s=30., preview_fps=10., max_age_s=1.):
        self.factory = source_factory
        self.idle_s = idle_s
        self.preview_interval = 1./preview_fps
        self.max_age_s = max_age_s
        self.condition = threading.Condition()
        self.worker = None
        self.closed = False
        self.error = None
        self.source = None
        self.phase = "idle"
        self.leases = 0
        self.touched = 0.
        self.preview_until = 0.
        self.preview_waiters = 0
        self.sequence = 0
        self.latest = None
        self.jpeg = None
        self.jpeg_sequence = 0
        self.jpeg_stamp_ns = 0
        self.jpeg_size = None

    def _ensure(self, *, reset_error=False):
        if self.closed:
            raise RuntimeError("camera service is closing")
        if self.error and reset_error and not self.leases:
            if self.worker is not None:
                raise RuntimeError("camera owner is still closing; retry preview after it has stopped")
            self.error = None
        if self.error:
            raise RuntimeError("camera stream failed; explicitly restart preview after checking the camera")
        self.touched = time.monotonic()
        if self.worker is None:
            self.worker = threading.Thread(target=self._run, name="evoarm-camera-owner", daemon=True)
            self.worker.start()
        self.condition.notify_all()

    def start_preview(self):
        with self.condition:
            self._ensure(reset_error=True)
            self.preview_until = self.touched + 3.

    def health(self):
        with self.condition:
            age = None if self.latest is None else max(0., (time.monotonic_ns()-self.latest.monotonic_ns)/1e6)
            return {"state": self.phase, "error_type": self.error, "frame_sequence": self.sequence,
                    "frame_age_ms": age, "recording_readers": self.leases}

    def reader(self):
        return SharedCameraReader(self)

    def _wait(self, predicate, timeout):
        deadline = time.monotonic()+timeout
        while True:
            if self.error or self.closed:
                raise RuntimeError("camera stream is unavailable; check camera service health")
            value = predicate()
            if value is not None:
                return value
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("no fresh camera frame; preview is paused")
            self.condition.wait(remaining)

    def preview(self, after=0, *, timeout=10.):
        with self.condition:
            self._ensure()
            self.preview_until = self.touched+3.
            def fresh():
                age = (time.monotonic_ns()-self.jpeg_stamp_ns)/1e9
                if self.jpeg is not None and self.jpeg_sequence > after and 0 <= age <= self.max_age_s:
                    return {"jpeg": self.jpeg, "sequence": self.jpeg_sequence,
                            "stamp_ns": self.jpeg_stamp_ns, "width": self.jpeg_size[0],
                            "height": self.jpeg_size[1], "age_ms": age*1000.}
            self.preview_waiters += 1
            try:
                return self._wait(fresh, timeout)
            finally:
                self.preview_waiters -= 1

    def _run(self):
        source = None
        try:
            while True:
                with self.condition:
                    idle = not self.leases and time.monotonic()-self.touched >= self.idle_s
                    stopping = self.closed or self.error is not None
                if source is not None and (idle or stopping):
                    source.stop()
                    source = None
                    with self.condition:
                        self.source = None
                        self.latest = self.jpeg = None
                        self.phase = "error" if self.error else "idle"
                        self.condition.notify_all()
                with self.condition:
                    if self.closed:
                        return
                    if self.error or (not self.leases and time.monotonic()-self.touched >= self.idle_s):
                        self.condition.wait(.25)
                        continue
                    if source is None:
                        self.phase = "warming"
                if source is None:
                    source = self.factory()
                    with self.condition:
                        self.source = source
                    source.start()
                frame = source.read()
                if not isinstance(frame, CameraFrame):
                    raise TypeError("camera owner requires timestamped CameraFrame objects")
                with self.condition:
                    if self.latest is not None and frame.monotonic_ns <= self.latest.monotonic_ns:
                        raise ValueError("camera frame timestamps must increase")
                    self.sequence += 1
                    sequence = self.sequence
                    self.latest = frame
                    self.phase = "live"
                    encode = (self.preview_waiters > 0 or time.monotonic() < self.preview_until) and (
                        self.jpeg is None or (frame.monotonic_ns-self.jpeg_stamp_ns)/1e9 >= self.preview_interval)
                    self.condition.notify_all()
                if encode:
                    import cv2
                    ok, jpeg = cv2.imencode(".jpg", frame.rgb_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ok:
                        raise ValueError("cannot encode camera preview")
                    with self.condition:
                        self.jpeg = jpeg.tobytes()
                        self.jpeg_sequence = sequence
                        self.jpeg_stamp_ns = frame.monotonic_ns
                        self.jpeg_size = frame.rgb_bgr.shape[1], frame.rgb_bgr.shape[0]
                        self.condition.notify_all()
        except Exception as exc:
            print("[CAM] Stream stopped: "+type(exc).__name__+": "+str(exc).replace("\n", " ")[:300],flush=True)
            with self.condition:
                self.error = type(exc).__name__
                self.phase = "error"
                self.latest = self.jpeg = None
                self.condition.notify_all()
        finally:
            if source is not None:
                try:
                    source.stop()
                except Exception:
                    pass
            with self.condition:
                self.source = None
                self.worker = None
                self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
            worker = self.worker
        if worker is not None:
            worker.join(timeout=12.)
            if worker.is_alive():
                raise RuntimeError("camera worker did not stop; do not start another SDK owner")


class SharedCameraReader:
    def __init__(self, hub):
        self.hub = hub
        self.started = False
        self.after = 0

    @property
    def frame_provider(self):
        with self.hub.condition:
            return getattr(self.hub.source, "frame_provider", None)

    def start(self):
        with self.hub.condition:
            if self.started:
                raise RuntimeError("shared camera reader already started")
            self.hub._ensure(reset_error=True)
            self.hub.leases += 1
            self.started = True
            self.after = self.hub.sequence

    def read(self):
        with self.hub.condition:
            if not self.started:
                raise RuntimeError("shared camera reader has not started")
            def fresh():
                frame = self.hub.latest
                if frame is not None and self.hub.sequence > self.after:
                    age = (time.monotonic_ns()-frame.monotonic_ns)/1e9
                    if 0 <= age <= self.hub.max_age_s:
                        self.after = self.hub.sequence
                        return frame
            return self.hub._wait(fresh, 10.)

    def stop(self):
        with self.hub.condition:
            if self.started:
                self.hub.leases -= 1
                self.hub.touched = time.monotonic()
                self.started = False
                self.hub.condition.notify_all()
