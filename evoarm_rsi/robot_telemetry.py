"""Read-only RoArm telemetry capture through the existing localhost controller.

This module never opens the serial port and never sends motion commands.
The browser/ArmService remains the *only* serial owner. Host monotonic clock
timestamps allow joining camera frames and arm feedback in later analysis.
"""
from __future__ import annotations

import json
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen


def _validate_status_url(url: str):
    parts = urlsplit(url)
    if (parts.scheme != "http" or parts.hostname not in ("127.0.0.1", "localhost")
            or parts.username or parts.password or parts.fragment or parts.query
            or parts.path != "/api/status" or parts.port is None):
        raise ValueError("robot status URL must be local http://127.0.0.1:PORT/api/status")
    return url


class ReadOnlyRobotSampler:
    """Sampling stops at the trial end; failures are recorded, never fabricated."""
    def __init__(self, status_url: str, *, interval_s=0.5, timeout_s=1.5):
        self.url = _validate_status_url(status_url)
        if not 0.1 <= interval_s <= 5:
            raise ValueError("interval_s out of bounds")
        if not 0.1 <= timeout_s <= 10:
            raise ValueError("timeout_s out of bounds")
        self.interval_s = interval_s
        self.timeout_s = timeout_s
        self._stop = threading.Event()
        self._thread = None
        self.samples = 0
        self.errors = 0
        self.first_error = None
        self._store = None
        self._trial_id = None

    def start(self, store, trial_id):
        if self._thread is not None:
            raise RuntimeError("sampler already started")
        self._store, self._trial_id = store, trial_id
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="rsi-read-only-robot-sampler", daemon=True
        )
        self._thread.start()

    def _fetch(self):
        # No auth, no redirects to external hosts are allowed. Explicitly
        # reject HTTP redirect responses rather than following arbitrary URLs.
        from urllib.request import build_opener, HTTPRedirectHandler, Request
        class NoRedirect(HTTPRedirectHandler):
            def redirect_request(self, req, fp, code, msg, headers, newurl):
                return None
        opener = build_opener(NoRedirect)
        with opener.open(Request(self.url, method="GET"), timeout=self.timeout_s) as resp:
            raw = resp.read(256 * 1024 + 1)
            if len(raw) > 256 * 1024:
                raise ValueError("robot status response too large")
            value = json.loads(raw.decode("utf-8"))
            if not isinstance(value, dict):
                raise ValueError("invalid robot status")
            return value

    def _run(self):
        while not self._stop.is_set():
            wall_ns = time.time_ns()
            begin_ns = time.monotonic_ns()
            try:
                result = self._fetch()
                end_ns = time.monotonic_ns()
                self._store.event(self._trial_id, "trajectory", {
                    "source": "real_roarm_readonly_api",
                    "host_wall_ns": wall_ns,
                    "sample_start_monotonic_ns": begin_ns,
                    "sample_end_monotonic_ns": end_ns,
                    "robot_status": result,
                })
                self.samples += 1
            except Exception as exc:
                self.errors += 1
                # Never log credentials or arbitrary server response bodies.
                detail = type(exc).__name__
                if self.first_error is None:
                    self.first_error = detail
                self._store.event(self._trial_id, "events", {
                    "kind": "robot_readonly_poll_failed",
                    "error_type": detail,
                    "host_monotonic_ns": time.monotonic_ns(),
                })
            self._stop.wait(self.interval_s)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout_s + 2)
            if self._thread.is_alive():
                raise RuntimeError("robot sampler did not stop")
        self._thread = None
        return {
            "real_samples": self.samples,
            "poll_errors": self.errors,
            "first_error_type": self.first_error,
            "read_only": True,
        }
