"""Exercise the actual shared capture worker with a synthetic RGB-D source."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np

from evoarm_rsi.camera import CameraFrame
from evoarm_rsi.camera_hub import CameraHub
from evoarm_rsi.core import TrialStore
from evoarm_rsi.observe import record_observation


class SyntheticSource:
    def __init__(self):
        self.calls = []
        self.fail = False
        self.frame_provider = self
        self.color_intrinsics = {"fx": 100., "fy": 100., "cx": 80., "cy": 48.,
                                 "width": 160, "height": 96, "verified": False}

    def start(self):
        self.calls.append(("start", threading.get_ident()))

    def read(self):
        self.calls.append(("read", threading.get_ident()))
        time.sleep(.01)
        if self.fail:
            raise OSError("synthetic camera disconnected")
        return CameraFrame(np.full((96, 160, 3), 100, np.uint8), time.monotonic_ns(),
                           np.full((96, 160), 400, np.uint16))

    def stop(self):
        self.calls.append(("stop", threading.get_ident()))


def wait_for(predicate, timeout=2.):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= end:
            raise AssertionError("condition not reached")
        time.sleep(.005)


class CameraHubTests(unittest.TestCase):
    def hub(self, **kwargs):
        sources = []
        def factory():
            source = SyntheticSource()
            sources.append(source)
            return source
        hub = CameraHub(factory, **kwargs)
        self.addCleanup(hub.close)
        return hub, sources

    def test_live_preview_and_rgbd_record_share_one_sdk_owner_and_intrinsics(self):
        hub, sources = self.hub()
        hub.start_preview()
        first = hub.preview()
        self.assertTrue(first["jpeg"].startswith(b"\xff\xd8"))
        reader = hub.reader()
        reader.start()
        old_sequence = reader.after
        frame = reader.read()
        self.assertGreater(reader.after, old_sequence)
        self.assertEqual(frame.depth_mm.shape, (96, 160))
        with tempfile.TemporaryDirectory() as temp:
            result = record_observation(camera="gemini", seconds=.18,
                source=hub.reader(), store=TrialStore(Path(temp)))
            folder = Path(result["folder"])
            self.assertEqual(json.loads((folder/"gemini_color_intrinsics.json").read_text()),
                             sources[0].color_intrinsics)
            depth = np.load(folder/"depth_samples.npz")
            self.assertTrue(depth.files)
            self.assertGreater(result["capture"]["frames"], 2)
        self.assertEqual(hub.leases, 1)
        newer = hub.preview(first["sequence"])
        self.assertGreater(newer["sequence"], first["sequence"])
        reader.stop()
        hub.close()
        self.assertEqual(len(sources), 1)
        self.assertEqual(sum(c[0] == "start" for c in sources[0].calls), 1)
        self.assertEqual(sum(c[0] == "stop" for c in sources[0].calls), 1)
        self.assertEqual(len({c[1] for c in sources[0].calls}), 1)

    def test_failure_is_latched_and_cached_frame_is_never_returned(self):
        hub, sources = self.hub()
        hub.start_preview()
        hub.preview()
        sources[0].fail = True
        wait_for(lambda: hub.error and hub.worker is None)
        with self.assertRaises(RuntimeError):
            hub.preview()
        self.assertEqual(len(sources), 1)
        self.assertIsNone(hub.jpeg)
        hub.start_preview()  # explicit operator retry only
        hub.preview()
        self.assertEqual(len(sources), 2)

    def test_idle_releases_sdk_and_next_request_reopens_on_same_worker(self):
        hub, sources = self.hub(idle_s=.06)
        hub.start_preview()
        first = hub.preview()
        wait_for(lambda: hub.phase == "idle" and hub.source is None)
        self.assertIsNone(hub.latest)
        newer = hub.preview(first["sequence"])
        self.assertGreater(newer["sequence"], first["sequence"])
        self.assertEqual(len(sources), 2)
        self.assertEqual(sum(c[0] == "stop" for c in sources[0].calls), 1)

    def test_wait_requires_new_sequence_and_leased_reader_blocks_idle(self):
        hub, sources = self.hub(idle_s=.04)
        reader = hub.reader()
        reader.start()
        reader.read()
        with self.assertRaises(TimeoutError):
            hub.preview(after=2**53, timeout=.12)
        self.assertEqual(hub.phase, "live")
        self.assertEqual(len(sources), 1)
        reader.stop()
