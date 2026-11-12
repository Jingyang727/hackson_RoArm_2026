"""Owner-only UNIX Gemini camera daemon contract; no hardware involved."""
import json
import os
from pathlib import Path
import socket
import stat
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

from evoarm_rsi.camera_service import CameraServer, client_request, preview_frame, daemon_lock
from evoarm_rsi.camera_hub import CameraHub
from tests.test_camera_hub import SyntheticSource, wait_for


class CameraDaemonTests(unittest.TestCase):
    def test_live_jpeg_client_and_recording_share_capture_worker(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);sock=root/"camera.sock";sources=[]
            def factory():
                sources.append(SyntheticSource());return sources[-1]
            srv=CameraServer(sock,os.getuid(),os.getgid(),root,hub=CameraHub(factory))
            thread=threading.Thread(target=srv.serve_forever,daemon=True);thread.start()
            try:
                with patch("evoarm_rsi.camera_service.socket_path",return_value=sock):
                    self.assertTrue(client_request(root,"POST","/preview/start",{})["started"])
                    frame=preview_frame(root)
                    self.assertEqual((frame["width"],frame["height"]),(160,96))
                    self.assertTrue(frame["jpeg"].startswith(b"\xff\xd8"))
                    with ThreadPoolExecutor(max_workers=1) as pool:
                        future=pool.submit(client_request,root,"POST","/record",{"seconds":.35})
                        wait_for(lambda:client_request(root,"GET","/health")["recording"])
                        concurrent_frame=preview_frame(root,frame["sequence"])
                        self.assertGreater(concurrent_frame["sequence"],frame["sequence"])
                        result=future.result(timeout=3)
                    self.assertGreater(result["capture"]["frames"],2)
                    self.assertGreater(preview_frame(root,frame["sequence"])["sequence"],frame["sequence"])
                    for path in ("/preview?after=-1","/preview?after=1&after=2","/preview?motor=1"):
                        with self.assertRaisesRegex(RuntimeError,"HTTP 400"):
                            client_request(root,"GET",path)
                self.assertEqual(len(sources),1)
            finally:
                srv.shutdown();srv.server_close();thread.join(timeout=2)

    def test_second_daemon_cannot_replace_the_socket_or_open_sdk(self):
        with tempfile.TemporaryDirectory() as temp:
            with daemon_lock(temp,os.getuid(),os.getgid()):
                with self.assertRaisesRegex(RuntimeError,"already running"):
                    with daemon_lock(temp,os.getuid(),os.getgid()):
                        self.fail("second owner acquired the lock")
            with daemon_lock(temp,os.getuid(),os.getgid()):
                pass

    def test_owner_only_socket_health_and_camera_only_capabilities(self):
        with tempfile.TemporaryDirectory() as temp:
            sock=Path(temp)/"camera.sock"
            srv=CameraServer(sock,os.getuid(),os.getgid(),temp)
            thread=threading.Thread(target=srv.serve_forever,daemon=True)
            thread.start()
            try:
                with patch("evoarm_rsi.camera_service.socket_path",
                           return_value=sock):
                    response=client_request(Path(temp),"GET","/health")
                self.assertEqual(response["camera_service"],"ready")
                self.assertIs(response["robot_control"],False)
                self.assertIs(response["recording"],False)
                self.assertIsNone(response["recording_started_monotonic_ns"])
                self.assertEqual(stat.S_IMODE(sock.stat().st_mode),0o600)
            finally:
                srv.shutdown()
                srv.server_close()
                thread.join(timeout=2)

    def test_simulated_record_http_does_not_open_camera_or_robot(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            sock=root/"camera.sock"
            srv=CameraServer(sock,os.getuid(),os.getgid(),root)
            thread=threading.Thread(target=srv.serve_forever,daemon=True)
            thread.start()
            def mock_record(*,camera,seconds,store,on_recording_start,source):
                self.assertIs(source.hub,srv.hub)
                self.assertEqual(camera,"gemini")
                self.assertEqual(seconds,1)
                folder=store.root/("a"*32)
                folder.mkdir(parents=True)
                (folder/"manifest.json").write_text(
                    json.dumps({"simulated":True,"test_only":True})
                )
                on_recording_start(time.monotonic(),seconds)
                return {"trial_id":"a"*32,"folder":str(folder),
                        "capture":{"frames":1,"test_only":True}}
            try:
                with (patch("evoarm_rsi.camera_service.socket_path",
                            return_value=sock),
                      patch("evoarm_rsi.camera_service.record_observation",
                            side_effect=mock_record)):
                    response=client_request(root,"POST","/record",{"seconds":1})
                    self.assertEqual(response["trial_id"],"a"*32)
                    self.assertEqual(response["capture"]["frames"],1)
                    with self.assertRaisesRegex(RuntimeError,"HTTP 400"):
                        client_request(root,"POST","/record",{
                            "seconds":1,"command":"move_robot"
                        })
                self.assertEqual((root/".evoarm"/"rsi"/"experiments"/("a"*32)/
                                  "manifest.json").stat().st_uid,os.getuid())
            finally:
                srv.shutdown()
                srv.server_close()
                thread.join(timeout=2)


if __name__=="__main__":
    unittest.main()
