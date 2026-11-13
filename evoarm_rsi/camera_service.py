"""Persistent privileged Gemini capture service over owner-only UNIX socket.

Run manually once in the terminal that can open Orbbec UVC. The AI coding
process calls the owner-only socket as the ordinary user. No TCP listener,
no arbitrary paths, no shell execution and NO robot motor endpoints.

Mac:
    sudo /path/to/evoarm/python -m evoarm_rsi.camera_service serve
    /path/to/evoarm/python -m evoarm_rsi.camera_service record --seconds 10

The socket is in the invoking user's ~/.evoarm/ directory (chmod 0600).
Only the explicit dedicated camera process is elevated; it restores
ownership of generated trial artifacts after recording.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import http.client
import json
import os
from pathlib import Path
import pwd
import socket
import socketserver
import stat
import threading
import time
from http.server import BaseHTTPRequestHandler
from urllib.parse import urlsplit, parse_qs

from .core import TrialStore
from .observe import record_observation
from .camera_hub import CameraHub


def owner_context():
    uid=int(os.environ.get("SUDO_UID",os.getuid()))
    gid=int(os.environ.get("SUDO_GID",os.getgid()))
    if uid == 0:
        raise RuntimeError("service requires a non-root local owner (launch via sudo from user)")
    home=Path(pwd.getpwuid(uid).pw_dir)
    return uid,gid,home


def socket_path(home):
    return Path(home)/".evoarm"/"rsi_camera.sock"


class CameraServer(socketserver.ThreadingMixIn,socketserver.UnixStreamServer):
    allow_reuse_address=False
    daemon_threads=True
    def __init__(self,path,uid,gid,home,*,hub=None):
        self.uid,self.gid,self.home=uid,gid,Path(home)
        self.recording=False
        self.phase="idle"
        self.started_monotonic_ns=None
        self.recording_request_id=None
        self.capture_lock=threading.Lock()
        self.hub=hub if hub is not None else CameraHub()
        super().__init__(str(path),CameraHandler)
        os.chown(path,uid,gid)
        os.chmod(path,0o600)

    def server_close(self):
        try:
            self.hub.close()
        finally:
            super().server_close()


class CameraHandler(BaseHTTPRequestHandler):
    server: CameraServer

    def log_message(self,*args):
        return  # keep API request lines out of logs

    def _respond(self,code,data):
        payload=json.dumps(data,ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length",str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        parsed=urlsplit(self.path)
        if parsed.path=="/preview":
            try:
                args=parse_qs(parsed.query,strict_parsing=True)
                if set(args)-{"after"} or len(args.get("after",["0"]))!=1:
                    raise ValueError("only one frame sequence is accepted")
                after=int(args.get("after",["0"])[0])
                if not 0<=after<=2**53:
                    raise ValueError("invalid frame sequence")
            except ValueError:
                self._respond(400,{"error":"invalid preview query"})
                return
            try:
                frame=self.server.hub.preview(after)
            except (OSError,RuntimeError,TimeoutError,ValueError):
                self._respond(503,{"error":"no fresh camera preview; check the camera service and USB"})
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type","image/jpeg")
                self.send_header("Content-Length",str(len(frame["jpeg"])))
                self.send_header("Cache-Control","no-store")
                for key,value in (("Sequence",frame["sequence"]),("Width",frame["width"]),
                                  ("Height",frame["height"]),("Stamp-Ns",frame["stamp_ns"]),
                                  ("Age-Ms",round(frame["age_ms"],2))):
                    self.send_header("X-Camera-"+key,str(value))
                self.end_headers()
                self.wfile.write(frame["jpeg"])
            except OSError:
                pass
            return
        if self.path!="/health":
            self._respond(404,{"error":"not found"})
            return
        self._respond(200,{"camera_service":"ready","robot_control":False,
                           "privileged_capture":True,
                           "recording":self.server.recording,
                           "phase":self.server.phase,
                           "live_preview":True,"preview":self.server.hub.health(),
                           "recording_request_ids":True,
                           "recording_request_id":self.server.recording_request_id,
                           "recording_started_monotonic_ns":self.server.started_monotonic_ns})

    def do_POST(self):
        if self.path=="/preview/start":
            try:
                size=int(self.headers.get("Content-Length","0"))
                if not 0<=size<=16:
                    raise ValueError("preview start accepts no settings")
                if json.loads(self.rfile.read(size).decode("utf-8") or "{}")!={}:
                    raise ValueError("preview start accepts no settings")
                self.server.hub.start_preview()
                self._respond(200,{"started":True,"robot_control":False})
            except (ValueError,UnicodeError):
                self._respond(400,{"error":"preview start accepts only an empty object"})
            except (OSError,RuntimeError):
                self._respond(503,{"error":"camera owner is not ready; check camera service health"})
            return
        if self.path!="/record":
            self._respond(404,{"error":"not found"})
            return
        if not self.server.capture_lock.acquire(blocking=False):
            self._respond(409,{"error":"camera already recording"})
            return
        try:
            length=int(self.headers.get("Content-Length","0"))
            if not 0 <= length <= 2048:
                raise ValueError("invalid request length")
            args=json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(args,dict) or "seconds" not in args or set(args)-{"seconds","request_id"}:
                raise ValueError("only seconds and an optional request ID are accepted")
            seconds=args["seconds"]
            if type(seconds) not in (float,int) or not 0<seconds<=20:
                raise ValueError("seconds must be in (0,20]")
            request_id=args.get("request_id")
            if request_id is not None and (not isinstance(request_id,str) or len(request_id)!=32
                    or any(c not in "0123456789abcdef" for c in request_id)):
                raise ValueError("request ID must be a lowercase UUID hex string")
        except (ValueError,TypeError,UnicodeError,json.JSONDecodeError):
            self.server.capture_lock.release()
            self._respond(400,{"error":"invalid capture request"})
            return
        self.server.recording=True
        self.server.phase="warming"
        self.server.started_monotonic_ns=None
        self.server.recording_request_id=request_id
        try:
            store=TrialStore(self.server.home/".evoarm"/"rsi"/"experiments")
            store.root.mkdir(parents=True,exist_ok=True)
            before=set(p.name for p in store.root.iterdir() if p.is_dir())
            def on_recording_start(start, duration):
                self.server.phase="recording"
                self.server.started_monotonic_ns=int(start*1e9)
            try:
                result=record_observation(
                    camera="gemini",seconds=seconds,store=store,
                    on_recording_start=on_recording_start,source=self.server.hub.reader()
                )
                if request_id is not None:
                    manifest=store.read(result["trial_id"])
                    store.write(result["trial_id"],"manifest.json",{**manifest,"camera_request_id":request_id})
            finally:
                # Make newly created files owner-accessible even on camera
                # failure. Only affect this service's newly created trial
                # UUID directories, never calibration or existing trials.
                for trial_folder in store.root.iterdir():
                    name=trial_folder.name
                    if (name in before or not trial_folder.is_dir() or
                            len(name)!=32 or any(ch not in "0123456789abcdef" for ch in name)):
                        continue
                    for path in trial_folder.rglob("*"):
                        if path.is_symlink():
                            raise RuntimeError("unexpected symlink in trial artifacts")
                        os.chown(path,self.server.uid,self.server.gid)
                    os.chown(trial_folder,self.server.uid,self.server.gid)
            self._respond(200,{**result,"request_id":request_id})
        except Exception as exc:
            self._respond(500,{"error_type":type(exc).__name__,
                               "error":"Gemini recording failed; inspect the camera service terminal"})
        finally:
            self.server.recording=False
            self.server.phase="idle"
            self.server.started_monotonic_ns=None
            self.server.recording_request_id=None
            self.server.capture_lock.release()


class UnixHTTPConnection(http.client.HTTPConnection):
    def __init__(self,path,timeout=45):
        super().__init__("localhost",timeout=timeout)
        self.socket_path=str(path)

    def connect(self):
        self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.socket_path)


def client_request(home,method,path,data=None):
    sockpath=socket_path(home)
    conn=UnixHTTPConnection(sockpath)
    try:
        payload=json.dumps(data).encode("utf-8") if data is not None else None
        headers={"Content-Type":"application/json"} if payload else {}
        conn.request(method,path,body=payload,headers=headers)
        resp=conn.getresponse()
        body=json.loads(resp.read().decode("utf-8"))
        if resp.status!=200:
            raise RuntimeError(f"camera service HTTP {resp.status}: {body.get('error')}")
        return body
    finally:
        conn.close()


def preview_frame(home,after=0):
    """Ordinary-user JPEG client; timestamp/sequence checks stay authoritative."""
    if type(after) is not int or not 0<=after<=2**53:
        raise ValueError("invalid camera frame sequence")
    conn=UnixHTTPConnection(socket_path(home),timeout=13)
    try:
        conn.request("GET","/preview?after="+str(after))
        response=conn.getresponse()
        raw=response.read(2_000_001)
        if response.status!=200:
            raise RuntimeError(f"camera preview HTTP {response.status}; update/restart the camera service if the endpoint is missing")
        if len(raw)>2_000_000 or response.getheader("Content-Type")!="image/jpeg":
            raise ValueError("invalid camera preview response")
        values={key:response.getheader("X-Camera-"+key) for key in ("Sequence","Width","Height","Stamp-Ns","Age-Ms")}
        frame={"jpeg":raw,"sequence":int(values["Sequence"]),"width":int(values["Width"]),
               "height":int(values["Height"]),"stamp_ns":int(values["Stamp-Ns"]),"age_ms":float(values["Age-Ms"])}
        if frame["sequence"]<=after or min(frame["width"],frame["height"],frame["stamp_ns"])<=0 or not 0<=frame["age_ms"]<=1500:
            raise ValueError("stale or invalid camera preview metadata")
        return frame
    finally:
        conn.close()


@contextmanager
def daemon_lock(home,uid,gid):
    """A second service cannot unlink a live socket and open another SDK."""
    path=Path(home)/".evoarm"/"rsi_camera.lock"
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    parent=path.parent.lstat()
    if not stat.S_ISDIR(parent.st_mode) or parent.st_uid not in (uid,os.geteuid()):
        raise RuntimeError("unexpected camera service directory")
    os.chown(path.parent,uid,gid)
    fd=os.open(path,os.O_RDWR|os.O_CREAT|getattr(os,"O_NOFOLLOW",0),0o600)
    try:
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink!=1 or info.st_uid not in (uid,os.geteuid()):
            raise RuntimeError("unexpected camera service lock file")
        try:
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("camera service is already running; do not start a second SDK owner") from exc
        os.fchown(fd,uid,gid)
        os.fchmod(fd,0o600)
        yield
    finally:
        os.close(fd)


def serve():
    uid,gid,home=owner_context()
    with daemon_lock(home,uid,gid):
        _serve_locked(uid,gid,home)


def _serve_locked(uid,gid,home):
    path=socket_path(home)
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists() or path.is_symlink():
        metadata=path.lstat()
        if not stat.S_ISSOCK(metadata.st_mode) or metadata.st_uid!=uid:
            raise RuntimeError("refusing to replace unexpected camera socket")
        path.unlink()
    service=CameraServer(path,uid,gid,home)
    print(f"[CAM SERVICE] local UNIX socket {path}; NO robot controls",flush=True)
    try:
        service.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        service.server_close()
        if path.exists() and stat.S_ISSOCK(path.lstat().st_mode):
            path.unlink()


def main(argv=None):
    parser=argparse.ArgumentParser(description="Gemini privileged camera-only service")
    actions=parser.add_subparsers(dest="command",required=True)
    actions.add_parser("serve")
    actions.add_parser("health")
    rec=actions.add_parser("record")
    rec.add_argument("--seconds",type=float,default=10.0)
    args=parser.parse_args(argv)
    if args.command=="serve":
        serve()
    else:
        # Normal user client; should never be launched with sudo.
        home=Path.home()
        response=client_request(home,"GET","/health") if args.command=="health" else (
            client_request(home,"POST","/record",{"seconds":args.seconds})
        )
        print(json.dumps(response,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
