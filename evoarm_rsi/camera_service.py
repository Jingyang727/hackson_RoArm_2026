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

from .core import TrialStore
from .observe import record_observation


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
    def __init__(self,path,uid,gid,home):
        self.uid,self.gid,self.home=uid,gid,Path(home)
        self.recording=False
        self.phase="idle"
        self.started_monotonic_ns=None
        self.capture_lock=threading.Lock()
        super().__init__(str(path),CameraHandler)
        os.chown(path,uid,gid)
        os.chmod(path,0o600)


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
        if self.path!="/health":
            self._respond(404,{"error":"not found"})
            return
        self._respond(200,{"camera_service":"ready","robot_control":False,
                           "privileged_capture":True,
                           "recording":self.server.recording,
                           "phase":self.server.phase,
                           "recording_started_monotonic_ns":self.server.started_monotonic_ns})

    def do_POST(self):
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
            if not isinstance(args,dict) or set(args)!={"seconds"}:
                raise ValueError("only seconds is accepted")
            seconds=args["seconds"]
            if type(seconds) not in (float,int) or not 0<seconds<=20:
                raise ValueError("seconds must be in (0,20]")
        except (ValueError,TypeError,UnicodeError,json.JSONDecodeError):
            self.server.capture_lock.release()
            self._respond(400,{"error":"invalid capture request"})
            return
        self.server.recording=True
        self.server.phase="warming"
        self.server.started_monotonic_ns=None
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
                    on_recording_start=on_recording_start
                )
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
            self._respond(200,result)
        except Exception as exc:
            self._respond(500,{"error_type":type(exc).__name__,
                               "error":"Gemini recording failed; inspect the camera service terminal"})
        finally:
            self.server.recording=False
            self.server.phase="idle"
            self.server.started_monotonic_ns=None
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


def serve():
    uid,gid,home=owner_context()
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
