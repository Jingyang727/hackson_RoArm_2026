"""Persistent active calibration revisions; never replace a previous layout."""
from datetime import datetime,timezone
import json
from pathlib import Path
import re
import uuid

from control_ui.robot_calibration import _write_json_atomic
from .task_memory import valid_id


def calibration_root(home,setup_id):
    return Path(home)/'.evoarm/rsi/calibrations'/valid_id(setup_id)


def active_session(home,setup_id):
    root=calibration_root(home,setup_id);pointer=root/'active-session.json'
    if not pointer.exists():return root/'session.json'
    revision=json.loads(pointer.read_text())['revision_id']
    if not isinstance(revision,str) or not re.fullmatch('[a-f0-9]{32}',revision):
        raise ValueError('当前标定版本指针无效，请检查active-session.json；未回退旧布局')
    return root/'revisions'/revision/'session.json'


def begin_revision(home,setup_id):
    root=calibration_root(home,setup_id);previous=active_session(home,setup_id)
    revision=uuid.uuid4().hex;target=root/'revisions'/revision/'session.json'
    target.parent.mkdir(parents=True,exist_ok=False)
    # Commit the new, initially empty layout BEFORE capture. A camera/detection
    # failure must not silently reactivate geometry for devices that moved.
    _write_json_atomic(root/'active-session.json',{'revision_id':revision,
        'started_at':datetime.now(timezone.utc).isoformat(),'previous_session':str(previous)})
    return target
