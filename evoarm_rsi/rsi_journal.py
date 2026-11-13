"""Local trial journal, verified backup and durable future Langfuse outbox."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
import uuid
import zipfile

from .task_agent import _write


def append_event(root, name, **fields):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    entry = {'schema_version': 1, 'event_id': uuid.uuid4().hex,
             'name': name, 'at_unix_ns': time.time_ns(),
             'host_monotonic_ns': time.monotonic_ns(), **fields}
    with (root / 'journal.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
        stream.flush()
    return entry


def archive_trial(root, *, calibration=None, home=None):
    """Snapshot all available evidence; a controller failure is also a trial."""
    root = Path(root)
    home = Path(home) if home is not None else Path.home()
    base = home / '.evoarm/rsi'
    state = json.loads((root / 'state.json').read_text())
    result = state.get('result', {})
    if not result and (root/'attempt.json').is_file():
        result=json.loads((root/'attempt.json').read_text())
    capture_id = result.get('capture_trial_id')
    if calibration is not None:
        _write(root / 'calibration-snapshot.json', calibration)
    sources = [(root, 'trial')]
    if capture_id and len(capture_id) == 32 and all(c in '0123456789abcdef' for c in capture_id):
        sources.append((base / 'experiments' / capture_id, 'capture'))
    observation = state.get('plan', {}).get('scene', {}).get('source_trial_id')
    if observation and len(observation) == 32 and all(c in '0123456789abcdef' for c in observation):
        sources.append((base / 'experiments' / observation, 'observation'))
    recovery_capture=state.get('recovery',{}).get('verification_trial_id')
    if recovery_capture and len(recovery_capture)==32 and all(c in '0123456789abcdef' for c in recovery_capture):
        sources.append((base/'experiments'/recovery_capture,'recovery-observation'))
    dest = base / 'backups' / f'{root.name}.zip'
    dest.parent.mkdir(parents=True, exist_ok=True)
    temp = dest.with_suffix('.tmp')
    inventory = []
    try:
        with zipfile.ZipFile(temp, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
            for source, prefix in sources:
                for file in sorted(source.rglob('*')):
                    if not file.is_file() or file.is_symlink() or file.suffix == '.tmp':
                        continue
                    data = file.read_bytes()
                    name = prefix + '/' + file.relative_to(source).as_posix()
                    archive.writestr(name, data)
                    inventory.append({'path': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()})
            manifest = {'schema_version': 1, 'plan_id': root.name, 'created_at_unix': time.time(),
                        'files': inventory, 'contains_credentials': False}
            archive.writestr('archive-manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2))
        with zipfile.ZipFile(temp) as archive:
            if archive.testzip() is not None:
                raise OSError('试次备份校验失败')
        temp.replace(dest)
    finally:
        temp.unlink(missing_ok=True)
    # This is an export contract, not a network submission. Exporters can retry
    # using the stable local trace_id, without repeating robot actions.
    packet = {'schema': 'evoarm.rsi.trace.v1', 'trace_id': root.name,
              'setup_id': state['plan']['setup_id'], 'created_at_unix': time.time(),
              'export_status': 'pending', 'remote_submitted': False,
              'input': {'parameters': state['plan']['parameters'],
                        'ball_xyz_mm': state['plan']['scene']['ball']['center_robot_mm'],
                        'funnel_xyz_mm': state['plan']['scene']['funnel']['mouth_center_robot_mm'],
                        'calibration_fingerprint': state['calibration_fingerprint']},
              'output': {'result': result, 'review': state.get('review'), 'recovery': state.get('recovery')},
              'backup': {'path': str(dest), 'sha256': hashlib.sha256(dest.read_bytes()).hexdigest()},
              'events_file': str(root / 'journal.jsonl')}
    _write(base / 'langfuse-outbox' / f'{root.name}.json', packet)
    _write(root / 'backup.json', {'archive': str(dest), 'outbox_trace_id': root.name})
    return str(dest)


def pending_exports(home=None):
    base = (Path(home) if home is not None else Path.home()) / '.evoarm/rsi/langfuse-outbox'
    for file in sorted(base.glob('*.json')):
        packet = json.loads(file.read_text())
        if packet.get('export_status') == 'pending':
            yield packet


def backup_existing(home=None):
    """Backfill older executed trials without reading or commanding hardware."""
    home=Path(home) if home is not None else Path.home()
    base=home/'.evoarm/rsi/pick_place_runs'
    result={'backed_up':0,'already_backed_up':0,'failed':[]}
    for file in sorted(base.glob('*/state.json')):
        try:
            state=json.loads(file.read_text())
            if not state.get('result') and not (file.parent/'attempt.json').is_file():continue
            if (file.parent/'backup.json').is_file():
                result['already_backed_up']+=1;continue
            archive_trial(file.parent,home=home);result['backed_up']+=1
        except (OSError,ValueError,KeyError,TypeError):
            result['failed'].append(file.parent.name)
    return result


def publish_trace(packet, client):
    """Explicit adapter for a configured Langfuse client, outside motion."""
    with client.start_as_current_observation(
        as_type='span', name='evoarm.rsi.pick_place', input=packet['input'],
        metadata={'setup_id': packet['setup_id'], 'local_trace_id': packet['trace_id'],
                  'backup_sha256': packet['backup']['sha256'], 'schema': packet['schema']},
    ) as span:
        span.update(output=packet['output'])
    client.flush()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='Back up previous RSI trials; no hardware or network calls')
    parser.add_argument('command',choices=['backup-existing'])
    parser.parse_args()
    print(json.dumps(backup_existing(),ensure_ascii=False,indent=2))
