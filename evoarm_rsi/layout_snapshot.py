"""Verify, compare or explicitly install the pinned bench snapshot. No robot imports."""
from __future__ import annotations

import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path,PurePosixPath
import re
import uuid
import zipfile
from urllib.parse import urlsplit
from urllib.request import build_opener,ProxyHandler,HTTPRedirectHandler,Request

DEFAULT_INDEX=Path(__file__).resolve().parents[1]/'calibrations/bench-layout-v2/latest.json'


def digest(raw):return hashlib.sha256(raw).hexdigest()


def fingerprint(session):
    return digest(json.dumps({k:session[k] for k in ('session_id','reference_sha256','records')},
                             sort_keys=True,allow_nan=False).encode())


def bounded(base,name):
    if not isinstance(name,str) or not name or '\\' in name:raise ValueError('快照文件名无效')
    relative=PurePosixPath(name)
    if relative.is_absolute() or '..' in relative.parts:raise ValueError('快照文件路径越界')
    path=Path(base).joinpath(*relative.parts)
    if path.is_symlink() or not path.resolve().is_relative_to(Path(base).resolve()):raise ValueError('快照文件路径越界或为符号链接')
    return path


def load_bundle(index=DEFAULT_INDEX):
    index=Path(index);pin=json.loads(index.read_bytes());root=bounded(index.parent,pin['snapshot'])
    raw=bounded(root,pin['manifest']).read_bytes()
    if digest(raw)!=pin['manifest_sha256']:raise ValueError('导出清单SHA256不匹配')
    manifest=json.loads(raw);files={}
    for row in manifest['files']:
        name=row['archive_path']
        if name in files:raise ValueError('导出清单文件重复')
        data=bounded(root,name).read_bytes()
        if len(data)!=row['bytes'] or digest(data)!=row['sha256']:raise ValueError('快照文件SHA256不匹配：'+name)
        files[name]=data
    pointers=[n for n in files if re.fullmatch(r'calibrations/[A-Za-z0-9_-]+/active-session.json',n)]
    if len(pointers)!=1:raise ValueError('快照必须有唯一活动标定指针')
    setup=pointers[0].split('/')[1];pointer=json.loads(files[pointers[0]]);revision=pointer['revision_id']
    if not re.fullmatch('[a-f0-9]{32}',revision):raise ValueError('活动revision无效')
    session_name=f'calibrations/{setup}/revisions/{revision}/session.json';session=json.loads(files[session_name])
    scope=fingerprint(session)
    if (scope!=pin['calibration_fingerprint'] or scope!=manifest['calibration_fingerprint']
            or session['setup_id']!=setup):raise ValueError('标定指纹与导出清单不一致')
    if digest(json.dumps(session['reference'],sort_keys=True,allow_nan=False).encode())!=session['reference_sha256']:
        raise ValueError('标定参考内容校验失败')
    trial=manifest['successful_pick_place_plan_id']
    if not re.fullmatch('[a-f0-9]{32}',trial):raise ValueError('成功试次ID无效')
    state=json.loads(files[f'pick_place_runs/{trial}/state.json'])
    snapshot=files[f'pick_place_runs/{trial}/calibration-snapshot.json']
    if (snapshot!=files[session_name] or state['calibration_fingerprint']!=scope
            or state['review']['outcome']!='placed' or state['review'].get('ball_state')!='funnel'
            or not state['result'].get('motion_completed') or not state['result'].get('placement_attempted')
            or not state['result'].get('recording_covers_motion') or state['result'].get('motion_error') or state['result'].get('camera_error')):
        raise ValueError('成功试次与当前标定或完整执行记录不一致')
    policy_names={kind:f'{kind}/{setup}/{scope}.json' for kind in ('pick_place_policies','challenge_policies')}
    policies={kind:json.loads(files[name]) for kind,name in policy_names.items()}
    single=policies['pick_place_policies']
    if (state['plan']['parameters']!=single['parameters'] or single.get('successful_policy',{}).get('plan_id')!=trial):
        raise ValueError('成功试次实际参数与单球策略不一致')
    source=session['reference']['source_trial_id']
    if not re.fullmatch('[a-f0-9]{32}',source):raise ValueError('参考采集ID无效')
    expected={pointers[0],session_name,session_name[:-5]+'.png',*policy_names.values(),
              f'experiments/{source}/gemini_color_intrinsics.json'}
    if set(pin['install_paths'])!=expected or len(pin['install_paths'])!=len(expected):
        raise ValueError('安装清单不匹配；只允许活动标定、参考图、参数和参考内参')
    return {'pin':pin,'index_raw':index.read_bytes(),'manifest_raw':raw,'manifest':manifest,'files':files,'setup_id':setup,'scope':scope,
            'session':session,'session_name':session_name,'policies':policies,'policy_names':policy_names,'state':state}


def summary(bundle):
    reference=bundle['session']['reference']['source_trial_id']
    intrinsics=json.loads(bundle['files'][f'experiments/{reference}/gemini_color_intrinsics.json'])
    return {'snapshot':bundle['pin']['snapshot'],'exported_at':bundle['manifest']['created_at'],
        'setup_id':bundle['setup_id'],'calibration_fingerprint':bundle['scope'],
        'session_sha256':digest(bundle['files'][bundle['session_name']]),
        'source_runtime':bundle['manifest']['runtime'],
        'reference_color_intrinsics':intrinsics,
        'policies':{k:{'version':p['version'],'parameters':p['parameters'],
                       'sha256':digest(bundle['files'][bundle['policy_names'][k]])} for k,p in bundle['policies'].items()},
        'successful_trial_id':bundle['state']['id'],'source_files_verified':len(bundle['files']),
        'robot_control':False}


def compare(bundle,home=None):
    base=(Path(home) if home is not None else Path.home())/'.evoarm/rsi';rows=[]
    for name in bundle['pin']['install_paths']:
        path=bounded(base,name);raw=path.read_bytes() if path.is_file() else None
        rows.append({'path':name,'exists':raw is not None,'matches_snapshot':raw==bundle['files'][name],
                     'sha256':digest(raw) if raw is not None else None,'expected_sha256':digest(bundle['files'][name])})
    root=base/'calibrations'/bundle['setup_id'];active=root/'session.json';active_error=None;scope=None;session_sha=None
    try:
        pointer=root/'active-session.json'
        if pointer.exists():
            revision=json.loads(pointer.read_bytes())['revision_id']
            if not isinstance(revision,str) or not re.fullmatch('[a-f0-9]{32}',revision):raise ValueError('本机活动标定指针无效')
            active=root/'revisions'/revision/'session.json'
        raw=active.read_bytes();scope=fingerprint(json.loads(raw));session_sha=digest(raw)
    except (OSError,ValueError,KeyError,TypeError) as exc:active_error=str(exc)
    parameters={}
    for kind,name in bundle['policy_names'].items():
        try:
            policy=json.loads(bounded(base,name).read_bytes());parameters[kind]={
                'version':policy['version'],'parameters_match':policy['parameters']==bundle['policies'][kind]['parameters']}
        except (OSError,ValueError,KeyError,TypeError):parameters[kind]={'version':None,'parameters_match':False}
    geometry_match=scope==bundle['scope'] and session_sha==digest(bundle['files'][bundle['session_name']])
    return {**summary(bundle),'home':str(base),'active_session':str(active),'active_fingerprint':scope,
        'active_session_sha256':session_sha,'active_error':active_error,'calibration_matches':geometry_match,
        'local_policies':parameters,'files':rows,'matches_snapshot':geometry_match and all(r['matches_snapshot'] for r in rows)}


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):raise ValueError('本机诊断接口不接受重定向')


def runtime(controller,bundle,home=None):
    url=urlsplit(controller)
    if url.scheme!='http' or url.hostname not in ('localhost','127.0.0.1','::1') or url.username or url.password or url.query or url.fragment:
        raise ValueError('仅接受本机HTTP控制台地址')
    opener=build_opener(ProxyHandler({}),NoRedirect())
    def get(path):
        with opener.open(Request(controller.rstrip('/')+path,method='GET'),timeout=8) as response:
            return json.loads(response.read(4_000_000))
    bench=get('/api/bench/status');readiness=get('/api/bench/rsi/agent/readiness')
    local=compare(bundle,home);actual=bench.get('session')
    return {'code_revision':readiness.get('code_revision'),'controller_project_root':readiness.get('controller_project_root'),
        'active_session_from_api':actual,'calibration_fingerprint':readiness.get('calibration_fingerprint'),
        'session_path_matches_local':bool(actual and Path(actual).resolve()==Path(local['active_session']).resolve()),
        'fingerprint_matches_snapshot':readiness.get('calibration_fingerprint')==bundle['scope'],
        'pending':bool(bench.get('health',{}).get('pending')),'automation_active':bool(bench.get('health',{}).get('automation_owner'))}


def atomic(path,raw):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:temp.write_bytes(raw);os.replace(temp,path)
    finally:temp.unlink(missing_ok=True)


def install(bundle,home=None):
    """Explicit maintenance action; stop the web service first. Never install trial history."""
    base=(Path(home) if home is not None else Path.home())/'.evoarm/rsi'
    names=bundle['pin']['install_paths'];old={name:(bounded(base,name).read_bytes() if bounded(base,name).is_file() else None) for name in names}
    changed=[name for name in names if old[name]!=bundle['files'][name]]
    if not changed:return {'installed_files':[],'backup_directory':None,'comparison':compare(bundle,home),'robot_control':False}
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
    backup=base/'layout_backups'/stamp;backup.mkdir(parents=True,exist_ok=False)
    receipt={'snapshot':bundle['pin']['snapshot'],'status':'prepared','files':[]}
    for name in changed:
        raw=old[name]
        if raw is not None:atomic(bounded(backup,name),raw)
        receipt['files'].append({'path':name,'previously_existed':raw is not None,'before_sha256':digest(raw) if raw is not None else None,
                                 'after_sha256':digest(bundle['files'][name])})
    atomic(backup/'restore-manifest.json',json.dumps(receipt,ensure_ascii=False,indent=2).encode())
    written=[]
    try:
        # Switch the active pointer only after its revision and policies exist.
        ordered=sorted(changed,key=lambda n:n.endswith('/active-session.json'))
        for name in ordered:
            dest=bounded(base,name);current=dest.read_bytes() if dest.is_file() else None
            if current!=old[name]:raise RuntimeError('安装期间本机数据发生变化，请停止网页后重试：'+name)
            written.append(name);atomic(dest,bundle['files'][name])
        result=compare(bundle,home)
        if not result['matches_snapshot']:raise RuntimeError('安装后SHA256或活动指纹核验失败')
        receipt['status']='completed'
    except Exception:
        for name in reversed(written):
            dest=bounded(base,name)
            if old[name] is None:dest.unlink(missing_ok=True)
            else:atomic(dest,old[name])
        receipt['status']='rolled_back';raise
    finally:atomic(backup/'restore-manifest.json',json.dumps(receipt,ensure_ascii=False,indent=2).encode())
    return {'installed_files':changed,'backup_directory':str(backup),'comparison':result,
            'restart_web_service':True,'robot_control':False}


TRANSFER_README='''# 成功电脑配置迁移包（不安装或升级Python环境）

本包复制成功电脑2026-11-16导出的在用数据。使用同一套机械臂、相机，且相机、底座和桌面位置保持不变时使用。

解压后进入本文件所在目录，用普通用户的python3执行。无需git、pip或sudo；不会打开设备，不会发送运动或复位。

1. 先查看本包完整性及本机差异（compare发现不同返回退出码1，是正常诊断结果）：

    python3 tools/install_layout.py verify
    python3 tools/install_layout.py compare

2. 结束现有任务、等待机械臂静止，并停止网页服务。然后备份并导入配置：

    python3 tools/install_layout.py install

3. 用原目录和原Python重新启动网页服务，然后核对实际加载：

    python3 tools/install_layout.py compare --controller http://127.0.0.1:8765

核对：matches_snapshot=true、calibration_matches=true、runtime.session_path_matches_local=true、runtime.fingerprint_matches_snapshot=true。
local_policies应为单球v64、挑战v13，parameters_match均为true。确认后再在网页开始测试。

标定指纹：05368598e3e0f107d87f4f4d174a2cfaf88fdf9efffd91f97b0fbccb9dd7eed4。
compare使用的是当前用户目录；若网页用另一用户启动，运行路径核验会指出不同。不要用sudo运行导入程序。

导入只复制活动指针、session、参考图、单球策略、挑战策略和参考内参，共6个文件。
不同的原文件先保存到~/.evoarm/rsi/layout_backups/，写入失败会回滚；相同文件不重复写入。
历史成功试次只保留在包里作为来源证据，不安装到同事电脑的执行历史。不包含token，不更改API、串口或环境配置。

本包不包含程序代码部署，也不改变原有代码。代码应为已合入main的连续挑战版本。
成功电脑4aad187与main的8d15d54代码树相同；后续只增加配置归档的main提交号可以不同。
本包源导出与本机文件的SHA256均可核验。继续学习后策略发生变化，再次compare可能不同，不表示标定自动丢失。

SUCCESS_DEMO_COMPUTER_MANIFEST.json记录8个原始文件的SHA256；PACKAGE_MANIFEST.json另记录整个迁移包文件的SHA256。
若核对仍失败，将compare输出回传，不要重复抓取来猜是哪份文件。
'''


def package(bundle,output):
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    base='evoarm-success-demo-transfer';data={f'{base}/README.md':TRANSFER_README.encode(),
        f'{base}/tools/install_layout.py':Path(__file__).read_bytes(),
        f'{base}/calibrations/bench-layout-v2/latest.json':bundle['index_raw'],
        f"{base}/calibrations/bench-layout-v2/{bundle['pin']['snapshot']}/{bundle['pin']['manifest']}":bundle['manifest_raw']}
    for name,raw in bundle['files'].items():data[f"{base}/calibrations/bench-layout-v2/{bundle['pin']['snapshot']}/{name}"]=raw
    manifest={'source_archive_sha256':bundle['pin']['source_archive_sha256'],'calibration_fingerprint':bundle['scope'],
        'files':[{'path':name.split('/',1)[1],'bytes':len(raw),'sha256':digest(raw)} for name,raw in sorted(data.items())]}
    data[f'{base}/PACKAGE_MANIFEST.json']=json.dumps(manifest,ensure_ascii=False,indent=2).encode()
    temp=output.with_name(output.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with zipfile.ZipFile(temp,'w',compression=zipfile.ZIP_DEFLATED) as archive:
            for name,raw in sorted(data.items()):archive.writestr(name,raw)
        with zipfile.ZipFile(temp) as archive:
            if archive.testzip() is not None:raise ValueError('迁移ZIP校验失败')
        os.replace(temp,output)
    finally:temp.unlink(missing_ok=True)
    return {'package':str(output),'sha256':digest(output.read_bytes()),'bytes':output.stat().st_size,'robot_control':False}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['verify','compare','install','package'])
    parser.add_argument('--index',type=Path,default=DEFAULT_INDEX)
    parser.add_argument('--home',type=Path,default=None)
    parser.add_argument('--controller',help='Optional read-only runtime check; compare only')
    parser.add_argument('--output',type=Path,help='Output ZIP path; package only')
    args=parser.parse_args(argv)
    try:
        bundle=load_bundle(args.index)
        if args.controller and args.action!='compare':raise ValueError('--controller只用于只读compare')
        if args.action=='package':
            if args.output is None:raise ValueError('package需要--output ZIP路径')
            print(json.dumps(package(bundle,args.output),ensure_ascii=False,indent=2));return 0
        value=summary(bundle) if args.action=='verify' else compare(bundle,args.home) if args.action=='compare' else install(bundle,args.home)
        if args.controller:value['runtime']=runtime(args.controller,bundle,args.home)
        print(json.dumps(value,ensure_ascii=False,indent=2))
        if args.action=='compare':return 0 if value['matches_snapshot'] and (not args.controller or
            value['runtime']['fingerprint_matches_snapshot'] and value['runtime']['session_path_matches_local']) else 1
        return 0
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as exc:
        print(json.dumps({'error':str(exc),'robot_control':False},ensure_ascii=False));return 2


if __name__=='__main__':raise SystemExit(main())
