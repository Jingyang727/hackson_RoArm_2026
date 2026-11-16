"""Pinned real export integrity and transactional migration, no hardware calls."""
import copy
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile
import tempfile
import unittest
from unittest.mock import patch

from evoarm_rsi import layout_snapshot as layout
from evoarm_rsi.coordinate_test import fingerprint
from evoarm_rsi.bench_calibration import fit_points


class Snapshot(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.root=Path(temp.name);self.home=self.root/'home'
        self.bundle=layout.load_bundle()

    def local_bundle(self):
        target=self.root/'source';shutil.copytree(layout.DEFAULT_INDEX.parent,target)
        return target/'latest.json'

    def update_manifest(self,index,mutate):
        pin=json.loads(index.read_bytes());root=index.parent/pin['snapshot'];manifest_path=root/pin['manifest']
        manifest=json.loads(manifest_path.read_bytes());mutate(root,manifest)
        raw=json.dumps(manifest).encode();manifest_path.write_bytes(raw);pin['manifest_sha256']=layout.digest(raw)
        index.write_text(json.dumps(pin))

    def test_export_matches_original_session_fit_and_successful_actual_parameters(self):
        bundle=self.bundle
        self.assertEqual(bundle['scope'],fingerprint(bundle['session']))
        self.assertEqual(bundle['scope'],'05368598e3e0f107d87f4f4d174a2cfaf88fdf9efffd91f97b0fbccb9dd7eed4')
        fit=fit_points(bundle['session']['records']);self.assertTrue(fit['planar_fit_validated'])
        self.assertAlmostEqual(fit['validation_error_mm'],3.7081351091353385)
        self.assertEqual(bundle['policies']['pick_place_policies']['version'],64)
        self.assertEqual(bundle['state']['policy_version'],63)  # v64 retains the v63 successful trial parameters.
        self.assertEqual(bundle['state']['plan']['parameters'],bundle['policies']['pick_place_policies']['parameters'])
        self.assertEqual(bundle['policies']['challenge_policies']['version'],13)

    def test_install_is_byte_exact_and_excludes_trial_history_and_credentials(self):
        secret=self.home/'.evoarm/company_llm.env';secret.parent.mkdir(parents=True);secret.write_bytes(b'fixture-existing-settings')
        result=layout.install(self.bundle,self.home);base=self.home/'.evoarm/rsi'
        self.assertTrue(result['comparison']['matches_snapshot']);self.assertEqual(len(result['installed_files']),6)
        for name in self.bundle['pin']['install_paths']:self.assertEqual((base/name).read_bytes(),self.bundle['files'][name])
        self.assertFalse((base/'pick_place_runs').exists());self.assertEqual(secret.read_bytes(),b'fixture-existing-settings')
        self.assertFalse(result['robot_control'])

    def test_reinstall_does_not_write_or_create_new_backups(self):
        layout.install(self.bundle,self.home)
        with patch.object(layout,'atomic',side_effect=AssertionError('already matches')):
            result=layout.install(self.bundle,self.home)
        self.assertEqual(result['installed_files'],[]);self.assertIsNone(result['backup_directory'])

    def test_existing_policy_is_backed_up_and_differences_are_reported(self):
        layout.install(self.bundle,self.home);name=self.bundle['policy_names']['pick_place_policies']
        target=self.home/'.evoarm/rsi'/name;old=json.loads(target.read_bytes());old['parameters']['pickup_xy_mm']=[0,0]
        raw=json.dumps(old).encode();target.write_bytes(raw)
        report=layout.compare(self.bundle,self.home)
        self.assertTrue(report['calibration_matches']);self.assertFalse(report['matches_snapshot'])
        self.assertFalse(report['local_policies']['pick_place_policies']['parameters_match'])
        installed=layout.install(self.bundle,self.home)
        self.assertEqual((Path(installed['backup_directory'])/name).read_bytes(),raw)
        self.assertTrue(installed['comparison']['matches_snapshot'])

    def test_active_pointer_wins_over_correct_but_inactive_revision(self):
        layout.install(self.bundle,self.home);root=self.home/'.evoarm/rsi/calibrations/bench-layout-v2'
        (root/'active-session.json').write_text(json.dumps({'revision_id':'0'*32}))
        report=layout.compare(self.bundle,self.home)
        self.assertFalse(report['calibration_matches']);self.assertIn('0'*32,report['active_session'])
        result=layout.install(self.bundle,self.home)
        self.assertEqual(result['installed_files'],['calibrations/bench-layout-v2/active-session.json'])

    def test_corrupt_export_is_rejected_before_installation(self):
        index=self.local_bundle();name=self.bundle['policy_names']['challenge_policies']
        target=index.parent/self.bundle['pin']['snapshot']/name;target.write_bytes(target.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'SHA256'):layout.load_bundle(index)
        self.assertFalse(self.home.exists())

    def test_apparent_success_without_complete_motion_is_not_valid_evidence(self):
        index=self.local_bundle()
        def corrupt(root,manifest):
            row=next(r for r in manifest['files'] if r['archive_path'].endswith('/state.json'))
            path=root/row['archive_path'];state=json.loads(path.read_bytes());state['result']['motion_completed']=False
            raw=json.dumps(state).encode();path.write_bytes(raw);row.update(sha256=layout.digest(raw),bytes=len(raw))
        self.update_manifest(index,corrupt)
        with self.assertRaisesRegex(ValueError,'成功试次'):layout.load_bundle(index)

    def test_arbitrary_install_paths_are_rejected(self):
        index=self.local_bundle();pin=json.loads(index.read_bytes());pin['install_paths'].append('../company_llm.env')
        index.write_text(json.dumps(pin))
        with self.assertRaisesRegex(ValueError,'安装清单'):layout.load_bundle(index)
        with self.assertRaises(ValueError):layout.bounded(self.home,'../outside')

    def test_failure_rolls_back_and_preserves_original_bytes(self):
        base=self.home/'.evoarm/rsi';names=self.bundle['pin']['install_paths']
        existing=base/self.bundle['policy_names']['pick_place_policies'];existing.parent.mkdir(parents=True)
        existing.write_bytes(b'original local policy bytes');before={n:(base/n).read_bytes() if (base/n).is_file() else None for n in names}
        original=layout.atomic;failed=[]
        def write(path,raw):
            if str(path)==str(base/self.bundle['policy_names']['challenge_policies']) and not failed:
                failed.append(True);raise OSError('synthetic install failure')
            return original(path,raw)
        with patch.object(layout,'atomic',side_effect=write):
            with self.assertRaisesRegex(OSError,'synthetic'):layout.install(self.bundle,self.home)
        for name in names:
            path=base/name;self.assertEqual(path.read_bytes() if path.is_file() else None,before[name])
        receipt=next((base/'layout_backups').glob('*/restore-manifest.json'))
        self.assertEqual(json.loads(receipt.read_bytes())['status'],'rolled_back')

    @unittest.skipIf(os.geteuid()==0,'root bypasses ordinary directory write permissions')
    def test_unwritable_experiments_is_detected_before_any_configuration_changes(self):
        base=self.home/'.evoarm/rsi';experiments=base/'experiments';experiments.mkdir(parents=True)
        policy=base/self.bundle['policy_names']['pick_place_policies'];policy.parent.mkdir(parents=True);policy.write_bytes(b'old policy')
        experiments.chmod(0o500);self.addCleanup(experiments.chmod,0o700)
        with self.assertRaises(layout.TargetPermissionError) as raised:layout.install(self.bundle,self.home)
        self.assertEqual(raised.exception.path,str(experiments));self.assertEqual(policy.read_bytes(),b'old policy')
        self.assertFalse((base/'calibrations').exists());self.assertFalse((base/'layout_backups').exists())
        experiments.chmod(0o700)
        self.assertTrue(layout.install(self.bundle,self.home)['comparison']['matches_snapshot'])

    @unittest.skipIf(os.geteuid()==0,'root bypasses ordinary directory write permissions')
    def test_matching_data_needs_no_write_permission(self):
        layout.install(self.bundle,self.home);experiments=self.home/'.evoarm/rsi/experiments'
        experiments.chmod(0o500);self.addCleanup(experiments.chmod,0o700)
        self.assertEqual(layout.install(self.bundle,self.home)['installed_files'],[])

    def test_failed_unreadable_destination_is_not_revisited_during_rollback(self):
        base=self.home/'.evoarm/rsi';name=self.bundle['policy_names']['pick_place_policies']
        target=base/name;target.parent.mkdir(parents=True);target.write_bytes(b'old policy')
        blocked=next(n for n in self.bundle['pin']['install_paths'] if n.startswith('experiments/'))
        original_write=layout.atomic;original_bounded=layout.bounded;failed=[]
        def write(path,raw):
            if path==base/blocked:
                failed.append(True);raise PermissionError('synthetic destination failure')
            return original_write(path,raw)
        def resolve(folder,path):
            if folder==base and path==blocked and failed:raise PermissionError('destination now inaccessible')
            return original_bounded(folder,path)
        with patch.object(layout,'atomic',side_effect=write),patch.object(layout,'bounded',side_effect=resolve):
            with self.assertRaisesRegex(PermissionError,'synthetic destination'):layout.install(self.bundle,self.home)
        self.assertEqual(target.read_bytes(),b'old policy')
        self.assertFalse((base/self.bundle['session_name']).exists())
        receipt=next((base/'layout_backups').glob('*/restore-manifest.json'))
        self.assertEqual(json.loads(receipt.read_bytes())['status'],'rolled_back')

    def test_runtime_check_uses_read_only_localhost_routes(self):
        layout.install(self.bundle,self.home);calls=[];report=layout.compare(self.bundle,self.home)
        class Reader:
            def open(inner,req,timeout):
                calls.append((req.get_method(),req.full_url))
                value={'session':report['active_session'],'health':{}} if req.full_url.endswith('/status') else {
                    'code_revision':'test-running-code','controller_project_root':'/test/project','calibration_fingerprint':self.bundle['scope']}
                return io.BytesIO(json.dumps(value).encode())
        with patch.object(layout,'build_opener',return_value=Reader()):result=layout.runtime('http://127.0.0.1:8765',self.bundle,self.home)
        self.assertTrue(result['session_path_matches_local']);self.assertTrue(result['fingerprint_matches_snapshot'])
        self.assertEqual([m for m,_ in calls],['GET','GET'])
        with self.assertRaises(ValueError):layout.runtime('https://example.com',self.bundle,self.home)

    def test_portable_zip_runs_outside_git_without_project_imports(self):
        output=self.root/'transfer.zip';layout.package(self.bundle,output)
        unpacked=self.root/'unpacked'
        with zipfile.ZipFile(output) as archive:archive.extractall(unpacked)
        root=unpacked/'evoarm-success-demo-transfer';script=root/'tools/install_layout.py'
        manifest=json.loads((root/'PACKAGE_MANIFEST.json').read_bytes())
        for row in manifest['files']:
            raw=(root/row['path']).read_bytes()
            self.assertEqual(len(raw),row['bytes']);self.assertEqual(layout.digest(raw),row['sha256'])
        def run(*args):return json.loads(subprocess.check_output([sys.executable,str(script),*args],cwd=self.root,text=True))
        self.assertEqual(run('verify')['policies']['pick_place_policies']['version'],64)
        result=run('install','--home',str(self.home));self.assertTrue(result['comparison']['matches_snapshot'])
        self.assertTrue(run('compare','--home',str(self.home))['matches_snapshot'])

    def test_published_zip_is_current_and_all_downloaded_files_verify(self):
        path=layout.DEFAULT_INDEX.parent/'transfers/evoarm-success-demo-transfer-20261116.zip'
        expected=path.with_suffix('.zip.sha256').read_text().split()[0]
        self.assertEqual(layout.digest(path.read_bytes()),expected)
        base='evoarm-success-demo-transfer/'
        with zipfile.ZipFile(path) as archive:
            self.assertEqual(archive.read(base+'tools/install_layout.py'),Path(layout.__file__).read_bytes())
            self.assertEqual(archive.read(base+'calibrations/bench-layout-v2/latest.json'),self.bundle['index_raw'])
            manifest=json.loads(archive.read(base+'PACKAGE_MANIFEST.json'))
            for row in manifest['files']:
                raw=archive.read(base+row['path']);self.assertEqual(layout.digest(raw),row['sha256'])
            for name,raw in self.bundle['files'].items():
                self.assertEqual(archive.read(base+'calibrations/bench-layout-v2/'+self.bundle['pin']['snapshot']+'/'+name),raw)

