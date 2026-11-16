"""The sudo SDK process must not leave per-user recording roots owned by root."""
import os
from pathlib import Path
import stat
from types import SimpleNamespace
import unittest
from unittest.mock import call,patch

from evoarm_rsi.camera_service import ensure_recording_root_owner


class RecordingRootOwner(unittest.TestCase):
    def test_privileged_creator_returns_both_directory_levels_to_user(self):
        root=Path('/example/home/.evoarm/rsi/experiments')
        metadata=SimpleNamespace(st_uid=0,st_gid=0,st_mode=stat.S_IFDIR|0o500)
        with patch.object(Path,'is_symlink',return_value=False),patch.object(Path,'stat',return_value=metadata),\
             patch('evoarm_rsi.camera_service.os.geteuid',return_value=0),\
             patch('evoarm_rsi.camera_service.os.chown') as chown,patch('evoarm_rsi.camera_service.os.chmod') as chmod:
            ensure_recording_root_owner(root,501,20)
        self.assertEqual(chown.call_args_list,[call(root.parent,501,20),call(root,501,20)])
        self.assertEqual(chmod.call_args_list,[call(root.parent,0o700),call(root,0o700)])

    def test_existing_correct_directories_are_not_rewritten(self):
        root=Path('/example/home/.evoarm/rsi/experiments')
        metadata=SimpleNamespace(st_uid=501,st_gid=20,st_mode=stat.S_IFDIR|0o755)
        with patch.object(Path,'is_symlink',return_value=False),patch.object(Path,'stat',return_value=metadata),\
             patch('evoarm_rsi.camera_service.os.chown') as chown,patch('evoarm_rsi.camera_service.os.chmod') as chmod:
            ensure_recording_root_owner(root,501,20)
        chown.assert_not_called();chmod.assert_not_called()

    def test_unrelated_user_directory_is_not_taken_over(self):
        metadata=SimpleNamespace(st_uid=777,st_gid=20,st_mode=stat.S_IFDIR|0o755)
        with patch.object(Path,'is_symlink',return_value=False),patch.object(Path,'stat',return_value=metadata),\
             patch('evoarm_rsi.camera_service.os.geteuid',return_value=0),patch('evoarm_rsi.camera_service.os.chown') as chown:
            with self.assertRaisesRegex(RuntimeError,'another user'):ensure_recording_root_owner(Path('/example/rsi/experiments'),501,20)
        chown.assert_not_called()

    def test_ordinary_camera_process_does_not_require_chown_on_existing_parent(self):
        with patch('evoarm_rsi.camera_service.os.geteuid',return_value=501),\
             patch('evoarm_rsi.camera_service.os.chown') as chown,patch('evoarm_rsi.camera_service.os.chmod') as chmod:
            ensure_recording_root_owner(Path('/example/root-owned-rsi/experiments'),501,20)
        chown.assert_not_called();chmod.assert_not_called()
