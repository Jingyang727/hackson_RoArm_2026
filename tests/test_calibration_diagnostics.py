"""Explain held-out error and prove replacement/refit actually use new samples."""
import copy
import json
import unittest
from unittest.mock import patch

import numpy as np

from evoarm_rsi import bench_calibration as bench
from tests import test_bench_ui_api as fixtures


def operator_records():
    rotation=np.array([[-.021340137839272612,.9984272420087487,.05184246263252524],
                       [.9997721389537799,.021284540396125,.0016243496881655861],
                       [.0005183519890763493,.051865313600995784,-.9986539543587087]])
    offset=np.array([306.814230746007,-92.44329827092471,-121.7117101719465])
    records={}
    for key,grid in bench.POINTS.items():
        board=[value*24.92 for value in grid]
        records[key]={'id':key,'board_mm':board,'robot_xyz_mm':(rotation@np.array(board+[0.])+offset).tolist()}
    records['V1']['robot_xyz_mm']=[380.945,5.844,-117.004]
    return records


class Residuals(unittest.TestCase):
    def test_operator_residual_is_about_one_board_row_not_a_twenty_six_mm_height_error(self):
        fit=bench.fit_points(operator_records());v1=fit['point_diagnostics'][-1]
        self.assertEqual(v1['id'],'V1');self.assertEqual(v1['grid'],[4,2])
        np.testing.assert_allclose(v1['predicted_xyz_mm'],[354.44865955,8.27481003,-119.07507362],atol=1e-7)
        np.testing.assert_allclose(v1['board_xy_error_mm'],[-2.99461816,26.51034633],atol=1e-7)
        self.assertAlmostEqual(v1['height_error_mm'],.6985988,places=6)
        self.assertAlmostEqual(v1['error_mm'],26.68809174,places=6)
        self.assertEqual(fit['suggested_recheck'],'V1')
        self.assertEqual([reason['metric'] for reason in fit['failure_reasons']],['validation_error_mm'])
        self.assertEqual(fit['limits_mm'],{'train_rms_mm':6.,'train_max_mm':9.,'validation_error_mm':10.})

    def test_changing_only_v1_keeps_rms_but_recomputes_independent_error(self):
        records=operator_records();new=bench.fit_points(records)
        records['V1']['robot_xyz_mm']=[379.778,7.574,-118.139];old=bench.fit_points(records)
        self.assertAlmostEqual(old['train_rms_mm'],new['train_rms_mm'])
        self.assertAlmostEqual(old['validation_error_mm'],25.35631788,places=6)
        self.assertGreater(new['validation_error_mm'],old['validation_error_mm']+1.)


class ReplacementAPI(unittest.TestCase):
    setUp=fixtures.BenchUIApi.setUp
    request=fixtures.BenchUIApi.request

    def seed(self):
        saved=json.loads(self.path.read_text());saved['records']=operator_records()
        for record in saved['records'].values():
            record.update(tool_pose_rad={'pitch':0.,'roll':0.,'gripper':float(np.pi)},captured_at='2026-11-14T06:49:10+00:00')
        saved['reference']['square_mm']=24.92;saved['reference_sha256']=bench._digest(saved['reference'])
        saved['fit']=bench.fit_points(saved['records']);saved['capture_revision']=5
        self.path.write_text(json.dumps(saved));return saved

    def test_replace_v1_generates_receipt_archives_old_point_and_refits_current_sample(self):
        before=self.seed();expected=before['fit']['point_diagnostics'][-1]['predicted_xyz_mm']
        self.service.arm.torque=0;self.service.arm.xyz=expected
        response,result=self.request('/api/bench/capture',{'point_id':'V1','replace':True,
            'aligned_and_supported':True,'session_id':before['session_id']})
        self.assertEqual(response['status'],200,result)
        self.assertLess(result['fit']['validation_error_mm'],1e-8)
        self.assertAlmostEqual(result['fit']['train_rms_mm'],before['fit']['train_rms_mm'])
        receipt=result['last_capture'];self.assertEqual(receipt['query_count'],3)
        self.assertGreater(receipt['change_norm_mm'],26.);self.assertFalse(receipt['xyz_unchanged'])
        self.assertEqual(receipt['capture_revision'],6);self.assertEqual(result['fit']['input_capture_revision'],6)
        after=json.loads(self.path.read_text());self.assertEqual(after['history'][-1],before['records']['V1'])
        self.assertEqual(after['records']['V1']['capture_id'],receipt['capture_id'])
        self.assertEqual(self.service.arm.commands,[])
        response,repeat=self.request('/api/bench/capture',{'point_id':'V1','replace':True,'aligned_and_supported':True})
        self.assertEqual(response['status'],200,repeat)
        self.assertNotEqual(repeat['last_capture']['capture_id'],receipt['capture_id'])
        self.assertTrue(repeat['last_capture']['xyz_unchanged'])
        self.assertEqual(repeat['last_capture']['change_norm_mm'],0.)

    def test_old_failed_fit_gets_diagnostics_readonly_without_requiring_another_capture(self):
        saved=self.seed()
        for name in ('point_diagnostics','failure_reasons','suggested_recheck','limits_mm'):saved['fit'].pop(name)
        self.path.write_text(json.dumps(saved));before=self.path.read_bytes()
        with patch.object(self.service,'status',side_effect=AssertionError('no fresh robot reads')):
            response,result=self.request('/api/bench/status',method='GET')
        self.assertEqual(response['status'],200,result);self.assertEqual(len(result['fit']['point_diagnostics']),5)
        self.assertEqual(self.path.read_bytes(),before);self.assertEqual(self.service.arm.commands,[])

    def test_stale_tab_cannot_replace_a_new_calibration_before_reading_robot(self):
        before=self.path.read_bytes()
        with patch.object(self.service,'status',side_effect=AssertionError('do not sample another session')):
            response,result=self.request('/api/bench/capture',{'point_id':'V1','replace':True,
                'aligned_and_supported':True,'session_id':'another-session'})
        self.assertEqual(response['status'],409,result);self.assertEqual(self.path.read_bytes(),before)
