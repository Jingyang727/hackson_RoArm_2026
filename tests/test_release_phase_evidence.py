"""Reported 78c7c889 timing, phase misclassification and unilateral release."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from evoarm_rsi import grok_review
from evoarm_rsi.grasp_once import export_keyframes
from evoarm_rsi.pick_place import build_plan,DEFAULT_PARAMETERS
from evoarm_rsi.release_geometry import upgrade_release_reference
from evoarm_rsi.rsi_policy import application,project
from evoarm_rsi.visual_phases import bind_phase_frames
from tests.test_release_and_retry import reported_scene
from tests.test_rsi_learning import FactoryArm,verdict,phase_facts

TIMING=json.loads(Path(__file__).with_name('fixtures').joinpath('reported_release_timing.json').read_text())


class PhaseEvidence(unittest.TestCase):
    def test_old_archive_no_longer_wastes_last_slot_on_a_64ms_duplicate(self):
        selected=grok_review.select_frames(TIMING['old_frames'])
        ids={r['frame_index'] for r in selected}
        self.assertIn(174,ids)  # Actual transport, omitted by the old selector.
        self.assertTrue({0,95,144,210,300}<=ids)
        self.assertNotIn(94,ids)

    def test_real_release_window_exports_preopening_and_inside_opening_not_just_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);writer=cv2.VideoWriter(str(root/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),15,(160,90))
            for _ in range(301):writer.write(np.full((90,160,3),160,np.uint8))
            writer.release();stamps=[round(i/15*1e9) for i in range(301)]
            (root/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
            events=TIMING['events']
            export_keyframes(root,events,stages=('close_completed','lift_completed','transport_completed','release_completed','finish','fault'),sample_count=6)
            rows=json.loads((root/'keyframes.json').read_text())['frames']
            chosen=grok_review.select_frames(rows)
            labels={label:r for r in chosen for label in r['labels']}
            self.assertEqual(len(chosen),6)
            self.assertTrue({'before','close_review','lift_review','carry_review','release_motion','last'}<=set(labels))
            start=next(e['host_monotonic_ns']/1e9 for e in events if e['stage']=='release')
            end=next(e['host_monotonic_ns']/1e9 for e in events if e['stage']=='release_completed')
            self.assertLess(labels['carry_review']['elapsed_s'],start)
            self.assertTrue(start<labels['release_motion']['elapsed_s']<end)
            # No phantom opening evidence when execution never sent release.
            export_keyframes(root,[e for e in events if e['host_monotonic_ns']<start*1e9],sample_count=6)
            self.assertFalse(any('release_motion' in r['labels'] or 'carry_review' in r['labels']
                                 for r in json.loads((root/'keyframes.json').read_text())['frames']))
            # The controller can log release start, then reject its preflight
            # without sending an open command. Do not turn that into a drop.
            export_keyframes(root,events,sample_count=6,release_attempted=False)
            self.assertFalse(any('release_motion' in r['labels'] or 'release_review' in r['labels']
                                 for r in json.loads((root/'keyframes.json').read_text())['frames']))

    def test_held_then_missed_release_cannot_become_a_pickup_correction(self):
        value=verdict('grasp_missed','image_right')
        value['phases']={
            'pickup':{'result':'held','frames':[144],'note':'球随夹爪离桌'},
            'transport':{'result':'held','frames':[194],'note':'开爪前仍持球'},
            'release':{'result':'missed','frames':[202,300],'note':'开爪后掉到口外'}}
        checked=grok_review.validate_verdict(value)
        self.assertEqual(checked['outcome'],'placement_missed')
        self.assertEqual(checked['reported_outcome'],'grasp_missed')
        self.assertEqual(checked['direction'],'none');self.assertEqual(checked['adjustment'],'unknown')
        self.assertEqual(grok_review.validate_verdict(checked),checked)

    def test_occluded_history_does_not_prove_the_ball_never_left_the_table(self):
        value=verdict('grasp_missed');value['phases']=phase_facts('grasp_missed',95)
        value['phases']['pickup']={'result':'unknown','frames':[],'note':'闭爪时被挡住'}
        self.assertEqual(grok_review.validate_verdict(value)['outcome'],'unknown')
        value['phases']['pickup']={'result':'missed','frames':[95],'note':'夹爪空闭，球在旁边'}
        self.assertEqual(grok_review.validate_verdict(value)['outcome'],'grasp_missed')

    def test_phase_facts_cannot_cite_unattached_or_previous_only_images(self):
        value={**verdict('grasp_missed'),'phases':phase_facts('grasp_missed',95)}
        for frames in ([{'role':'current','frame_index':94}],[{'role':'previous','frame_index':95}]):
            with self.assertRaises(ValueError):bind_phase_frames(value,frames)
        bind_phase_frames(value,[{'role':'current','frame_index':95}])

    def test_new_api_reply_requires_phases_and_retains_six_image_768px_budget(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);capture=root/'capture';capture.mkdir();rows=[]
            for i,label in enumerate(('before','close_review','lift_review','carry_review','release_motion','last')):
                file=f'{i}.jpg';cv2.imwrite(str(capture/file),np.full((720,1280,3),160,np.uint8))
                rows.append({'file':file,'label':label,'frame_index':i*30,'elapsed_s':float(i)})
            (capture/'keyframes.json').write_text(json.dumps({'frames':rows}))
            context={'plan_id':'a'*32,'video_path':str(capture/'rgb.mp4')}
            config={'configured':True,'model':'DeepSeek-V4.1-Flash','backend':'company_api'}
            answer=verdict('grasp_missed')
            with patch.object(grok_review,'configuration',return_value=config),patch.object(grok_review,'complete_content',return_value=json.dumps(answer)),self.assertRaisesRegex(ValueError,'分阶段'):
                grok_review.review(context,root,threading.Event())
            answer['phases']=phase_facts('grasp_missed',30)
            with patch.object(grok_review,'configuration',return_value=config),patch.object(grok_review,'complete_content',return_value=json.dumps(answer)) as api:
                actual=grok_review.review(context,root,threading.Event())
            self.assertEqual(actual,answer)
            request=json.loads((root/'grok-request.json').read_text())
            self.assertEqual(len(request['reviewed_frames']),6)
            self.assertTrue(all(max(r['model_image_size_px'])<=768 for r in request['reviewed_frames']))
            self.assertEqual(sum(p['type']=='image_url' for p in api.call_args.args[1]),6)


class ReleaseReference(unittest.TestCase):
    def test_endpoint_is_image_right_half_radius_without_radial_or_double_offset(self):
        saved,scene=reported_scene();p=copy.deepcopy(DEFAULT_PARAMETERS)
        arm=FactoryArm();arm.send({'T':100})
        plan=build_plan(saved,arm.status(),scene,p);tip=np.array(plan['release_tip_xyz_mm'])
        center=np.array(plan['object_geometry']['release_ball_center_xyz_mm']);center[2]=tip[2]
        shift=np.array(project(scene,tip))-project(scene,center)
        self.assertGreater(shift[0],0.)
        self.assertAlmostEqual(shift[1],0.,places=6)
        self.assertAlmostEqual(math.dist(tip[:2],center[:2]),19.)
        self.assertGreater(plan['release_policy']['minimum_tip_clearance_mm'],0.)

    def test_reference_migration_backs_out_only_old_release_xy_and_is_idempotent(self):
        p=copy.deepcopy(DEFAULT_PARAMETERS);p['pickup_xy_mm']=[15.,8.];p['release_xy_mm']=[-.56,15.99]
        old={'version':28,'parameters':p,'reviews':[{'plan_id':'a'*32,'after':copy.deepcopy(p)}],
             'successful_policy':{'parameters':copy.deepcopy(p),'plan_id':'b'*32}}
        original=copy.deepcopy(old);new,record=upgrade_release_reference(old)
        self.assertEqual(old,original);self.assertEqual(new['parameters']['release_xy_mm'],[0.,0.])
        for key in p:
            if key!='release_xy_mm':self.assertEqual(p[key],new['parameters'][key])
        self.assertEqual(new['reviews'],old['reviews'])
        self.assertNotIn('successful_policy',new)
        self.assertEqual(new['successful_policy_before_release_reference'],old['successful_policy'])
        self.assertTrue(application(new['parameters'],new['parameters'],new['reviews'],new['baseline_resets'])['parameters_verified'])
        self.assertIsNone(upgrade_release_reference(new)[1])
