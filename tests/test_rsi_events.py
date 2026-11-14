"""Public event replay, streaming lifecycle and no hardware access."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch

from fastapi import FastAPI

from control_ui.rsi_trial import install
from evoarm_rsi.rsi_events import LearningEvents,stream_events


class PublicEvents(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        item=patch.object(Path,'home',return_value=self.home);item.start();self.addCleanup(item.stop)
        self.events=LearningEvents();self.key='a'*32;self.events.reset(self.key)

    def test_snapshot_replay_and_model_text_survive_reload_without_duplicate_deltas(self):
        self.events.emit('phase',phase='analyzing',attempt=1)
        self.events.model({'type':'started','preflight':False,'model':'grok-test'},attempt=1,plan_id='b'*32)
        cursor=self.events.after()['cursor']
        self.events.model({'type':'text','delta':'球仍在桌面'},attempt=1,plan_id='b'*32)
        replay=self.events.after(cursor)
        self.assertFalse(replay['reset']);self.assertEqual(len(replay['events']),1)
        self.assertEqual(replay['events'][0]['delta'],'球仍在桌面')
        fresh=self.events.after();self.assertTrue(fresh['reset'])
        self.assertEqual(fresh['model_output']['text'],'球仍在桌面')
        self.assertNotIn('grok_delta',[r['kind'] for r in fresh['events']])
        self.assertEqual(self.events.after(fresh['cursor'])['events'],[])
        lines=(self.home/'.evoarm/rsi/learning_sessions'/self.key/'events.jsonl').read_text().splitlines()
        self.assertEqual([json.loads(r)['sequence'] for r in lines],[1,2,3])

    def test_invalid_or_old_session_cursor_resets_and_snapshots_are_copies(self):
        self.events.emit('phase',phase='descend')
        for cursor in ('invalid','x:999','b'*32+':1',self.key+':999'):
            snapshot=self.events.after(cursor);self.assertTrue(snapshot['reset'])
            snapshot['events'][0]['phase']='corrupted'
        self.assertEqual(self.events.after()['events'][0]['phase'],'descend')

    def test_ring_overflow_restores_latest_public_text_and_file_keeps_full_history(self):
        self.events.model({'type':'started','preflight':False,'model':'grok-test'},attempt=1,plan_id='b'*32)
        for _ in range(520):self.events.model({'type':'text','delta':'x'},attempt=1,plan_id='b'*32)
        old=self.events.after(self.key+':1');self.assertTrue(old['reset'])
        self.assertEqual(old['model_output']['text'],'x'*520)
        self.assertEqual(len((self.home/'.evoarm/rsi/learning_sessions'/self.key/'events.jsonl').read_text().splitlines()),521)

    def test_process_log_failure_is_visible_and_does_not_throw_into_motor_execution(self):
        with patch.object(Path,'mkdir',side_effect=OSError('disk unavailable')):
            self.events.emit('motion',phase='close')
        self.assertIn('磁盘',self.events.after()['persistence_error'])
        self.assertEqual(self.events.after()['events'][0]['phase'],'close')

    def test_sse_endpoint_replays_then_sends_new_event_and_never_reads_robot(self):
        provider=Mock(side_effect=AssertionError('events must not touch hardware'))
        app=FastAPI();install(app,provider)
        routes=[child for r in app.routes for child in (r.original_router.routes if hasattr(r,'original_router') else [r])]
        endpoint=next(r.endpoint for r in routes if getattr(r,'path',None)=='/api/bench/rsi/learning/events')
        class Request:
            headers={};query_params={}
            async def is_disconnected(self):return False
        async def check():
            response=await endpoint(Request())
            self.assertIn('text/event-stream',response.headers['content-type'])
            self.assertEqual(response.headers['x-accel-buffering'],'no')
            first=await anext(response.body_iterator)
            self.assertIn('event: snapshot',first)
            await response.body_iterator.aclose()
            stream=stream_events(Request(),self.events)
            await anext(stream);self.events.emit('motion',phase='close')
            second=await anext(stream);self.assertIn('event: rsi',second);self.assertIn('"close"',second)
            await stream.aclose()
        asyncio.run(check());provider.assert_not_called()
