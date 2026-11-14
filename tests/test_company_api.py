"""Direct company HTTP protocol and local credential discovery, no network."""
import io
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import MagicMock,patch
from urllib import request,error

from evoarm_rsi import company_api as api


class CompanyAPI(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        for item in (patch.object(Path,'home',return_value=self.home),patch.dict(os.environ,{},clear=True)):
            item.start();self.addCleanup(item.stop)

    def explicit(self):
        os.environ.update(EVOARM_COMPANY_LLM_BASE_URL='https://company.invalid/v1',
                          EVOARM_COMPANY_LLM_TOKEN='fake-test-token',EVOARM_COMPANY_LLM_MODEL='grok-4.7-build')

    def company_env(self,content):
        path=self.home/'.evoarm/company_llm.env';path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(content);return path

    def test_user_company_env_file_is_read_as_literals_without_shell_or_environment_mutation(self):
        file=self.company_env('export OPENAI_BASE_URL="https://inference.tesla.com/v1"\nOPENAI_API_KEY=\'fake-$(`never-execute`)\'\nOPENAI_MODEL=grok-4.7-build\n')
        before=file.read_bytes();environment=dict(os.environ)
        with patch('subprocess.Popen',side_effect=AssertionError('no shell')):cfg=api.settings()
        self.assertEqual(cfg.base_url,'https://inference.tesla.com/v1');self.assertEqual(cfg.token,'fake-$(`never-execute`)')
        self.assertEqual(cfg.source,'company_env_file');self.assertEqual(dict(os.environ),environment)
        self.assertEqual(file.read_bytes(),before);self.assertNotIn(cfg.token,json.dumps(api.configuration()))

    def test_complete_explicit_configuration_takes_priority_even_over_invalid_saved_file(self):
        self.company_env('not a valid assignment');self.explicit()
        self.assertEqual(api.settings().source,'environment')

    def test_env_base_can_reuse_user_auth_copy_and_auth_only_uses_known_company_endpoint(self):
        self.provider({},auth={'type':'api','key':'fake-auth-key'})
        self.company_env('OPENAI_BASE_URL=https://inference.tesla.com/v1\n')
        self.assertEqual(api.settings().token,'fake-auth-key');self.assertEqual(api.settings().source,'company_env_file')
        (self.home/'.evoarm/company_llm.env').unlink()
        self.assertEqual(api.settings().base_url,'https://inference.tesla.com/v1')
        self.assertEqual(api.settings().source,'company_auth_file')

    def test_partial_custom_endpoint_does_not_receive_another_endpoint_env_token(self):
        self.company_env('OPENAI_API_KEY=fake-company-key\n')
        os.environ['EVOARM_COMPANY_LLM_BASE_URL']='https://different.invalid/v1'
        self.assertFalse(api.configuration()['configured'])

    def test_invalid_saved_file_error_does_not_expose_content(self):
        self.company_env('fake-private-data-without-assignment')
        result=api.configuration();self.assertFalse(result['configured'])
        self.assertNotIn('fake-private-data',json.dumps(result))

    def provider(self,options,auth=None):
        folder=self.home/'.config/opencode';folder.mkdir(parents=True,exist_ok=True)
        cfg={'provider':{'bottlerocket':{'npm':'@ai-sdk/openai-compatible','options':options,
                                       'models':{'grok-4.7-build':{'id':'grok-4.7-build'}}}}}
        # Existing JSONC configuration is read only; no opencode executable needed.
        file=folder/'opencode.jsonc';file.write_text('// company settings\n'+json.dumps(cfg))
        if auth:
            dest=self.home/'.local/share/opencode/auth.json';dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_text(json.dumps({'bottlerocket':auth}))
        return file

    def test_explicit_api_configuration_does_not_expose_credentials(self):
        self.explicit();cfg=api.settings();public=api.configuration()
        self.assertTrue(public['configured']);self.assertEqual(public['backend'],'company_api')
        self.assertNotIn('fake-test-token',repr(cfg));self.assertNotIn('fake-test-token',json.dumps(public))
        self.assertNotIn('company.invalid',repr(cfg))

    def test_reuse_existing_provider_env_key_without_modifying_files(self):
        os.environ['EXAMPLE_COMPANY_KEY']='fake-profile-key'
        file=self.provider({'baseURL':'https://company.invalid/v1','apiKey':'{env:EXAMPLE_COMPANY_KEY}'})
        before=file.read_bytes()
        with patch('subprocess.Popen',side_effect=AssertionError('no process')):cfg=api.settings()
        self.assertEqual(cfg.token,'fake-profile-key');self.assertEqual(cfg.source,'local_provider_settings')
        self.assertEqual(file.read_bytes(),before)

    def test_reuse_existing_auth_file_and_custom_auth_headers(self):
        self.provider({'baseURL':'https://company.invalid/v1'},auth={'type':'api','key':'fake-auth-key'})
        self.assertEqual(api.settings().token,'fake-auth-key')
        self.provider({'baseURL':'https://company.invalid/v1','headers':{'x-api-key':'fake-header-key'}})
        self.assertEqual(api.settings().headers['x-api-key'],'fake-header-key')

    def test_jsonc_parser_preserves_url_and_string_comma_braces(self):
        parsed=api._jsonc('{/*comment*/"url":"https://example.invalid/a//b", "text":"literal ,}",//line\n"list":[1,2,],}')
        self.assertEqual(parsed,{'url':'https://example.invalid/a//b','text':'literal ,}','list':[1,2]})

    def test_direct_request_is_multimodal_and_has_no_tools_or_reasoning_input(self):
        self.explicit();opener=MagicMock();response=opener.open.return_value.__enter__.return_value
        response.read.return_value=json.dumps({'choices':[{'message':{'content':'{"ready":true,"note":"visible"}',
                                                        'reasoning_content':'private reasoning must stay unused'}}]}).encode()
        image={'type':'image_url','image_url':{'url':'data:image/jpeg;base64,AAAA'}}
        with patch.object(api.request,'build_opener',return_value=opener):
            final=api.complete_content('scene check',[image],threading.Event())
        req=opener.open.call_args.args[0];body=json.loads(req.data)
        self.assertEqual(req.full_url,'https://company.invalid/v1/chat/completions')
        self.assertEqual(req.get_header('Authorization'),'Bearer fake-test-token')
        self.assertEqual(body['model'],'grok-4.7-build');self.assertFalse(body['stream'])
        self.assertEqual(body['messages'][1]['content'],[image]);self.assertNotIn('tools',body)
        self.assertEqual(json.loads(final)['ready'],True);self.assertNotIn('private reasoning',final)

    def test_http_error_is_not_retried_and_does_not_expose_response_or_token(self):
        self.explicit();opener=MagicMock()
        opener.open.side_effect=error.HTTPError('https://company.invalid/v1',401,'denied',{},io.BytesIO(b'secret provider body'))
        with patch.object(api.request,'build_opener',return_value=opener),self.assertRaises(RuntimeError) as caught:
            api.complete_content('review',[],threading.Event())
        self.assertIn('HTTP 401',str(caught.exception));self.assertNotIn('secret',str(caught.exception))
        self.assertNotIn('fake-test-token',str(caught.exception));opener.open.assert_called_once()

    def sse(self,packets):
        data=b''.join(b'data: '+(p.encode() if isinstance(p,str) else json.dumps(p,ensure_ascii=False).encode())+b'\n\n' for p in packets)
        opener=MagicMock();response=opener.open.return_value.__enter__.return_value
        response.headers={'Content-Type':'text/event-stream; charset=utf-8'}
        response.readline.side_effect=io.BytesIO(data).readline
        return opener

    def test_stream_sends_public_unicode_content_before_completion_and_ignores_reasoning(self):
        self.explicit();pieces=[];modes=[]
        opener=self.sse([{'choices':[{'delta':{'reasoning_content':'private must not appear','content':'球仍在'}}]},
                         {'choices':[{'delta':{'content':'桌面。'}}]},
                         {'choices':[{'delta':{},'finish_reason':'stop'}]},'[DONE]'])
        with patch.object(api.request,'build_opener',return_value=opener):
            final=api.complete_content('review',[],threading.Event(),on_delta=pieces.append,on_mode=modes.append)
        self.assertEqual(pieces,['球仍在','桌面。']);self.assertEqual(final,'球仍在桌面。')
        self.assertEqual(modes,['streaming']);self.assertNotIn('private',''.join(pieces))
        self.assertTrue(json.loads(opener.open.call_args.args[0].data)['stream'])

    def test_truncated_stream_is_not_a_final_result_even_when_prefix_looks_complete(self):
        self.explicit();pieces=[]
        opener=self.sse([{'choices':[{'delta':{'content':'{"ready":true,"note":"visible"}'}}]}])
        with patch.object(api.request,'build_opener',return_value=opener),self.assertRaises(ValueError):
            api.complete_content('scene',[],threading.Event(),on_delta=pieces.append)
        self.assertEqual(len(pieces),1)

    def test_first_delta_reaches_caller_while_http_stream_is_still_open(self):
        self.explicit();first=threading.Event();release=threading.Event();result=[];pieces=[]
        packets=[{'choices':[{'delta':{'content':'正在观察'}}]},
                 {'choices':[{'delta':{'content':'，球未离桌。'},'finish_reason':'stop'}]},'[DONE]']
        opener=self.sse(packets);response=opener.open.return_value.__enter__.return_value
        read=response.readline.side_effect;lines=0
        def gated(limit):
            nonlocal lines
            if lines==2:release.wait(2)
            lines+=1;return read(limit)
        response.readline.side_effect=gated
        def observe(text):pieces.append(text);first.set()
        def worker():
            try:result.append(api.complete_content('review',[],threading.Event(),on_delta=observe))
            except Exception as exc:result.append(exc)
        with patch.object(api.request,'build_opener',return_value=opener):
            thread=threading.Thread(target=worker);thread.start()
            try:
                self.assertTrue(first.wait(1));self.assertTrue(thread.is_alive());self.assertEqual(result,[])
                self.assertEqual(pieces,['正在观察'])
            finally:release.set();thread.join(2)
        self.assertEqual(result,['正在观察，球未离桌。'])

    def test_length_limited_or_malformed_stream_never_returns_a_completed_reply(self):
        self.explicit()
        for packet in ({'choices':[{'delta':{'content':'partial'},'finish_reason':'length'}]},'not json'):
            opener=self.sse([packet,'[DONE]'])
            with patch.object(api.request,'build_opener',return_value=opener),self.assertRaises(ValueError):
                api.complete_content('review',[],threading.Event(),on_delta=lambda text:None)

    def test_explicit_unsupported_stream_falls_back_once_but_auth_failure_does_not(self):
        self.explicit();modes=[];opener=MagicMock();response=MagicMock()
        response.headers={'Content-Type':'application/json'}
        response.read.return_value=b'{"choices":[{"message":{"content":"final buffered reply"}}]}'
        opener.open.side_effect=[error.HTTPError('https://company.invalid/v1',400,'invalid',{},io.BytesIO(b'{"error":"stream is not supported"}')),
                                MagicMock(__enter__=MagicMock(return_value=response))]
        with patch.object(api.request,'build_opener',return_value=opener):
            final=api.complete_content('review',[],threading.Event(),on_delta=lambda text:None,on_mode=modes.append)
        self.assertEqual(final,'final buffered reply');self.assertEqual(opener.open.call_count,2)
        self.assertEqual(modes,['buffered_fallback'])
        bodies=[json.loads(call.args[0].data) for call in opener.open.call_args_list]
        self.assertEqual([b['stream'] for b in bodies],[True,False])
        opener.open.reset_mock();opener.open.side_effect=error.HTTPError('https://company.invalid/v1',401,'denied',{},io.BytesIO(b'secret stream auth'))
        with patch.object(api.request,'build_opener',return_value=opener),self.assertRaises(RuntimeError):
            api.complete_content('review',[],threading.Event(),on_delta=lambda text:None)
        opener.open.assert_called_once()

    def test_provider_buffered_response_is_labeled_and_has_no_fake_deltas(self):
        self.explicit();pieces=[];modes=[];opener=MagicMock()
        response=opener.open.return_value.__enter__.return_value;response.headers={'Content-Type':'application/json'}
        response.read.return_value=b'{"choices":[{"message":{"content":"complete"}}]}'
        with patch.object(api.request,'build_opener',return_value=opener):
            self.assertEqual(api.complete_content('review',[],threading.Event(),on_delta=pieces.append,on_mode=modes.append),'complete')
        self.assertEqual(pieces,[]);self.assertEqual(modes,['buffered'])

    def test_cancelled_stream_does_not_publish_late_chunks_or_return_a_result(self):
        self.explicit();stop=threading.Event();release=threading.Event();pieces=[]
        def slow(*args,**options):
            options['on_delta']('first');release.wait(2);options['on_delta']('late');return 'late result'
        def observe(text):pieces.append(text);stop.set()
        with patch.object(api,'_http_content',side_effect=slow),self.assertRaises(RuntimeError):
            try:api.complete_content('review',[],stop,on_delta=observe)
            finally:release.set()
        self.assertEqual(pieces,['first'])

    def test_credentials_are_never_redirected_to_another_host(self):
        req=request.Request('https://company.invalid/v1/chat/completions',headers={'Authorization':'Bearer fake-key'})
        with self.assertRaises(error.HTTPError):
            api._NoRedirect().redirect_request(req,io.BytesIO(),302,'redirect',{},'https://other.invalid')

    def test_cancel_discards_late_api_response(self):
        self.explicit();stop=threading.Event();started=threading.Event();release=threading.Event();result=[]
        def slow(*args):started.set();release.wait(3);return 'late result'
        def worker():
            try:result.append(api.complete_content('review',[],stop))
            except RuntimeError as exc:result.append(str(exc))
        with patch.object(api,'_http_content',side_effect=slow):
            thread=threading.Thread(target=worker);thread.start();self.assertTrue(started.wait(1))
            stop.set();thread.join(1);release.set();self.assertFalse(thread.is_alive())
        self.assertEqual(len(result),1);self.assertIn('用户停止',result[0]);self.assertNotEqual(result[0],'late result')

    def test_missing_or_invalid_configuration_never_chooses_a_public_fallback(self):
        self.assertFalse(api.configuration()['configured'])
        self.explicit();os.environ['EVOARM_COMPANY_LLM_BASE_URL']='http://company.invalid'
        self.assertFalse(api.configuration()['configured'])
        os.environ['EVOARM_COMPANY_LLM_BASE_URL']='https://token@company.invalid'
        self.assertFalse(api.configuration()['configured'])
