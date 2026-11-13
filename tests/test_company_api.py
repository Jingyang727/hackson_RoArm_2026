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
