"""Direct company Grok HTTP client; reuse local provider settings, never run CLI."""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import queue
import re
import threading
import time
from urllib import request, error
from urllib.parse import urlparse


@dataclass(frozen=True)
class APISettings:
    base_url: str = field(repr=False)
    token: str = field(repr=False)
    model: str = 'grok-4.7-build'
    source: str = 'environment'
    headers: dict = field(default_factory=dict, repr=False)


def _jsonc(text):
    """Remove JSON comments/trailing commas without changing quoted URLs/keys."""
    chars=list(text); quoted=False; escaped=False; index=0
    while index<len(chars):
        char=chars[index]
        if quoted:
            if escaped:escaped=False
            elif char=='\\':escaped=True
            elif char=='"':quoted=False
            index+=1;continue
        if char=='"':quoted=True;index+=1;continue
        if char=='/' and index+1<len(chars) and chars[index+1] in ('/','*'):
            block=chars[index+1]=='*';chars[index]=chars[index+1]=' ';index+=2
            while index<len(chars):
                if block and index+1<len(chars) and chars[index:index+2]==['*','/']:
                    chars[index]=chars[index+1]=' ';index+=2;break
                if not block and chars[index]=='\n':break
                if chars[index]!='\n':chars[index]=' '
                index+=1
            continue
        index+=1
    quoted=False;escaped=False
    for index,char in enumerate(chars):
        if quoted:
            if escaped:escaped=False
            elif char=='\\':escaped=True
            elif char=='"':quoted=False
        elif char=='"':quoted=True
        elif char==',':
            next_index=index+1
            while next_index<len(chars) and chars[next_index].isspace():next_index+=1
            if next_index<len(chars) and chars[next_index] in '}]':chars[index]=' '
    value=json.loads(''.join(chars))
    if not isinstance(value,dict):raise ValueError('公司模型配置需为JSON对象')
    return value


def _merge(base, value):
    for key,item in value.items():
        if isinstance(item,dict) and isinstance(base.get(key),dict):_merge(base[key],item)
        else:base[key]=item


def _resolve(value):
    if not isinstance(value,str):return ''
    env=re.fullmatch(r'\{env:([A-Za-z_][A-Za-z0-9_]*)\}',value)
    if env:return os.getenv(env[1],'').strip()
    if '{env:' in value:
        return re.sub(r'\{env:([A-Za-z_][A-Za-z0-9_]*)\}',lambda m:os.getenv(m[1],''),value).strip()
    file=re.fullmatch(r'\{file:(.+)\}',value)
    if file:
        path=Path(file[1]).expanduser()
        if path.stat().st_size>65536:raise ValueError('公司API凭据文件过大')
        return path.read_text().strip()
    return value.strip()


def _validate(settings):
    if not all(isinstance(value,str) for value in (settings.base_url,settings.token,settings.model)) or not isinstance(settings.headers,dict):
        raise ValueError('公司API配置字段格式无效')
    url=urlparse(settings.base_url)
    if url.scheme!='https' or not url.hostname or url.username or url.password or url.query or url.fragment:
        raise ValueError('公司API地址需为HTTPS，且不在URL中包含凭据')
    if not settings.token and not any(k.lower() in ('authorization','x-api-key','api-key') and bool(v) for k,v in settings.headers.items()):
        raise ValueError('未找到公司Grok API凭据；请配置本机环境或已有provider凭据，不要贴到聊天')
    if not settings.model or 'grok' not in settings.model.lower():
        raise ValueError('请选择已批准的公司Grok模型')
    for key,value in settings.headers.items():
        if not re.fullmatch(r'[A-Za-z0-9-]+',key) or not isinstance(value,str) or '\n' in value or '\r' in value:
            raise ValueError('公司API请求头配置无效')
    if '\n' in settings.token or '\r' in settings.token:raise ValueError('公司API凭据格式无效')
    return settings


def settings():
    selected=os.getenv('EVOARM_GROK_MODEL',os.getenv('EVOARM_OPENCODE_MODEL','bottlerocket/grok-4.7-build'))
    provider,model=selected.split('/',1) if '/' in selected else ('bottlerocket',selected)
    explicit=os.getenv('EVOARM_COMPANY_LLM_BASE_URL','').strip()
    if explicit:
        return _validate(APISettings(explicit.rstrip('/'),os.getenv('EVOARM_COMPANY_LLM_TOKEN','').strip(),
                                    os.getenv('EVOARM_COMPANY_LLM_MODEL',model),'environment'))
    home=Path.home();config=Path(os.getenv('XDG_CONFIG_HOME',str(home/'.config')))/'opencode'
    # Auto-discovery is limited to user-owned global settings. A repository
    # config must not redirect a globally stored company credential elsewhere.
    # A user can explicitly select another trusted file with OPENCODE_CONFIG.
    files=[config/'config.json',config/'opencode.json',config/'opencode.jsonc']
    if os.getenv('OPENCODE_CONFIG'):files.append(Path(os.environ['OPENCODE_CONFIG']).expanduser())
    merged={}
    for file in files:
        if file.is_file():_merge(merged,_jsonc(file.read_text()))
    if os.getenv('OPENCODE_CONFIG_CONTENT'):_merge(merged,_jsonc(os.environ['OPENCODE_CONFIG_CONTENT']))
    item=merged.get('provider',{}).get(provider,{})
    options=item.get('options',{})
    base=_resolve(options.get('baseURL') or options.get('baseUrl'))
    if not base:raise ValueError('未找到公司API地址；可复用provider的baseURL，或设置 EVOARM_COMPANY_LLM_BASE_URL')
    npm=item.get('npm','@ai-sdk/openai-compatible')
    if npm not in ('@ai-sdk/openai-compatible','@ai-sdk/openai','@ai-sdk/xai'):
        raise ValueError('现有provider不是已知Chat Completions协议；请显式配置公司Grok API地址与凭据')
    token=_resolve(options.get('apiKey'))
    if not token:
        data=Path(os.getenv('XDG_DATA_HOME',str(home/'.local/share')))/'opencode/auth.json'
        alternatives=[data,home/'Library/Application Support/opencode/auth.json']
        for file in alternatives:
            if not file.is_file():continue
            auth=json.loads(file.read_text()).get(provider,{})
            if auth.get('type')=='oauth' and auth.get('expires'):
                try:expires=float(auth['expires'])
                except (ValueError,TypeError):raise ValueError('公司API登录过期时间格式无效') from None
                expires=expires/1000 if expires>100_000_000_000 else expires
                if expires<time.time():raise ValueError('公司API登录凭据已过期，请在本机更新登录')
            token=_resolve(auth.get('key') or auth.get('apiKey') or auth.get('access') or auth.get('accessToken'))
            if token:break
    headers={k:_resolve(v) for k,v in options.get('headers',{}).items()
             if k.lower() not in ('host','content-length','connection','proxy-authorization')}
    canonical=item.get('models',{}).get(model,{}).get('id',model)
    return _validate(APISettings(base.rstrip('/'),token,canonical,'local_provider_settings',headers))


def configuration():
    try:
        cfg=settings()
        return {'backend':'company_api','configured':True,'model':cfg.model,'reason':None,'settings_source':cfg.source}
    except (OSError,ValueError,TypeError,KeyError,AttributeError):
        # Do not expose paths, provider configuration values or credentials.
        return {'backend':'company_api','configured':False,'model':'grok-4.7-build',
                'reason':'公司Grok API未配置完整；请只读运行 python -m evoarm_rsi.company_api 检查',
                'settings_source':None}


class _NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self,req,fp,code,msg,headers,newurl):
        raise error.HTTPError(req.full_url,code,'company API redirects are disabled',headers,fp)


def _http_content(cfg,system,content,timeout_s):
    endpoint=cfg.base_url if cfg.base_url.endswith('/chat/completions') else cfg.base_url+'/chat/completions'
    headers={'Content-Type':'application/json',**cfg.headers}
    if cfg.token and not any(k.lower() in ('authorization','x-api-key','api-key') for k in headers):headers['Authorization']='Bearer '+cfg.token
    body={'model':cfg.model,'stream':False,'messages':[{'role':'system','content':system},{'role':'user','content':content}]}
    req=request.Request(endpoint,data=json.dumps(body,ensure_ascii=False).encode(),headers=headers,method='POST')
    try:
        with request.build_opener(_NoRedirect()).open(req,timeout=timeout_s) as response:
            raw=response.read(2_000_001)
    except error.HTTPError as exc:
        code=exc.code;exc.close()
        raise RuntimeError(f'公司Grok API HTTP {code}；未自动重试') from None
    except (OSError,error.URLError):raise RuntimeError('公司Grok API连接失败；未自动重试') from None
    if len(raw)>2_000_000:raise ValueError('公司Grok API回复过大')
    try:value=json.loads(raw)['choices'][0]['message']['content']
    except (ValueError,KeyError,TypeError,IndexError):raise ValueError('公司Grok API未返回最终文本') from None
    if isinstance(value,list):value=''.join(p.get('text','') for p in value if isinstance(p,dict) and p.get('type')=='text')
    if not isinstance(value,str):raise ValueError('公司Grok API最终回复不是文本')
    return value


def complete_content(system,content,stop,*,timeout_s=120):
    if stop.is_set():raise RuntimeError('用户已停止，不发送新的模型请求')
    cfg=settings();responses=queue.Queue(maxsize=1)
    def perform():
        try:responses.put((True,_http_content(cfg,system,content,timeout_s)))
        except Exception as exc:responses.put((False,exc))
    # Network work has no motor access or persistence. Cancellation discards
    # its eventual result; it cannot restart a stopped learning task.
    threading.Thread(target=perform,name='evoarm-company-grok-http',daemon=True).start()
    until=time.monotonic()+timeout_s
    while True:
        if stop.is_set():raise RuntimeError('用户停止了模型分析；迟到回复不会触发动作')
        remaining=until-time.monotonic()
        if remaining<=0:raise RuntimeError('公司Grok API分析超时；未自动重试')
        try:ok,value=responses.get(timeout=min(.1,remaining))
        except queue.Empty:continue
        if stop.is_set():raise RuntimeError('用户停止了模型分析')
        if not ok:raise value
        return value


if __name__=='__main__':
    info=configuration()
    if not info['configured']:
        try:settings()
        except (OSError,ValueError,TypeError,KeyError,AttributeError) as exc:
            # These are fixed validation messages, never the underlying file content.
            info['reason']=str(exc) if isinstance(exc,ValueError) and not isinstance(exc,json.JSONDecodeError) else '本机公司API配置无法读取或解析'
    print(json.dumps(info,ensure_ascii=False,indent=2))
