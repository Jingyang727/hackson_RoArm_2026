"""Read-only public RSI event stream; never queries or commands hardware."""
from __future__ import annotations

import copy
from collections import deque
import json
from pathlib import Path
import threading
import time


class LearningEvents:
    def __init__(self):
        self.lock=threading.RLock();self.reset(None)

    def reset(self,session_id):
        with self.lock:
            self.session_id=session_id;self.sequence=0;self.rows=deque(maxlen=512);self.model_output=None
            self.started_at_unix=time.time();self.persistence_error=None

    def emit(self,kind,**data):
        with self.lock:
            if not self.session_id:return
            self.sequence+=1
            row={'id':f'{self.session_id}:{self.sequence}','sequence':self.sequence,
                 'session_id':self.session_id,'kind':kind,'at_unix':time.time(),**copy.deepcopy(data)}
            folder=Path.home()/'.evoarm/rsi/learning_sessions'/self.session_id
            try:
                folder.mkdir(parents=True,exist_ok=True)
                with (folder/'events.jsonl').open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            except OSError:self.persistence_error='过程日志未能写入磁盘；请检查本机存储。'
            self.rows.append(row)

    def model(self,update,*,attempt,plan_id):
        with self.lock:
            kind=update['type']
            if kind=='started':
                self.model_output={'plan_id':plan_id,'attempt':attempt,'preflight':update['preflight'],
                                   'model':update['model'],'status':'generating','text':'','mode':'connecting'}
            output=self.model_output
            if output is None or output['plan_id']!=plan_id:return
            if kind=='text':
                delta=update['delta'][:max(0,16000-len(output['text']))]
                if not delta:return
                output['text']+=delta
                self.emit('grok_delta',attempt=attempt,plan_id=plan_id,delta=delta)
                return
            if kind=='mode':output['mode']=update['mode']
            if kind in ('validated','failed','stopped'):
                output['status']=kind;output['verdict']=update.get('verdict');output['error']=update.get('error')
            self.emit('grok_'+kind,attempt=attempt,plan_id=plan_id,**{k:v for k,v in update.items() if k!='type'})

    def after(self,cursor=None):
        with self.lock:
            latest=f'{self.session_id or "idle"}:{self.sequence}'
            try:session,number=(cursor or '').rsplit(':',1);number=int(number)
            except (ValueError,AttributeError):session=None;number=-1
            oldest=self.rows[0]['sequence'] if self.rows else self.sequence+1
            if session==(self.session_id or 'idle') and oldest-1<=number<=self.sequence:
                return {'reset':False,'cursor':latest,'events':copy.deepcopy([row for row in self.rows if row['sequence']>number])}
            return {'reset':True,'cursor':latest,'session_id':self.session_id,'started_at_unix':self.started_at_unix,
                    'persistence_error':self.persistence_error,
                    'events':copy.deepcopy([row for row in self.rows if row['kind']!='grok_delta']),
                    'model_output':copy.deepcopy(self.model_output)}


def sse_packet(kind,data,cursor):
    return f'id: {cursor}\nevent: {kind}\ndata: {json.dumps(data,ensure_ascii=False,allow_nan=False)}\n\n'


async def stream_events(request,events):
    import asyncio
    cursor=request.headers.get('last-event-id') or request.query_params.get('after')
    if cursor and len(cursor)>128:cursor=None
    heartbeat=time.monotonic()
    while not await request.is_disconnected():
        batch=events.after(cursor)
        if batch['reset']:
            yield sse_packet('snapshot',batch,batch['cursor']);heartbeat=time.monotonic()
        else:
            for row in batch['events']:
                yield sse_packet('rsi',row,row['id']);heartbeat=time.monotonic()
        cursor=batch['cursor']
        if time.monotonic()-heartbeat>=12:
            yield ': keepalive\n\n';heartbeat=time.monotonic()
        await asyncio.sleep(.1)
