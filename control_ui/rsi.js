/* One explicitly started trial, then qualitative review and bounded next parameters. */
const $=id=>document.getElementById(id),prefix='/api/bench/rsi';
const declaration='已核对黄球、漏斗和完整抓放路径，现场有人看护且双手离开运动范围';
let state=null,busy=false,polling=false,revision=0,planReady=false,displayed=null,reviewed=null;
const stream={active:false,sequence:0,generation:0,controller:null,timer:null,url:null};
const phases={idle:'尚未开始。',observing:'正在采集当前RGB-D并计算完整路径…',ready:'计划已生成，请核对黄球、漏斗与整臂路径。',
  recording_warmup:'正在启动本轮10秒录像…',raise_current:'先从当前低位抬起…',open:'正在张爪…',align:'正在调整接近姿态…',
  approach_ball:'移动到黄球上方…',descend:'下降接近黄球…',close:'闭爪并核对反馈…',lift:'提起黄球…',
  transit_arc:'沿已检查的过渡路径运输…',transport:'移动到漏斗开口上方…',release:'松爪投放…',
  saving_recording:'正在保存录像、深度和关键帧…',stopping:'已停止后续指令，当前已发动作可能继续。'};
const outcomes={placed:'已复核：抓起并进入漏斗',grasp_missed:'抓空',pushed_away:'推走了球',slipped:'滑落',placement_missed:'投偏',bounced_out:'弹出',execution_failed:'执行未完成',unknown:'尚未确认'};
const number=v=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(1):'—';
const xyz=v=>Array.isArray(v)?v.map(number).join(', '):'—';
async function api(path,method='GET',body){
  const response=await fetch(path,{method,cache:'no-store',headers:{'Content-Type':'application/json'},...(body===undefined?{}:{body:JSON.stringify(body)})});
  const raw=await response.text();let value;try{value=JSON.parse(raw);}catch{throw Error(`HTTP ${response.status}：服务未返回JSON，请检查日志。`);}
  if(!response.ok)throw Error(`HTTP ${response.status}：${typeof value.detail==='string'?value.detail:value.detail?.message||'操作失败'}`);return value;
}
function showError(error){$('error').textContent=error.message||String(error);$('error').hidden=false;}
function stopStream(){stream.active=false;stream.generation++;stream.controller?.abort();clearTimeout(stream.timer);}
function showImage(url,kind){
  stopStream();planReady=false;displayed=url;
  $('graspFrame').onload=()=>{if(displayed===url){planReady=kind==='plan';render();}};
  $('graspFrame').onerror=()=>{planReady=false;$('imageCaption').textContent='图像未加载成功，请重新观察。';render();};
  $('graspFrame').src=url;$('graspFrame').hidden=false;$('graspEmpty').hidden=true;
  $('imageCaption').textContent=kind==='plan'?'本轮RGB-D识别与完整路径预览 · 非实时':kind==='diagnostic'?'本次观察画面 · 尚未执行动作':'本轮实测关键帧 · 根据画面复核抓起与投放结果';
  $('viewBadge').textContent=kind==='plan'?'计划预览':kind==='diagnostic'?'识别诊断':'实测关键帧';
}
function render(){
  const h=state?.health||{},j=state?.job||{},p=state?.plan,r=j.result;
  const running=j.status==='running',preparing=j.status==='preparing';
  const locked=Array.isArray(h.torque_status)&&h.torque_status.length===6&&h.torque_status.every(v=>v===1||v===true);
  $('setupBadge').textContent=state?.setup_id||'当前布置';$('connection').textContent=h.connected?'机械臂已连接':'机械臂未连接';
  $('connection').className='status-chip '+(h.connected&&!h.fault?'connected':'');
  $('motionBadge').textContent=running?'执行中':h.motion_enabled?'快速试验模式':'只读模式';
  $('connect').hidden=Boolean(h.connected);$('connect').disabled=busy||running;
  $('recoveryLink').hidden=!h.fault;
  $('readiness').textContent=h.fault?h.fault.message:!h.connected?'先连接机械臂，串口初始化可能触发回位，请在现场看护。':
    !locked?'六关节扭矩反馈尚未全部确认锁定。':running?'本轮完整抓放、录像与反馈同步记录。':'沿用已有标定；先观察预览，再执行一轮完整抓放。';
  $('prepare').disabled=busy||running||preparing||!h.connected||!locked||Boolean(h.fault)||Boolean(h.pending);
  $('prepare').textContent=r?'1 · 重新观察下一轮':'1 · 观察并规划抓放';
  const ready=p?.status==='prepared'&&p.seconds_left>0&&planReady&&displayed===p.preview_url;
  $('confirmed').disabled=busy||running||!ready;
  $('run').disabled=busy||running||!ready||!h.connected||!locked||!h.motion_enabled||Boolean(h.fault)||Boolean(h.pending)||!$('confirmed').checked;
  $('stop').hidden=!running;$('stop').disabled=j.phase==='stopping';
  $('phase').textContent=running||preparing?phases[j.phase]||j.phase:j.status==='failed'?'本轮未完成：'+(j.error||'请核对记录'):j.status==='completed'?'完整动作序列已执行，接下来复核实际结果。':phases[j.phase]||'尚未开始。';
  $('planExpiry').textContent=p?.status==='prepared'?`计划还可用 ${Math.ceil(p.seconds_left)} 秒；只执行一次。`:'';
  const warnings=p?.warnings||[];$('observationWarnings').hidden=!warnings.length;$('observationWarnings').textContent=warnings.join('\n');
  if(p){
    const ball=p.scene.ball,funnel=p.scene.funnel,depth=ball.depth_evidence;
    $('sceneFacts').textContent=`球心 XYZ：${xyz(ball.center_robot_mm)} mm\n${depth.depth_used?'已使用RGB-D球面拟合':'使用保存桌面与34mm球径估计'}；有效深度 ${depth.valid_samples} 个，拟合残差 ${number(depth.fit_residual_mm)} mm\n漏斗口 XYZ：${xyz(funnel.mouth_center_robot_mm)} mm；口径76mm，高45.4mm。`;
    $('planFacts').textContent=`参数版本 ${p.policy_version} · 抓取俯仰 ${number(p.pitch_deg)}°\n${p.steps.join(' → ')}`;
    const v=p.parameters;$('speedFacts').textContent=`XYZ运输 spd=${v.travel_speed}，接近 spd=${v.approach_speed}；夹爪 spd=${v.gripper_speed}步/秒，acc=${v.gripper_acc}。T104没有额外acc字段。`;
    $('depthPanel').hidden=false;if($('depthFrame').getAttribute('src')!==p.depth_url)$('depthFrame').src=p.depth_url;
  }
  $('runResult').hidden=!r;
  if(r){
    $('resultTitle').textContent=r.motion_completed?'完整抓放动作已执行':'完整抓放动作未完成';
    $('resultText').textContent=`闭爪尝试：${r.grasp_attempted?'已执行':'未执行'}；松爪投放：${r.placement_attempted?'已执行':'未执行'}。动作耗时 ${number(r.physical_elapsed_s)} 秒；10秒内：${r.within_10s===true?'是':r.within_10s===false?'否':'未知'}。视觉成功仍需复核。`;
    const available=Boolean(r.capture_trial_id);$('keyframesLink').hidden=!available;
    $('keyframesLink').href=`${prefix}/plans/${j.plan_id}/keyframes`;
    $('trialVideo').hidden=!available;
    if(available&&$('trialVideo').dataset.planId!==j.plan_id){$('trialVideo').src=`${prefix}/plans/${j.plan_id}/video`;$('trialVideo').dataset.planId=j.plan_id;}
  }
  $('reviewPanel').hidden=!r||running;
  $('policyVersion').textContent=p?`执行参数 v${p.policy_version}`:'';
  $('saveReview').disabled=busy||running||!r||Boolean(p?.review);
  $('grokContext').disabled=busy||running||!r;
  $('direction').disabled=!['grasp_missed','placement_missed'].includes($('outcome').value);
  if(p?.review){$('reviewResult').textContent=`${outcomes[p.review.outcome]} · ${p.review.source==='company_grok'?'Grok复核':'人工复核'}\n${p.review.changes.join('\n')}\n下一轮参数版本 v${p.review.after_version}，请重新观察。`;}
}
async function refresh(){
  if(polling||busy||document.hidden)return;polling=true;const rev=revision;
  try{
    const next=await api(prefix+'/state');if(rev!==revision)return;
    const previousPlan=state?.plan?.plan_id,wasRunning=state?.job?.status==='running';state=next;
    if(previousPlan!==state.plan?.plan_id){$('confirmed').checked=false;planReady=false;reviewed=null;$('reviewResult').textContent='';$('grokPrompt').hidden=true;$('outcome').value='unknown';$('direction').value='none';$('reviewNote').value='';}
    if(state.job.status==='running'){
      if(!stream.active&&state.job.phase!=='saving_recording')startStream();
    }else if(state.plan?.status==='prepared'&&displayed!==state.plan.preview_url){showImage(state.plan.preview_url,'plan');}
    else if(state.job.diagnostic_url&&displayed!==state.job.diagnostic_url){showImage(state.job.diagnostic_url,'diagnostic');}
    else if((wasRunning||!displayed)&&state.job.result?.keyframes_file){showImage(`${prefix}/plans/${state.job.plan_id}/keyframes`,'evidence');loadHistory();}
    if(state.job.status!=='running'&&stream.active)stopStream();
    render();
  }catch(error){showError(error);}finally{polling=false;}
}
async function action(fn){if(busy)return;busy=true;revision++;$('error').hidden=true;render();try{await fn();}catch(error){showError(error);}finally{busy=false;await refresh();render();}}
$('connect').onclick=()=>action(async()=>{await api('/api/auto-connect','POST',{});});
$('prepare').onclick=()=>action(async()=>{
  $('confirmed').checked=false;planReady=false;stopStream();displayed=null;$('graspFrame').hidden=true;$('graspEmpty').hidden=false;
  $('trialVideo').hidden=true;$('grokPrompt').hidden=true;await api(prefix+'/prepare','POST',{});
});
$('confirmed').onchange=render;
$('run').onclick=()=>{if($('run').disabled)return;const id=state.plan.plan_id;return action(async()=>{$('confirmed').checked=false;await api(prefix+'/run','POST',{plan_id:id,declaration});});};
$('stop').onclick=async()=>{if($('stop').disabled)return;$('stop').disabled=true;try{await api(prefix+'/stop','POST',{plan_id:state.job.plan_id});await refresh();}catch(error){showError(error);}};
$('outcome').onchange=()=>{if(!['grasp_missed','placement_missed'].includes($('outcome').value))$('direction').value='none';render();};
$('saveReview').onclick=()=>action(async()=>{
  const result=await api(prefix+'/review','POST',{plan_id:state.job.plan_id,outcome:$('outcome').value,direction:$('direction').value,source:'human',note:$('reviewNote').value});
  $('reviewResult').textContent=result.changes.join('\n');loadHistory();
});
$('grokContext').onclick=()=>action(async()=>{
  const data=await api(prefix+'/context/'+state.job.plan_id);
  $('grokPrompt').value='请使用公司Grok复核本轮本地录像和关键帧，并将定性结果POST到 http://127.0.0.1:8765/api/bench/rsi/review。不要执行下一轮运动。\n'+JSON.stringify(data,null,2);
  $('grokPrompt').hidden=false;
});
function startStream(){
  stopStream();stream.active=true;stream.sequence=0;planReady=false;displayed='live';
  $('graspFrame').onload=null;$('graspFrame').onerror=null;$('viewBadge').textContent='实时执行';
  nextFrame(stream.generation);
}
async function nextFrame(generation){
  if(!stream.active||generation!==stream.generation)return;stream.controller=new AbortController();let url;
  try{
    const response=await fetch('/api/bench/live-frame?after='+stream.sequence,{cache:'no-store',signal:stream.controller.signal});
    if(!response.ok)throw Error('实时预览暂不可用，动作与录像状态仍会更新。');
    const sequence=Number(response.headers.get('X-Frame-Sequence')),age=Number(response.headers.get('X-Frame-Age-Ms'));
    const width=Number(response.headers.get('X-Frame-Width')),height=Number(response.headers.get('X-Frame-Height'));
    if(!Number.isSafeInteger(sequence)||sequence<=stream.sequence||age<0||age>1500||!Number.isFinite(age)||width!==state.plan?.scene.image_size[0]||height!==state.plan?.scene.image_size[1])throw Error('实时帧信息不匹配。');
    url=URL.createObjectURL(await response.blob());const decoded=new Image();decoded.src=url;await decoded.decode();
    if(!stream.active||generation!==stream.generation){URL.revokeObjectURL(url);return;}
    if(decoded.naturalWidth!==width||decoded.naturalHeight!==height)throw Error('实时图像尺寸不匹配。');
    if(stream.url)URL.revokeObjectURL(stream.url);stream.url=url;url=null;$('graspFrame').src=stream.url;
    $('graspFrame').hidden=false;$('graspEmpty').hidden=true;$('streamNotice').hidden=true;
    $('imageCaption').textContent='实时执行画面 · 10秒RGB-D由相机服务独立录制';stream.sequence=sequence;
    stream.timer=setTimeout(()=>nextFrame(generation),100);
  }catch(error){if(url)URL.revokeObjectURL(url);if(!stream.active||generation!==stream.generation)return;stopStream();$('streamNotice').hidden=false;$('streamNotice').textContent=error.message;}
}
async function loadHistory(){
  try{const data=await api(prefix+'/history');$('trials').replaceChildren();$('count').textContent=data.trials.length+' 次';$('emptyTrials').hidden=Boolean(data.trials.length);
    for(const trial of data.trials){const row=document.createElement('article'),title=document.createElement('strong'),summary=document.createElement('p');row.className='trial-row';
      title.textContent=`试次 ${trial.id.slice(0,10)} · 参数 v${trial.policy_version}`;
      summary.textContent=`${trial.review?outcomes[trial.review.outcome]:'等待复核'}；动作 ${number(trial.result?.physical_elapsed_s)} 秒；状态 ${trial.status}`;
      row.append(title,summary);$('trials').append(row);}
  }catch(error){showError(error);}
}
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopStream();else refresh();});
window.addEventListener('pagehide',()=>{stopStream();if(stream.url)URL.revokeObjectURL(stream.url);});
setInterval(refresh,700);refresh();loadHistory();
