/* The controller owns actions; Grok reviews, the browser displays progress. */
const $=id=>document.getElementById(id),prefix='/api/bench/rsi';
const declaration='现场已清空，允许自动抓放、失败后恢复官方姿态并连续重试';
const outcomes={placed:'已抓起并进入漏斗',grasp_missed:'没夹住',pushed_away:'球被推走',slipped:'球滑落',placement_missed:'投放偏了',bounced_out:'球弹出了漏斗',execution_failed:'动作未完成',unknown:'画面不足以判断'};
const phases={idle:'尚未开始。',starting:'检查当前状态…',observing:'观察黄球与漏斗，规划本轮路径…',scene_check:'Grok正在确认当前球与漏斗…',
  analyzing:'Grok正在分析本轮10秒录像…',learning:'根据Grok判断更新策略…',recovering:'正在恢复官方初始姿态…',replanning:'已复位，重新定位并规划…',reset_completed:'已恢复官方初始姿态。',
  completed:'本轮成功，策略已保存。',exhausted:'本次尝试次数已用完，策略和证据已保存。',stopping:'正在停止；不会继续恢复或重试。',stopped:'自动任务已停止。',
  recording_warmup:'启动本轮录像…',raise_current:'抬离桌面…',open:'张爪…',align:'调整接近姿态…',approach_ball:'移动到球上方，同时张爪…',
  descend:'下降接近黄球…',close:'合抓…',lift:'提升…',transit_arc:'经过过渡点…',transport:'移动到漏斗上方…',release:'释放小球…',saving_recording:'保存本轮录像与反馈…'};
let state=null,busy=false,polling=false,revision=0,historyKey=null;
const stream={active:false,sequence:0,generation:0,controller:null,timer:null,url:null};
const number=v=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(1):'—';
const xyz=v=>Array.isArray(v)?v.map(number).join(', '):'—';
const parameterNames={pickup_xy_mm:'抓取 XY 补偿（mm）',release_xy_mm:'投放 XY 补偿（mm）',grasp_z_offset_mm:'抓取高度补偿（mm）',finger_offset_mm:'指尖偏移（mm）',squeeze_deg:'预紧角度（°）',travel_speed:'移动速度',release_clearance_mm:'投放间隙（mm）'};
function targets(plan){
  const overlay=$('targetOverlay'),caption=$('targetCaption');overlay.replaceChildren();overlay.toggleAttribute('hidden',!plan);caption.hidden=!plan;
  if(!plan)return;
  const [width,height]=plan.scene.image_size;overlay.setAttribute('viewBox',`0 0 ${width} ${height}`);
  const add=(type,attrs,text)=>{const node=document.createElementNS('http://www.w3.org/2000/svg',type);for(const [key,value] of Object.entries(attrs))node.setAttribute(key,String(value));if(text)node.textContent=text;overlay.append(node);};
  for(const [point,label,color] of [[plan.scene.ball.pixel_center,'黄球','#f4ca24'],[plan.scene.funnel.pixel_center,'漏斗口','#48dfb8']]){
    const [x,y]=point;add('circle',{cx:x,cy:y,r:width*.022,fill:'none',stroke:color,'stroke-width':3});
    add('text',{x:Math.max(8,Math.min(width-95,x+width*.026)),y:Math.max(28,y-10),fill:color,'font-size':width*.02,stroke:'#152824','stroke-width':3,'paint-order':'stroke'},label);
  }
  const side=plan.scene.funnel.pixel_center[0]<plan.scene.ball.pixel_center[0]?'左':'右';
  caption.textContent=`本轮识别：漏斗在黄球的画面${side}侧。圆圈是观察时的位置，并非实时跟踪；画面未镜像，左右不等于机械臂 XYZ 正负。`;
}
async function api(path,method='GET',body){
  const response=await fetch(path,{method,cache:'no-store',headers:{'Content-Type':'application/json'},...(body===undefined?{}:{body:JSON.stringify(body)})});
  const raw=await response.text();let value;try{value=JSON.parse(raw);}catch{throw Error(`HTTP ${response.status}：服务未返回JSON，请检查日志。`);}
  if(!response.ok)throw Error(`HTTP ${response.status}：${typeof value.detail==='string'?value.detail:value.detail?.message||'操作失败'}`);
  return value;
}
function showError(error){$('error').textContent=error.message||String(error);$('error').hidden=false;}
function render(){
  const h=state?.health||{},a=state?.learning||{status:'idle'},j=state?.job||{},p=state?.plan,r=j.result;
  const running=a.status==='running'||j.status==='running',locked=Array.isArray(h.torque_status)&&h.torque_status.length===6&&h.torque_status.every(v=>v===1||v===true);
  $('setupBadge').textContent=state?.setup_id||'当前布置';$('connection').textContent=h.connected?'机械臂已连接':'机械臂未连接';
  $('connection').className='status-chip '+(h.connected?'connected':'');
  $('connect').hidden=Boolean(h.connected);$('connect').disabled=busy||running;
  $('readiness').textContent=!h.connected?'先连接机械臂，现场看护串口初始化。':!locked?'六关节扭矩反馈尚未全部确认锁定。':h.fault?'启动后先确认新鲜静止反馈，直接恢复官方姿态。':'沿用已有标定。发生规划或动作异常时自动恢复官方姿态。';
  const grok=state?.grok||{};$('grokStatus').textContent=grok.configured?'公司Grok API已配置 · 直接调用':grok.reason||'正在检查公司Grok API配置。';
  const compatible=state?.api_version===4;
  if(state&&!compatible)$('grokStatus').textContent='网页服务仍是旧版，请更新并重启服务后再开始。';
  $('start').disabled=busy||running||!compatible||!h.connected||!locked||!h.motion_enabled||Boolean(h.pending)||!grok.configured||!$('confirmed').checked;
  $('reset').hidden=running;$('reset').disabled=busy||running||!h.connected||!locked||!h.motion_enabled||!$('confirmed').checked;
  $('confirmed').disabled=running||busy;$('maxAttempts').disabled=running||busy;
  $('stop').hidden=!running;$('stop').disabled=a.phase==='stopping'||j.phase==='stopping';
  $('attemptBadge').textContent=a.attempt?`第 ${a.attempt} / ${a.max_attempts} 轮`:'尚未开始';
  const phase=a.status==='running'?(a.phase==='executing'?j.phase:a.phase):j.status==='running'?j.phase:a.phase;
  $('phase').textContent=a.status==='failed'?(a.error?.startsWith('已恢复官方初始姿态')?a.error:'任务已停止：'+(a.error||'请查看本轮记录')):phases[phase]||phases[j.phase]||'尚未开始。';
  $('nextChange').textContent=(a.changes||[]).join('\n');
  $('sessionBackupLink').hidden=!a.session_backup_file;
  if(a.session_backup_file)$('sessionBackupLink').href=`${prefix}/sessions/${a.session_id}/archive`;
  const review=p?.review||a.review;$('grokVerdict').hidden=!review;
  if(review)$('grokVerdict').textContent=`${review.source==='human'?'既有人工记录':'Grok判断'}：${outcomes[review.outcome]||review.outcome}\n${review.note||''}`;
  const proposal=review?.proposal;$('changeDetails').hidden=!proposal;
  if(proposal){
    const changes=(proposal.parameters_changed||[]).map(key=>`${parameterNames[key]||key}：${JSON.stringify(proposal.before[key])} → ${JSON.stringify(proposal.after[key])}`);
    if(proposal.robot_delta_mm)changes.push(`本次机械臂 XY 增量：${xyz(proposal.robot_delta_mm)} mm；经标定换算，不能按画面左右直接解读正负号。`);
    if(proposal.predicted_pixel_delta)changes.push(`相机模型预测的像素增量：${xyz(proposal.predicted_pixel_delta)} px；实际改善待下一轮录像验证。`);
    $('changeFacts').textContent=changes.join('\n')||'没有应用新的参数。';
  }
  targets(p);
  $('recordings').hidden=!p;
  if(p){
    const ball=p.scene.ball,funnel=p.scene.funnel,depth=ball.depth_evidence;
    $('policyVersion').textContent=`策略 v${p.policy_version}`;
    $('sceneFacts').textContent=`原始画面像素：黄球 (${xyz(ball.pixel_center)})；漏斗 (${xyz(funnel.pixel_center)})；u 向右、v 向下\n球心 XYZ：${xyz(ball.center_robot_mm)} mm\n${depth.depth_used?'使用球面深度拟合':'深度不足，使用保存桌面与34mm球径估计'}；深度样本 ${depth.valid_samples} 个\n漏斗口 XYZ：${xyz(funnel.mouth_center_robot_mm)} mm`;
    if($('depthFrame').getAttribute('src')!==p.depth_url)$('depthFrame').src=p.depth_url;
    const scale=p.scene.depth_scale;$('depthLegend').textContent=scale?`颜色范围：到相机 ${number(scale.near_mm)} ～ ${number(scale.far_mm)} mm；黑色表示深度缺失。`:'颜色表示表面到相机的距离，单位毫米。';
    $('planFacts').textContent=`路径：${p.steps.join(' → ')}\n本轮策略 v${p.policy_version}；${p.policy_application?.parameters_verified?'已核对新参数进入本轮计划。':'参数核对信息待更新。'}\n${p.policy_application?.source_trial_id?'参数来自试次 '+p.policy_application.source_trial_id.slice(0,10)+'。':''}`;
    $('archiveLink').hidden=!p.backup_url;if(p.backup_url)$('archiveLink').href=p.backup_url;
    $('backupStatus').textContent=p.backup_url?'完整试次已备份；Langfuse待同步记录已生成。':r?'正在整理本轮备份…':'观察数据已保存；动作结束后生成完整备份。';
  }
  $('trialVideo').hidden=!r?.capture_trial_id;$('rawVideoLink').hidden=!r?.capture_trial_id;$('keyframesLink').hidden=!r?.keyframes_file;
  if(r){
    const timing=Number.isFinite(r.physical_elapsed_s)?`动作耗时 ${number(r.physical_elapsed_s)} 秒`:'动作未完整结束，时序已保留';
    $('resultText').textContent=`闭爪${r.grasp_attempted?'已发送':'未发送'}；投放${r.placement_attempted?'已发送':'未发送'}；${timing}。${r.motion_error?'\n动作记录：'+r.motion_error:''}`;
    if(r.capture_trial_id&&$('trialVideo').dataset.planId!==j.plan_id){$('trialVideo').src=`${prefix}/plans/${j.plan_id}/video`;$('trialVideo').dataset.planId=j.plan_id;}
    $('rawVideoLink').href=`${prefix}/plans/${j.plan_id}/raw-video`;$('keyframesLink').href=`${prefix}/plans/${j.plan_id}/keyframes`;
  }else $('resultText').textContent=j.status==='running'?'本轮正在执行，结束后显示动作结果与录像。':'当前观察图与定位诊断；尚未执行本轮抓放。';
}
async function refresh(){
  if(polling||busy||document.hidden)return;polling=true;const rev=revision;
  try{
    const next=await api(prefix+'/state');if(rev!==revision)return;state=next;render();
    if(!stream.active)startStream();
    const key=`${state.job?.plan_id}:${state.job?.status}:${state.plan?.review?.after_version}:${state.plan?.backup_url}:${state.learning?.status}`;
    if(historyKey!==key){historyKey=key;loadHistory();}
  }catch(error){showError(error);}finally{polling=false;}
}
async function action(fn){if(busy)return;busy=true;revision++;$('error').hidden=true;render();try{await fn();}catch(error){showError(error);}finally{busy=false;await refresh();render();}}
$('connect').onclick=()=>action(()=>api('/api/auto-connect','POST',{}));
$('confirmed').onchange=render;
$('start').onclick=()=>{if($('start').disabled)return;return action(async()=>{await api(prefix+'/learning/start','POST',{declaration,max_attempts:Number($('maxAttempts').value)});$('confirmed').checked=false;});};
$('reset').onclick=()=>{if($('reset').disabled)return;return action(async()=>{await api(prefix+'/learning/reset','POST',{declaration});$('confirmed').checked=false;});};
$('stop').onclick=async()=>{if($('stop').disabled)return;$('stop').disabled=true;try{if(state.learning?.status==='running')await api(prefix+'/learning/stop','POST',{});else await api(prefix+'/stop','POST',{plan_id:state.job.plan_id});await refresh();}catch(error){showError(error);}};
$('trialVideo').onerror=()=>showError(Error('回放暂不可用，可下载原录像或查看关键帧；请检查回放编码器依赖。'));
function stopStream(){stream.active=false;stream.generation++;stream.controller?.abort();clearTimeout(stream.timer);}
function startStream(){stopStream();stream.active=true;stream.sequence=0;nextFrame(stream.generation);}
async function nextFrame(generation){
  if(!stream.active||generation!==stream.generation)return;stream.controller=new AbortController();let url;
  try{
    const response=await fetch('/api/bench/live-frame?after='+stream.sequence,{cache:'no-store',signal:stream.controller.signal});
    if(!response.ok)throw Error('实时画面暂不可用；录像与任务进度会单独显示。');
    const sequence=Number(response.headers.get('X-Frame-Sequence')),age=Number(response.headers.get('X-Frame-Age-Ms'));
    const width=Number(response.headers.get('X-Frame-Width')),height=Number(response.headers.get('X-Frame-Height'));
    const expected=state?.plan?.scene.image_size;
    if(!Number.isSafeInteger(sequence)||sequence<=stream.sequence||!Number.isFinite(age)||age<0||age>1500||width<1||height<1||(expected&&(width!==expected[0]||height!==expected[1])))throw Error('实时帧信息已过期或尺寸不匹配。');
    url=URL.createObjectURL(await response.blob());const decoded=new Image();decoded.src=url;await decoded.decode();
    if(!stream.active||generation!==stream.generation){URL.revokeObjectURL(url);return;}
    if(decoded.naturalWidth!==width||decoded.naturalHeight!==height)throw Error('实时图像尺寸不匹配。');
    if(stream.url)URL.revokeObjectURL(stream.url);stream.url=url;url=null;$('workspaceFrame').src=stream.url;
    $('workspaceFrame').hidden=false;$('frameEmpty').hidden=true;$('streamNotice').hidden=true;$('viewBadge').textContent='实时画面';$('imageCaption').textContent='实时画面 · 每轮RGB-D由独立相机服务录制';
    stream.sequence=sequence;stream.timer=setTimeout(()=>nextFrame(generation),150);
  }catch(error){if(url)URL.revokeObjectURL(url);if(!stream.active||generation!==stream.generation)return;stopStream();$('streamNotice').hidden=false;$('streamNotice').textContent=error.message;$('viewBadge').textContent='预览暂停';$('imageCaption').textContent='当前画面已暂停；录像状态单独显示。';}
}
async function loadHistory(){
  try{const data=await api(prefix+'/history');$('trials').replaceChildren();$('count').textContent=data.trials.length+' 次';$('emptyTrials').hidden=Boolean(data.trials.length);
    for(const trial of data.trials){const row=document.createElement('article'),title=document.createElement('strong'),summary=document.createElement('p');row.className='trial-row';
      title.textContent=`试次 ${trial.id.slice(0,10)} · 参数 v${trial.policy_version}`;
      const timing=Number.isFinite(trial.result?.physical_elapsed_s)?`动作 ${number(trial.result.physical_elapsed_s)} 秒`:'动作未完整结束';
      summary.textContent=`${trial.review?(trial.review.source==='human'?'人工记录：':'Grok：')+(outcomes[trial.review.outcome]||trial.review.outcome):'未分析'}；${timing}`;
      row.append(title,summary);if(trial.review?.changes?.length){const changes=document.createElement('p');changes.className='quiet';changes.textContent=trial.review.changes.join('\n');row.append(changes);}if(trial.backup_url){const link=document.createElement('a');link.href=trial.backup_url;link.textContent='下载本轮备份';link.setAttribute('download','');row.append(link);}$('trials').append(row);}
  }catch(error){showError(error);}
}
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopStream();else refresh();});
window.addEventListener('pagehide',()=>{stopStream();if(stream.url)URL.revokeObjectURL(stream.url);});
setInterval(refresh,800);refresh();loadHistory();
