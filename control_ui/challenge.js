/* Multi-ball desktop challenge. Loading or previewing never starts motion. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const declaration = '现场已清空，允许连续抓球入碗及故障时自动回位';
  const phases = {idle:'尚未开始',ready:'已完成只读定位',starting:'开始挑战',observing:'观察桌面上的球',
    observed:'桌面定位完成',refreshing:'刷新 RGB-D 位置',planning:'检查完整抓放路径',executing:'执行抓放',
    recovering:'恢复官方初始姿态',checking_home:'检查起步姿态',reviewing:'分析本轮录像',reobserving:'重新扫描剩余球',
    waiting_view:'等待清晰画面',blocked:'剩余球路径不可达',container_locked:'已锁定本次漏斗位置',
    recording_warmup:'启动20秒录像',approach_ball:'接近目标球并张爪',raise_current:'原 XY 向上抬升',
    clearance_adjust:'抬升到通行高度',orient_transport:'高处调整运输姿态',orient_clear:'高处调整姿态',
    align_above_ball:'在球顶上方对齐',descend:'竖直下探',close:'合抓',lift:'抬升',transport:'高处运输至收集容器',
    transit_arc:'经过高处途径点',lower_to_release:'停稳在容器上方',release:'释放球',saving_recording:'保存录像',
    finished:'本轮动作结束',stopping:'正在停止后续动作',stopped:'挑战已停止',failed:'挑战暂停',completed:'桌面已清空，挑战完成',cleared:'桌面已清空，部分入碗结果未确认',verifying_table:'最后确认桌面画面'};
  const colours = {red:'#d74c4c',orange:'#e99436',yellow:'#ebca36',green:'#489756',cyan:'#47b8be',blue:'#517dcc',purple:'#995bbb',pink:'#df7dab',white:'#fafafa',black:'#30373b'};
  const names = {red:'红球',orange:'橙球',yellow:'黄球',green:'绿球',cyan:'青球',blue:'蓝球',purple:'紫球',pink:'粉球',white:'白球',black:'黑球'};
  const outcomes = {placed:'已进碗',grasp_missed:'没抓住',pushed_away:'球被推开',slipped:'抓起后滑落',placement_missed:'投放偏了',bounced_out:'投放后弹出',execution_failed:'动作未完成',unknown:'结果未确认'};
  let state = null, busy = false, observation = '', historyKey = '', sequence = 0, objectURL = null, actionError = '';
  const media = (job,trial,kind) => `/api/challenge/sessions/${encodeURIComponent(job)}/trials/${encodeURIComponent(trial)}/${kind}`;
  async function api(path,body) {
    const response=await fetch('/api/challenge/'+path,{cache:'no-store',...(body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})});
    const raw=await response.text();let value;
    try {value=JSON.parse(raw);} catch {throw new Error('服务回复无效（HTTP '+response.status+'），请查看服务日志。');}
    if(!response.ok)throw new Error(typeof value.detail==='string'?value.detail:JSON.stringify(value.detail||value));
    return value;
  }
  function error(text){$('error').textContent=text||'';$('error').hidden=!text;}
  function controls() {
    const h=state?.health||{},c=state?.challenge||{},running=c.status==='running',observing=c.status==='observing';
    const ready=state?.challenge_api_version===2&&h.connected&&h.motion_enabled&&h.teach_active===false&&
      h.torque_status?.length===6&&h.torque_status.every(v=>v===1||v===true)&&!h.pending&&!h.automation_owner&&state.calibrated&&state.model?.configured&&!state.feedback_error;
    $('start').disabled=busy||running||observing||!ready||!$('confirmed').checked;
    $('observe').disabled=busy||running||observing||!!h.automation_owner||!!h.pending||!state?.calibrated||!state?.model?.configured;
    $('confirmed').disabled=busy||running||observing;$('stop').hidden=!running;$('stop').disabled=busy;
    $('connection').textContent=h.connected?'机械臂已连接':'机械臂未连接';
    $('readiness').textContent=state&&state.challenge_api_version!==2?'请重启网页服务以加载新版连续挑战模式。':running?'连续抓放中；正常轮次不回初始姿态。':!h.connected?'请先在坐标验证页连接机械臂。':
      !h.motion_enabled?'当前服务为只读模式，需要开启实物执行。':h.teach_active?'请先锁定六关节扭矩并结束手动调整。':
      !h.torque_status?.every(v=>v===1||v===true)?'尚未确认六关节扭矩全部锁定。':state?.feedback_error||state?.calibration_error||
      (!state?.model?.configured?state?.model?.reason:null)||(h.automation_owner||h.pending?'其他任务正在占用控制器。':
      c.status==='failed'?'本次挑战已暂停，具体原因见下方；重新开始会重新定位漏斗。':'沿用现有标定、容器与高处运输路径；可以开始连续抓放。');
  }
  function renderScene(c) {
    if(!c.scene)return;
    $('observation').hidden=false;
    const key=c.observation_session_id+'/'+c.observation_id;
    if(c.observation_session_id&&c.observation_id&&key!==observation){observation=key;$('observationFrame').src='/api/challenge/sessions/'+key.replace('/','/observations/');}
    $('balls').replaceChildren();
    for(const ball of c.scene.balls||[]){
      const li=document.createElement('li'),dot=document.createElement('span'),label=document.createElement('span'),xyz=document.createElement('small');
      dot.className='dot';dot.style.background=colours[ball.color]||'#87949b';label.textContent=ball.label||names[ball.color]||ball.color;
      xyz.textContent='XYZ '+ball.center_robot_mm.map(v=>Number(v).toFixed(1)).join(', ')+' mm · '+(ball.depth_evidence?.depth_used?'深度球面拟合':'保存桌面与球径估计');
      label.append(xyz);li.append(dot,label);$('balls').append(li);
    }
    const bowl=c.scene.bowl;$('bowl').textContent=bowl?'容器口中心 XYZ '+bowl.mouth_center_robot_mm.map(v=>Number(v).toFixed(1)).join(', ')+' mm'+
      (c.scene.bowl_location?.mode==='challenge_locked'?' · 沿用本次挑战开始时的位置':' · 本次画面定位'):'';
  }
  function renderTrials(c) {
    const rows=c.trials||[],key=JSON.stringify([c.session_id,rows]);if(key===historyKey)return;historyKey=key;
    $('trials').replaceChildren();$('trialCount').textContent=rows.length+' 次尝试';$('trialEmpty').hidden=rows.length>0;
    for(const row of [...rows].reverse()){
      const box=document.createElement('div');box.className='trial';const title=document.createElement('strong');
      title.textContent=(names[row.target_color]||row.target_color)+' · '+(row.review_status==='pending'?'后台复核中':outcomes[row.outcome]||'未确认');box.append(title);
      const p=document.createElement('p');p.className='quiet';p.textContent='试次 '+row.plan_id.slice(0,10)+' · 动作 '+(row.physical_elapsed_s==null?'未确认':row.physical_elapsed_s.toFixed(1)+' 秒');box.append(p);
      for(const [kind,text] of [['archive','下载完整备份'],['keyframes','查看关键帧'],['state','查看参数与反馈']]){
        const a=document.createElement('a');a.href=media(c.session_id,row.plan_id,kind);a.textContent=text;a.target='_blank';a.rel='noopener';box.append(a);
      }
      const details=document.createElement('details'),summary=document.createElement('summary');summary.textContent='查看本轮录像';details.append(summary);
      details.addEventListener('toggle',()=>{if(details.open&&!details.querySelector('video')){const video=document.createElement('video');video.controls=true;video.preload='none';video.src=media(c.session_id,row.plan_id,'video');details.append(video);}});
      box.append(details);$('trials').append(box);
    }
  }
  function renderProcess(process) {
    if(!process)return;
    if(process.reset){$('events').replaceChildren();for(const row of process.events||[])addEvent(row);}
    const output=process.model_output;
    if(output){$('modelLabel').textContent=(output.preflight?'当前画面确认':'后台分析 · 第 '+output.attempt+' 次')+' · '+(output.model||'视觉模型')+' · '+(output.status==='generating'?'正在分析':'公开回复');$('modelText').textContent=output.text||'';}
    $('storageError').textContent=process.persistence_error||'';
  }
  function addEvent(row) {
    if(row.kind==='grok_delta'){$('modelText').textContent+=row.delta;return;}
    if(row.kind==='grok_started')$('modelText').textContent='';
    if(row.kind!=='phase'&&row.kind!=='review'&&row.kind!=='target')return;
    const li=document.createElement('li'),time=new Date(row.at_unix*1000).toLocaleTimeString('zh-CN',{hour12:false});
    const description=row.kind==='phase'?(phases[row.phase]||row.phase)+(row.message?'：'+row.message:''):
      row.kind==='target'?'目标：'+row.ball.label:'视觉判断：'+(outcomes[row.verdict?.outcome]||'未确认');
    li.textContent=time+' · '+description;$('events').append(li);while($('events').children.length>150)$('events').firstChild.remove();
  }
  function render(value) {
    state=value;const c=value.challenge||{};controls();
    $('modelStatus').textContent='视觉模型：'+(value.model?.model||'未配置')+' · 失败时后台分析，空桌时确认';
    $('recordingStatus').textContent=c.adaptive_recording===false?'当前相机服务仍等待20秒录像；重启新版相机服务后，可在动作结束后停录。':c.adaptive_recording===true?'动作完成后保留短暂投放画面即可停录，最多20秒。':'';
    $('containerStatus').textContent=c.container_reference?'漏斗位置已锁定：本次挑战内沿用，遮挡口沿不会触发重新定位。移动漏斗后请停止并重新开始。':
      '每次开始挑战时定位漏斗，本次挑战内沿用该位置；移动漏斗后请停止并重新开始。';
    $('remaining').textContent=c.remaining??'';$('placed').textContent=String((c.confirmed_colors||[]).length);
    $('attempt').textContent=c.attempt?'第 '+c.attempt+' 次抓放':'尚未开始';$('phase').textContent=phases[c.phase]||c.phase||'尚未开始';
    $('target').textContent=c.status==='running'&&c.target?'当前目标：'+c.target.label:'';
    $('verdict').textContent=c.last_review?(c.review_source==='rgbd'?'RGB-D画面确认：':'最近视觉判断：')+(outcomes[c.last_review.outcome]||c.last_review.outcome)+'\n'+(c.last_review.note||c.last_review.evidence||c.last_review.reason||''):'';
    $('changes').textContent=(c.changes||[]).join('\n');$('success').hidden=c.status!=='completed'||c.placement_confirmed!==true;
    $('clearNotice').hidden=c.status!=='completed'||c.placement_confirmed===true;
    $('clearNotice').textContent='桌面已清空，已确认入碗 '+(c.confirmed_colors||[]).length+' 个；其他球的去向未确认，录像已保存。';
    $('pauseReason').hidden=c.status!=='failed';
    $('pauseText').textContent=c.status==='failed'?(c.failure_phase?'发生阶段：'+(phases[c.failure_phase]||c.failure_phase)+'\n':'')+
      (c.error||'服务未返回具体原因，请查看挑战过程及逐球记录。'):'';
    error(c.error||actionError);renderScene(c);renderTrials(c);renderProcess(value.process);
  }
  async function refresh(){try{render(await api('state'));}catch(e){error(e.message);}}
  async function action(path,body){busy=true;actionError='';error('');controls();try{await api(path,body);await refresh();}catch(e){actionError=e.message;error(actionError);}finally{busy=false;controls();}}
  $('confirmed').addEventListener('change',controls);
  $('observe').addEventListener('click',()=>action('observe',{}));
  $('start').addEventListener('click',()=>action('start',{declaration}));
  $('stop').addEventListener('click',()=>action('stop',{}));
  async function frame(){
    try{
      const response=await fetch('/api/bench/live-frame?after='+sequence,{cache:'no-store',signal:AbortSignal.timeout(12000)});
      if(response.status===204)return;if(!response.ok)throw new Error('相机暂不可用');
      sequence=Number(response.headers.get('X-Frame-Sequence'))||sequence;const url=URL.createObjectURL(await response.blob());
      const previous=objectURL;objectURL=url;$('liveFrame').src=url;if(previous)URL.revokeObjectURL(previous);$('cameraStatus').textContent='实时画面 · 不叠加旧定位标记';
    }catch(e){$('cameraStatus').textContent='实时画面暂不可用；'+e.message;}
    finally{if(!document.hidden)setTimeout(frame,300);else setTimeout(frame,1500);}
  }
  const stream=new EventSource('/api/challenge/events');
  stream.addEventListener('snapshot',e=>renderProcess(JSON.parse(e.data)));
  stream.addEventListener('rsi',e=>{addEvent(JSON.parse(e.data));});
  refresh();setInterval(refresh,1500);frame();
})();
