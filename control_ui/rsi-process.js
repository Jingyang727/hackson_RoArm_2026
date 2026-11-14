/* Public observations and controller decisions. This module only performs GETs. */
(() => {
  const el=id=>document.getElementById(id);
  const outcomes={placed:'已抓起并进入漏斗',grasp_missed:'没夹住',pushed_away:'球被推走',slipped:'球滑落',placement_missed:'投放偏了',bounced_out:'球弹出了漏斗',execution_failed:'动作未完成',unknown:'画面不足以判断'};
  const directions={image_left:'夹爪在目标的画面左侧',image_right:'夹爪在目标的画面右侧',image_up:'夹爪在目标的画面上方',image_down:'夹爪在目标的画面下方',too_high:'抓取或投放位置偏高',too_low:'抓取或投放位置偏低',none:'没有明确偏差方向'};
  const adjustments={keep:'保留当前策略',pickup_xy:'调整抓取平面偏移',grasp_height:'调整抓取高度',grip_tighten:'增加夹持预紧',transport_slower:'降低运输速度',release_xy:'调整投放平面偏移',release_height:'调整投放高度',unknown:'没有明确调整类别'};
  const ballStates={table:'桌面上',gripper:'夹爪中',funnel:'漏斗中',unknown:'无法确认'};
  const magnitudes={small:'小',medium:'中',large:'大',unknown:'不确定'};
  const effects={improved:'偏差缩小',worsened:'偏差变大',overshot:'越过目标',unchanged:'偏差没有明显变化',unknown:'看不清效果',not_comparable:'两轮不可直接比较',not_compared:'本轮没有可比前轮'};
  const movements={as_expected:'向预期方向移动',opposite:'与预期方向相反',no_visible_motion:'没有可见的相应移动',unknown:'移动方向无法确认'};
  const phases={starting:'任务开始',observing:'观察与规划',scene_check:'核对球与漏斗',executing:'执行本轮抓放',analyzing:'分析本轮录像',learning:'更新经验参数',recovering:'恢复官方姿态',replanning:'回位后重新定位',reset_completed:'官方复位已确认',completed:'本轮成功，策略已保存',exhausted:'尝试结束，记录已保存',failed:'任务停止',stopping:'收到停止指令',stopped:'已停止',recording_warmup:'启动10秒录像',raise_current:'原XY竖直抬升',clearance_adjust:'竖直到通行高度',orient_clear:'高处调整姿态',align_above_ball:'球顶上方竖直对齐',lower_to_release:'竖直到漏斗上方并停稳',open:'张爪',align:'调整接近姿态',approach_ball:'接近黄球并张爪',descend:'下降',close:'合抓',lift:'提升',transit_arc:'经过过渡点',transport:'运输至漏斗',release:'释放',saving_recording:'保存录像',finished:'本轮动作与录像已结束'};
  let session=null,started=0,sequence=-1,rows=[],output=null,source=null,paired=null;
  function verdict(value){
    if(!value)return '';
    if(typeof value.ready==='boolean')return `场景检查：${value.ready?'已确认球与漏斗':'暂不能确认'}\n${value.note||''}`;
    const lines=[`${value.source==='human'?'人工记录':'Grok判断'}：${outcomes[value.outcome]||value.outcome}`];
    if(value.ball_state)lines.push(`球的位置：${ballStates[value.ball_state]||value.ball_state}`);
    if(value.direction)lines.push(`${directions[value.direction]||value.direction}${value.magnitude?'；粗略偏差：'+(magnitudes[value.magnitude]||value.magnitude):''}`);
    if(value.adjustment)lines.push(`Grok建议：${adjustments[value.adjustment]||value.adjustment}`);
    const comparison=value.comparison?.assessment||value.comparison;
    if(comparison?.effect)lines.push(`前后对照：${effects[comparison.effect]||comparison.effect}；${movements[comparison.movement]||comparison.movement}\n${comparison.note||''}`);
    if(typeof value.confidence==='number')lines.push(`模型自报置信度：${Math.round(value.confidence*100)}%`);
    if(value.visual_review){
      lines.push(`Grok画面依据：${value.visual_review.note||''}`);
      if(value.note)lines.push(`执行Agent决策：${value.note}`);
    }else if(value.note)lines.push(`画面依据：${value.note}`);
    return lines.join('\n');
  }
  function describe(row){
    if(row.kind==='phase'||row.kind==='motion')return (phases[row.phase]||row.phase)+(row.error?'：'+row.error:row.message?'：'+row.message:'');
    if(row.kind==='plan')return `本轮规划完成 · 策略 v${row.policy_version} · ${row.depth_used?'使用球面深度':'使用保存桌面与球径估计'}${row.policy_application?.parameters_verified?' · 参数已核对':''}`;
    if(row.kind==='grok_started')return `Grok开始${row.preflight?'核对当前场景':'复核本轮录像'} · ${row.frame_count} 张图`;
    if(row.kind==='grok_validated')return verdict(row.verdict);
    if(row.kind==='grok_failed'||row.kind==='grok_stopped')return row.error||'Grok回复未完成';
    if(row.kind==='policy')return '程序采用的修正：\n'+(row.changes||[]).join('\n');
    if(row.kind==='comparison')return row.comparison.available?'已提供前后两轮对应动作的实帧和参数变化':'本轮对照：'+row.comparison.reason;
    if(row.kind==='recovery')return row.status==='completed'?'官方初始姿态已确认':'官方复位未确认';
    return '';
  }
  function publicText(text){
    const prefix=text.split('{',1)[0].trim();if(prefix)return prefix;
    const note=/"note"\s*:\s*"((?:\\.|[^"\\])*)/.exec(text);
    if(note){try{return JSON.parse('"'+note[1]+'"');}catch{}}
    return text?'正在生成结构化结果，可展开查看完整公开回复。':'';
  }
  function render(){
    const list=el('processEvents'),nearBottom=list.scrollHeight-list.scrollTop-list.clientHeight<45;
    list.replaceChildren();
    for(const row of rows){
      const message=describe(row);if(!message)continue;
      const item=document.createElement('li'),meta=document.createElement('span'),text=document.createElement('p');
      meta.className='process-time';meta.textContent=`${row.attempt?'第 '+row.attempt+' 轮 · ':''}${new Date(row.at_unix*1000).toLocaleTimeString()}`;
      text.textContent=message;item.append(meta,text);list.append(item);
    }
    el('processEmpty').hidden=Boolean(list.childElementCount);if(nearBottom)list.scrollTop=list.scrollHeight;
    showComparison(paired);
    el('grokLive').hidden=!output;
    if(output){
      const status=output.status,mode=output.mode;
      el('grokLiveTitle').textContent=`Grok${output.preflight?'场景检查':'录像复核'} · 第 ${output.attempt} 轮`;
      el('grokReplyStatus').textContent=status==='validated'?'回复已完成并通过校验':status==='failed'?'回复未完成或未通过校验，未采用':status==='stopped'?'已停止，未采用部分回复':mode==='streaming'?'正在流式回复，本轮结论尚未确认':mode==='buffered_fallback'?'接口不支持流式，等待完整回复':mode==='buffered'?'等待接口返回完整回复':'等待Grok开始回复…';
      el('grokLiveText').textContent=publicText(output.text)||output.verdict?.note||'分析开始后，公开观察与建议会显示在这里。';
      el('grokRawText').textContent=output.text||'';
    }
  }
  function showComparison(value){
    paired=value||null;el('effectComparison').hidden=!paired;if(!paired)return;
    const assessment=paired.assessment,feedback=paired.feedback||{};
    el('comparisonSummary').textContent=paired.available?`前轮 ${paired.reference_plan_id?.slice(0,10)} → 本轮 ${paired.plan_id?.slice(0,10)}；${assessment?effects[assessment.effect]+'；'+movements[assessment.movement]:'Grok正在对照两轮实帧。'}`:paired.reason||'本轮没有可比前轮。';
    const lines=(paired.parameter_changes||[]).map(row=>`${row.name}：${JSON.stringify(row.before)} → ${JSON.stringify(row.after)}`);
    const vector=v=>v.map(n=>Number(n).toFixed(2)).join(', ');
    if(paired.requested_xy_delta_mm)lines.push(`要求改变的 XY 补偿：[${vector(paired.requested_xy_delta_mm)}] mm`);
    if(paired.expected_image_delta_px)lines.push(`标定预测的画面位移：[${vector(paired.expected_image_delta_px)}] px（计算值）`);
    if(feedback.available)lines.push(`固件报告的 XY 补偿变化：[${vector(feedback.reported_xy_delta_mm)}] mm（已扣除目标位置变化；不是图像实测）`);
    else lines.push('对应动作的固件位移反馈不足，不把命令当成已经发生的移动。');
    if(assessment?.note)lines.push(`视频对照依据：${assessment.note}`);
    el('comparisonFacts').textContent=lines.join('\n');
    const image=el('comparisonFrame');image.hidden=!paired.panel_url;if(paired.panel_url&&image.getAttribute('src')!==paired.panel_url)image.src=paired.panel_url;
  }
  function snapshot(data){
    if(!data)return;
    const next=Number(data.cursor?.split(':').pop());
    if(data.session_id===session&&next<=sequence)return;
    if(data.session_id!==session&&(data.started_at_unix||0)<started)return;
    session=data.session_id;started=data.started_at_unix||0;sequence=next;
    rows=data.events||[];output=data.model_output||null;
    const last=[...rows].reverse().find(r=>r.kind==='comparison'||r.kind==='policy'&&r.review?.comparison);
    paired=last?(last.kind==='comparison'?last.comparison:last.review.comparison):null;
    el('processStorageError').textContent=data.persistence_error||'';render();
  }
  function event(row){
    if(row.session_id!==session||row.sequence<=sequence)return;sequence=row.sequence;
    if(row.kind==='grok_started')output={plan_id:row.plan_id,attempt:row.attempt,preflight:row.preflight,text:'',status:'generating',mode:'connecting'};
    if(output&&row.plan_id===output.plan_id){
      if(row.kind==='grok_delta')output.text=(output.text+row.delta).slice(0,16000);
      if(row.kind==='grok_mode')output.mode=row.mode;
      if(['grok_validated','grok_failed','grok_stopped'].includes(row.kind)){output.status=row.kind.slice(5);output.verdict=row.verdict;}
    }
    if(row.kind==='comparison')paired=row.comparison;
    if(row.kind==='policy'&&row.review?.comparison)paired=row.review.comparison;
    if(row.kind!=='grok_delta'){rows.push(row);if(rows.length>512)rows.shift();}render();
  }
  function connect(){
    if(source||document.hidden)return;
    if(!window.EventSource){el('processConnection').textContent='通过状态刷新显示';return;}
    const current=new EventSource('/api/bench/rsi/learning/events');source=current;
    current.onopen=()=>{if(source===current)el('processConnection').textContent='实时过程已连接';};
    current.onerror=()=>{if(source===current)el('processConnection').textContent='正在重连，状态刷新继续';};
    current.addEventListener('snapshot',message=>{if(source===current)snapshot(JSON.parse(message.data));});
    current.addEventListener('rsi',message=>{if(source===current)event(JSON.parse(message.data));});
  }
  function close(){const current=source;source=null;current?.close();}
  document.addEventListener('visibilitychange',()=>{if(document.hidden)close();else connect();});
  window.addEventListener('pagehide',close);
  window.RSIProcess={snapshot,connect,close,verdict,showComparison};connect();
})();
