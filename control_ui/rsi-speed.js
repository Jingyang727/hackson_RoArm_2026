/* Edit the persisted policy while idle. Saving sends no movement command. */
(() => {
  const el=id=>document.getElementById(id),travel=el('travelSpeed'),approach=el('approachSpeed');
  let latest=null,baseline=null,dirty=false,saving=false,blocked=true,message='';
  const valid=value=>typeof value==='number'&&Number.isFinite(value)&&value>=.05&&value<=1;
  const sameBasis=(a,b)=>a&&b&&a.policy_version===b.policy_version&&a.calibration_fingerprint===b.calibration_fingerprint;
  const changed=()=>document.dispatchEvent(new Event('rsi-speed-change'));
  function load(){
    baseline=latest;dirty=false;message='';
    if(baseline){travel.value=String(baseline.travel_speed);approach.value=String(baseline.approach_speed);}
  }
  function draw(){
    const stale=dirty&&!sameBasis(baseline,latest),available=Boolean(latest)&&!blocked&&!saving;
    travel.disabled=approach.disabled=el('speedFast').disabled=!available;
    el('saveSpeed').disabled=!available||!dirty||stale||!valid(Number(travel.value))||!valid(Number(approach.value));
    el('reloadSpeed').disabled=!available||!dirty;
    el('speedSummary').textContent=latest?`抓放速度 · 转移 ${latest.travel_speed.toFixed(2)} / 下探 ${latest.approach_speed.toFixed(2)}`:'抓放速度设置';
    el('speedStatus').textContent=saving?'正在保存速度，不发送运动…':message.startsWith('保存失败')?message:stale?'策略已变化，请重新载入后再修改速度。':
      dirty?'速度尚未保存；保存后下一轮重新规划。':message||'沿用当前策略；修改并保存后供下一轮使用。RSI仍可按实际结果调整速度。';
  }
  function configure(data,externalBusy=false){
    const incoming=data?.speed_settings;
    if(data?.speed_settings_version===1&&valid(incoming?.travel_speed)&&valid(incoming?.approach_speed)){
      // A render immediately after saving can still contain the previous poll.
      if(!latest||incoming.calibration_fingerprint!==latest.calibration_fingerprint||incoming.policy_version>=latest.policy_version)latest=incoming;
      if(!dirty&&!saving&&!sameBasis(baseline,latest))load();
      blocked=externalBusy||data.learning?.status==='running'||['running','preparing'].includes(data.job?.status)||Boolean(data.health?.pending);
    }else{
      latest=null;blocked=true;message=incoming?.error?'速度设置暂不可用：'+incoming.error:'请更新并重启网页服务以加载速度设置。';
    }
    draw();
  }
  function edit(){
    dirty=Boolean(baseline)&&(Number(travel.value)!==baseline.travel_speed||Number(approach.value)!==baseline.approach_speed);
    message='';draw();changed();
  }
  travel.oninput=approach.oninput=edit;
  el('speedFast').onclick=()=>{if(el('speedFast').disabled)return;travel.value='1';approach.value='0.5';edit();};
  el('reloadSpeed').onclick=()=>{if(el('reloadSpeed').disabled)return;load();draw();changed();};
  el('saveSpeed').onclick=async()=>{
    if(el('saveSpeed').disabled)return;
    const body={travel_speed:Number(travel.value),approach_speed:Number(approach.value),
      policy_version:baseline.policy_version,calibration_fingerprint:baseline.calibration_fingerprint};
    saving=true;draw();changed();
    try{
      const response=await fetch('/api/bench/rsi/speed',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
      const raw=await response.text();let value;try{value=JSON.parse(raw);}catch{throw Error(`HTTP ${response.status}：服务未返回JSON`);}
      if(!response.ok)throw Error(typeof value.detail==='string'?value.detail:'保存速度失败');
      if(!valid(value.travel_speed)||!valid(value.approach_speed))throw Error('服务返回的速度设置无效，请重新载入');
      latest=value;load();message='速度已保存；下一轮将按新速度重新规划。';
      document.dispatchEvent(new Event('rsi-speed-saved'));
    }catch(error){message='保存失败：'+error.message;}
    finally{saving=false;draw();changed();}
  };
  window.RSISpeed={configure,isDirty:()=>dirty||saving};
})();
