/* Optional pure planning preview; no run, torque, reset or authorization API. */
(() => {
  const el=id=>document.getElementById(id),prefix='/api/bench/rsi';
  let snapshot=null,busy=false,result=null,observation=null,timer=null,generation=0,rendered=null,basisPlanId=null;
  const checkbox=el('workspaceVisible'),overlay=el('workspaceOverlay'),note=el('workspaceStatus');
  async function api(path,method='GET',body){
    const response=await fetch(prefix+path,{method,cache:'no-store',headers:{'Content-Type':'application/json'},...(body===undefined?{}:{body:JSON.stringify(body)})});
    const text=await response.text();let data;try{data=JSON.parse(text);}catch{throw Error(`HTTP ${response.status}：工作区接口未返回JSON`);}
    if(!response.ok)throw Error(typeof data.detail==='string'?data.detail:data.detail?.message||'工作区计算失败');
    return data;
  }
  function changed(){document.dispatchEvent(new Event('rsi-workspace-change'));}
  const canonical=value=>Array.isArray(value)?value.map(canonical):value&&typeof value==='object'?Object.fromEntries(Object.keys(value).sort().map(k=>[k,canonical(value[k])])):value;
  const same=(a,b)=>JSON.stringify(canonical(a))===JSON.stringify(canonical(b));
  function draw(){
    const hidden=!checkbox.checked||!result?.grid?.length;
    overlay.toggleAttribute('hidden',hidden);
    if(hidden)return;
    const signature=`${result.workspace_id}:${result.completed}:${result.status}`;
    if(signature===rendered)return;rendered=signature;overlay.replaceChildren();
    const [width,height]=result.image_size;overlay.setAttribute('viewBox',`0 0 ${width} ${height}`);
    const paths={'+':'','-':''},w=width/result.columns,h=height/result.rows;
    result.grid.forEach((row,y)=>[...row].forEach((value,x)=>{
      if(value in paths)paths[value]+=`M${x*w} ${y*h}h${w}v${h}h${-w}Z`;
    }));
    for(const [value,d] of Object.entries(paths)){
      const path=document.createElementNS('http://www.w3.org/2000/svg','path');
      path.setAttribute('d',d);path.setAttribute('fill',value==='+'?'#1c9477':'#626b77');path.setAttribute('fill-opacity',value==='+'?'.22':'.12');overlay.append(path);
    }
  }
  function describe(){
    draw();el('workspaceRecompute').hidden=!result;el('workspaceRecompute').disabled=busy||checkbox.disabled;
    if(!result)return;
    if(result.status==='failed'){note.textContent='区域计算未完成：'+result.error;return;}
    const lines=[result.status==='computing'?`正在检查完整抓放候选路径：${result.completed||0} / ${result.total||'…'} 个采样点`:
      `绿色：找到完整抓放路径（${result.planned_samples||0}点）；灰色：当前搜索未通过；空白：未完成采样。`];
    const ball=result.selected_ball;
    if(ball?.status==='planned')lines.push(`本次观察的球：找到完整路径；抓取俯仰 ${ball.grasp_pitch_deg.toFixed(0)}°，抬升/运输高度 ${ball.transfer_height_mm.toFixed(0)}mm。`);
    else if(ball?.diagnosis)lines.push(`本次观察的球：${ball.diagnosis.summary}。${ball.diagnosis.suggestion}`);
    else if(ball)lines.push('本次观察的球：计算窗口内尚未确认完整路径。');
    if(result.initial_xyz_mm)lines.push(`按计算时的起步 XYZ (${result.initial_xyz_mm.map(v=>v.toFixed(0)).join(', ')}) mm、当前漏斗和策略估计。边界为采样参考，执行时仍检查实际路径。`);
    note.textContent=lines.join('\n');
  }
  function invalidate(message){checkbox.checked=false;overlay.toggleAttribute('hidden',true);result=null;observation=null;rendered=null;el('workspaceRecompute').hidden=true;note.textContent=message;}
  function configure(data,externalBusy=false){
    snapshot=data;
    checkbox.disabled=externalBusy||data?.learning?.status==='running'||data?.job?.status==='running'||data?.workspace_preview_version!==1;
    if(!busy&&result&&observation&&data?.plan&&data.plan.plan_id!==basisPlanId){
      const pose=data.plan.initial_pose;
      const moved=(a,b)=>Array.isArray(a)&&Array.isArray(b)&&Math.hypot(...a.map((v,i)=>v-b[i]))>3;
      const changedPose=pose&&(moved(pose.xyz_mm,result.initial_xyz_mm)||
        Math.abs(pose.pitch_rad-result.initial_pitch_deg*Math.PI/180)>.02||
        Math.abs(pose.roll_rad-result.initial_roll_rad)>.02);
      if(changedPose||moved(data.plan.scene.funnel?.mouth_center_robot_mm,result.funnel_xyz_mm)||
        !same(data.plan.parameters,result.parameters)||!same(data.plan.scene.camera_geometry,observation.scene.camera_geometry))
        invalidate('起步规划条件或策略已变化，请重新计算区域。');
      basisPlanId=data.plan.plan_id;
    }
    el('workspaceRecompute').disabled=busy||checkbox.disabled;
    if(result)describe();
    else if(data&&data.workspace_preview_version!==1)note.textContent='网页服务尚未加载工作区预览，请更新并重启服务。';
  }
  async function calculate(){
    if(busy)return;
    busy=true;result=null;observation=null;rendered=null;basisPlanId=snapshot?.plan?.plan_id||null;overlay.toggleAttribute('hidden',true);const token=++generation;changed();
    note.textContent='采集当前球与漏斗并读取起步姿态，随后计算区域…';
    async function poll(id){
      if(token!==generation)return;
      try{
        result=await api('/workspace/'+id);describe();
        if(result.status==='computing')timer=setTimeout(()=>poll(id),500);
        else{busy=false;changed();}
      }catch(error){busy=false;note.textContent=error.message;changed();}
    }
    try{
      observation=await api('/observe','POST',{});
      const job=await api('/workspace','POST',{observation_id:observation.observation_id});
      await poll(job.workspace_id);
    }catch(error){busy=false;note.textContent=error.message;checkbox.checked=false;changed();}
  }
  checkbox.onchange=()=>{draw();changed();if(checkbox.checked&&!result&&!busy)calculate();};
  el('workspaceRecompute').onclick=()=>{checkbox.checked=true;calculate();};
  window.RSIWorkspace={configure,isBusy:()=>busy,sceneForOverlay:()=>checkbox.checked?observation?.scene:null};
  document.addEventListener('visibilitychange',()=>{if(!document.hidden&&result)draw();});
  window.addEventListener('pagehide',()=>{generation++;clearTimeout(timer);});
})();
