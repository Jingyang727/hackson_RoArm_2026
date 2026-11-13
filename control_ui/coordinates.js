const $ = id => document.getElementById(id);
const declaration = '已核对XYZ目标和路径，现场有人看护且双手离开运动范围';
let state=null,plan=null,busy=false,polling=false,revision=0,depth=null,verifiedID=null;
let selectionMode='point',targetInitialized=false;
const view={yaw:.8,pitch:.6,zoom:1};
const canvas=$('scene'),ctx=canvas.getContext('2d');
const number = value => typeof value==='number' && Number.isFinite(value) ? value.toFixed(1) : '—';
const target = () => ['x','y','z'].map(id => $(id).value.trim()==='' ? NaN : Number($(id).value));
const coordinateText = values => values.map(number).join(', ');
async function api(path,method='GET',body) {
  const response=await fetch(path,{method,cache:'no-store',headers:{'Content-Type':'application/json'},...(body===undefined?{}:{body:JSON.stringify(body)})});
  const raw=await response.text();let value;
  try{value=JSON.parse(raw);}catch{throw Error(`HTTP ${response.status}：服务未返回JSON，请检查服务日志。`);}
  if(!response.ok)throw Error(`HTTP ${response.status}：${typeof value.detail==='string'?value.detail:JSON.stringify(value.detail||value.error||'操作失败')}`);
  return value;
}
function error(exc){$('error').textContent=exc.message||String(exc);$('error').hidden=false;}
function resetPlan(){plan=null;$('confirmed').checked=false;$('planNote').textContent='';render();}
function planeZ(x,y){const p=state?.table_plane;return p?-(p.d_mm+p.normal_up[0]*x+p.normal_up[1]*y)/p.normal_up[2]:null;}
function targetHeight(){const t=target(),p=state?.table_plane;return p&&t.every(Number.isFinite)?p.normal_up.reduce((sum,n,i)=>sum+n*t[i],p.d_mm):null;}
function syncHeight(){
  const h=targetHeight();if(h===null)return;
  $('heightRange').max=Math.max(400,Math.ceil(h/25)*25);
  $('heightRange').value=Math.max(25,Math.round(h));
}
function applyHeight(x,y){
  x=Math.round(x*10)/10;y=Math.round(y*10)/10;
  const point=CoordinateView.atHeight(x,y,Number($('heightRange').value),state?.table_plane);
  if(!point)return;
  // Round upward so the 25mm slider stop cannot round below table clearance.
  for(const [i,id] of ['x','y','z'].entries())$(id).value=i===2?Math.ceil(point[i]*100)/100:point[i];
  targetInitialized=true;resetPlan();
}
function fillCurrent(){
  if(!state?.robot)return;
  for(const id of ['x','y','z'])$(id).value=state.robot[id].toFixed(2);
  targetInitialized=true;syncHeight();
  if(selectionMode==='point'&&state.table_plane)applyHeight(state.robot.x,state.robot.y);
  else resetPlan();
}
function setSelectionMode(mode){
  if(busy||state?.health?.pending)return;
  selectionMode=mode;
  $('selectionHint').textContent=mode==='point'?'点左侧桌面网格选择位置，再拖动滑块设置离桌高度；点图不会移动机械臂。':'输入机械臂基坐标XYZ，Z=0不是桌面。';
  if(mode==='point'){
    if(target().every(Number.isFinite)){syncHeight();applyHeight(...target().slice(0,2));}
    else fillCurrent();
  }
  resetPlan();
}
function render(){
  const h=state?.health||{},r=state?.robot,result=state?.result;
  const moving=Boolean(h.pending),locked=Array.isArray(h.torque_status)&&h.torque_status.length===6&&h.torque_status.every(v=>v===1);
  $('setupBadge').textContent=state?.setup_id||'当前布置';
  $('connection').textContent=h.connected?'机械臂已连接':'机械臂未连接';$('connection').className='status-chip '+(h.connected?'connected':'');
  $('motionMode').textContent=h.motion_enabled?'实物模式':'只读模式';
  $('readiness').textContent=h.fault?h.fault.message:state?.calibration_error?'无法读取已完成标定：'+state.calibration_error:
    !h.connected?'先连接机械臂，串口初始化可能使设备移动，请在现场看护。':moving?'动作执行中，正在读取实际反馈。':
    !locked?'请先确认六关节扭矩均已锁定。':!h.motion_enabled?'网页服务仍为只读，请按启动说明开启实物模式。':'已沿用现有标定；不需要棋盘或相机在线。';
  $('connect').hidden=Boolean(h.connected);$('connect').disabled=busy;
  $('recover').hidden=!h.fault;$('recover').disabled=busy||moving||!h.connected;
  for(const [id,k] of [['actualX','x'],['actualY','y'],['actualZ','z']])$(id).textContent=number(r?.[k]);
  $('height').textContent=typeof state?.height_mm==='number'?`当前名义末端离桌 ${number(state.height_mm)} mm。保持俯仰 ${number(r.tit*180/Math.PI)}°、旋转 ${number(r.r*180/Math.PI)}°、夹爪 ${number(r.g*180/Math.PI)}°。`:'';
  const t=target(),valid=t.every(Number.isFinite),height=targetHeight();
  const graphical=selectionMode==='point';
  $('pointMode').setAttribute('aria-pressed',String(graphical));$('numericMode').setAttribute('aria-pressed',String(!graphical));
  $('pointMode').disabled=$('numericMode').disabled=busy||moving;
  $('heightControl').hidden=!graphical;$('heightRange').disabled=busy||moving||!state?.table_plane;
  $('heightValue').textContent=number(height)+' mm';$('useCurrent').textContent=graphical?'从当前位置选点':'填入当前 XYZ';
  for(const id of ['x','y','z'])$(id).readOnly=graphical;
  $('targetHeight').textContent=height===null?'请输入机械臂基坐标XYZ。':`输入目标离桌 ${number(height)} mm；坐标验证先使用离桌至少25mm的点。`;
  $('prepare').disabled=busy||moving||!h.connected||!locked||!valid||Boolean(state?.calibration_error);
  const ready=plan&&valid&&plan.plan.target_xyz_mm.every((v,i)=>v===t[i])&&performance.now()<plan.expires;
  $('confirmed').disabled=busy||moving||!ready;
  $('move').disabled=busy||moving||!ready||!$('confirmed').checked||!locked||!h.connected||!h.motion_enabled||Boolean(h.fault);
  $('useCurrent').disabled=busy||moving||!r;
  for(const id of ['x','y','z'])$(id).disabled=busy||moving;
  $('disarm').hidden=!moving;
  if(plan){
    const hold=plan.plan.gripper_hold;
    const gripNote=hold?.normalized?`夹爪反馈 ${number(hold.feedback_deg)}°；保持指令取合法边界 ${number(hold.command_deg)}°。原始反馈保留，未扩大开合限位。`:'保持当前夹爪开度。';
    $('planNote').textContent=ready?`预览有效；移动距离 ${number(plan.plan.distance_mm)} mm。保持当前俯仰和旋转。${gripNote}`:'预览已失效，请重新预览。';
  }
  const labels={prepared:'目标预览已保存，尚未发送运动。',moving:'已发送一次T104，等待实际反馈…',already_at_target:'当前反馈已在目标点附近，没有发送运动。',
    feedback_within_tolerance:'反馈已稳定并进入6mm参考带，实际准确度由你验收。',settled_with_offset:'反馈已稳定，但与目标有偏差；本次停止在实际位置，没有重发。',
    timeout_unverified:'反馈未在等待窗口内确认；请核对实际状态。',fault:'出现设备故障，请核对实际状态。',transmission_uncertain:'指令传输结果不确定，已锁定后续动作。'};
  if(result){
    const lines=[labels[result.status]||result.status,`目标 XYZ：${coordinateText(result.target_xyz_mm||[])} mm`];
    if(result.actual_xyz_mm)lines.push(`反馈 XYZ：${coordinateText(result.actual_xyz_mm)} mm`,`反馈－目标：${coordinateText(result.actual_minus_target_mm)} mm`,`三维差值：${number(result.position_error_mm)} mm`);
    if(result.movement_observed===false)lines.push('尚未检测到明显的反馈位移；不能据此判定实际移动成功。');
    if(result.pose_error_deg)lines.push(`姿态差：俯仰 ${number(result.pose_error_deg.pitch)}° / 旋转 ${number(result.pose_error_deg.roll)}° / 夹爪 ${number(result.pose_error_deg.gripper)}°`);
    $('summary').textContent=lines.join('\n');
    const finished=!['prepared','dispatching','moving','superseded'].includes(result.status);$('verification').hidden=!finished;
    if(result.plan_id!==verifiedID){verifiedID=result.plan_id;$('saved').textContent='';$('note').value='';document.querySelector('input[name=verified][value=uncertain]').checked=true;}
    if(result.physical_accuracy!=='not_checked')$('saved').textContent='实物验收已保存：'+({matches:'基本一致',offset:'有偏差',uncertain:'尚未确认'}[result.physical_accuracy]||'');
  }
  draw();
}
async function refresh(){
  if(polling||busy||document.hidden)return;polling=true;const rev=revision;
  try{
    const next=await api('/api/coordinates/state');if(rev!==revision)return;state=next;
    if(!targetInitialized&&selectionMode==='point'&&state.table_plane){
      const saved=state.result?.target_xyz_mm;
      if(Array.isArray(saved)&&saved.length===3&&saved.every(Number.isFinite)){
        for(const [i,id] of ['x','y','z'].entries())$(id).value=saved[i];targetInitialized=true;syncHeight();
      }else if(state.robot)fillCurrent();
    }
    render();
  }
  catch(exc){error(exc);}finally{polling=false;}
}
async function action(fn){if(busy)return;busy=true;revision++;$('error').hidden=true;render();try{await fn();}catch(exc){error(exc);}finally{busy=false;await refresh();render();}}
$('connect').onclick=()=>action(async()=>{await api('/api/auto-connect','POST',{});resetPlan();});
$('recover').onclick=()=>action(async()=>{await api('/api/recover','POST',{declaration:'已检查机械臂静止且运动区域安全'});resetPlan();});
$('useCurrent').onclick=fillCurrent;
$('pointMode').onclick=()=>setSelectionMode('point');$('numericMode').onclick=()=>setSelectionMode('numeric');
$('heightRange').oninput=()=>{if(!busy&&!state?.health?.pending&&target().slice(0,2).every(Number.isFinite))applyHeight(...target().slice(0,2));};
for(const id of ['x','y','z'])$(id).oninput=resetPlan;
$('confirmed').onchange=render;
$('prepare').onclick=()=>action(async()=>{resetPlan();const prepared=await api('/api/coordinates/prepare','POST',{xyz_mm:target()});plan={...prepared,expires:performance.now()+prepared.expires_in_s*1000};});
$('move').onclick=()=>{if($('move').disabled)return;const key=plan.plan_id;return action(async()=>{$('confirmed').checked=false;const result=await api('/api/coordinates/move','POST',{plan_id:key,declaration});state.result=result;state.health.pending=result.sent?{kind:'coordinate_test'}:null;plan=null;render();});};
$('disarm').onclick=async()=>{try{await api('/api/disarm','POST',{});$('summary').textContent+='\n已锁定后续运动；当前指令可能继续。';}catch(exc){error(exc);}};
$('saveCheck').onclick=()=>action(async()=>{await api('/api/coordinates/verify','POST',{plan_id:state.result.plan_id,result:document.querySelector('input[name=verified]:checked').value,note:$('note').value});});
// Draw a rotatable orthographic 3D schematic directly, with no CDN or SDK.
function draw(){
  const rect=canvas.getBoundingClientRect(),w=rect.width,h=rect.height,dpr=devicePixelRatio||1;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const project=point=>CoordinateView.project(point,{width:w,height:h,...view});
  function line(a,b,color,width=1,dash=[]){a=project(a);b=project(b);ctx.beginPath();ctx.setLineDash(dash);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.setLineDash([]);}
  function dot(point,color,r=5,label=''){const p=project(point);ctx.fillStyle=color;ctx.beginPath();ctx.arc(...p,r,0,Math.PI*2);ctx.fill();if(label){ctx.font='12px sans-serif';ctx.fillText(label,p[0]+9,p[1]-8);}}
  const table=state?.table_plane;
  if(table){
    const corners=[[-60,-260],[480,-260],[480,260],[-60,260]].map(([x,y])=>[x,y,planeZ(x,y)]);
    ctx.beginPath();corners.forEach((p,i)=>{const uv=project(p);i?ctx.lineTo(...uv):ctx.moveTo(...uv);});ctx.closePath();ctx.fillStyle='#eaf0ee';ctx.fill();
    for(let x=0;x<=450;x+=50)line([x,-250,planeZ(x,-250)],[x,250,planeZ(x,250)],'#d7e0de');
    for(let y=-250;y<=250;y+=50)line([0,y,planeZ(0,y)],[450,y,planeZ(450,y)],'#d7e0de');
    dot([430,230,planeZ(430,230)],'#80938e',2,'已标定桌面');line([0,0,planeZ(0,0)],[0,0,0],'#94a3ab',9);
  }
  for(const [axis,color,label] of [[[130,0,0],'#c64f4f','X'],[[0,130,0],'#4c9569','Y'],[[0,0,130],'#467ab5','Z']]){line([0,0,0],axis,color,2);dot(axis,color,2,label);}
  dot([0,0,0],'#45555e',5,'O');
  function arm(points,color,width,dash=[]){for(let i=1;i<points.length;i++)line(points[i-1],points[i],color,width,dash);for(const p of points)dot(p,color,width*.7);}
  if(plan)arm(plan.plan.predicted_joints_xyz_mm,'#9aa9b1',3,[4,4]);
  if(state?.model){arm(state.model.joints_xyz_mm,'#177d69',8);const tip=state.robot?['x','y','z'].map(k=>state.robot[k]):state.model.xyz_mm;dot(tip,'#177d69',6,'当前末端');}
  const t=target();if(t.every(Number.isFinite)){
    if(table){const foot=[t[0],t[1],planeZ(t[0],t[1])];dot(foot,'#cf7136',3);line(foot,t,'#cf7136',1,[2,4]);}
    dot(t,'#cf7136',6,'目标');if(state?.robot)line(['x','y','z'].map(k=>state.robot[k]),t,'#cf7136',2,[5,5]);
  }
  if(depth?.point?.valid)dot(depth.point.robot_xyz_mm,'#865bc0',5,'深度表面点');
}
let drag=null;
function pickTablePoint(event){
  if(selectionMode!=='point'||busy||state?.health?.pending)return;
  const rect=canvas.getBoundingClientRect(),point=CoordinateView.tablePoint([event.clientX-rect.left,event.clientY-rect.top],state?.table_plane,{width:rect.width,height:rect.height,...view});
  if(!point){$('selectionHint').textContent='这个视角无法确定桌面位置，请切换“三维视图”或“俯视XY”选点。';return;}
  if(point[0]<-60||point[0]>480||Math.abs(point[1])>260){$('selectionHint').textContent='请在桌面网格区域内选点。';return;}
  applyHeight(point[0],point[1]);$('selectionHint').textContent=`已选桌面位置 X=${number(point[0])}、Y=${number(point[1])} mm；拖动滑块选高度，再预览路径。`;
}
canvas.onpointerdown=e=>{if(e.button!==0)return;drag=[e.clientX,e.clientY,view.yaw,view.pitch,false];canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(!drag)return;if(Math.hypot(e.clientX-drag[0],e.clientY-drag[1])>4)drag[4]=true;if(!drag[4])return;view.yaw=drag[2]+(e.clientX-drag[0])*.008;view.pitch=Math.max(-1.5,Math.min(1.5,drag[3]+(e.clientY-drag[1])*.008));draw();};
canvas.onpointerup=e=>{const click=drag&&!drag[4];drag=null;if(click)pickTablePoint(e);};
canvas.onpointercancel=()=>{drag=null;};
canvas.addEventListener('wheel',e=>{e.preventDefault();view.zoom=Math.max(.5,Math.min(2.5,view.zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
$('view3d').onclick=()=>{Object.assign(view,{yaw:.8,pitch:.6,zoom:1});draw();};
$('viewTop').onclick=()=>{Object.assign(view,{yaw:0,pitch:Math.PI/2,zoom:1});draw();};
$('viewFront').onclick=()=>{Object.assign(view,{yaw:0,pitch:0,zoom:1});draw();};
new ResizeObserver(draw).observe(canvas);
$('readDepth').onclick=async()=>{
  $('readDepth').disabled=true;
  try{depth=await api('/api/coordinates/depth-snapshot');
    $('depthRGB').src=depth.rgb_url;$('depthImage').src=depth.depth_url;$('depthRGB').hidden=$('depthImage').hidden=false;
    $('depthSource').textContent=`非实时 · 录像 ${depth.trial_id.slice(0,10)} · 帧 ${depth.frame_index}`;
    $('depthLegend').textContent=`深度图 · ${number(depth.color_range_mm[0])}–${number(depth.color_range_mm[1])}mm · 有效 ${number(depth.valid_fraction*100)}% · 黑色为空洞`;
  }catch(exc){error(exc);}finally{$('readDepth').disabled=false;}
};
async function readDepthPoint(event){
  if(!depth)return;const snapshot=depth,rect=event.currentTarget.getBoundingClientRect(),[w,h]=snapshot.image_size;
  const u=Math.min(w-1,Math.max(0,Math.floor((event.clientX-rect.left)/rect.width*w))),v=Math.min(h-1,Math.max(0,Math.floor((event.clientY-rect.top)/rect.height*h)));
  try{const point=await api('/api/coordinates/depth-point','POST',{snapshot_id:snapshot.snapshot_id,u,v});if(depth!==snapshot)return;snapshot.point=point;
    $('depthPoint').textContent=point.valid?`像素 (${u}, ${v})：实测深度 ${number(point.depth_mm)} mm\n相机 XYZ：${coordinateText(point.camera_xyz_mm)} mm\n机械臂 XYZ：${coordinateText(point.robot_xyz_mm)} mm\n这是可见表面测量点，不是球心；未发送运动。`:point.reason;draw();
  }catch(exc){error(exc);}
}
$('depthRGB').onclick=$('depthImage').onclick=readDepthPoint;
document.addEventListener('visibilitychange',()=>{if(!document.hidden)refresh();});
setInterval(refresh,500);refresh();render();
