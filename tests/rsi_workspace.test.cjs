const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
class Element{
  constructor(){this.checked=false;this.disabled=false;this.hidden=false;this.textContent='';this.children=[];this.attrs={};}
  setAttribute(key,value){this.attrs[key]=String(value);}
  toggleAttribute(key,value){if(value)this.attrs[key]='';else delete this.attrs[key];if(key==='hidden')this.hidden=value;}
  replaceChildren(...items){this.children=items;}
  append(...items){this.children.push(...items);}
}
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function page({partial=false,fail=false}={}){
  const elements=new Map(['workspaceVisible','workspaceOverlay','workspaceStatus','workspaceRecompute'].map(id=>[id,new Element()]));
  const calls=[],parameters={pickup_xy_mm:[12,0],travel_speed:.65},geometry={K:[[600,0,640],[0,600,360],[0,0,1]]};
  const result={workspace_id:'b'.repeat(32),status:partial?'partial':'completed',grid:partial?['+?','??']:['++','--'],
    rows:2,columns:2,image_size:[1280,720],completed:partial?1:4,total:4,planned_samples:partial?1:2,
    parameters,selected_ball:{status:'planned',grasp_pitch_deg:80,transfer_height_mm:158},initial_xyz_mm:[346,0,223]};
  const context={document:{getElementById:id=>elements.get(id),createElementNS:()=>new Element(),addEventListener(){},dispatchEvent(){},hidden:false},
    window:{addEventListener(){}},Event:class{},setTimeout,clearTimeout,console,
    fetch:async(url,options={})=>{
      calls.push([options.method||'GET',url]);
      let data=url.endsWith('/observe')?{observation_id:'a'.repeat(32),scene:{camera_geometry:geometry}}:
        url.endsWith('/workspace')?{workspace_id:result.workspace_id}:result;
      const bad=fail&&url.endsWith('/workspace');if(bad)data={detail:'请先连接以读取起步姿态'};
      return {ok:!bad,status:bad?409:200,text:async()=>JSON.stringify(data)};
    }};
  vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'..','control_ui','rsi-workspace.js'),'utf8'),context);
  const state={workspace_preview_version:1,learning:{status:'idle'},job:{status:'idle'},
    plan:{plan_id:'previous',parameters:{travel_speed:.3,pickup_xy_mm:[0,0]},scene:{camera_geometry:geometry}}};
  context.window.RSIWorkspace.configure(state);
  return {elements,calls,result,parameters,geometry,state,api:context.window.RSIWorkspace};
}

test('workspace preview captures and computes without a run, reset, torque or authorization request',async()=>{
  const p=page(),box=p.elements.get('workspaceVisible');box.checked=true;box.onchange();
  assert.equal(p.api.isBusy(),true);await flush();await flush();
  assert.equal(p.api.isBusy(),false);assert.equal(p.elements.get('workspaceOverlay').hidden,false);
  assert.deepEqual(p.calls,[['POST','/api/bench/rsi/observe'],['POST','/api/bench/rsi/workspace'],['GET','/api/bench/rsi/workspace/'+'b'.repeat(32)]]);
  assert.match(p.elements.get('workspaceStatus').textContent,/完整抓放路径/);
  assert.equal(p.elements.get('workspaceOverlay').attrs.viewBox,'0 0 1280 720');
  box.checked=false;box.onchange();assert.equal(p.elements.get('workspaceOverlay').hidden,true);
});

test('an old displayed plan does not invalidate a current preview, while a new changed recipe does',async()=>{
  const p=page();p.elements.get('workspaceVisible').checked=true;p.elements.get('workspaceVisible').onchange();await flush();
  p.api.configure(p.state);assert.equal(p.elements.get('workspaceVisible').checked,true);
  const same={...p.state,plan:{plan_id:'next',parameters:{travel_speed:.65,pickup_xy_mm:[12,0]},scene:{camera_geometry:p.geometry}}};
  p.api.configure(same);assert.equal(p.elements.get('workspaceVisible').checked,true);
  p.api.configure({...same,plan:{...same.plan,plan_id:'changed',parameters:{...p.parameters,travel_speed:1}}});
  assert.equal(p.elements.get('workspaceVisible').checked,false);assert.equal(p.elements.get('workspaceOverlay').hidden,true);
  assert.match(p.elements.get('workspaceStatus').textContent,/重新计算/);assert.equal(p.calls.length,3);
});

test('uncomputed samples remain blank instead of being shaded as rejected',async()=>{
  const p=page({partial:true});p.elements.get('workspaceVisible').checked=true;p.elements.get('workspaceVisible').onchange();await flush();
  const paths=p.elements.get('workspaceOverlay').children;
  assert.equal(paths.filter(e=>e.attrs.d).length,1);assert.equal(paths.find(e=>e.attrs.d).attrs.fill,'#1c9477');
  assert.match(p.elements.get('workspaceStatus').textContent,/空白：未完成采样/);
});

test('a changed funnel or starting pose hides a workspace computed under different conditions',async()=>{
  for(const change of ['funnel','pose']){
    const p=page();p.result.initial_pitch_deg=0;p.result.initial_roll_rad=0;p.result.funnel_xyz_mm=[300,200,-70];
    p.elements.get('workspaceVisible').checked=true;p.elements.get('workspaceVisible').onchange();await flush();
    const scene={camera_geometry:p.geometry,funnel:{mouth_center_robot_mm:change==='funnel'?[300,250,-70]:[300,200,-70]}};
    const plan={plan_id:'changed',parameters:p.parameters,scene,initial_pose:{xyz_mm:change==='pose'?[346,0,180]:[346,0,223],pitch_rad:0,roll_rad:0}};
    p.api.configure({...p.state,plan});assert.equal(p.elements.get('workspaceVisible').checked,false);
    assert.equal(p.elements.get('workspaceOverlay').hidden,true);
  }
});

test('a preview input error is shown without starting any physical task',async()=>{
  const p=page({fail:true});p.elements.get('workspaceVisible').checked=true;p.elements.get('workspaceVisible').onchange();await flush();
  assert.equal(p.api.isBusy(),false);assert.equal(p.elements.get('workspaceVisible').checked,false);
  assert.match(p.elements.get('workspaceStatus').textContent,/读取起步姿态/);assert.equal(p.calls.length,2);
});
