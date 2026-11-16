const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
function page({fail=false}={}){
  const ids=['travelSpeed','approachSpeed','speedFast','saveSpeed','reloadSpeed','speedSummary','speedStatus'];
  const elements=new Map(ids.map(id=>[id,{value:'',textContent:'',disabled:true}])),calls=[],events=[];
  const context={window:{},Event:class{constructor(type){this.type=type;}},
    document:{getElementById:id=>elements.get(id),dispatchEvent:event=>events.push(event.type)},
    fetch:async(url,options)=>{
      const body=JSON.parse(options.body);calls.push([url,body]);
      return {ok:!fail,status:fail?409:200,text:async()=>JSON.stringify(fail?{detail:'策略已变化'}:{...body,policy_version:body.policy_version+1})};
    }};
  vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'..','control_ui','rsi-speed.js'),'utf8'),context);
  const state={speed_settings_version:1,speed_settings:{policy_version:7,calibration_fingerprint:'calibration',travel_speed:.65,approach_speed:.3},learning:{status:'idle'},job:{status:'idle'}};
  context.window.RSISpeed.configure(state);
  return {elements,calls,events,state,api:context.window.RSISpeed};
}
test('loading speed settings is read only and fast is an explicit saved configuration',async()=>{
  const p=page();assert.equal(p.calls.length,0);assert.equal(p.elements.get('travelSpeed').value,'0.65');
  p.elements.get('speedFast').onclick();assert.equal(p.api.isDirty(),true);assert.equal(p.calls.length,0);
  await p.elements.get('saveSpeed').onclick();
  assert.deepEqual(p.calls,[['/api/bench/rsi/speed',{travel_speed:1,approach_speed:.5,policy_version:7,calibration_fingerprint:'calibration'}]]);
  assert.equal(p.api.isDirty(),false);assert.match(p.elements.get('speedStatus').textContent,/速度已保存/);
  p.api.configure(p.state);assert.equal(p.elements.get('travelSpeed').value,'1'); // Late old poll cannot undo a saved value.
});
test('polls preserve edits and a newer policy requires reload before saving',()=>{
  const p=page();p.elements.get('speedFast').onclick();p.api.configure(p.state);
  assert.equal(p.elements.get('travelSpeed').value,'1');assert.equal(p.elements.get('saveSpeed').disabled,false);
  p.api.configure({...p.state,speed_settings:{...p.state.speed_settings,policy_version:8,travel_speed:.55}});
  assert.equal(p.elements.get('saveSpeed').disabled,true);assert.match(p.elements.get('speedStatus').textContent,/策略已变化/);
  p.elements.get('reloadSpeed').onclick();assert.equal(p.elements.get('travelSpeed').value,'0.55');assert.equal(p.api.isDirty(),false);
});
test('task execution or unavailable configuration disables speed changes',async()=>{
  const p=page();p.elements.get('speedFast').onclick();p.api.configure({...p.state,learning:{status:'running'}});
  assert.equal(p.elements.get('travelSpeed').disabled,true);await p.elements.get('saveSpeed').onclick();assert.equal(p.calls.length,0);
  p.api.configure({});assert.equal(p.elements.get('speedFast').disabled,true);assert.equal(p.calls.length,0);
});
test('failed saves retain the draft and show the actual failure',async()=>{
  const p=page({fail:true});p.elements.get('speedFast').onclick();await p.elements.get('saveSpeed').onclick();
  assert.equal(p.api.isDirty(),true);assert.equal(p.elements.get('travelSpeed').value,'1');
  assert.match(p.elements.get('speedStatus').textContent,/保存失败：策略已变化/);
});
test('invalid numeric input cannot be saved',()=>{
  const p=page();for(const value of ['',0,1.1,'NaN']){
    p.elements.get('travelSpeed').value=String(value);p.elements.get('travelSpeed').oninput();
    assert.equal(p.elements.get('saveSpeed').disabled,true);assert.equal(p.api.isDirty(),true);
  }
});
