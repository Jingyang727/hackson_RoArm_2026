/* Browser pick-page smoke tests. All robot/serial commands are mocked. */
const test=require("node:test");
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const vm=require("node:vm");

class FakeElement{
  constructor(id){
    this.id=id;this.style={};this.disabled=false;this.hidden=id==="pickTorquePrompt";
    this.checked=false;this.textContent="";this.value="";this.dataset={};
    this.videoWidth=1920;this.videoHeight=1080;this.children=[];
    this.classList={toggle(){},add(){},remove(){}};
  }
  addEventListener(){}
  play(){return Promise.resolve()}
  getContext(){return {
    drawImage(){},beginPath(){},arc(){},stroke(){},fillText(){},clearRect(){}
  }}
  toBlob(callback){callback({type:"image/png",byteLength:20})}
}
const flush=()=>new Promise(r=>setImmediate(r));
function loadPage(db,initialTorque=1){
  const html=fs.readFileSync(path.join(__dirname,"..","control_ui","pick.html"),"utf8");
  const script=html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
  assert.ok(script);
  const elems=new Map([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>[m[1],new FakeElement(m[1])]));
  const pointButtons=["grab","travel","place","open","close"].map(kind=>{
    const b=new FakeElement(kind);b.dataset.teach=kind;return b;
  });
  const requests=[];
  let torque=initialTorque;
  let sendCount=0;
  const fetcher=async(url,opts={})=>{
    const method=opts.method||"GET";
    requests.push(method+" "+url);
    let data;
    if(url==="/api/health")data={
      connected:true,port:"/dev/mock",teach_enabled:true,teach_active:torque===0,
      torque_status:Array(6).fill(torque),fault:null,pending:null
    };
    else if(url==="/api/pick/execution")data={
      has_session:false,session_id:null,next_index:0,total:0,next_step:null,
      hardware_execution_enabled:false,arm_pending:null,arm_fault:null
    };
    else if(url==="/api/pick/profile")data={
      profile:db.profile||{},saved:Boolean(db.saved),
      source:db.profile?"/mock/pick_profile.json":null
    };
    else if(url.startsWith("/api/vision/analyze?")){
      const hasBall=db.ballPresent!==false;
      data={
        image:{width:1920,height:1080},
        checkerboard:{
          detected:true,source:"reference_partial_verified",
          corners_px:Array.from({length:54},(_,i)=>[200+(i%9)*25,200+Math.floor(i/9)*25])
        },
        ball:hasBall?{
          detected:true,center_px:[310.5,269],radius_px:26.5,
          board_mm_xy:[96,60],inside_inner_board:true
        }:{detected:false},
        warnings:[]
      };
    }
    else if(url==="/api/pick/preview"){
      if(db.previewError){
        return {ok:false,json:async()=>({detail:db.previewError})};
      }
      data={
        ball_robot_xy_mm:[250,-21],place_robot_xy_mm:[230,10],
        number_of_steps:1,requirements:["逐段人工确认"],
        waypoints:[{stage:"hover_ball",kind:"cartesian",
                    xyz_mm:[250,-21,140],reason:"仅预览"}]
      };
    }
    else if(url==="/api/pick/teach")data={
      x_mm:200,y_mm:20,z_mm:110,pitch_rad:0.1,roll_rad:0.05,gripper_rad:1.5
    };
    else if(url==="/api/pick/draft"){
      db.profile=JSON.parse(opts.body).values;
      data={saved:true,path:"/mock/pick_profile_draft.json"};
    }
    else if(url==="/api/teach/torque"){
      const body=JSON.parse(opts.body);
      assert.match(body.declaration,/已托住机械臂/);
      assert.equal(method,"POST");
      torque=body.enabled?1:0;
      sendCount+=1;
      data={verified:true,torque_state:Array(6).fill(torque),message:"已确认",
            command_sent:true,verification_attempts:1};
    }
    else if(url==="/api/status"){
      data={T:1051,...Object.fromEntries("BSETRG".split("").map(k=>["torswitch"+k,torque]))};
    }else throw Error("Unexpected request "+method+" "+url);
    return {ok:true,json:async()=>data};
  };
  const sandbox={
    fetch:fetcher,console,
    document:{
      getElementById:id=>elems.get(id)||null,
      querySelectorAll:()=>pointButtons
    },
    navigator:{mediaDevices:{getUserMedia:async()=>({
      getTracks:()=>[{stop(){}}],getVideoTracks:()=>[{getSettings:()=>({})}]
    })}},
    window:{addEventListener(){}},
    setInterval:()=>0,
    confirm:()=>true
  };
  vm.runInNewContext(script,sandbox,{filename:"pick.html",timeout:5000});
  const el=id=>{const value=elems.get(id);assert.ok(value,"Missing "+id);return value};
  return {el,buttons:pointButtons,requests,sendCount:()=>sendCount};
}

test("pick torque buttons require confirmation and are fully local to the page",async()=>{
 const p=loadPage({});
 for(let i=0;i<4;i++)await flush();
 assert.equal(p.el("pickTorqueRelease").disabled,false);
 assert.equal(p.el("pickTorqueLock").disabled,true);
 p.el("pickTorqueRelease").onclick();
 assert.equal(p.el("pickTorquePrompt").hidden,false);
 assert.equal(p.sendCount(),0);
 p.el("pickTorqueAck").checked=true;p.el("pickTorqueAck").onchange();
 assert.equal(p.el("pickTorqueConfirm").disabled,false);
 await p.el("pickTorqueConfirm").onclick();
 assert.equal(p.sendCount(),1);
 assert.match(p.el("pickTorqueState").textContent,/全部释放/);
 assert.equal(p.el("pickTorqueLock").disabled,false);
 p.el("pickTorqueLock").onclick();
 assert.equal(p.sendCount(),1);
 p.el("pickTorqueAck").checked=true;p.el("pickTorqueAck").onchange();
 await p.el("pickTorqueConfirm").onclick();
 assert.equal(p.sendCount(),2);
 assert.match(p.el("pickTorqueState").textContent,/全部锁定/);
 assert.ok(!p.requests.includes("POST /api/connect"));
});

test("read gripper pose autosaves draft; next page load restores it",async()=>{
 const db={};
 const p=loadPage(db);
 for(let i=0;i<4;i++)await flush();
 assert.equal(p.buttons[0].disabled,false);
 await p.buttons[0].onclick();
 assert.equal(db.profile.grip_z_mm,110);
 assert.equal(db.profile.pitch_rad,0.1);
 assert.match(p.el("pickSaveState").textContent,/自动保存/);
 const second=loadPage(db);
 for(let i=0;i<4;i++)await flush();
 assert.match(second.el("grabShow").textContent,/110 mm/);
 assert.match(second.el("pickSaveState").textContent,/已恢复/);
 assert.ok(!second.requests.includes("POST /api/connect"));
});

test("one click opens camera, detects ball, and previews when saved profile exists",async()=>{
 const p=loadPage({saved:true,profile:{
   grip_z_mm:-116.18,place_z_mm:-103.87,travel_z_mm:-38.82,
   place_xy_mm:[230,10],pitch_rad:1.24,roll_rad:0.01,
   open_g_rad:2.34,close_g_rad:2.53,ball_diameter_mm:34.7
 }});
 for(let i=0;i<5;i++)await flush();
 assert.equal(p.el("preview").disabled,false,"preview should never be grey solely because ball not detected");
 await p.el("preview").onclick();
 assert.ok(p.requests.includes("POST /api/pick/preview"));
 assert.ok(p.requests.some(x=>x.startsWith("POST /api/vision/analyze?square_mm=24&use_reference=true")));
 assert.match(p.el("previewStatus").textContent,/共 1 步/);
 assert.match(p.el("waypoints").textContent,/hover_ball/);
 assert.equal(p.el("preview").disabled,false);
 assert.ok(!p.requests.includes("POST /api/pick/step"));
});

test("ball recognition failure gives useful guidance and preview button remains clickable",async()=>{
 const p=loadPage({saved:true,ballPresent:false,profile:{grip_z_mm:100}});
 for(let i=0;i<5;i++)await flush();
 assert.equal(p.el("preview").disabled,false);
 await p.el("preview").onclick();
 assert.match(p.el("previewStatus").textContent,/未检测到黄色圆形球/);
 assert.match(p.el("ballResult").textContent,/没有检测到黄色球/);
 assert.equal(p.el("preview").disabled,false);
 assert.ok(!p.requests.includes("POST /api/pick/preview"));
});

test("unsafe place is rejected by planner instead of bypassing reachability",async()=>{
 const p=loadPage({saved:true,previewError:"放置位置距基座水平半径396.3mm，超出当前受控区70～370mm"});
 for(let i=0;i<5;i++)await flush();
 await p.el("preview").onclick();
 assert.match(p.el("previewStatus").textContent,/396.3mm/);
 assert.equal(p.el("preview").disabled,false);
 assert.ok(!p.requests.includes("POST /api/pick/step"));
});
