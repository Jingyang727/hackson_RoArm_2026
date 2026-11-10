/* Browser DOM smoke test: verifies calibration torque buttons actually bind
 * and change visible UI state. Mock serial backend: NEVER sends hardware commands.
 */
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

class FakeElement {
  constructor(id) {
    this.id = id;
    this.disabled = false;
    this.hidden = id === "torquePrompt";
    this.checked = false;
    this.textContent = "";
    this.value = id === "square" ? "24" : "";
    this.children = [];
    this.style = {};
    this.events = {};
    this.classList = {add() {},remove() {},contains() {return false}};
  }
  addEventListener(type,callback) {this.events[type]=callback}
  setAttribute(name,value) {this[name]=value}
  replaceChildren(...children) {this.children=children}
  append(...children) {this.children.push(...children)}
  scrollIntoView() {this.scrolled=true}
  getContext() {return {
    clearRect() {},drawImage() {},fillText() {},strokeText() {},
    beginPath() {},arc() {},moveTo() {},lineTo() {},fill() {},stroke() {}
  }}
}

function makePage({initiallyConnected=true,previousSession=null}={}) {
  const html = fs.readFileSync(path.join(__dirname,"..","control_ui","calibration.html"),"utf8");
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
  const elements = new Map(ids.map(id=>[id,new FakeElement(id)]));
  const script = html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
  assert.ok(script,"inline JS must exist");
  let torque = 1;
  let connected=initiallyConnected;
  let autoConnectCalls=0;
  let teachActive = false;
  let torquePosts = 0;
  let statusGets = 0;
  let rejectedPoint="P3";
  let recordedPoints=[];
  const captures=[];
  const requests = [];
  const apiFetch = async (url,opts={}) => {
    const method=opts.method||"GET";
    requests.push(method+" "+url);
    let data;
    if(url==="/api/ports") data=[{device:"/dev/mock-usb"}];
    else if(url==="/api/calibration/status") data=previousSession || {
      session_id:null,records:[],square_mm:24,fit:null,ready:false
    };
    else if(url==="/api/auto-connect"){
      autoConnectCalls++;connected=true;
      data={connected:true,port:"/dev/mock-usb",reused:false};
    }
    else if(url==="/api/health") data={
      connected,port:connected?"/dev/mock-usb":null,teach_enabled:true,
      teach_active:teachActive,torque_status:Array(6).fill(torque),
      pending:null,fault:null,
    };
    else if(url==="/api/status") {
      statusGets++;
      data={
        T:1051,x:200,y:30,z:120,
        ...Object.fromEntries([...("BSETRG")].map(k=>["torswitch"+k,torque]))
      };
    }
    else if(url==="/api/calibration/capture") {
      const req=JSON.parse(opts.body);
      captures.push(req.point_id);
      if(req.point_id===rejectedPoint){
        return {ok:false,status:409,json:async()=>({detail:
          "末端姿态与 P1 不一致：Pitch 相差 11.0°，Roll 相差 0.0°（上限均为5°）"
        })};
      }
      recordedPoints=recordedPoints.filter(r=>r.id!==req.point_id);
      recordedPoints.push({
        id:req.point_id,grid:[req.col,req.row],
        robot_xyz_mm:[200+req.col,30+req.row,110]
      });
      data={records:recordedPoints.slice(),ready:false,fit:null,
            message:"还有待完成的标定点"};
    }
    else if(url==="/api/teach/torque") {
      const payload=JSON.parse(opts.body);
      assert.equal(method,"POST");
      assert.ok(payload.declaration.includes("已托住机械臂"));
      torquePosts++;
      torque=payload.enabled?1:0;
      teachActive=!payload.enabled;
      data={
        verified:true,command_sent:true,verification_attempts:1,
        message:"模拟扭矩操作成功",torque_state:Array(6).fill(torque)
      };
    }
    else throw Error("unexpected network request "+url);
    return {ok:true,json:async()=>data};
  };
  const sandbox={
    document:{
      getElementById:id=>elements.get(id)||null,
      createElement:tag=>new FakeElement(tag),
    },
    window:{addEventListener() {}},
    navigator:{mediaDevices:{enumerateDevices:async()=>[]}},
    fetch:apiFetch,console,
    setInterval:()=>0,
    requestAnimationFrame:()=>0,cancelAnimationFrame:()=>{},
    confirm:()=>true,
  };
  vm.createContext(sandbox);
  vm.runInContext(script,sandbox,{filename:"calibration.html",timeout:5000});
  const el=id=>{
    assert.ok(elements.has(id),"DOM id missing: "+id);
    return elements.get(id);
  };
  return {
    el,requests,
    torquePosts:()=>torquePosts,statusGets:()=>statusGets,
    autoConnectCalls:()=>autoConnectCalls,
    primeSession:()=>vm.runInContext('session="test-session";summary={records:[],ready:false,fit:null,message:""};vision={checkerboard:{detected:true,corners_px:Array.from({length:54},(_,i)=>[100+(i%9)*40,100+Math.floor(i/9)*40])}};shot={width:1920,height:1080};render()',sandbox),
    refresh:()=>vm.runInContext("render()",sandbox),
    pointRow:(index)=>el("targets").children[index],
    captured:()=>captures.slice(),
    allowPoint:(point)=>{if(rejectedPoint===point)rejectedPoint=null},
  };
}
const flush=()=>new Promise(resolve=>setImmediate(resolve));

test("torque release button opens in-page confirmation and sends nothing early",async()=>{
  const page=makePage();
  await flush();await flush();
  assert.equal(page.el("torqueOff").disabled,false);
  assert.equal(typeof page.el("torqueOff").onclick,"function");
  page.el("torqueOff").onclick();
  assert.equal(page.el("torquePrompt").hidden,false);
  assert.match(page.el("torqueActionFeedback").textContent,/释放所有关节/);
  assert.match(page.el("torqueMessage").textContent,/尚未发送|还没有发送/);
  assert.equal(page.torquePosts(),0);
  page.el("torqueAcknowledged").checked=true;
  page.el("torqueAcknowledged").onchange();
  assert.equal(page.el("torqueConfirm").disabled,false);
  await page.el("torqueConfirm").onclick();
  assert.equal(page.torquePosts(),1);
  assert.match(page.el("torqueMessage").textContent,/确认成功/);
});

test("read actual status button reports result locally even before torque release",async()=>{
  const page=makePage();
  await flush();await flush();
  assert.equal(page.el("torqueRefresh").disabled,false);
  const before=page.statusGets();
  await page.el("torqueRefresh").onclick();
  assert.ok(page.statusGets()>before);
  assert.match(page.el("torqueActionFeedback").textContent,/读取实际扭矩状态/);
  assert.match(page.el("torqueMessage").textContent,/六关节全部锁定/);
  assert.equal(page.torquePosts(),0);
});

test("disabled relock button becomes enabled after confirmed release",async()=>{
  const page=makePage();
  await flush();await flush();
  assert.equal(page.el("torqueOn").disabled,true);
  page.el("torqueOff").onclick();
  page.el("torqueAcknowledged").checked=true;
  page.el("torqueAcknowledged").onchange();
  await page.el("torqueConfirm").onclick();
  assert.equal(page.el("torqueOn").disabled,false);
  page.el("torqueOn").onclick();
  assert.equal(page.el("torquePrompt").hidden,false);
  assert.match(page.el("torquePromptTitle").textContent,/重新锁定/);
  assert.equal(page.torquePosts(),1); // confirmation required again
});

test("new calibration tab automatically reuses/opens single shared serial connection",async()=>{
  const connectedPage=makePage();
  await flush();await flush();await flush();
  assert.equal(connectedPage.autoConnectCalls(),0);
  assert.equal(connectedPage.el("connectionToast").hidden,true);
  assert.match(connectedPage.el("connectionChip").textContent,/已连接/);

  const freshPage=makePage({initiallyConnected:false});
  await flush();await flush();await flush();await flush();
  assert.equal(freshPage.autoConnectCalls(),1);
  assert.equal(freshPage.el("connectionToast").hidden,true);
  assert.match(freshPage.el("connectionChip").textContent,/已连接/);
  assert.ok(!freshPage.requests.includes("POST /api/connect"),
    "page should use backend shared auto-connect, not repeatedly POST manual connect");
});

test("P1 P2 remain recorded; P3 reject shows local reason and retry succeeds; P4 and V1 can save",async()=>{
 const page=makePage();
 await flush();await flush();await flush();
 page.primeSession();
 assert.equal(page.el("targets").children.length,5);
 const record=index=>page.pointRow(index).children[1].children[1];
 const feedback=index=>page.pointRow(index).children[2].textContent;
 const stableP3=record(2);
 for(const index of [0,1]) await record(index).onclick();
 assert.match(page.pointRow(0).children[0].textContent,/✓/);
 assert.match(page.pointRow(1).children[0].textContent,/✓/);

 // Simulate periodic /api/health repaint DURING a mouse press; same DOM node
 // must survive, or the browser may discard the click.
 page.refresh();page.refresh();
 assert.strictEqual(record(2),stableP3);
 assert.strictEqual(record(4),page.pointRow(4).children[1].children[1]);

 await record(2).onclick();
 assert.match(feedback(2),/Pitch 相差 11\.0/);
 assert.match(page.el("captureStatus").textContent,/P3 记录失败/);
 assert.doesNotMatch(page.pointRow(2).children[0].textContent,/✓/);
 assert.match(page.pointRow(0).children[0].textContent,/✓/);
 assert.match(page.pointRow(1).children[0].textContent,/✓/);

 page.allowPoint("P3");
 for(const index of [2,3,4])await record(index).onclick();
 assert.deepEqual(page.captured(),["P1","P2","P3","P3","P4","V1"]);
 for(let i=0;i<5;i++){
   assert.match(page.pointRow(i).children[0].textContent,/✓/, "missing checkmark "+i);
   assert.match(feedback(i),/已记录/);
 }
 assert.match(page.el("captureStatus").textContent,/V1 记录成功/);
});

test("existing P1 and P2 are recovered after page refresh without starting a new session",async()=>{
 const oldRecords=[
   {id:"P1",grid:[1,1],robot_xyz_mm:[210,30,110]},
   {id:"P2",grid:[7,1],robot_xyz_mm:[245,30,110]},
 ];
 const page=makePage({previousSession:{
   session_id:"existing-session",records:oldRecords,
   next_id:"P3",square_mm:24,image_size:[1920,1080],
   message:"继续标定",fit:null,ready:false
 }});
 for(let i=0;i<5;i++)await flush();
 assert.match(page.pointRow(0).children[0].textContent,/✓/);
 assert.match(page.pointRow(1).children[0].textContent,/✓/);
 assert.equal(page.pointRow(2).children[1].children[1].disabled,true,
   "recovered session must require a fresh board scan before capturing more points");
 assert.equal(page.requests.filter(v=>v==="POST /api/calibration/start").length,0);
});
