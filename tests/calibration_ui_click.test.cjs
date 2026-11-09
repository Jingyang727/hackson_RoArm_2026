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
  replaceChildren(...children) {this.children=children}
  append(...children) {this.children.push(...children)}
  scrollIntoView() {this.scrolled=true}
  getContext() {return {
    clearRect() {},drawImage() {},fillText() {},strokeText() {},
    beginPath() {},arc() {},moveTo() {},lineTo() {},fill() {},stroke() {}
  }}
}

function makePage() {
  const html = fs.readFileSync(path.join(__dirname,"..","control_ui","calibration.html"),"utf8");
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
  const elements = new Map(ids.map(id=>[id,new FakeElement(id)]));
  const script = html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
  assert.ok(script,"inline JS must exist");
  let torque = 1;
  let teachActive = false;
  let torquePosts = 0;
  let statusGets = 0;
  const requests = [];
  const apiFetch = async (url,opts={}) => {
    const method=opts.method||"GET";
    requests.push(method+" "+url);
    let data;
    if(url==="/api/ports") data=[{device:"/dev/mock-usb"}];
    else if(url==="/api/health") data={
      connected:true,port:"/dev/mock-usb",teach_enabled:true,
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
  vm.runInNewContext(script,sandbox,{filename:"calibration.html",timeout:5000});
  const el=id=>{
    assert.ok(elements.has(id),"DOM id missing: "+id);
    return elements.get(id);
  };
  return {el,requests,torquePosts:()=>torquePosts,statusGets:()=>statusGets};
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
  assert.match(page.el("torqueMessage").textContent,/尚未发送/);
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
