/* Reproduce the real /teach unlock click with non-JSON HTTP 500 and JSON 409. */
const test=require("node:test");
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const vm=require("node:vm");

class Element {
 constructor(id){this.id=id;this.value="";this.children=[];this.textContent="";
  this.classList={toggle(){}};this.dataset={};this.style={};}
 replaceChildren(...items){this.children=items}
 append(...items){this.children.push(...items)}
}

async function pageWithUnlockResponse(status,raw){
 const html=fs.readFileSync(path.join(__dirname,"..","control_ui","fast_teach.html"),"utf8");
 const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
 const elements=new Map([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>[m[1],new Element(m[1])]));
 const calls=[];
 const health={connected:true,motion_enabled:true,armed:false,pending:null,fault:null,
  teach_active:false,torque_status:[1,1,1,1,1,1],allowed_joints:[1],
  joints:[{id:1,label:"底座",field:"b",min:-75,max:75}]};
 const context={document:{getElementById:id=>elements.get(id),
  createElement:id=>new Element(id),querySelectorAll:()=>[]},
  setInterval:()=>1,console,
  fetch:async(url,options={})=>{
   calls.push((options.method||"GET")+" "+url);
   if(url==="/api/unlock-once")return {ok:false,status,text:async()=>raw};
   let data;
   if(url==="/api/health")data=health;
   else if(url==="/api/status")data={x:200,y:0,z:100,b:0,tit:0,r:0,g:2};
   else if(url==="/api/teach/poses")data={poses:{}};
   else throw Error("unexpected endpoint "+url);
   return {ok:true,status:200,text:async()=>JSON.stringify(data)};
  }};
 vm.runInNewContext(script,context,{filename:"fast_teach.html"});
 for(let i=0;i<8;i++)await new Promise(resolve=>setImmediate(resolve));
 await elements.get("unlock").onclick();
 return {message:elements.get("motionState").textContent,calls};
}

test("unlock plain 500 shows endpoint/status instead of JSON syntax error and never retries",async()=>{
 const page=await pageWithUnlockResponse(500,"Internal Server Error");
 assert.match(page.message,/HTTP 500/);
 assert.match(page.message,/\/api\/unlock-once/);
 assert.doesNotMatch(page.message,/Unexpected token/);
 assert.equal(page.calls.filter(c=>c==="POST /api/unlock-once").length,1);
 assert.ok(!page.calls.some(c=>c.includes("/api/teach/move")||c.includes("/api/recover")));
});

test("unlock JSON 409 exposes original serial driver detail",async()=>{
 const page=await pageWithUnlockResponse(409,JSON.stringify({detail:"串口清空接收缓冲区失败：termios.error (6, 'Device not configured')"}));
 assert.match(page.message,/HTTP 409/);
 assert.match(page.message,/Device not configured/);
 assert.match(page.message,/termios.error/);
});
