/* EvoArm dashboard startup smoke tests, no actual USB, camera or motor access. */
const test=require("node:test");
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const vm=require("node:vm");

class Element {
 constructor(id){
  this.id=id;this.value=id==="stepSize"?"5":id==="gripperStep"?"15":id==="homeKind"?"saved":"";
  this.textContent="";this.disabled=false;this.hidden=id==="connectionToast";
  this.style={};this.dataset={};this.children=[];this.listeners={};
  this.classList={add(x){this.active?.add(x)},remove(x){this.active?.delete(x)},active:new Set()};
 }
 append(...args){this.children.push(...args)}
 replaceChildren(...args){this.children=args}
 addEventListener(name,callback){this.listeners[name]=callback}
}

function makeDashboard(initiallyConnected){
 const html=fs.readFileSync(path.join(__dirname,"..","control_ui","index.html"),"utf8");
 const js=html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
 assert.ok(js,"dashboard script must exist");
 const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
 const elements=new Map(ids.map(id=>[id,new Element(id)]));
 const events=[];
 let connected=initiallyConnected;
 const api=async (url,options={})=>{
  const method=options.method||"GET";
  events.push(method+" "+url);
  let data;
  if(url==="/api/auto-connect"){
   connected=true;data={connected:true,port:"/dev/cu.usbserial-130",reused:false};
  }else if(url==="/api/health"){
   data={
    connected,port:connected?"/dev/cu.usbserial-130":null,
    teach_enabled:false,teach_active:false,
    motion_enabled:false,armed:false,pending:null,fault:null,
    last_command:null,arm_seconds_left:0,allowed_joints:[1,2,3,4,5,6],
    joints:[{id:1,field:"b",label:"底座",min:-75,max:75},
            {id:2,field:"s",label:"肩部",min:-50,max:50},
            {id:3,field:"e",label:"肘部",min:35,max:145},
            {id:4,field:"t",label:"腕俯仰",min:-60,max:60},
            {id:5,field:"r",label:"腕旋转",min:-90,max:90},
            {id:6,field:"g",label:"夹爪",min:65,max:180}],
   };
  }else if(url==="/api/ports"){
   data=[{device:"/dev/cu.usbserial-130",description:"Mock Arm"}];
  }else if(url==="/api/status"){
   data={T:1051,x:200,y:40,z:110,b:0,s:0,e:1.57,t:0,r:0,g:2.1};
  }else if(url.startsWith("/api/home/preview")){
   data={available:false,rows:[]};
  }else throw Error("Unexpected request "+url);
  return {ok:true,json:async()=>data};
 };
 const context={
  document:{
   getElementById:(id)=>elements.get(id)||null,
   createElement:id=>new Element(id),
   querySelectorAll:()=>[],
  },
  window:{addEventListener(){}},
  fetch:api,setInterval:()=>1,console,confirm:()=>true
 };
 vm.runInNewContext(js,context,{filename:"index.html",timeout:5000});
 return {events,element:id=>elements.get(id)};
}
const tick=()=>new Promise(resolve=>setImmediate(resolve));

test("dashboard reuses existing serial without POSTing a connect",async()=>{
 const page=makeDashboard(true);
 for(let i=0;i<8;i++)await tick();
 assert.ok(!page.events.some(x=>x.includes("auto-connect")));
 assert.ok(!page.events.includes("POST /api/connect"));
 assert.match(page.element("connectionChip").textContent,/已连接/);
 assert.equal(page.element("connectionToast").hidden,true);
 assert.equal(page.element("port").value,"/dev/cu.usbserial-130");
});

test("dashboard automatically attaches only once when initially offline",async()=>{
 const page=makeDashboard(false);
 for(let i=0;i<8;i++)await tick();
 assert.equal(page.events.filter(x=>x==="POST /api/auto-connect").length,1);
 assert.ok(!page.events.includes("POST /api/connect"));
 assert.equal(page.element("connectionToast").hidden,true);
 assert.match(page.element("connectionChip").textContent,/已连接/);
});
