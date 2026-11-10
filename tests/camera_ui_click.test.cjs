/* Camera-page browser smoke test: mocked camera and API, no robot access. */
const test=require("node:test");
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const vm=require("node:vm");
class Node {
 constructor(id){
  this.id=id;this.value="";this.textContent="";this.disabled=false;
  this.style={};this.children=[];this.videoWidth=1920;this.videoHeight=1080;
 }
 append(child){this.children.push(child)}
 replaceChildren(...children){this.children=children}
 play(){return Promise.resolve()}
 getContext(){return {drawImage(){},beginPath(){},arc(){},fill(){},stroke(){},
   moveTo(){},lineTo(){},fillText(){}}}
 toBlob(callback){callback({type:"image/png"})}
}
const tick=()=>new Promise(resolve=>setImmediate(resolve));
test("camera page uses readable light colors and restores saved board size before detecting ball",async()=>{
 const html=fs.readFileSync(path.join(__dirname,"..","control_ui","camera.html"),"utf8");
 assert.match(html,/main\{[^}]*background:#fff;/);
 assert.match(html,/body\{[^}]*background:#f5f7fa;/);
 assert.ok(!html.includes("background:#1a293f"),"regression: unreadable dark card restored");
 assert.ok(html.includes('/assets/theme.css'));
 const code=html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
 assert.ok(code);
 const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
 const elems=new Map(ids.map(id=>[id,new Node(id)]));
 let analyzed=0,opened=0;
 const mockStream={
  getTracks:()=>[{stop(){}}],
  getVideoTracks:()=>[{getSettings:()=>({deviceId:"usb",frameRate:30})}]
 };
 const fetcher=async url=>{
  let data;
  if(url==="/api/calibration/status")data={saved:true,ready:true,square_mm:24};
  else if(url.includes("/api/vision/analyze")){
   analyzed++;
   assert.ok(url.includes("use_reference=true"));
   assert.ok(url.includes("square_mm=24"));
   data={
    image:{width:1920,height:1080},
    checkerboard:{detected:true,source:"reference_partial_verified",
      corners_count:54,corners_px:Array.from({length:54},(_,i)=>[250+(i%9)*25,300+Math.floor(i/9)*25])},
    ball:{detected:true,center_px:[340,370],radius_px:22,
      inside_inner_board:true,board_grid_xy:[3,2],board_mm_xy:[72,48]},
    warnings:[]
   };
  }else throw Error("Unexpected HTTP "+url);
  return {ok:true,json:async()=>data};
 };
 const mock={
  fetch:fetcher,console,
  document:{
   getElementById:id=>elems.get(id)||null,
   createElement:type=>new Node(type)
  },
  navigator:{
   permissions:{query:async()=>({state:"granted"})},
   mediaDevices:{
    enumerateDevices:async()=>[{kind:"videoinput",deviceId:"usb",label:"USB 1080p"}],
    getUserMedia:async()=>{opened++;return mockStream}
   }
  },
  window:{addEventListener(){}}
 };
 vm.runInNewContext(code,mock,{filename:"camera.html",timeout:5000});
 for(let i=0;i<10;i++)await tick();
 const el=id=>elems.get(id);
 assert.equal(el("squareSize").value,"24");
 assert.equal(opened,1,"existing camera permission should open webcam automatically");
 assert.equal(el("analyze").disabled,false);
 await el("analyze").onclick();
 assert.equal(analyzed,1);
 assert.match(el("analysisDetails").textContent,/棋盘位置/);
 assert.match(el("analysisDetails").textContent,/毫米|mm/);
});
