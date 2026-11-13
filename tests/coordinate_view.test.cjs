const test=require('node:test');
const assert=require('node:assert/strict');
const {project,tablePoint,atHeight,torqueState}=require('../control_ui/coordinate_view.js');
const close=(a,b)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<1e-7,`${a} != ${b}`));

test('table clicks round-trip in rotated, top, zoomed and mobile views',()=>{
  const plane={normal_up:[0,0,1],d_mm:120};
  for(const [width,height] of [[760,480],[315,370]])for(const [yaw,pitch,zoom] of [[.8,.6,1],[0,Math.PI/2,1],[1.8,.9,1.5]]){
    const view={width,height,yaw,pitch,zoom};
    for(const point of [[350,30,-120],[150,-100,-120],[400,50,-120]])close(tablePoint(project(point,view),plane,view),point);
  }
});

test('tilted calibrated table determines XY and slider sets actual plane clearance',()=>{
  const length=Math.hypot(.03,-.02,1),plane={normal_up:[.03/length,-.02/length,1/length],d_mm:117};
  const point=atHeight(350,50,0,plane),view={width:760,height:480,yaw:.8,pitch:.6,zoom:1};
  close(tablePoint(project(point,view),plane,view),point);
  const raised=atHeight(point[0],point[1],170,plane);
  assert.equal(raised[0],350);assert.equal(raised[1],50);
  assert.ok(Math.abs(plane.normal_up.reduce((sum,n,i)=>sum+n*raised[i],plane.d_mm)-170)<1e-7);
});

test('side view or missing calibration cannot invent a depth for a click',()=>{
  const view={width:760,height:480,yaw:0,pitch:0,zoom:1};
  assert.equal(tablePoint([300,200],{normal_up:[0,0,1],d_mm:120},view),null);
  assert.equal(tablePoint([300,200],null,view),null);
  assert.equal(atHeight(NaN,0,100,{normal_up:[0,0,1],d_mm:120}),null);
});

test('numeric and boolean torque replies agree, while missing replies stay unknown',()=>{
  for(const value of [1,true])assert.equal(torqueState(value),'locked');
  for(const value of [0,false])assert.equal(torqueState(value),'released');
  for(const value of [undefined,null,'1','0',2,NaN])assert.equal(torqueState(value),'unknown');
});
