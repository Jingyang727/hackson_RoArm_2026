/* Orthographic view math shared by rendering and table-point picking. */
(function(root){
  function basis(view){
    const {width:w,height:h,yaw,pitch,zoom=1}=view;
    const cy=Math.cos(yaw),sy=Math.sin(yaw),sp=Math.sin(pitch),cp=Math.cos(pitch);
    return {scale:Math.min(w/700,h/620)*zoom,center:[170,0,-10],origin:[w*.48,h*.48],
      right:[cy,-sy,0],down:[sy*sp,cy*sp,-cp],ray:[sy*cp,cy*cp,sp]};
  }
  const dot=(a,b)=>a.reduce((sum,value,i)=>sum+value*b[i],0);
  function project(point,view){
    const b=basis(view),relative=point.map((v,i)=>v-b.center[i]);
    const x=b.origin[0]+dot(relative,b.right)*b.scale;
    return [view.mirrored?view.width-x:x,b.origin[1]+dot(relative,b.down)*b.scale];
  }
  function tablePoint(pixel,plane,view){
    if(!plane)return null;
    const b=basis(view),denominator=dot(plane.normal_up,b.ray);
    if(!Number.isFinite(denominator)||Math.abs(denominator)<.08||b.scale<=0)return null;
    const x=view.mirrored?view.width-pixel[0]:pixel[0];
    const u=(x-b.origin[0])/b.scale,v=(pixel[1]-b.origin[1])/b.scale;
    const start=b.center.map((n,i)=>n+u*b.right[i]+v*b.down[i]);
    const distance=-(dot(plane.normal_up,start)+plane.d_mm)/denominator;
    const point=start.map((n,i)=>n+distance*b.ray[i]);
    return point.every(Number.isFinite)?point:null;
  }
  function atHeight(x,y,height,plane){
    if(!plane||!Number.isFinite(x)||!Number.isFinite(y)||!Number.isFinite(height)||Math.abs(plane.normal_up[2])<.1)return null;
    const [nx,ny,nz]=plane.normal_up;
    return [x,y,(height-plane.d_mm-nx*x-ny*y)/nz];
  }
  function torqueState(value){
    return value===1||value===true?'locked':value===0||value===false?'reported_zero':'unknown';
  }
  root.CoordinateView={project,tablePoint,atHeight,torqueState};
  if(typeof module!=='undefined')module.exports=root.CoordinateView;
})(globalThis);
