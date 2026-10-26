/* APOGEE v2.0 — dependency-free orbital mission viewer.
   The browser only visualizes samples returned by the Python physics engine.
   No orbital positions are synthesized by the visual renderer. */
'use strict';

const $ = id => document.getElementById(id);

// Web deployment adapter. GitHub Pages serves this HTML; Python runs on Render.
// The URL is public configuration, NOT a secret or authentication credential.
const APOGEE_DEFAULT_API = String(window.APOGEE_CONFIG?.apiBaseUrl || '').trim();
const APOGEE_STORAGE_KEY = 'apogee_public_api_base_v1';
function isStaticSite() { return window.APOGEE_CONFIG?.staticSite !== false; }
function readSavedApi() { try { return localStorage.getItem(APOGEE_STORAGE_KEY) || ''; } catch (_) { return ''; } }
function writeSavedApi(value) { try { if(value) localStorage.setItem(APOGEE_STORAGE_KEY,value); else localStorage.removeItem(APOGEE_STORAGE_KEY); } catch (_) {} }
function normalizeApiBase(input) {
  const value = String(input || '').trim().replace(/\/+$/, '');
  if (!value) return '';
  const u = new URL(value);
  const isLocal = ['localhost','127.0.0.1','[::1]'].includes(u.hostname);
  if (u.protocol !== 'https:' && !(isLocal && u.protocol === 'http:')) throw Error('Use an HTTPS API URL (HTTP only for localhost).');
  if (u.username || u.password || u.search || u.hash || u.pathname !== '/' && u.pathname !== '') throw Error('Enter only the API origin, e.g. https://service.onrender.com');
  return u.origin;
}
let currentApiBase = (()=> { try { return normalizeApiBase(readSavedApi() || APOGEE_DEFAULT_API); } catch (_) { return ''; } })();
function apiUrl(path) {
  if (!path.startsWith('/api/')) throw Error('Invalid API route');
  if (isStaticSite() && !currentApiBase) throw Error('Connect the Python API in the bar above to run this simulation.');
  return `${currentApiBase}${path}`;
}
function apiFetch(path,options) { return fetch(apiUrl(path),options); }
function connectionUI(text,type='info') {
  const feedback=$('connection-feedback');
  if(feedback) {feedback.textContent=text;feedback.style.color=type==='ok'?'#93e3c7':type==='error'?'#ffb5a7':'#f5cb87';}
}
function connectionHeader(status,message) {
  $('connection-label').textContent=`PHYSICS API · ${status}`;
  $('connection-explanation').textContent=message;
}
async function checkConnection({rerun=false}={}) {
  if(isStaticSite() && !currentApiBase) {
    setText('engine-status','DEMO');
    connectionHeader('NOT CONNECTED','Viewing a precomputed orbit. Connect the Python service to calculate new trajectories.');
    return false;
  }
  try {
    const response=await apiFetch('/api/health',{signal:AbortSignal.timeout(25000)});
    if (!response.ok) throw Error(`HTTP ${response.status}`);
    const data=await response.json();
    if(data.application!=='APOGEE') throw Error('This URL does not identify an APOGEE backend.');
    connectionHeader('CONNECTED',`Python physics engine v${data.version || '?'} · all 26 modes available`);
    connectionUI('Connected to '+(currentApiBase || location.origin),'ok');
    setText('engine-status','ONLINE');
    if(rerun) await submitRequest('orbit');
    return true;
  } catch(err) {
    connectionHeader('OFFLINE','Unable to reach the Python engine. Check URL, service status, and CORS settings.');
    connectionUI(`Connection failed: ${err.message}`,'error');
    setText('engine-status','OFFLINE');
    return false;
  }
}
function registerConnectionUI() {
  $('api-base-input').value=currentApiBase;
  $('connection-form').addEventListener('submit',async e=>{
    e.preventDefault();
    const button=$('connection-save');button.disabled=true;connectionUI('Checking the physics service…');
    try {
      const candidate=normalizeApiBase($('api-base-input').value);
      if(isStaticSite() && !candidate) throw Error('Enter the deployed HTTPS Python API URL.');
      currentApiBase=candidate;
      writeSavedApi(candidate);
      await checkConnection({rerun:true});
    } catch(err) {connectionUI(err.message,'error');} finally {button.disabled=false;}
  });
  $('connection-clear').addEventListener('click',()=>{
    currentApiBase='';writeSavedApi('');$('api-base-input').value='';
    if(isStaticSite()) {setText('engine-status','DEMO');connectionHeader('NOT CONNECTED','Connect the Python service to run the full simulator.');connectionUI('Saved API URL cleared.');}
    else {connectionHeader('SAME ORIGIN','This site can use the Python server on the same origin.');connectionUI('Remote API URL cleared.');}
  });
}
async function loadDemoOrbit() {
  const resp=await fetch('./demo/orbit-default.json');
  if(!resp.ok) throw Error('Bundled preview unavailable');
  applyResponse(await resp.json());
  setText('engine-status','DEMO');
  connectionHeader('DEMO PREVIEW','Precomputed numerical orbit only. Connect the Python API to run the simulator.');
  connectionUI('This sample is not a live calculation. Other modes require the Python API.');
}

const R = 6378.137;
const TAU = 2*Math.PI;
const scene = $('scene');
const ctx = scene.getContext('2d');
const chart = $('drift-chart');
const chartCtx = chart.getContext('2d');
const app = {
  mode:'orbit', data:null, playing:true, simTime:0, speed:4,
  yaw:0.48, pitch:-0.39, zoom:1.0, dragging:false,
  pointerX:0, pointerY:0, lastFrame:0, maxRadius:R+400,
  presets:null, loading:false, sceneW:0, sceneH:0,
};

function formatNumber(value,decimals=2){return Number.isFinite(value) ? (Math.abs(value)<0.5*Math.pow(10,-decimals)?0:value).toLocaleString('en-US',{minimumFractionDigits:decimals,maximumFractionDigits:decimals}) : '—';}
function sci(num){return Number.isFinite(num) ? num.toExponential(2) : '—';}
function timeLabel(t){
  if(!Number.isFinite(t))return 'T+ --:--:--';
  const s=Math.max(0,Math.floor(t));
  const pad=n=>String(n).padStart(2,'0');
  return `T+ ${pad(Math.floor(s/3600))}:${pad(Math.floor(s%3600/60))}:${pad(s%60)}`;
}
function setText(id,value){$(id).textContent=String(value);}
function resizeCanvas(canvas,context){
  const rect=canvas.getBoundingClientRect();
  const dpr=Math.min(2,window.devicePixelRatio||1);
  const w=Math.max(2,Math.round(rect.width*dpr));
  const h=Math.max(2,Math.round(rect.height*dpr));
  if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}
  context.setTransform(dpr,0,0,dpr,0,0);
  return {w:rect.width,h:rect.height};
}
function cameraProject(position,scale,w,h){
  const [x,y,z]=position;
  const cy=Math.cos(app.yaw),sy=Math.sin(app.yaw);
  const cp=Math.cos(app.pitch),sp=Math.sin(app.pitch);
  const rx=cy*x-sy*y;
  const ry=sy*x+cy*y;
  const rz=sp*ry+cp*z;
  const ry2=cp*ry-sp*z;
  return {x:w/2+rx*scale,y:h/2-ry2*scale,depth:rz,xkm:rx,ykm:ry2};
}
function p3(latDeg,lonDeg,radius=R,rotation=0){
  const lat=latDeg*Math.PI/180;
  const lon=lonDeg*Math.PI/180+rotation;
  return [radius*Math.cos(lat)*Math.cos(lon),radius*Math.cos(lat)*Math.sin(lon),radius*Math.sin(lat)];
}
function strokePolyline(points,color,width=1,alpha=1,dash=[]){
  if(points.length<2)return;
  ctx.beginPath();ctx.moveTo(points[0].x,points[0].y);
  for(let i=1;i<points.length;i++)ctx.lineTo(points[i].x,points[i].y);
  ctx.strokeStyle=color;ctx.lineWidth=width;ctx.globalAlpha=alpha;
  ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=1;
}
function globeGrid(scale,w,h,spin){
  ctx.save();ctx.beginPath();ctx.arc(w/2,h/2,R*scale,0,TAU);ctx.clip();
  function visibleGridLine(positions,color,alpha){
    let segment=[];
    for(const pt of positions){
      const projected=cameraProject(pt,scale,w,h);
      if(projected.depth>0){segment.push(projected)}
      else {strokePolyline(segment,color,.65,alpha);segment=[];}
    }
    strokePolyline(segment,color,.65,alpha);
  }
  for(let lat=-75;lat<=75;lat+=15){const pts=[];for(let lon=-180;lon<=180;lon+=3)pts.push(p3(lat,lon,R*1.001,spin));visibleGridLine(pts,'#72cbd7',.11);}
  for(let lon=-180;lon<180;lon+=20){const pts=[];for(let lat=-89;lat<=89;lat+=3)pts.push(p3(lat,lon,R*1.001,spin));visibleGridLine(pts,'#72cbd7',.10);}
  // Schematic coastline outlines (non-geodetic visual texture).
  const land=[
    [[-167,62],[-150,69],[-128,60],[-124,49],[-130,42],[-117,32],[-111,26],[-98,17],[-90,19],[-82,25],[-80,32],[-65,45],[-60,50],[-80,55],[-95,72],[-115,72],[-143,69],[-167,62]],
    [[-81,12],[-76,5],[-79,-7],[-74,-15],[-70,-23],[-72,-32],[-69,-42],[-72,-53],[-61,-54],[-51,-41],[-48,-24],[-34,-7],[-43,2],[-54,7],[-66,10],[-81,12]],
    [[-11,36],[-5,44],[7,44],[13,46],[30,45],[37,53],[27,60],[10,61],[-7,59],[-11,50],[-11,36]],
    [[-16,32],[-5,36],[11,36],[24,30],[35,23],[44,11],[48,0],[42,-12],[32,-28],[18,-35],[12,-32],[10,-17],[2,-5],[-10,5],[-16,20],[-16,32]],
    [[35,42],[50,46],[65,55],[90,61],[115,54],[135,52],[149,45],[135,34],[126,23],[115,18],[108,9],[100,4],[95,13],[80,9],[70,20],[58,24],[47,30],[35,42]],
    [[113,-22],[128,-14],[138,-12],[148,-20],[153,-29],[145,-38],[134,-35],[120,-33],[113,-22]],
    [[46,-13],[49,-17],[48,-24],[45,-26],[44,-19],[46,-13]],
    [[-51,59],[-45,64],[-42,74],[-37,81],[-49,83],[-59,78],[-55,67],[-51,59]],
  ];
  for(const continent of land){
    let line=[];
    for(let i=0;i<continent.length;i++){
      const point=cameraProject(p3(continent[i][1],continent[i][0],R*1.004,spin),scale,w,h);
      if(point.depth>0){line.push(point)}else{strokePolyline(line,'#92dfd7',.9,.30);line=[];}
    }
    strokePolyline(line,'#92dfd7',.9,.30);
  }
  ctx.restore();
}
function drawEarth(scale,w,h,t){
  const cx=w/2,cy=h/2,rad=R*scale;
  ctx.save();
  const haze=ctx.createRadialGradient(cx,cy,rad*.82,cx,cy,rad*1.34);
  haze.addColorStop(0,'rgba(58,170,217,0)');haze.addColorStop(.5,'rgba(58,170,217,.055)');haze.addColorStop(1,'rgba(58,170,217,0)');
  ctx.fillStyle=haze;ctx.beginPath();ctx.arc(cx,cy,rad*1.35,0,TAU);ctx.fill();
  const earth=ctx.createRadialGradient(cx-rad*.4,cy-rad*.43,rad*.08,cx+rad*.15,cy+rad*.15,rad*1.24);
  earth.addColorStop(0,'#1d617d');earth.addColorStop(.36,'#194a68');earth.addColorStop(.73,'#12314b');earth.addColorStop(1,'#061321');
  ctx.beginPath();ctx.arc(cx,cy,rad,0,TAU);ctx.fillStyle=earth;ctx.fill();
  ctx.strokeStyle='rgba(91,187,217,.5)';ctx.lineWidth=1;ctx.stroke();
  globeGrid(scale,w,h,t/86164*TAU);
  ctx.restore();
}
function drawOrbitLine(points,scale,w,h,isFront,color,alpha,width=1.2,start=0,end=points.length-1){
  if(!points||points.length<2)return;
  let previous=null;
  ctx.beginPath();let hasSegment=false;
  for(let i=start;i<=end;i++){
    const p=cameraProject(points[i],scale,w,h);
    const r2=p.xkm*p.xkm+p.ykm*p.ykm;
    const front=p.depth>0 || r2>=R*R;
    if(front===isFront){
      if(!previous||previous!==i-1)ctx.moveTo(p.x,p.y);
      else ctx.lineTo(p.x,p.y);
      hasSegment=true;
    }else if(previous===i-1&&r2>=R*R){ctx.lineTo(p.x,p.y)}
    previous=front===isFront?i:null;
  }
  if(hasSegment){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.globalAlpha=alpha;ctx.stroke();ctx.globalAlpha=1;}
}
function makeOrbitCircle(radius,inclination){
  const rad=inclination*Math.PI/180;
  const result=[];
  for(let i=0;i<=240;i++){
    const angle=i/240*TAU;
    result.push([radius*Math.cos(angle),radius*Math.sin(angle)*Math.cos(rad),radius*Math.sin(angle)*Math.sin(rad)]);
  }
  return result;
}
function drawAxes(scale,w,h){
  const ox=w-62,oy=h-62;
  const length=19;
  const axes=[{v:[1,0,0],c:'#e68f8e',k:'X'},{v:[0,1,0],c:'#81e6bd',k:'Y'},{v:[0,0,1],c:'#80b9ee',k:'Z'}];
  ctx.lineWidth=1.5;ctx.font='10px ui-monospace,monospace';
  for(const axis of axes){
    const p=cameraProject(axis.v,1,0,0);
    ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox+(p.x)*length,oy+(p.y)*length);
    ctx.strokeStyle=axis.c;ctx.stroke();ctx.fillStyle=axis.c;
    ctx.fillText(axis.k,ox+(p.x)*length*1.22-2,oy+(p.y)*length*1.22+3);
  }
}
function currentIndex(){
  const traj=app.data?.trajectory;if(!traj)return 0;
  const t=traj.time_s;
  // Burn boundaries introduce nonuniform timestamps. Use binary nearest-neighbor lookup.
  let lo=0,hi=t.length-1;
  while(lo<hi){const m=(lo+hi)>>1;if(t[m]<app.simTime)lo=m+1;else hi=m;}
  return lo>0&&Math.abs(t[lo-1]-app.simTime)<Math.abs(t[lo]-app.simTime)?lo-1:lo;
}
function drawScene(){
  const {w,h}=resizeCanvas(scene,ctx);
  app.sceneW=w;app.sceneH=h;
  ctx.clearRect(0,0,w,h);
  // Fixed deterministic star field.
  for(let i=0;i<105;i++){
    const a=Math.sin(i*12.9898+4.23)*43758.5453;
    const b=Math.sin(i*78.233+6.92)*22912.241;
    const x=(a-Math.floor(a))*w,y=(b-Math.floor(b))*h;
    ctx.fillStyle=`rgba(171,215,238,${.12+(i%5)*.085})`;
    ctx.fillRect(x,y,i%17===0?1.6:.85,i%17===0?1.6:.85);
  }
  const maxR=app.maxRadius;
  const scale=Math.min(w*.39,h*.42)/(maxR*1.04)*app.zoom;
  const positions=app.data?.trajectory.position_km;
  const current=currentIndex();
  let departure=null,destination=null;
  if(app.data?.transfer){
    const tr=app.data.transfer;
    departure=makeOrbitCircle(R+tr.start_altitude_km,app.data.input.inclination_deg);
    destination=makeOrbitCircle(R+tr.target_altitude_km,app.data.input.inclination_deg);
  }
  if(departure){drawOrbitLine(departure,scale,w,h,false,'#456b87',.34,1);drawOrbitLine(destination,scale,w,h,false,'#456b87',.36,1);}
  const comparisonBaseline=app.data?.comparison?.baseline_trajectory?.position_km || app.data?.guidance?.reference_position_km;
  if(comparisonBaseline)drawOrbitLine(comparisonBaseline,scale,w,h,false,'#ffa970',.34,1.5);
  if(positions)drawOrbitLine(positions,scale,w,h,false,'#53afc4',.32,1.4);
  drawEarth(scale,w,h,app.simTime);
  if(departure){drawOrbitLine(departure,scale,w,h,true,'#66899d',.42,1);drawOrbitLine(destination,scale,w,h,true,'#66899d',.52,1);}
  if(positions){
    if(comparisonBaseline){drawOrbitLine(comparisonBaseline,scale,w,h,true,'#ffa970',.77,1.6);
      const t=app.data.comparison?.baseline_trajectory?.time_s || app.data.guidance.time_s;
      let idx=0;while(idx<t.length-1&&t[idx]<app.simTime)idx++;
      const bp=cameraProject(comparisonBaseline[idx],scale,w,h);
      ctx.fillStyle='#ffb079';ctx.beginPath();ctx.arc(bp.x,bp.y,3,0,TAU);ctx.fill();}
    drawOrbitLine(positions,scale,w,h,true,'#54bdcf',.70,1.65);
    drawOrbitLine(positions,scale,w,h,true,'#7deaf1',.98,2.2,0,current);
    const allEvents=app.data.events||[];
    let displayed=0;
    for(const event of allEvents){
      if(!['burn_cutoff','ignition','ignition_command','fuel_exhaustion','impulse','departure_impulse','opposite_apsis_arrival','guidance_burn'].includes(event.type))continue;
      const eventPos=event.position_km || (event.type==='guidance_burn'?positions[Math.max(0,app.data.trajectory.time_s.findIndex(t=>Math.abs(t-event.time_s)<.01))]:null);
      if(!eventPos)continue;
      const ep=cameraProject(eventPos,scale,w,h);
      if(ep.depth<0&&ep.xkm*ep.xkm+ep.ykm*ep.ykm<R*R)continue;
      const color=event.type==='fuel_exhaustion'?'#ff8e8e':'#ffc784';
      ctx.beginPath();ctx.arc(ep.x,ep.y,6,0,TAU);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();
      ctx.beginPath();ctx.arc(ep.x,ep.y,2.5,0,TAU);ctx.fillStyle=color;ctx.fill();
      ctx.font='10px ui-monospace,monospace';ctx.fillStyle=color;
      const label=event.type==='burn_cutoff'?'ENGINE OFF':event.type==='ignition'?'IGNITION':event.type==='fuel_exhaustion'?'FUEL OUT':event.type==='opposite_apsis_arrival'?'ARRIVAL BURN':event.type==='departure_impulse'?'DEPARTURE BURN':(event.burn_name||event.name||'IMPULSE').toUpperCase();
      ctx.fillText(label,ep.x+12,ep.y+15+displayed%2*13);
      displayed++;
    }
    const obj=cameraProject(positions[current],scale,w,h);
    const occluded=obj.depth<0 && obj.xkm*obj.xkm+obj.ykm*obj.ykm<R*R;
    if(!occluded){
      const glow=ctx.createRadialGradient(obj.x,obj.y,1,obj.x,obj.y,17);
      glow.addColorStop(0,'rgba(255,215,155,.40)');glow.addColorStop(1,'rgba(255,215,155,0)');
      ctx.fillStyle=glow;ctx.beginPath();ctx.arc(obj.x,obj.y,17,0,TAU);ctx.fill();
      ctx.fillStyle='#ffe4a9';ctx.beginPath();ctx.arc(obj.x,obj.y,4,0,TAU);ctx.fill();
      ctx.strokeStyle='#fff7e5';ctx.lineWidth=1.5;ctx.stroke();
      // Actual body +X axis, derived from backend-integrated quaternion samples.
      if(app.data?.attitude){
        const axis=app.data.attitude.body_x_inertial[current];
        const offset=cameraProject([positions[current][0]+axis[0]*850,positions[current][1]+axis[1]*850,positions[current][2]+axis[2]*850],scale,w,h);
        ctx.beginPath();ctx.moveTo(obj.x,obj.y);ctx.lineTo(offset.x,offset.y);
        ctx.strokeStyle='#ffc77a';ctx.lineWidth=2.5;ctx.stroke();
        ctx.fillStyle='#ffc77a';ctx.font='10px ui-monospace,monospace';ctx.fillText('BODY +X',offset.x+5,offset.y);
      }
      ctx.font='10px ui-monospace,monospace';ctx.fillStyle='#e5f6fb';ctx.fillText('SC-001',obj.x+10,obj.y-10);
      ctx.beginPath();ctx.moveTo(obj.x+7,obj.y-7);ctx.lineTo(obj.x+24,obj.y-18);ctx.strokeStyle='#8eacb9';ctx.lineWidth=.7;ctx.stroke();
    }
  }
  drawAxes(scale,w,h);
}
function chartLine(ctx2,w,h,series,minValue,maxValue,color,validMask=null){
  if(!series||series.length<2)return;
  const left=54,right=w-16,top=12,bottom=h-22;
  ctx2.beginPath();
  let started=false;
  series.forEach((raw,i)=>{
    if(validMask && !validMask[i]) {started=false;return;}
    const value=Math.max(minValue,raw);
    const xx=left+i/(series.length-1)*(right-left);
    const yy=bottom-(Math.log10(value)-Math.log10(minValue))/(Math.log10(maxValue)-Math.log10(minValue))*(bottom-top);
    if(!started){ctx2.moveTo(xx,yy);started=true;}else ctx2.lineTo(xx,yy);
  });
  ctx2.strokeStyle=color;ctx2.lineWidth=1.5;ctx2.stroke();
}
function drawComparisonChart(){
  const {w,h}=resizeCanvas(chart,chartCtx),c=chartCtx;
  const comparison=app.data.comparison,ys=comparison.separation_km;
  c.clearRect(0,0,w,h);
  const left=55,right=w-16,top=14,bottom=h-25;
  const peak=Math.max(1e-10,...ys)*1.15;
  c.font='10px ui-monospace, monospace';
  for(let j=0;j<=4;j++){
    const y=top+(bottom-top)*j/4;
    c.strokeStyle='#244057';c.setLineDash([3,4]);c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();
    c.fillStyle='#99b0c3';c.textAlign='right';c.fillText(formatNumber((4-j)/4*peak,peak<.01?5:2),left-8,y+3);
  }
  c.setLineDash([]);c.textAlign='left';c.fillText('T+ 0',left,bottom+16);
  c.textAlign='right';c.fillText(timeLabel(comparison.time_s.at(-1)),right,bottom+16);
  if(ys.length<2)return;
  c.beginPath();
  for(let i=0;i<ys.length;i++){
    const x=left+(right-left)*i/(ys.length-1);
    const y=bottom-ys[i]/peak*(bottom-top);
    if(i)c.lineTo(x,y);else c.moveTo(x,y);
  }
  c.strokeStyle='#f2a46e';c.lineWidth=2;c.stroke();
}
function drawUncertaintyChart(){
  const {w,h}=resizeCanvas(chart,chartCtx), c=chartCtx;
  const series=app.data.uncertainty.position_95pct_conservative_radius_m;
  c.clearRect(0,0,w,h);
  const l=55,r=w-16,top=14,bottom=h-25;
  const maximum=Math.max(1,...series)*1.10;
  c.font='10px ui-monospace, monospace';
  for(let j=0;j<=4;j++){
    const y=top+(bottom-top)*j/4;
    c.strokeStyle='#244057';c.setLineDash([3,4]);c.beginPath();c.moveTo(l,y);c.lineTo(r,y);c.stroke();
    c.fillStyle='#99b0c3';c.textAlign='right';c.fillText(formatNumber((4-j)/4*maximum,1),l-8,y+3);
  }
  c.setLineDash([]);c.textAlign='left';c.fillText('0',l,bottom+16);
  c.textAlign='right';c.fillText('END · 95% RADIUS (m)',r,bottom+16);
  c.beginPath();series.forEach((v,i)=>{
    const x=l+(r-l)*i/(series.length-1),y=bottom-v/maximum*(bottom-top);
    if(i===0)c.moveTo(x,y);else c.lineTo(x,y);
  });
  c.strokeStyle='#f2a46e';c.lineWidth=2;c.stroke();
}
function drawGuidanceChart(){
  const {w,h}=resizeCanvas(chart,chartCtx),c=chartCtx;
  const g=app.data.guidance,one=g.guided_position_error_m,two=g.unguided_position_error_m;
  c.clearRect(0,0,w,h);
  const left=59,right=w-18,top=15,bottom=h-27;
  const maximum=Math.max(1,...one,...two)*1.08;
  c.font='10px ui-monospace,monospace';
  for(let j=0;j<=4;j++){
    let y=top+(bottom-top)*j/4;
    c.strokeStyle='#254259';c.setLineDash([3,4]);c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();
    c.fillStyle='#a5bbc8';c.textAlign='right';c.fillText(formatNumber(maximum*(1-j/4),0),left-8,y+3);
  }
  c.setLineDash([]);
  const plot=(values,color)=>{
    c.strokeStyle=color;c.lineWidth=2;c.beginPath();values.forEach((value,i)=>{
      let x=left+(right-left)*i/Math.max(1,values.length-1),y=bottom-value/maximum*(bottom-top);
      if(i===0)c.moveTo(x,y);else c.lineTo(x,y);
    });c.stroke();
  };
  plot(two,'#f2a46e');plot(one,'#50d5e4');
  c.fillStyle='#a5bbc8';c.textAlign='left';c.fillText('0',left,bottom+16);
  c.textAlign='right';c.fillText('END · METERS',right,bottom+16);
}
function drawAttitudeChart(){
  const {w,h}=resizeCanvas(chart,chartCtx),c=chartCtx;
  c.clearRect(0,0,w,h);
  const data=app.data.attitude,values=data.pointing_error_deg;
  const left=54,right=w-16,top=14,bottom=h-26,peak=Math.max(1,...values,...((app.data.adcs||app.data.precision_adcs)?.attitude_estimation_error_deg||[]))*1.12;
  c.font='10px ui-monospace,monospace';
  for(let j=0;j<=4;j++){
    const y=top+(bottom-top)*j/4;
    c.strokeStyle='#233c50';c.setLineDash([3,4]);c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();
    c.fillStyle='#a7bfcb';c.textAlign='right';c.fillText(formatNumber(peak*(1-j/4),1),left-6,y+3);
  }
  c.setLineDash([]);c.beginPath();values.forEach((v,i)=>{
    const x=left+(right-left)*i/Math.max(1,values.length-1),y=bottom-v/peak*(bottom-top);
    if(i)c.lineTo(x,y);else c.moveTo(x,y);
  });c.lineWidth=2;c.strokeStyle='#53d6e1';c.stroke();
  if(app.data.adcs || app.data.precision_adcs){
    c.beginPath();(app.data.adcs||app.data.precision_adcs).attitude_estimation_error_deg.forEach((v,i)=>{
      const x=left+(right-left)*i/Math.max(1,values.length-1),y=bottom-v/peak*(bottom-top);
      if(i)c.lineTo(x,y);else c.moveTo(x,y);
    });c.strokeStyle='#f2a46e';c.lineWidth=1.7;c.stroke();
  }
  const bs=app.data.input.burn_start_s,be=bs+app.data.input.burn_duration_s,dur=app.data.trajectory.time_s.at(-1);
  c.fillStyle='rgba(245,164,103,.12)';c.fillRect(left+(right-left)*bs/dur,top,(right-left)*Math.min(be,dur)/dur-(right-left)*bs/dur,bottom-top);
  c.fillStyle='#a5bbc8';c.textAlign='left';c.fillText('0',left,bottom+16);
  c.textAlign='right';c.fillText('END · DEGREES',right,bottom+16);
}
function drawIntegratedNavChart(){
  const {w,h}=resizeCanvas(chart,chartCtx),c=chartCtx;
  const n=app.data.integrated_navigation;
  const one=n.position_error_m,two=n.position_1sigma_rss_m;
  const L=54,R=w-16,T=14,B=h-26,M=Math.max(1,...one,...two)*1.12;
  c.clearRect(0,0,w,h);c.font='10px ui-monospace,monospace';
  for(let i=0;i<=4;i++){
    const y=T+(B-T)*i/4;c.beginPath();c.moveTo(L,y);c.lineTo(R,y);
    c.strokeStyle='#254259';c.setLineDash([3,4]);c.stroke();
    c.fillStyle='#a7bfcb';c.textAlign='right';c.fillText(formatNumber(M*(1-i/4),0),L-6,y+3);
  }
  c.setLineDash([]);
  for(const [series,color] of [[one,'#50d5e4'],[two,'#f2a46e']]){
    c.beginPath();series.forEach((v,i)=>{const x=L+(R-L)*i/Math.max(1,series.length-1),y=B-v/M*(B-T);
      if(i)c.lineTo(x,y);else c.moveTo(x,y);});
    c.strokeStyle=color;c.lineWidth=2;c.stroke();
  }
  c.fillStyle='#a7bfcb';c.textAlign='left';c.fillText('0',L,B+17);
  c.textAlign='right';c.fillText('END · METERS',R,B+17);
}
function drawDriftChart(){
  if(app.data?.ground_navigation){
    const n=app.data.ground_navigation;
    const previous=app.data.integrated_navigation;
    app.data.integrated_navigation=n;
    try{drawIntegratedNavChart()}finally{app.data.integrated_navigation=previous;}
    return;
  }
  if(app.data?.observability){
    const n=app.data.observability;
    const previous=app.data.integrated_navigation;
    app.data.integrated_navigation=n;
    try{drawIntegratedNavChart()}finally{app.data.integrated_navigation=previous;}
    return;
  }
  if(app.data?.adaptive_navigation){
    const n=app.data.adaptive_navigation;
    const previous=app.data.integrated_navigation;
    app.data.integrated_navigation=n;
    try{drawIntegratedNavChart()}finally{app.data.integrated_navigation=previous;}
    return;
  }
  if(app.data?.joint_integrity){
    const n=app.data.joint_integrity;
    const previous=app.data.integrated_navigation;
    app.data.integrated_navigation=n;
    try{drawIntegratedNavChart()}finally{app.data.integrated_navigation=previous;}
    return;
  }
  if(app.data?.integrity){
    const n=app.data.integrity;
    const previous=app.data.integrated_navigation;
    app.data.integrated_navigation=n;
    try{drawIntegratedNavChart()}finally{app.data.integrated_navigation=previous;}
    return;
  }
  if(app.data?.joint_navigation){
    const n=app.data.joint_navigation;
    const previous=app.data.integrated_navigation;
    app.data.integrated_navigation=n;
    try{drawIntegratedNavChart()}finally{app.data.integrated_navigation=previous;}
    return;
  }
  if(app.data?.integrated_navigation){drawIntegratedNavChart();return;}
  if(app.data?.attitude){drawAttitudeChart();return;}
  if(app.data?.guidance){drawGuidanceChart();return;}
  if(app.data?.uncertainty){drawUncertaintyChart();return;}
  if(app.data?.comparison){drawComparisonChart();return;}

  const {w,h}=resizeCanvas(chart,chartCtx);const c=chartCtx;
  c.clearRect(0,0,w,h);
  const tr=app.data?.trajectory;
  const xLeft=54,yTop=12,xRight=w-15,yBottom=h-22;
  const seriesE=tr?.relative_energy_drift||[];
  const seriesH=tr?.relative_angular_momentum_drift||[];
  const max=Math.max(1e-12,...seriesE,...seriesH);
  const maxP=Math.ceil(Math.log10(max));
  const minP=maxP-5;
  c.font='9px ui-monospace,monospace';c.textAlign='right';
  for(let i=0;i<=5;i++){
    const yy=yTop+i/5*(yBottom-yTop);
    c.strokeStyle='#23374b';c.lineWidth=.7;c.setLineDash([3,4]);
    c.beginPath();c.moveTo(xLeft,yy);c.lineTo(xRight,yy);c.stroke();
    c.fillStyle='#829bb0';c.fillText('1e'+(maxP-i),xLeft-8,yy+3);
  }
  c.setLineDash([]);
  c.fillStyle='#829bb0';c.textAlign='left';c.fillText('0',xLeft,yBottom+14);
  c.textAlign='right';c.fillText('END',xRight,yBottom+14);
  if(app.data?.metrics.conservation_applicable===false)return;
  if(tr){const mask=tr.coast_diagnostics_valid||null;chartLine(c,w,h,seriesE,10**minP,10**maxP,'#50d5e4',mask);chartLine(c,w,h,seriesH,10**minP,10**maxP,'#9c99ff',mask);}
}
function analysisRow(label,value){
  const el=document.createElement('div');el.className='analysis-row';
  const a=document.createElement('span');a.textContent=label;
  const b=document.createElement('strong');b.textContent=value;
  el.append(a,b);return el;
}
function updateAnalysis(){
  const d=app.data;if(!d)return;
  const container=$('analysis-rows');container.replaceChildren();
  const list=[];
  if(d.ground_navigation){
    const n=d.ground_navigation,q=n.summary,mc=n.monte_carlo;
    $('analysis-heading').textContent='Causal 7-state EKF · rotating ground stations';
    list.push(['Final estimated position error',formatNumber(q.final_error_m,2)+' m'],
      ['Final estimated 1σ RSS',formatNumber(q.final_1sigma_rss_m,2)+' m'],
      ['Actual / estimated thrust',formatNumber(d.input.true_thrust_scale,4)+'× / '+formatNumber(q.final_thrust_scale,4)+'×'],
      ['Packets produced / received',q.packet_count+' / '+q.received_count],
      ['Packets still in flight',String(q.in_flight_count)],
      ['Visible measurement intervals',String(q.visible_intervals)],
      ['Mean innovation statistic',q.mean_nis_received==null?'N/A':formatNumber(q.mean_nis_received,2)],
      ['Monte Carlo median final error',formatNumber(mc.median_final_position_error_m,1)+' m'],
      ['Monte Carlo 90th percentile',formatNumber(mc.p90_final_position_error_m,1)+' m'],
      ['UTC reference epoch',d.input.epoch_utc]);
  }else if(d.observability){
    const n=d.observability, info=n.effective_information;
    const summary=n.monte_carlo[n.method];
    $('analysis-heading').textContent='Batch calibration · robust estimator comparison';
    list.push(['Engine scale identifiable?',info.observable?'YES · within assumed noise':'NO · insufficient independent information'],
      ['Effective scale information',formatNumber(info.effective_scale_information,1)],
      ['Scale 1σ from observations',info.scale_1sigma_from_data===null?'Not identifiable':formatNumber(info.scale_1sigma_from_data*100,3)+'%'],
      ['Synthetic true / estimated scale',formatNumber(n.actual_thrust_scale,4)+' / '+formatNumber(n.estimated_thrust_scale,4)],
      ['Selected estimator',n.method.replace('_',' ')],
      ['Downweighted observations',String(n.downweighted_fixes)],
      ['Position error · first seed',formatNumber(n.position_error_m.at(-1),2)+' m'],
      ['MC median position · selected',formatNumber(summary.median_final_position_error_m,2)+' m'],
      ['MC 90th percentile',formatNumber(summary.p90_final_position_error_m,2)+' m'],
      ['Empirical nominal 95% coverage',formatNumber(summary.position_95pct_coverage*100,1)+'%'],
      ['Least squares · median',formatNumber(n.monte_carlo.least_squares.median_final_position_error_m,2)+' m'],
      ['Huber · median',formatNumber(n.monte_carlo.huber.median_final_position_error_m,2)+' m'],
      ['Gate · median',formatNumber(n.monte_carlo.gate.median_final_position_error_m,2)+' m']);
  }else if(d.adaptive_navigation){
    const n=d.adaptive_navigation,m=n.monte_carlo;
    $('analysis-heading').textContent='13-state EKF · engine calibration and sensor recovery';
    list.push(['Final position error',formatNumber(n.final_position_error_m,2)+' m'],
      ['Estimated position 1σ RSS',formatNumber(n.final_position_1sigma_rss_m,2)+' m'],
      ['Actual / estimated engine scale',formatNumber(n.actual_thrust_scale,4)+' / '+formatNumber(n.estimated_thrust_scale,4)],
      ['Engine-scale 1σ',formatNumber(n.thrust_scale_1sigma*100,2)+'%'],
      ['Position / tracker alarms',n.alarms_by_channel.position+' / '+n.alarms_by_channel.tracker],
      ['Rejected / quarantined packets',n.rejected_packets+' / '+n.quarantined_packets],
      ['Position sensor noise multiplier',formatNumber(n.final_noise_multipliers.position,2)+'×'],
      ['Tracker noise multiplier',formatNumber(n.final_noise_multipliers.tracker,2)+'×'],
      ['Monte Carlo median error',formatNumber(m.median_final_position_error_m,2)+' m'],
      ['Monte Carlo worst error',formatNumber(m.worst_final_position_error_m,2)+' m']);
  }else if(d.joint_integrity){
    const n=d.joint_integrity,m=n.monte_carlo;
    $('analysis-heading').textContent='12-state joint navigation · multisensor integrity';
    list.push(['Final position error',formatNumber(n.final_position_error_m,1)+' m'],
      ['Final 1σ RSS position',formatNumber(n.final_position_1sigma_rss_m,1)+' m'],
      ['Final attitude error',formatNumber(n.final_attitude_error_deg,3)+'°'],
      ['Channel classification',n.classification.replaceAll('_',' ')],
      ['Position / tracker alarms',n.alarms_by_channel.position+' / '+n.alarms_by_channel.tracker],
      ['Rejected / received packets',n.rejected_packets+' / '+n.received_packets],
      ['Faulted packets with alarms',n.fault_alarms+' / '+n.fault_measurements_received],
      ['NIS threshold · χ² (3 DoF)',formatNumber(n.gate_chi2_3dof,2)],
      ['MC median position error',formatNumber(m.median_final_position_error_m,1)+' m'],
      ['MC worst position error',formatNumber(m.worst_final_position_error_m,1)+' m'],
      ['Monte Carlo trials',String(m.runs)]);
  }else if(d.integrity){
    const n=d.integrity,m=n.monte_carlo;
    $('analysis-heading').textContent='Sensor integrity · Monte Carlo fault monitoring';
    list.push(['Final position error · first run',formatNumber(n.final_position_error_m,1)+' m'],
      ['Final estimated 1σ RSS',formatNumber(n.final_position_1sigma_rss_m,1)+' m'],
      ['NIS χ² threshold · 3 DoF',formatNumber(n.gate_threshold_chi2_3dof,2)],
      ['Fault alarms / faulted samples',n.fault_alarms+' / '+n.fault_measurements],
      ['Pre-fault false alarms',n.pre_fault_alarms+' / '+n.pre_fault_measurements],
      ['First fault detection',n.first_fault_alarm_s===null?'Not detected':timeLabel(n.first_fault_alarm_s)],
      ['Engine calibration · realized',formatNumber(n.true_thrust_calibration_factor*100,2)+'%'],
      ['Monte Carlo · median final error',formatNumber(m.median_final_error_m,1)+' m'],
      ['Monte Carlo · worst final error',formatNumber(m.worst_final_error_m,1)+' m'],
      ['Sweep fault detection rate',m.fault_detection_rate===null?'No injected fault':formatNumber(m.fault_detection_rate*100,1)+'%'],
      ['Sweep pre-fault alarm rate',formatNumber(m.false_alarm_rate_pre_fault*100,1)+'%'],
      ['Sweep repetitions',String(m.runs)]);
  }else if(d.joint_navigation){
    const n=d.joint_navigation;
    $('analysis-heading').textContent='12-state joint orbit + attitude navigation';
    list.push(['Final position error',formatNumber(n.final_position_error_m,2)+' m'],
      ['Position uncertainty · 1σ RSS',formatNumber(n.final_position_1sigma_rss_m,2)+' m'],
      ['Final attitude error',formatNumber(n.final_attitude_estimation_error_deg,3)+'°'],
      ['Attitude uncertainty · 1σ RSS',formatNumber(n.final_attitude_1sigma_rss_deg,3)+'°'],
      ['Maximum position/attitude correlation',formatNumber(n.max_position_attitude_abs_correlation,3)],
      ['Packets assimilated / generated',n.received_at_finish+' / '+n.packets_generated],
      ['Final position error / 1σ',formatNumber(n.final_error_over_position_sigma_rss,2)+' ×'],
      ['Estimator state', '12-dimensional correlated error covariance']);
  }else if(d.integrated_navigation){
    const n=d.integrated_navigation;
    $('analysis-heading').textContent='Time-tagged orbit + attitude navigation';
    list.push(['Final position estimate error',formatNumber(n.final_position_error_m,2)+' m'],
      ['Final 1σ position RSS',formatNumber(n.final_position_1sigma_rss_m,2)+' m'],
      ['Final velocity estimate error',formatNumber(n.final_velocity_error_m_s,3)+' m/s'],
      ['Attitude estimate error',formatNumber(n.final_attitude_estimation_error_deg,3)+'°'],
      ['Packets received / generated',n.received_at_finish+' / '+n.packets_generated],
      ['Pending packets at end',String(n.late_or_pending_at_finish)],
      ['Tracker fixes',String(n.tracker_fixes)],
      ['Powered covariance',n.attitude_covariance_augmented?'Augmented (approximate)':'Original 6-state'],
      ['Error / final 1σ RSS',formatNumber(n.final_position_error_over_sigma_rss,2)+' ×'],
      ['Model class', '6+6 loosely coupled; no cross covariance']);
  }else if(d.precision_adcs){
    const a=d.precision_adcs,m=d.metrics;
    $('analysis-heading').textContent='MEKF · sensor geometry & magnetic unloading';
    list.push(['Final pointing error',formatNumber(a.final_pointing_error_deg,3)+'°'],
      ['Final attitude estimation error',formatNumber(a.final_estimation_error_deg,3)+'°'],
      ['Final estimated 3-axis σ',formatNumber(a.final_attitude_3axis_sigma_deg,3)+'°'],
      ['Mean NIS · three observable axes',a.mean_nis===null?'No measurements':formatNumber(a.mean_nis,2)],
      ['Mean NEES · six error states',formatNumber(a.mean_nees,2)],
      ['Tracker fixes',String(a.tracker_fixes)],
      ['Blocked Sun / Earth fixes',a.tracker_blocked_sun+' / '+a.tracker_blocked_earth],
      ['Final wheel momentum',formatNumber(a.final_wheel_momentum_norm_nms,3)+' N·m·s'],
      ['Magnetorquer active steps',String(a.magnetorquer_active_steps)],
      ['Fuel used',formatNumber(m.propellant_used_kg,3)+' kg']);
  }else if(d.adcs){
    const a=d.adcs,m=d.metrics;
    $('analysis-heading').textContent='ADCS · sensor fusion & momentum';
    list.push(['Final pointing error',formatNumber(a.final_pointing_error_deg,3)+'°'],
      ['Final attitude estimate error',formatNumber(a.final_estimation_error_deg,3)+'°'],
      ['RMS estimate error',formatNumber(a.rms_estimation_error_deg,3)+'°'],
      ['Star tracker fixes',String(a.tracker_fixes)],
      ['Wheel-limited decision steps',String(a.wheel_limit_steps)],
      ['Peak wheel momentum',formatNumber(a.max_wheel_momentum_nms,3)+' N·m·s'],
      ['Max applied motor torque',formatNumber(a.max_wheel_torque_nm,3)+' N·m'],
      ['Fuel used',formatNumber(m.propellant_used_kg,3)+' kg']);
  }else if(d.attitude){
    const a=d.attitude,m=d.metrics;
    $('analysis-heading').textContent='Orbit + attitude integrated dynamics';
    list.push(['Initial pointing error',formatNumber(a.pointing_error_deg[0],2)+'°'],
      ['Final pointing error',formatNumber(a.final_pointing_error_deg,3)+'°'],
      ['Mean burn pointing error',a.mean_burn_pointing_error_deg==null?'N/A':formatNumber(a.mean_burn_pointing_error_deg,3)+'°'],
      ['Peak body rotation',formatNumber(a.peak_body_rate_deg_s,4)+' °/s'],
      ['Max commanded torque',formatNumber(a.max_control_torque_nm_observed,4)+' N·m'],
      ['Fuel consumed',formatNumber(m.propellant_used_kg,4)+' kg'],
      ['Engine Δv integral',formatNumber(m.ideal_thrust_delta_v_m_s,3)+' m/s'],
      ['Final orbit a',formatNumber(d.final_orbital_elements.semi_major_axis_km,3)+' km']);
  }else if(d.guidance){
    const m=d.metrics,g=d.guidance;
    $('analysis-heading').textContent='Closed-loop navigation performance';
    list.push(['Unguided final error',formatNumber(m.final_unguided_position_error_m,2)+' m'],
      ['Guided final error',formatNumber(m.final_guided_position_error_m,2)+' m'],
      ['Position error reduction',formatNumber(m.improvement_pct,2)+' %'],
      ['Final estimation error',formatNumber(m.final_estimation_position_error_m,2)+' m'],
      ['Estimated 1σ position RSS',formatNumber(m.final_estimated_position_1sigma_rss_m,2)+' m'],
      ['Total guidance Δv',formatNumber(m.total_guidance_delta_v_m_s,4)+' m/s'],
      ['Corrections executed',String(m.correction_count)],
      ['Position measurements',String(m.measurement_count)],
      ['Final guided velocity error',formatNumber(m.final_guided_velocity_error_m_s,3)+' m/s']);
    if(d.robust){
      $('analysis-heading').textContent='Robust GNC · finite thrust and ground tracking';
      list.push(['Visible ground measurements',String(m.measurement_count)],
        ['Visible station epochs',String(m.visible_measurement_epochs)],
        ['Fuel consumed',formatNumber(m.fuel_consumed_kg,5)+' kg'],
        ['Actual integrated engine Δv',formatNumber(m.actual_integrated_delta_v_m_s,4)+' m/s'],
        ['Mean NIS · 2D sensor',m.mean_nis===null?'N/A':formatNumber(m.mean_nis,3)],
        ['Final NEES · 6D state',m.final_nees===null?'N/A':formatNumber(m.final_nees,2)],
        ['Force-model agreement',d.input.truth_j2_enabled===d.input.filter_j2_enabled?'Shared J₂ setting':'J₂ model mismatch']);
    }
    for(const e of d.events||[])list.push(['Burn at T+'+formatNumber(e.time_s/60,1)+' min',formatNumber(e.delta_v_m_s,4)+' m/s'+(d.robust?' · '+formatNumber(e.duration_s,3)+' s':'')]);
  }else if(d.finite_optimization){
    const o=d.finite_optimization,c=o.selected;
    $('analysis-heading').textContent='Engine-duration optimization';
    list.push(['Selected departure phase',formatNumber(c.phase_deg,1)+'°'],
      ['Departure burn',formatNumber(c.departure_duration_s,2)+' s'],
      ['Arrival burn',formatNumber(c.arrival_duration_s,2)+' s'],
      ['Apsis arrival time',formatNumber(c.apsis_event_s/60,3)+' min'],
      ['Propellant consumed',formatNumber(c.propellant_used_kg,3)+' kg'],
      ['Propellant remaining',formatNumber(c.propellant_remaining_kg,3)+' kg'],
      ['Delivered engine Δv · ideal integral',formatNumber(c.delivered_ideal_delta_v_m_s,3)+' m/s'],
      ['Apsis radius residual',formatNumber(c.target_apsis_residual_m,5)+' m'],
      ['Postburn eccentricity',formatNumber(c.postburn_eccentricity,7)],
      ['Postburn perigee / apogee',formatNumber(c.postburn_periapsis_altitude_km,2)+' / '+formatNumber(c.postburn_apoapsis_altitude_km,2)+' km'],
      ['Final radius error',formatNumber(c.arrival_radius_error_km*1000,1)+' m'],
      ['Phase grid',String(o.phase_candidates)+' sampled']);
    for(const t of o.thrust_strategy_comparison||[]){
      const level=t.relative_thrust+'× thrust';
      list.push([level,t.error?'No viable solution':formatNumber(t.fuel_used_kg,2)+' kg fuel · e='+formatNumber(t.postburn_eccentricity,5)]);
    }
  }else if(d.uncertainty){
    const u=d.uncertainty, last=u.position_95pct_conservative_radius_m.at(-1);
    $('analysis-heading').textContent='Covariance & sensitivity';
    list.push(['Initial position σ',formatNumber(u.input_position_sigma_m,3)+' m / axis'],
      ['Initial velocity σ',formatNumber(u.input_velocity_sigma_mm_s,3)+' mm/s / axis'],
      ['Monte Carlo realizations',String(u.monte_carlo_samples)],
      ['Random seed',String(u.random_seed)],
      ['Final conservative 95% radius',formatNumber(last,2)+' m'],
      ['Final 1σ principal axis',formatNumber(u.position_max_axis_sigma_m.at(-1),2)+' m'],
      ['Final MC position RMS',formatNumber(u.final_mc_position_rms_m,2)+' m'],
      ['STM final 1σ axis',formatNumber(u.final_stm_position_1sigma_max_m,2)+' m'],
      ['Sample fraction inside final radius',formatNumber(100*u.empirical_fraction_within_final_95pct_radius,1)+' %'],
      ['Uncertainty frame','Nominal radial / along / cross-track']);
  }else if(d.optimization){
    const o=d.optimization,c=o.selected,b=o.fuel_budget;
    $('analysis-heading').textContent='Minimum-fuel phase search · ideal impulses';
    list.push(['Target',formatNumber(o.starting_altitude_km,1)+' → '+formatNumber(o.target_altitude_km,1)+' km'],
      ['Selected departure phase',formatNumber(c.phase_deg,1)+'°'],
      ['Phase grid',String(o.phase_candidates)+' discrete candidates'],
      ['Departure burn',formatNumber(c.departure_delta_v_m_s,4)+' m/s'],
      ['Arrival circularization',formatNumber(c.arrival_delta_v_m_s,4)+' m/s'],
      ['Total ideal Δv',formatNumber(c.total_delta_v_m_s,4)+' m/s'],
      ['Required fuel',formatNumber(c.fuel_required_kg,4)+' kg'],
      ['Fuel margin',formatNumber(b.propellant_margin_kg,3)+' kg'],
      ['Apsis event residual',formatNumber(c.target_radius_residual_m,5)+' m'],
      ['Event time',formatNumber(c.arrival_time_s/60,3)+' min'],
      ['Propellant feasibility',b.feasible?'✓ FEASIBLE':'✕ SHORTFALL']);
  }else if(d.validation){
    const v=d.validation;
    $('analysis-heading').textContent='Matched-epoch reference residuals';
    list.push(['Reference frame',v.frame],['Epoch timescale',v.time_scale],
      ['Reference samples',String(v.samples)],
      ['Position RMS residual',formatNumber(v.position_rms_km*1000,3)+' m'],
      ['Position maximum',formatNumber(v.position_max_km*1000,3)+' m'],
      ['Position final',formatNumber(v.position_final_km*1000,3)+' m'],
      ['Velocity RMS',formatNumber(v.velocity_rms_m_s,5)+' m/s'],
      ['Comparison','Point-mass vs imported states']);
  }else if(d.comparison){
    $('analysis-heading').textContent='Trajectory comparison · common timeline';
    const c=d.comparison;
    list.push(['Baseline',d.validation?'Imported Earth-centered ICRF reference vectors':'Two-body gravity + identical burns'],
      ['Target',c.target_model],
      ['Final position separation',formatNumber(c.final_separation_km,5)+' km'],
      ['Maximum separation',formatNumber(c.max_separation_km,5)+' km'],
      ['Final velocity difference',formatNumber(c.final_velocity_difference_m_s,5)+' m/s'],
      ['Compare duration',formatNumber(c.comparison_duration_s/60,2)+' min'],
      ...(d.validation?[]:[['Target propellant remaining',formatNumber(c.target_final_fuel_kg,3)+' kg']]));
  }else if(d.targeting){
    const t=d.targeting,b=t.fuel_budget;
    $('analysis-heading').textContent='Numerical apsis target';
    list.push(['Target',formatNumber(t.start_altitude_km,1)+' → '+formatNumber(t.target_altitude_km,1)+' km'],
      ['Departure impulse',formatNumber(t.departure_delta_v_m_s,3)+' m/s'],
      ['Arrival circularization',formatNumber(t.arrival_delta_v_m_s,3)+' m/s'],
      ['Total Δv',formatNumber(t.total_delta_v_m_s,3)+' m/s'],
      ['Bisection iterations',String(t.iterations)],
      ['Apsis residual',formatNumber(t.solver_residual_km*1000,5)+' m'],
      ['Propagated arrival error',formatNumber(t.numerical_arrival_radius_error_km*1000,4)+' m'],
      ['Half-step refinement',formatNumber(t.step_refinement_position_difference_m,5)+' m'],
      ['Time to opposite apsis',formatNumber(t.transfer_time_s/60,2)+' min'],
      ['Required propellant',formatNumber(b.required_propellant_kg,3)+' kg'],
      ['Fuel feasibility',b.feasible?'✓ FEASIBLE':'✕ SHORTFALL']);
  }else if(d.reference){
    $('analysis-heading').textContent='Imported reference vectors';
    list.push(['Source',d.reference.origin], ['Reference system','Earth-centered ICRF'],
      ['Reference timescale','TDB (not UTC)'],
      ['First JD (TDB)',formatNumber(d.reference.epoch_start_jd_tdb,6)],
      ['Last JD (TDB)',formatNumber(d.reference.epoch_end_jd_tdb,6)],
      ['Imported samples',String(d.metrics.samples)],
      ['Model error comparison','NOT ATTEMPTED']);
  }else if(d.satellite){
    $('analysis-heading').textContent='Catalog orbit · SGP4';
    list.push(['Satellite',d.satellite.name],['NORAD catalog',String(d.satellite.catalog_number)],
              ['OMM epoch',d.satellite.epoch_utc],['Reference frame','TEME'],
              ['Min / max altitude',formatNumber(d.metrics.min_altitude_km,1)+' / '+formatNumber(d.metrics.max_altitude_km,1)+' km']);
  }else if(d.model?.force_terms?.includes('Earth J2 geopotential') || d.input?.drag_enabled || d.input?.j2_enabled===false){
    $('analysis-heading').textContent='Perturbation analysis';
    list.push(['Active forces',d.model.force_terms.join(' + ')],
              ['Δ RAAN',formatNumber(d.metrics.delta_raan_deg,5)+'°'],
              ['Δ semi-major axis',formatNumber(d.metrics.delta_semi_major_axis_km,4)+' km'],
              ['Energy change',formatNumber(d.metrics.relative_specific_energy_change*100,5)+' %'],
              ['Min altitude',formatNumber(d.metrics.min_altitude_km,2)+' km']);
  }else if(d.transfer){
    $('analysis-heading').textContent='Transfer maneuver budget';
    const tr=d.transfer;
    list.push(['Departure impulse',formatNumber(tr.departure_delta_v_km_s*1000,2)+' m/s'],
              ['Arrival impulse',formatNumber(tr.arrival_delta_v_km_s*1000,2)+' m/s'],
              ['Total Δv',formatNumber(tr.total_delta_v_km_s*1000,2)+' m/s'],
              ['Transfer flight time',formatNumber(tr.time_of_flight_s/60,2)+' min'],
              ['Arrival radius error',formatNumber(tr.arrival_radius_error_km*1000,3)+' m']);
    if(tr.fuel_budget){
      const b=tr.fuel_budget;
      list.push(['Required propellant',formatNumber(b.required_propellant_kg,2)+' kg'],
                ['Fuel available',formatNumber(b.available_propellant_kg,2)+' kg'],
                ['Fuel margin',formatNumber(b.propellant_margin_kg,2)+' kg'],
                ['Mission feasibility',b.feasible?'✓ FUEL SUFFICIENT':'✕ FUEL SHORTFALL'],
                ['Insertion eccentricity',formatNumber(tr.post_circularization_eccentricity,7)]);
    }
  }else if(d.burn_reports){
    $('analysis-heading').textContent='Engine & propellant budget';
    const m=d.metrics;
    list.push(['Wet / dry mass',formatNumber(d.spacecraft_mass_kg,1)+' / '+formatNumber(m.dry_mass_kg,1)+' kg'],
              ['Propellant consumed',formatNumber(m.propellant_used_kg,3)+' kg'],
              ['Propellant remaining',formatNumber(m.propellant_remaining_kg,3)+' kg'],
              ['Engine Δv · ideal integral',formatNumber(m.ideal_thrust_delta_v_m_s,2)+' m/s'],
              ['Final semi-major axis',formatNumber(d.final_orbital_elements.semi_major_axis_km,1)+' km']);
    for(const [i,b] of d.burn_reports.entries()){
      list.push([`Burn ${i+1} · ${b.direction}`,formatNumber(b.fuel_used_kg,2)+' kg / '+formatNumber(b.delivered_ideal_delta_v_m_s,2)+' m/s']);
    }
  }else{
    $('analysis-heading').textContent='Orbit characteristics';
    const el=d.orbital_elements;
    list.push(['Semi-major axis',formatNumber(el.semi_major_axis_km,2)+' km'],
              ['Perigee altitude',formatNumber(el.periapsis_altitude_km,2)+' km'],
              ['Apogee altitude',el.apoapsis_altitude_km==null?'Unbound':formatNumber(el.apoapsis_altitude_km,2)+' km'],
              ['Inclination',formatNumber(el.inclination_deg,2)+'°'],
              ['RAAN',formatNumber(el.raan_deg,2)+'°']);
  }
  for(const [label,value] of list)container.appendChild(analysisRow(label,value));
}
function updateTelemetry(){
  const d=app.data;if(!d)return;
  const idx=currentIndex();const tr=d.trajectory;
  const r=tr.position_km[idx],v=tr.velocity_km_s[idx];
  setText('stat-altitude',formatNumber(tr.altitude_km[idx],1));
  setText('stat-speed',formatNumber(tr.speed_km_s[idx],3));
  setText('stat-period',d.orbital_elements.period_s?formatNumber(d.orbital_elements.period_s/60,2):'N/A');
  setText('stat-eccentricity',formatNumber(d.orbital_elements.eccentricity,5));
  if(tr.wet_mass_kg){
    const remaining=tr.propellant_remaining_kg[idx];
    const initial=d.metrics.initial_propellant_kg;
    setText('stat-fuel',formatNumber(remaining,1));
    setText('live-wet-mass',formatNumber(tr.wet_mass_kg[idx],2)+' kg');
    setText('live-prop',formatNumber(remaining,2)+' kg');
    $('fuel-progress').style.width=(initial>0?Math.max(0,Math.min(100,remaining/initial*100)):0)+'%';
    const active=(d.input.burns||[]).some(b=>app.simTime>=b.start_s&&app.simTime<b.start_s+b.duration_s&&remaining>1e-8);
    setText('engine-telemetry-status',active?'● FIRING':'○ COAST');
    setText('live-delivered-dv',formatNumber(tr.cumulative_ideal_delta_v_m_s[idx],2)+' m/s');
  }
  ['x','y','z'].forEach((axis,j)=>{
    setText('pos-'+axis,formatNumber(r[j],3));setText('vel-'+axis,formatNumber(v[j],5));
  });
  setText('current-time',timeLabel(app.simTime));
  $('scrubber').value=Math.round(app.simTime/(tr.time_s.at(-1)||1)*1000);
}
function updateDiagnostics(){
  const d=app.data;if(!d)return;
  const metrics=d.metrics;
  setText('step-value',formatNumber(metrics.step_s,2)+' s');
  setText('count-value',metrics.samples.toLocaleString());
  const applicable=metrics.conservation_applicable!==false;
  const comparing=Boolean(d.comparison);
  setText('energy-value',applicable?sci(metrics.max_relative_energy_drift):'N/A');
  setText('momentum-value',applicable?sci(metrics.max_relative_angular_momentum_drift):'N/A');
  const badge=$('conservation-badge');
  const trustworthy=applicable && metrics.max_relative_energy_drift<1e-5&&metrics.max_relative_angular_momentum_drift<1e-5;
  badge.className=applicable?(trustworthy?'pass-label ok':'pass-label warn'):'pass-label';
  badge.textContent=applicable?(trustworthy?'● NUMERICALLY STABLE':'● CHECK RESOLUTION'):'● NOT APPLICABLE';
  const powered=Boolean(d.burn_reports||d.attitude);
  $('integrity-title').textContent=d.ground_navigation?'Position error (cyan) vs 1σ RSS (orange) · m':d.observability?'Batch-fit position error vs nominal 1σ (m)':d.adaptive_navigation?'Adaptive position error vs 1σ RSS (meters)':d.joint_integrity?'Joint position error vs 1σ RSS (meters)':d.integrity?'Position error (cyan) vs estimated 1σ RSS (orange) · meters':d.joint_navigation?'Position error (cyan) vs joint 1σ RSS (orange) · meters':d.integrated_navigation?'Position error (cyan) vs 1σ filter RSS (orange) · meters':(d.adcs||d.precision_adcs)?'True pointing (cyan) vs estimation error (orange) · degrees':d.attitude?'Body +X pointing error · degrees':d.robust?'Robust finite-burn guidance · meters':d.guidance?'Guided vs unguided error · meters':d.uncertainty?'Position uncertainty envelope · meters':d.finite_optimization?'Finite-thrust optimizer · checked':comparing?'Trajectory separation vs. elapsed time':d.optimization?'Event-based apsis targeting · checked':d.reference?'Reference vectors · no integration':powered?'Coast-only solver integrity':applicable?'Conservation monitor':'Nonconservative model';
  $('integrity-desc').textContent=d.ground_navigation?'Only received range and range-rate packets update the 7-state EKF at their capture epoch. DOP853 truth is independent of RK4 prediction.':d.observability?'Cyan: noncausal batch-fit error against independent nonlinear synthetic truth. Orange: conditional 1σ from the assumed local model; faults and time correlation can invalidate it.':d.adaptive_navigation?'Uncertain engine scale is estimated only from synthetic observations, with configurable fault-response policies.':d.joint_integrity?'Sensor-specific 3-DoF innovation alarms; classification cannot distinguish a faulty position sensor from an engine-model error.':d.integrity?'Fault alarms and sensor correlation are reported in Mission Analysis. Chart compares observed error against nominal EKF covariance.':d.joint_navigation?'12-state joint EKF: cross-covariance, finite body-axis thrust, delayed packet replay. Synthetic sensors only.':d.integrated_navigation?'Causal packet replay with optional approximate powered attitude covariance. It does not correct thrust-model bias or capture position/attitude cross correlation.':(d.adcs||d.precision_adcs)?'Cyan: actual engine-axis pointing error. Orange: attitude-estimator angular error; both are compared to simulated truth for diagnosis, not fed to the controller.':d.attitude?'Numerically propagated spacecraft attitude: angular error between the body-fixed engine axis and desired thrust direction. Orange shading marks scheduled burn.':d.robust?'Cyan: finite-burn guided error. Orange: uncorrected orbit; range and range-rate fixes from hypothetical rotating stations.':d.guidance?'Cyan: error under estimated-state feedback. Orange: identical initial error with no corrections. Idealized model and synthetic measurements.':d.uncertainty?'The orange line shows the conservative 95% position radius from sampled initial-state uncertainty. It is not a collision-probability calculation.':d.finite_optimization?'Two finite burns are integrated with mass depletion; the departure apsis is targeted numerically and postburn eccentricity remains measurable.':d.optimization?'Targeting uses a propagated radial-velocity zero crossing with adaptive integration. See the mission analysis panel for the event-radius residual and fuel check.':d.reference?'These are imported reference samples, not a numerical propagation. No conservation or error comparison is implied.':d.validation?'The orange line shows the point-mass model residual against matched user-attested ICRF reference vectors, measured at identical Julian TDB epochs.':comparing?'The orange line shows inertial position separation in kilometers, with identical initial state and scheduled burns. This is a model sensitivity study, not validated ephemeris error.':powered?'Coast invariants use the selected J₂ potential when enabled. With thrust, only coast segments are considered; drag disables conservation tests.':d.satellite?'SGP4 includes a fitted perturbation model. Two-body conservation charts are not valid.':d.input?.drag_enabled?'Drag physically dissipates orbital energy; conservation charts are intentionally disabled. Physical energy change is reported separately.':d.input?.j2_enabled?'Conserved quantities include the J₂ potential and axial angular momentum (h_z), not the two-body orbital-energy/h-magnitude pair.':'Errors are measured against initial two-body invariants.';
  document.querySelector('.chart-key').classList.toggle('hidden',!applicable);
  $('fuel-tile').classList.toggle('hidden',!powered);
  $('propulsion-telemetry').classList.toggle('hidden',!powered);
  document.querySelector('.metrics-grid').classList.toggle('with-fuel',powered);
  const notices=d.warnings||[];
  $('warning-box').classList.toggle('hidden',notices.length===0);
  $('warning-text').textContent=notices.join(' ');
  $('model-notice').textContent=d.ground_navigation?'Simulated spherical ground network with approximate sidereal rotation and finite burn. Not a real station, ITRS frame, operational sensor or external validation.':d.observability?'Offline seven-parameter linearized fit using DOP853 variational sensitivities. Not a causal or joint attitude navigation filter; Gaussian coverage is only a model assumption.':d.adaptive_navigation?'13-state error EKF with thrust-scale calibration, per-channel NIS monitoring, heuristic adaptive R and optional temporary quarantine. Educational and unvalidated.':d.joint_integrity?'Joint 12×12 EKF with separate position/tracker gates, causal packet replay, AR(1) noise and thrust-covariance inflation. Educational, not flight qualified.':d.integrity?'Independent DOP853 truth, RK4 six-state EKF, AR(1) sensor noise, engine scale uncertainty and chi-square innovation gating. Not flight validated.':d.joint_navigation?'True 12×12 error covariance with attitude/position cross-correlation; synthetic gyro, tracker and Cartesian measurements. Not flight-validated.':d.integrated_navigation?'Ideal inertial Cartesian or synthetic ground tracking plus separate attitude MEKF. Optional thrust covariance injection is not a joint filter; large burn mismatch may remain.':d.precision_adcs?'Six-state error MEKF and ideal dipole-limited magnetorquer unloading. Synthetic fixed-Sun and axial-dipole geometry; not flight grade.':d.adcs?'Three momentum-limited reaction wheels, noisy gyro and quaternion star tracker, and a complementary attitude filter. No wheel desaturation; educational model only.':d.attitude?'Rigid-body quaternion dynamics with finite control torques and body-fixed propulsion. Direct idealized PD torque, no navigation sensing or wheel saturation. Not flight software.':d.robust?'Finite-duration mass-depleting feedback burns with simulated engine errors, geometric station range/range-rate tracking and a 6×6 EKF. Hypothetical stations. Not flight software.':d.guidance?'Extended Kalman filter with noisy position fixes, numerical 6×6 STM and bounded ideal impulsive guidance. This is not operational GN&C.':d.uncertainty?'Monte Carlo and six-by-six Cartesian sensitivity; idealized model, no sensor error or process noise.':d.finite_optimization?'Two noninstantaneous engine burns with fuel depletion and optional J₂, bounded departure phases and reported circularization error.':d.reference?'User-supplied Earth-centered ICRF ephemeris, Julian TDB epochs. Do not subtract directly from TEME or idealized ECI trajectories.':d.validation?'Point-mass propagation initialized from imported Earth-centered ICRF six-state; samples at exact TDB epochs. Reference origin not authenticated.':d.optimization?'Finite grid minimum-fuel phase search with DOP853 event localization and ideal instantaneous burns. Not continuous or globally optimal.':d.targeting?'Apsidal impulse is solved by bisection, then the trajectory is numerically propagated. Assumes a circular initial orbit and ideal tangential burns.':comparing?'Two trajectories computed with identical initial conditions, burns and output times. Comparison isolates the effect of J₂ and optional drag.':d.burn_reports?'Coupled Earth point gravity, optional J₂ and atmosphere drag, plus continuously steered thrust and changing wet mass. Drag density is illustrative; no third bodies.':d.satellite?'Real catalog GP/OMM elements propagated by SGP4 in TEME; do not treat as a ground-fixed frame.':d.input?.density_at_400_km_kg_m3!==undefined?'Numerical Earth J₂ + optional co-rotating exponential drag. Density is user-provided and not linked to solar activity.':powered?'Earth point-mass gravity + continuously steered finite thrust, with mass loss. No drag, J₂, attitude dynamics, or third bodies.':d.transfer?'Earth two-body transfer; ideal tangential impulses with rocket-equation mass budget, no finite burn losses.':'Unperturbed Earth point-mass gravity. No drag, J₂, finite thrust or third-body forces in Orbit mode.';
  drawDriftChart();
}
function applyResponse(data){
  if(!data?.trajectory?.position_km?.length)throw Error('The solver returned no trajectory samples');
  app.data=data;
  app.simTime=0;app.playing=true;setText('play-pause','Ⅱ');
  const points=data.trajectory.position_km;
  app.maxRadius=Math.max(R+250,...points.map(p=>Math.hypot(...p)));
  if(data.comparison)app.maxRadius=Math.max(app.maxRadius,...data.comparison.baseline_trajectory.position_km.map(p=>Math.hypot(...p)));
  if(data.transfer)app.maxRadius=Math.max(app.maxRadius,R+data.transfer.start_altitude_km,R+data.transfer.target_altitude_km);
  setText('duration-time',timeLabel(data.trajectory.time_s.at(-1)));
  setText('view-title',data.ground_navigation?'Ground navigation · causal measurement replay':data.observability?'Thrust observability · robust calibration':data.adaptive_navigation?'Adaptive navigation · thrust calibration and recovery':data.joint_integrity?'Joint navigation · fault isolation':data.integrity?'Navigation integrity · fault monitoring':data.joint_navigation?'Joint orbit + attitude navigation · 12-state EKF':data.integrated_navigation?'Integrated time-tagged navigation · 6+6 estimators':data.precision_adcs?'Precision ADCS · MEKF and magnetic unloading':data.adcs?'ADCS · gyro, star tracker & reaction wheels':data.attitude?'Integrated attitude + orbit · body-fixed thrust':data.robust?'Robust GNC · finite burns and ground tracking':data.guidance?'Closed-loop guidance · EKF & corrections':data.uncertainty?'Navigation uncertainty · Monte Carlo':data.finite_optimization?'Optimized two-burn finite-thrust transfer':data.optimization?'Event-based ideal impulse optimization':data.validation?'Frame-aligned reference residuals':data.targeting?'Apsis-targeted maneuver · RK4 verification':data.reference?'Horizons reference vectors · imported':data.comparison?'Force-model comparison · matched burns':data.satellite?('Satellite · '+data.satellite.name):data.input?.density_at_400_km_kg_m3!==undefined?'Perturbed orbital dynamics':data.transfer?'Hohmann transfer + fuel budget':data.burn_reports?'Finite-thrust mission trajectory':'Earth orbit propagation');
  $('view-note').textContent=data.ground_navigation?'SYNTHETIC RANGE/RANGE-RATE · CAUSAL PACKETS':data.observability?'SEVEN-PARAMETER BATCH FIT · EMPIRICAL COVERAGE':data.adaptive_navigation?'JOINT 13×13 COVARIANCE · NIS SENSOR RESPONSE':data.joint_integrity?'JOINT 12×12 EKF · CHANNEL NIS GATES':data.integrity?'SYNTHETIC FAULT INJECTION · χ² INNOVATION GATE':data.joint_navigation?'JOINT 12×12 COVARIANCE · FINITE THRUST':data.integrated_navigation?'CAUSAL CAPTURE/RECEIPT · ORBIT + ATTITUDE':data.precision_adcs?'MEKF · TRACKER CONES · MAGNETIC UNLOADING':data.adcs?'CYAN: POINTING · AMBER: ATTITUDE ESTIMATE ERROR':data.attitude?'AMBER ARROW: ACTUAL BODY +X · INERTIAL':data.robust?'FINITE THRUST · STATION RANGE/RATE · EKF':data.guidance?'CYAN GUIDED · ORANGE REFERENCE · INERTIAL':data.uncertainty?'GAUSSIAN INITIAL STATE · NUMERIC STM':data.finite_optimization?'ENGINE FIRINGS · MASS DEPLETION':data.validation?'ICRF · EPOCH-MATCHED · USER-ATTESTED':data.optimization?'APSIDAL EVENT · SAMPLED PHASES':data.reference?'EARTH-CENTERED ICRF · JD TDB · IMPORTED':data.targeting?'TARGET IMPULSES · IDEAL TWO-BODY':data.comparison?'CYAN: J₂ / DRAG · ORANGE: TWO-BODY':data.satellite?'SGP4 · TEME FRAME · GLOBE IS SCHEMATIC':'DRAG TO ROTATE · SCROLL TO ZOOM';
  $('scene-loading').classList.add('hidden');
  updateAnalysis();updateDiagnostics();updateTelemetry();drawScene();
}
function readRobustForm(){
  return {altitude_km:Number($('rg-alt').value),inclination_deg:Number($('rg-inc').value),
        duration_minutes:Number($('rg-duration').value),measurement_interval_s:Number($('rg-period').value),
        range_sigma_m:Number($('rg-noise').value),range_rate_sigma_m_s:Number($('rg-rate').value),
        initial_position_error_m:Number($('rg-poserr').value),initial_velocity_error_m_s:Number($('rg-velerr').value),
        thrust_n:Number($('rg-thrust').value),specific_impulse_s:Number($('rg-isp').value),
        dry_mass_kg:Number($('rg-dry').value),fuel_mass_kg:Number($('rg-fuel').value),
        max_total_delta_v_m_s:Number($('rg-dv').value),corrections:Number($('rg-burns').value),
        thrust_error_pct:Number($('rg-thrust-error').value),pointing_error_deg:Number($('rg-pointing').value),
        station_count:Number($('rg-stations').value),seed:Number($('rg-seed').value),
        guidance_enabled:$('rg-enable').checked,truth_j2_enabled:$('rg-true-j2').checked,
        filter_j2_enabled:$('rg-filter-j2').checked};
}
async function submitRequest(mode,initial=false){
  if(app.loading)return;
  const btn=$(({orbit:'orbit-submit',transfer:'transfer-submit',mission:'mission-submit',compare:'compare-submit',perturb:'perturb-submit',satellite:'satellite-submit',target:'target-submit',reference:'reference-submit',optimize:'optimize-submit',finite:'finite-submit',uncertainty:'uncertainty-submit',guidance:'guidance-submit',robust:'robust-submit',attitude:'attitude-submit',adcs:'adcs-submit',precision:'precision-submit',integrated:'integrated-submit',joint:'joint-submit',integrity:'integrity-submit','joint-integrity':'joint-integrity-submit',adaptive:'adaptive-submit',observability:'observability-submit',ground:'ground-submit',validate:'validate-submit'})[mode]);
  app.loading=true;btn.disabled=true;
  $('scene-loading').textContent='Running the selected orbital model…';$('scene-loading').classList.remove('hidden');
  try{
    let payload={},endpoint;
    if(mode==='orbit'){
      endpoint='/api/propagate';
      const keys=['perigee_altitude_km','eccentricity','inclination_deg','raan_deg','argument_of_periapsis_deg','true_anomaly_deg','velocity_factor','spacecraft_mass_kg','duration_orbits','samples_per_orbit'];
      for(const k of keys)payload[k]=Number($(k).value);
    }else if(mode==='transfer'){
      endpoint='/api/transfer';
      payload={start_altitude_km:Number($('start_altitude_km').value),
        target_altitude_km:Number($('target_altitude_km').value),
        inclination_deg:Number($('transfer_inclination_deg').value),
        dry_mass_kg:Number($('transfer_dry_kg').value),
        propellant_kg:Number($('transfer_prop_kg').value),
        specific_impulse_s:Number($('transfer_isp_s').value),
        samples:Number($('transfer_samples').value)};
    }else if(mode==='perturb'){
      endpoint='/api/perturbations';
      payload={perigee_altitude_km:Number($('p-alt').value),eccentricity:Number($('p-ecc').value),
        inclination_deg:Number($('p-inc').value),raan_deg:Number($('p-raan').value),
        j2_enabled:$('p-j2').checked,drag_enabled:$('p-drag').checked,
        mass_kg:Number($('p-mass').value),cross_section_m2:Number($('p-area').value),
        drag_coefficient:Number($('p-cd').value),density_at_400_km_kg_m3:Number($('p-density').value),
        scale_height_km:Number($('p-scale').value),duration_hours:Number($('p-hours').value),
        samples:Number($('p-samples').value)};
    }else if(mode==='target'){
      endpoint='/api/target';
      payload={start_altitude_km:Number($('t-start').value),target_altitude_km:Number($('t-target').value),
        inclination_deg:Number($('t-inc').value),dry_mass_kg:Number($('t-dry').value),
        propellant_kg:Number($('t-prop').value),specific_impulse_s:Number($('t-isp').value),
        samples:Number($('t-samples').value)};
    }else if(mode==='optimize'){
      endpoint='/api/optimize';
      payload={start_altitude_km:Number($('o-start').value),target_altitude_km:Number($('o-target').value),
        inclination_deg:Number($('o-inc').value),dry_mass_kg:Number($('o-dry').value),
        propellant_kg:Number($('o-fuel').value),specific_impulse_s:Number($('o-isp').value),
        phase_candidates:Number($('o-phases').value),j2_enabled:$('o-j2').checked,samples:360};
    }else if(mode==='finite'){
      endpoint='/api/finite/optimize';
      payload={start_altitude_km:Number($('fb-start').value),target_altitude_km:Number($('fb-target').value),
        inclination_deg:Number($('fb-inc').value),dry_mass_kg:Number($('fb-dry').value),
        propellant_kg:Number($('fb-fuel').value),specific_impulse_s:Number($('fb-isp').value),
        thrust_n:Number($('fb-thrust').value),phase_candidates:Number($('fb-phases').value),
        j2_enabled:$('fb-j2').checked,samples:280};
    }else if(mode==='guidance'){
      endpoint='/api/guidance';
      payload={altitude_km:Number($('g-alt').value),inclination_deg:Number($('g-inc').value),
        duration_minutes:Number($('g-duration').value),measurement_interval_s:Number($('g-period').value),
        measurement_sigma_m:Number($('g-noise').value),initial_position_error_m:Number($('g-poserr').value),
        initial_velocity_error_m_s:Number($('g-velerr').value),max_total_delta_v_m_s:Number($('g-dv').value),
        corrections:Number($('g-burns').value),guidance_enabled:$('g-enable').checked,
        j2_enabled:$('g-j2').checked,seed:Number($('g-seed').value)};
    }else if(mode==='attitude'){
      endpoint='/api/attitude/mission';
      payload={altitude_km:Number($('a-alt').value),inclination_deg:Number($('a-inc').value),
        duration_minutes:Number($('a-duration').value),initial_misalignment_deg:Number($('a-error').value),
        target_mode:$('a-target').value,controller_enabled:$('a-control').checked,
        max_control_torque_nm:Number($('a-torque').value),kp_nm_per_rad:Number($('a-kp').value),kd_nms_per_rad:Number($('a-kd').value),
        burn_start_s:Number($('a-start').value),burn_duration_s:Number($('a-burn').value),
        thrust_n:Number($('a-thrust').value),specific_impulse_s:Number($('a-isp').value),
        dry_mass_kg:Number($('a-dry').value),fuel_mass_kg:Number($('a-fuel').value),
        j2_enabled:$('a-j2').checked,gravity_gradient_enabled:$('a-gradient').checked};
    }else if(mode==='adcs'){
      endpoint='/api/adcs/mission';
      payload={altitude_km:Number($('ad-alt').value),duration_minutes:Number($('ad-duration').value),
        initial_misalignment_deg:Number($('ad-offset').value),initial_estimate_error_deg:Number($('ad-estimate').value),
        target_mode:$('ad-target').value,controller_enabled:$('ad-control').checked,tracker_enabled:$('ad-tracker').checked,
        gyro_bias_deg_s:Number($('ad-bias').value),gyro_noise_deg_s:Number($('ad-gyro-noise').value),
        tracker_noise_deg:Number($('ad-star-noise').value),tracker_interval_s:Number($('ad-track-period').value),
        tracker_dropout_start_s:Number($('ad-blackout-start').value),tracker_dropout_duration_s:Number($('ad-blackout-time').value),
        wheel_max_torque_nm:Number($('ad-torque').value),wheel_momentum_limit_nms:Number($('ad-capacity').value),
        burn_start_s:Number($('ad-start').value),burn_duration_s:Number($('ad-burn').value),seed:Number($('ad-seed').value)};
    }else if(mode==='precision'){
      endpoint='/api/precision-adcs/mission';
      payload={altitude_km:Number($('px-alt').value),duration_minutes:Number($('px-duration').value),
        target_mode:$('px-target').value,controller_enabled:$('px-control').checked,
        tracker_enabled:$('px-tracker').checked,tracker_visibility_enabled:$('px-visibility').checked,
        tracker_noise_deg:Number($('px-star-noise').value),tracker_interval_s:Number($('px-track-period').value),
        star_tracker_sun_exclusion_deg:Number($('px-sun').value),star_tracker_earth_limb_margin_deg:Number($('px-earth').value),
        gyro_bias_deg_s:Number($('px-bias').value),gyro_bias_random_walk_deg_s_sqrt_s:Number($('px-rw').value),
        initial_wheel_momentum_nms:Number($('px-initial-h').value),wheel_momentum_limit_nms:Number($('px-capacity').value),
        desaturation_enabled:$('px-unload').checked,magnetorquer_max_dipole_am2:Number($('px-magnetic').value),
        desaturation_gain_per_s:Number($('px-gain').value),burn_start_s:Number($('px-start').value),
        burn_duration_s:Number($('px-burn').value),seed:Number($('px-seed').value)};
    }else if(mode==='ground'){
      endpoint='/api/navigation/ground-tracking';
      const duration=Number($('gn-duration').value);
      payload={altitude_km:Number($('gn-alt').value),duration_minutes:duration,
        samples:duration*4+1,station_count:Number($('gn-stations').value),
        min_elevation_deg:Number($('gn-elev').value),range_sigma_m:Number($('gn-range').value),
        range_rate_sigma_m_s:Number($('gn-rate').value),measurement_interval_s:Number($('gn-period').value),
        telemetry_delay_s:Number($('gn-delay').value),burn_start_s:180,
        burn_duration_s:Number($('gn-burn').value),true_thrust_scale:Number($('gn-scale').value),
        fault_bias_m:Number($('gn-fault').value),monte_carlo_runs:Number($('gn-runs').value),
        seed:Number($('gn-seed').value)};
    }else if(mode==='observability'){
      endpoint='/api/navigation/observability';
      const duration=Number($('ob-duration').value);
      payload={altitude_km:Number($('ob-alt').value),duration_minutes:duration,
        samples:duration*5+1,observation_interval_s:Number($('ob-period').value),
        burn_start_s:180,burn_duration_s:Number($('ob-burn').value),
        true_thrust_scale:Number($('ob-scale').value),observation_sigma_m:Number($('ob-noise').value),
        fault_bias_m:Number($('ob-fault').value),fault_duration_s:Number($('ob-fault-duration').value),
        fault_start_s:360,noise_correlation:Number($('ob-corr').value),
        estimator:$('ob-method').value,monte_carlo_runs:Number($('ob-runs').value),
        seed:Number($('ob-seed').value)};
    }else if(mode==='adaptive'){
      endpoint='/api/navigation/adaptive';
      const duration=Number($('av-duration').value);
      payload={altitude_km:Number($('av-alt').value),duration_minutes:duration,
        samples:duration*4,navigation_period_s:60,tracker_period_s:60,
        telemetry_latency_s:15,tracker_latency_s:15,
        actual_thrust_error_pct:Number($('av-true-thrust').value),
        engine_model_error_pct:Number($('av-assumed-thrust').value),
        fault_channel:$('av-fault-channel').value,position_fault_m:Number($('av-fault-m').value),
        sensor_correlation:Number($('av-corr').value),
        measurement_response:$('av-policy').value,monte_carlo_runs:Number($('av-runs').value),
        seed:Number($('av-seed').value),estimate_thrust_scale:$('av-calibrate').checked,
        adaptive_noise:$('av-adapt').checked,burn_start_s:180,burn_duration_s:120};
    }else if(mode==='joint-integrity'){
      endpoint='/api/navigation/joint-integrity';
      const duration=Number($('ji-duration').value);
      payload={altitude_km:Number($('ji-alt').value),duration_minutes:duration,
        samples:duration*4,navigation_period_s:60,tracker_period_s:60,
        telemetry_latency_s:15,tracker_latency_s:15,
        fault_channel:$('ji-channel').value,fault_start_s:Number($('ji-fault-start').value),
        position_fault_m:Number($('ji-pos-fault').value),tracker_fault_deg:Number($('ji-star-fault').value),
        sensor_correlation:Number($('ji-corr').value),engine_model_error_pct:Number($('ji-engine').value),
        thrust_sigma_pct:Number($('ji-thrust-sigma').value),
        monte_carlo_runs:Number($('ji-runs').value),reject_faults:$('ji-gate').checked,
        tracker_enabled:$('ji-tracker').checked,
        burn_start_s:180,burn_duration_s:120,seed:29};
    }else if(mode==='integrity'){
      endpoint='/api/navigation/integrity';
      payload={altitude_km:Number($('ni-alt').value),duration_minutes:Number($('ni-duration').value),
        measurement_sigma_m:Number($('ni-noise').value),correlation:Number($('ni-corr').value),
        fault_start_s:Number($('ni-fault-time').value),fault_magnitude_m:Number($('ni-fault').value),
        thrust_error_pct:Number($('ni-thrust').value),monte_carlo_runs:Number($('ni-runs').value),
        seed:Number($('ni-seed').value),reject_faults:$('ni-gate').checked,
        j2_enabled:$('ni-j2').checked};
    }else if(mode==='joint'){
      endpoint='/api/navigation/joint';
      const duration=Number($('jn-duration').value);
      payload={altitude_km:Number($('jn-alt').value),duration_minutes:duration,
        samples:Math.max(36,duration*4),navigation_period_s:60,tracker_period_s:60,
        telemetry_latency_s:Number($('jn-delay').value),tracker_latency_s:Number($('jn-tracker-delay').value),
        position_sigma_m:Number($('jn-noise').value),tracker_sigma_deg:Number($('jn-tracker-noise').value),
        tracker_enabled:$('jn-star').checked,j2_enabled:$('jn-j2').checked,
        burn_start_s:Number($('jn-start').value),burn_duration_s:Number($('jn-burn').value),
        seed:Number($('jn-seed').value)};
    }else if(mode==='integrated'){
      endpoint='/api/navigation/integrated';
      payload={altitude_km:Number($('in-alt').value),duration_minutes:Number($('in-duration').value),
        samples:(()=>{const d=Math.round(Number($('in-duration').value)*60),p=Number($('in-period').value);let a=d,b=p;while(b){const c=a%b;a=b;b=c;}const g=a;return Math.min(120,Math.round(d/(g/4))<=120?Math.round(d/(g/4)):(Math.round(d/(g/2))<=120?Math.round(d/(g/2)):Math.round(d/g)));})(),navigation_period_s:Number($('in-period').value),
        telemetry_latency_s:Number($('in-latency').value),position_sigma_m:Number($('in-noise').value),
        initial_position_offset_m:Number($('in-offset').value),measurement_mode:$('in-mode').value,
        tracker_enabled:$('in-star').checked,j2_enabled:$('in-j2').checked,
        propagate_attitude_covariance:$('in-cov').checked,
        burn_start_s:Number($('in-start').value),burn_duration_s:Number($('in-burn').value),
        seed:Number($('in-seed').value)};
    }else if(mode==='robust'){
      endpoint='/api/robust/guidance';
      payload=readRobustForm();
    }else if(mode==='uncertainty'){
      endpoint='/api/uncertainty';
      payload={altitude_km:Number($('u-alt').value),inclination_deg:Number($('u-inc').value),
        duration_minutes:Number($('u-minutes').value),position_sigma_m:Number($('u-pos').value),
        velocity_sigma_mm_s:Number($('u-vel').value),monte_carlo_samples:Number($('u-samples').value),
        seed:Number($('u-seed').value),j2_enabled:$('u-j2').checked,steps:120};
    }else if(mode==='validate'){
      if(!$('v-confirm').checked)throw Error('Confirm Horizons center, frame, timescale and unit settings.');
      endpoint='/api/reference/compare';
      payload={text:$('v-text').value,metadata:{center:'500@399',frame:'ICRF',plane:'FRAME',units:'KM-S',corrections:'NONE',time_scale:'TDB'}};
    }else if(mode==='reference'){
      if(!$('r-confirm').checked)throw Error('Confirm that you checked the requested Horizons metadata settings.');
      endpoint='/api/ephemeris/import';
      payload={text:$('r-text').value,metadata:{center:'500@399',frame:'ICRF',plane:'FRAME',units:'KM-S',corrections:'NONE',time_scale:'TDB'}};
    }else if(mode==='satellite'){
      endpoint='/api/satellite/propagate';
      let parsed;
      try{parsed=JSON.parse($('s-omm').value);}catch(_){throw Error('Load an OMM record by catalog number or paste JSON first.');}
      if(Array.isArray(parsed)){if(parsed.length!==1)throw Error('Provide a single OMM record');parsed=parsed[0];}
      payload={omm:parsed,duration_minutes:Number($('s-duration').value),samples:Number($('s-samples').value)};
    }else{
      endpoint=mode==='compare'?'/api/compare':'/api/mission';
      payload={j2_enabled:$('m-j2').checked,drag_enabled:$('m-drag').checked,
        cross_section_m2:Number($('m-area').value),drag_coefficient:Number($('m-cd').value),
        density_at_400_km_kg_m3:Number($('m-density').value),perigee_altitude_km:Number($('mission_altitude').value),
        eccentricity:Number($('mission_ecc').value),inclination_deg:Number($('mission_inc').value),
        raan_deg:Number($('mission_raan').value),dry_mass_kg:Number($('mission_dry').value),
        propellant_kg:Number($('mission_prop').value),duration_minutes:Number($('mission_duration').value),
        samples:Number($('mission_samples').value),burns:readBurns()};
    }
    const response=await apiFetch(endpoint,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
    if(!response.ok){
      const json=await response.json().catch(()=>({}));
      const details=Array.isArray(json.detail)?json.detail.map(e=>`${e.loc?.at(-1)}: ${e.msg}`).join('; '):json.detail;
      throw Error(details||`Server returned ${response.status}`);
    }
    applyResponse(await response.json());
    if(!initial)document.title='APOGEE · '+({orbit:'Orbit propagation',mission:'Coupled finite thrust',compare:'Force-model comparison',transfer:'Hohmann transfer',perturb:'J2 & drag',satellite:'SGP4 satellite',target:'Numerical targeting',reference:'Reference vectors',optimize:'Ideal impulse optimization',finite:'Finite engine optimization',uncertainty:'Navigation uncertainty',guidance:'Autonomous guidance',robust:'Finite-burn robust GNC',validate:'Reference residuals',attitude:'Coupled attitude and orbit',adcs:'Sensor fusion & reaction wheels',precision:'Precision attitude navigation',integrated:'Integrated time-aligned navigation',joint:'Joint orbit/attitude estimation',integrity:'Navigation integrity and fault detection','joint-integrity':'Joint integrity and fault isolation',adaptive:'Adaptive navigation and fault recovery',observability:'Thrust observability and robustness',ground:'Ground-station tracking and causal navigation'}[mode]);
    setText('engine-status','ONLINE');
  }catch(err){
    $('scene-loading').textContent=`Simulation error: ${err.message}`;
    if(isStaticSite() && !currentApiBase) connectionUI('Connect the Python API to calculate this mission.');
    if(initial)setText('engine-status','OFFLINE');
  }finally{app.loading=false;btn.disabled=false;}
}

async function submitGeodesy(){
  const button=$('geodesy-submit'),output=$('geodesy-result');
  button.disabled=true;
  output.textContent='Computing strict epoch-aware geometry…';
  const val=id=>Number($(id).value);
  const body={epoch_utc:$('geo-epoch').value.trim(),frame:$('geo-frame').value,
    station:{latitude_deg:val('geo-lat'),longitude_deg:val('geo-lon'),height_m:val('geo-height')},
    state:{position_km:[val('geo-x'),val('geo-y'),val('geo-z')],
    velocity_km_s:[val('geo-vx'),val('geo-vy'),val('geo-vz')]},provenance:'synthetic'};
  try{
    const res=await apiFetch('/api/navigation/epoch-ground-observation',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const data=await res.json();
    if(!res.ok)throw Error(typeof data.detail==='string'?data.detail:JSON.stringify(data.detail||data));
    const azimuth=(data.azimuth_deg==null?'Undefined (zenith)':data.azimuth_deg.toFixed(3)+'°');
    setText('gd-range',data.range_m.toLocaleString('en-US',{maximumFractionDigits:2}));
    setText('gd-rate',data.range_rate_m_s.toFixed(4));
    setText('gd-elev',data.elevation_deg.toFixed(3)+'°');
    setText('gd-az',azimuth);
    setText('gd-epoch',data.epoch_utc);
    setText('gd-frames',data.state_frame+' → '+data.measurement_frame);
    setText('gd-eop',data.eop_status);
    setText('gd-status',data.validity);
    setText('model-notice','WGS84 Earth-fixed (ITRS) observation; GCRS transformation strictly depends on Astropy/IERS. No real radar or light-time calibration.');
    output.textContent=`Range: ${data.range_m.toFixed(2)} m
Geometric range rate: ${data.range_rate_m_s.toFixed(4)} m/s
Elevation: ${data.elevation_deg.toFixed(3)}°
Azimuth: ${azimuth}
Frame: ${data.state_frame} → ITRS
EOP: ${data.eop_status}
${data.validity}`;
    output.style.whiteSpace='pre-wrap';
    setText('view-title','WGS84 station geometry · '+data.state_frame+' at UTC epoch');
    setText('view-note','INSTANTANEOUS GEOMETRIC OBSERVATION · NOT RADAR / LIGHT-TIME CORRECTED');
    $('scene-loading').textContent='Ground geometry calculated; orbit visualization still depicts the previously propagated mission.';
  }catch(err){
    output.textContent='Geometry unavailable: '+err.message;
    setText('gd-status','Geometry unavailable: '+err.message);
    output.style.whiteSpace='pre-wrap';
    setText('view-title','Ground geometry · strict frame and EOP validation');
  }finally{button.disabled=false;}
}

// v2.3 independent numerical software verification. Never synthesize a PASS for missing software.
let v22Report=null;
async function submitV22(){
  const button=$('vb-run');button.disabled=true;
  setText('vb-notice','Running independent numerical comparisons…');
  try{
    const response=await apiFetch('/api/verification/v24',{
      method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({steps_per_orbit:Number($('vb-steps').value),orbits:Number($('vb-orbits').value)})});
    const report=await response.json();
    if(!response.ok)throw Error(typeof report.detail==='string'?report.detail:JSON.stringify(report.detail));
    v22Report=report;
    const kepler=report.results.find(x=>x.case==='independent_kepler_ellipse');
    const sgp=report.results.find(x=>x.case==='sgp4_multi_regime');
    const frames=report.results.find(x=>x.case==='astropy_gcrs_itrs_roundtrip');
    setText('vb-analytic-status',kepler.status);
    setText('vb-analytic-error',`Maximum velocity deviation: ${kepler.max_velocity_error_m_s.toFixed(6)} m/s`);
    setText('vb-sgp-status',sgp.status);
    setText('vb-frames',frames.status);
    setText('vb-frame-error',frames.status==='SKIP' ? frames.reason : `Position roundtrip: ${frames.position_roundtrip_error_m.toFixed(6)} m`);
    setText('vb-sgp-error',sgp.status==='SKIP'?'SGP4 not installed: no vectors compared':
      sgp.status==='PASS'?`Maximum error: ${sgp.max_position_error_m.toFixed(5)} m`:
      (sgp.reason||`Error: ${sgp.max_position_error_m} m`));
    setText('vb-position',kepler.max_position_error_m.toFixed(5)+' m');
    setText('vb-overall',report.suite_status);
    setText('vb-gate', report.release_gate.status);
    const cov=report.results.find(x=>x.case==='vallado_full_corpus_coverage');
    const convergence=report.results.find(x=>x.case==='rk4_refinement');
    setText('vb-coverage',`${cov.cases_with_vectors} cases / ${cov.state_vectors} vectors · ${cov.status}`);
    setText('vb-convergence',`${convergence.error_reduction_factor.toFixed(2)}× improvement · ${convergence.status}`);
    setText('vb-tallies',`${report.passed} passed · ${report.failed} failed · ${report.skipped} skipped`);
    setText('vb-sha',report.reference_bundle.fixture_sha256 || report.reference_bundle.vectors_sha256 || '—');
    setText('vb-metadata',`${kepler.samples} Kepler points / ${sgp.sample_count} vectors · ${sgp.cases} catalogs`);
    setText('vb-notice',report.suite_status==='PARTIAL'?
      'PARTIAL: full-corpus coverage and/or optional dependencies are missing. NOT release-certified. Review individual statuses.':
      report.suite_status==='PASS'?'Software-verification gate passed. Real-flight accuracy is NOT validated.':
      'FAIL: one or more independent numerical comparisons exceeded its stated tolerance.');
    setText('vb-details',`Kepler threshold: ${kepler.position_tolerance_m.toFixed(1)} m maximum. Published subset: ${sgp.sample_count} TEME vectors, ${sgp.cases} catalog(s). Complete corpus threshold is 33 cases and 666 vectors. No upstream source signature verified.`);
  }catch(err){setText('vb-notice','Benchmark unavailable: '+err.message);setText('vb-overall','ERROR');}
  finally{button.disabled=false;}
}

function switchMode(mode){
  app.mode=mode;
  document.body.classList.toggle('geodesy-active',mode==='geodesy');
  document.body.classList.toggle('v22-active',mode==='v22');
  for(const id of ['orbit','transfer','mission','perturb','satellite','target','reference','optimize','finite','uncertainty','guidance','robust','attitude','adcs','precision','integrated','joint','integrity','joint-integrity','adaptive','observability','ground','geodesy','v22','epoch','validate']){
    $('mode-'+id).classList.toggle('active',mode===id);
    $('mode-'+id).setAttribute('aria-selected',String(mode===id));
    const el=$((id==='perturb'?'perturb':id)+'-form');el.classList.toggle('hidden',mode!==id);
  }
  if(mode==='geodesy'){submitGeodesy();return;}
  if(mode==='v22'){submitV22();return;}
  if(mode!=='satellite'&&mode!=='reference'&&mode!=='validate'&&mode!=='epoch')submitRequest(mode);
}

let burnCounter=0;
function addBurn(data={}){
  if($('burn-list').children.length>=8)return;
  const ordinal=++burnCounter;
  const wrapper=document.createElement('div');wrapper.className='burn-card';
  wrapper.innerHTML=`
    <div class="burn-card-head"><strong>ENGINE EVENT ${String(ordinal).padStart(2,'0')}</strong><button type="button" class="remove-burn" aria-label="Remove maneuver">×</button></div>
    <div class="two-fields">
      <label class="field"><span class="field-label">Start <span class="unit">min</span></span><input data-key="start_min" type="number" min="0" max="1440" step="0.1" value="12" required></label>
      <label class="field"><span class="field-label">Duration <span class="unit">s</span></span><input data-key="duration_s" type="number" min="1" max="10000" step="1" value="120" required></label>
    </div>
    <div class="two-fields">
      <label class="field"><span class="field-label">Thrust <span class="unit">N</span></span><input data-key="thrust_n" type="number" min="1" max="10000000" step="1" value="850" required></label>
      <label class="field"><span class="field-label">Isp <span class="unit">s</span></span><input data-key="specific_impulse_s" type="number" min="50" max="5000" value="315" required></label>
    </div>
    <div class="two-fields">
      <label class="field"><span class="field-label">Direction</span><select data-key="direction">
        <option value="prograde">Prograde</option><option value="retrograde">Retrograde</option>
        <option value="transverse">Local tangential +</option><option value="antitransverse">Local tangential −</option>
        <option value="radial_out">Radial outward</option><option value="radial_in">Radial inward</option>
        <option value="normal">Orbit normal +</option><option value="antinormal">Orbit normal −</option>
      </select></label>
      <label class="field"><span class="field-label">Throttle <span class="unit">%</span></span><input data-key="throttle_pct" type="number" min="1" max="100" value="100" required></label>
    </div>`;
  for(const [key,val] of Object.entries(data)){
    const input=wrapper.querySelector(`[data-key="${key}"]`);if(input)input.value=val;
  }
  wrapper.querySelector('.remove-burn').addEventListener('click',()=>wrapper.remove());
  $('burn-list').appendChild(wrapper);
}
function readBurns(){
  return [...document.querySelectorAll('#burn-list .burn-card')].map((node,i)=>{
    const read=key=>node.querySelector(`[data-key="${key}"]`).value;
    return {name:'Burn '+(i+1),start_s:Number(read('start_min'))*60,
      duration_s:Number(read('duration_s')),thrust_n:Number(read('thrust_n')),
      specific_impulse_s:Number(read('specific_impulse_s')),
      direction:read('direction'),throttle:Number(read('throttle_pct'))/100};
  });
}
function downloadFile(filename,text,mimetype){
  const blob=new Blob([text],{type:mimetype});
  const href=URL.createObjectURL(blob);
  const link=document.createElement('a');link.href=href;link.download=filename;
  document.body.appendChild(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(href),1000);
}
function exportJSON(){if(app.data)downloadFile('APOGEE-mission-report.json',JSON.stringify(app.data,null,2),'application/json');}
function exportCSV(){
  const tr=app.data?.trajectory;if(!tr)return;
  const hasMass=Boolean(tr.wet_mass_kg);
  const rows=['time_s,x_km,y_km,z_km,vx_km_s,vy_km_s,vz_km_s,altitude_km,speed_km_s'+(hasMass?',wet_mass_kg,propellant_remaining_kg,thrust_active':'')+',relative_energy_drift,relative_angular_momentum_drift'];
  for(let i=0;i<tr.time_s.length;i++)rows.push([
    tr.time_s[i],...tr.position_km[i],...tr.velocity_km_s[i],tr.altitude_km[i],tr.speed_km_s[i],
    ...(hasMass?[tr.wet_mass_kg[i],tr.propellant_remaining_kg[i],tr.thrust_active[i]]:[]),
    tr.relative_energy_drift?.[i]??'',tr.relative_angular_momentum_drift?.[i]??''
  ].join(','));
  downloadFile('APOGEE-trajectory.csv',rows.join('\n')+'\n','text/csv');
}
function registerEvents(){
  $('orbit-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('orbit');});
  $('transfer-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('transfer');});
  $('mission-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('mission');});
  $('compare-submit').addEventListener('click',()=>submitRequest('compare'));
  $('perturb-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('perturb');});
  $('satellite-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('satellite');});
  $('target-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('target');});
  $('reference-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('reference');});
  $('optimize-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('optimize');});
  $('finite-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('finite');});
  $('uncertainty-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('uncertainty');});
  $('guidance-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('guidance');});
  $('attitude-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('attitude');});
  $('adcs-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('adcs');});
  $('precision-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('precision');});
  $('integrity-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('integrity');});
  $('adaptive-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('adaptive');});
  $('ground-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('ground');});
  $('geodesy-form').addEventListener('submit',event=>{event.preventDefault();submitGeodesy();});
  $('observability-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('observability');});
  $('joint-integrity-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('joint-integrity');});
  $('joint-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('joint');});
  $('integrated-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('integrated');});
  $('epoch-form').addEventListener('submit',async event=>{
    event.preventDefault();const btn=$('epoch-submit'),out=$('epoch-output');btn.disabled=true;out.textContent='Computing absolute-time references…';
    try{
      const vec=id=>{const v=$(id).value.split(',').map(Number);if(v.length!==3||v.some(x=>!Number.isFinite(x)))throw Error('Vectors require three finite numbers');return v;};
      const payload={epoch_utc:$('ep-time').value,input_frame:$('ep-frame').value,position_km:vec('ep-r'),velocity_km_s:vec('ep-v'),include_igrf:$('ep-magnetic').checked};
      const response=await apiFetch('/api/references/epoch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
      const data=await response.json();if(!response.ok)throw Error(typeof data.detail==='string'?data.detail:JSON.stringify(data.detail));
      out.textContent=`UTC: ${data.epoch_utc}\n${data.input_frame} → ${data.output_frame}\nPosition km: ${data.converted.position_km.map(v=>v.toFixed(6)).join(', ')}\nVelocity km/s: ${data.converted.velocity_km_s.map(v=>v.toFixed(8)).join(', ')}\nSun unit GCRS: ${data.sun_direction_gcrs.map(v=>v.toFixed(7)).join(', ')}\n${data.igrf14?'IGRF-14: '+data.igrf14.magnitude_microtesla.toFixed(4)+' µT':'IGRF not requested'}\nIERS offline: Earth-orientation uncertainty depends on local tables.\nReference only: do not treat idealized ECI as GCRS.`;
    }catch(error){out.textContent='Reference error: '+error.message;}
    finally{btn.disabled=false;}
  });
  $('robust-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('robust');});
  $('rg-sweep').addEventListener('click',async()=>{
    const btn=$('rg-sweep'),display=$('rg-ensemble-status');btn.disabled=true;
    display.textContent='Computing reproducible seed ensemble…';
    try{
      const input={...readRobustForm(),seeds:Number($('rg-sweep-count').value)};
      const response=await apiFetch('/api/robust/sweep',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(input)});
      const r=await response.json();if(!response.ok)throw Error(JSON.stringify(r.detail));
      const a=r.summary;
      display.textContent=`${r.seed_count} synthetic noise seeds · median guided error ${formatNumber(a.median_guided_position_error_m,2)} m · worst ${formatNumber(a.max_guided_position_error_m,2)} m · improved in ${a.runs_improved_vs_baseline}/${r.seed_count} cases. Not an operational reliability estimate.`;
    }catch(error){display.textContent='Ensemble error: '+error.message;}
    finally{btn.disabled=false;}
  });
  $('validate-form').addEventListener('submit',event=>{event.preventDefault();submitRequest('validate');});
  $('v-file').addEventListener('change',async e=>{
    const file=e.target.files[0];if(!file)return;
    if(file.size>260000){$('v-text').value='';return;}
    $('v-text').value=await file.text();
  });
  $('r-file').addEventListener('change',async e=>{
    const file=e.target.files[0];if(!file)return;
    if(file.size>260000){$('f-output').textContent='Reference text file exceeds 260 KB';return;}
    $('r-text').value=await file.text();
  });
  $('f-transform').addEventListener('click',async()=>{
    const result=$('f-output');result.textContent='Transforming…';
    try{
      const vals=$('f-state').value.split(',').map(s=>Number(s.trim()));
      if(vals.length!==6||vals.some(v=>!Number.isFinite(v)))throw Error('Enter six comma-separated finite numbers');
      const payload={state:vals,greenwich_angle_deg_at_t0:Number($('f-angle').value),
        elapsed_s:Number($('f-time').value),direction:$('f-direction').value};
      const response=await apiFetch('/api/frames/spin',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
      const data=await response.json();if(!response.ok)throw Error(JSON.stringify(data.detail));
      result.textContent=data.direction+'\n'+data.state.map((v,i)=>['X','Y','Z','VX','VY','VZ'][i]+' = '+Number(v).toFixed(8)).join('\n')+'\nUnits: km / km s⁻¹';
    }catch(error){result.textContent='Error: '+error.message;}
  });
  $('s-fetch').addEventListener('click',async()=>{
    const button=$('s-fetch');button.disabled=true;$('s-source-status').textContent='Contacting CelesTrak…';
    try{
      const nr=Number($('s-catnr').value);
      if(!Number.isInteger(nr)||nr<1||nr>999999999)throw Error('Enter a valid catalog number');
      const response=await apiFetch('/api/catalog/'+nr);
      const d=await response.json();if(!response.ok)throw Error(d.detail||'Catalog lookup failed');
      $('s-omm').value=JSON.stringify(d.data,null,2);
      $('s-source-status').textContent='Loaded '+(d.data.OBJECT_NAME||nr)+' · epoch '+d.data.EPOCH+(d.sgp4_available?'':' · SGP4 not installed');
    }catch(err){$('s-source-status').textContent='Catalog fetch failed: '+err.message;}
    finally{button.disabled=false;}
  });
  $('s-file').addEventListener('change',async e=>{
    const file=e.target.files[0];if(!file)return;
    if(file.size>65536){$('s-source-status').textContent='JSON file must be under 64 KB';return;}
    $('s-omm').value=await file.text();$('s-source-status').textContent='Loaded local file '+file.name;
  });
  $('add-burn').addEventListener('click',()=>addBurn());
  $('mode-orbit').addEventListener('click',()=>switchMode('orbit'));
  $('mode-transfer').addEventListener('click',()=>switchMode('transfer'));
  $('mode-mission').addEventListener('click',()=>switchMode('mission'));
  $('mode-perturb').addEventListener('click',()=>switchMode('perturb'));
  $('mode-satellite').addEventListener('click',()=>switchMode('satellite'));
  $('mode-target').addEventListener('click',()=>switchMode('target'));
  $('mode-reference').addEventListener('click',()=>switchMode('reference'));
  $('mode-optimize').addEventListener('click',()=>switchMode('optimize'));
  $('mode-finite').addEventListener('click',()=>switchMode('finite'));
  $('mode-uncertainty').addEventListener('click',()=>switchMode('uncertainty'));
  $('mode-guidance').addEventListener('click',()=>switchMode('guidance'));
  $('mode-attitude').addEventListener('click',()=>switchMode('attitude'));
  $('mode-adcs').addEventListener('click',()=>switchMode('adcs'));
  $('mode-precision').addEventListener('click',()=>switchMode('precision'));
  $('mode-integrity').addEventListener('click',()=>switchMode('integrity'));
  $('mode-adaptive').addEventListener('click',()=>switchMode('adaptive'));
  $('mode-ground').addEventListener('click',()=>switchMode('ground'));
  $('mode-geodesy').addEventListener('click',()=>switchMode('geodesy'));
  $('mode-v22').addEventListener('click',()=>switchMode('v22'));
  $('v22-form').addEventListener('submit',event=>{event.preventDefault();submitV22();});
  $('vb-import').addEventListener('click',async()=>{
    const info=$('vb-import-status'),files=[$('vb-tle-file').files[0],$('vb-output-file').files[0]];
    if(files.some(f=>!f)){info.textContent='Select both official source files first.';return;}
    if(files[0].size>2000000||files[1].size>8000000){info.textContent='File size limit exceeded.';return;}
    const button=$('vb-import');button.disabled=true;info.textContent='Importing published vectors…';
    try{
      const [tle_text,output_text]=await Promise.all(files.map(f=>f.text()));
      const res=await apiFetch('/api/verification/v24',{
        method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({tle_text,vector_text:output_text,steps_per_orbit:240,orbits:2})});
      const report=await res.json();if(!res.ok)throw Error(report.detail||'Invalid corpus');
      v22Report=report;
      const c=report.results.find(x=>x.case==='sgp4_multi_regime'),r=report.import_summary,coverage=report.results.find(x=>x.case==='vallado_full_corpus_coverage');
      info.textContent=`${r.cases_with_vectors} cases / ${r.samples} TEME vectors · ${c.passed} PASS, ${c.failed} FAIL, ${c.skipped} SKIP · full coverage: ${coverage.status} · strict gate: ${report.release_gate.status}.`;
      setText('vb-sgp-status',c.status);setText('vb-sgp-error',`${r.samples} vectors across ${r.cases_with_vectors} cases`);
      setText('vb-tallies',`${c.passed} passed · ${c.failed} failed · ${c.skipped} skipped (SGP4)`);
      setText('vb-sha',report.reference_bundle.vectors_sha256);
      setText('vb-gate',report.release_gate.status);setText('vb-overall',report.suite_status);setText('vb-coverage',`${coverage.cases_with_vectors} cases / ${coverage.state_vectors} vectors · ${coverage.status}`);
      setText('vb-metadata',`Imported full-corpus file · ${r.samples} vectors`);
    }catch(err){info.textContent='Import rejected: '+err.message;}finally{button.disabled=false;}
  });
  $('vb-export').addEventListener('click' ,()=>{if(!v22Report)return;const blob=new Blob([JSON.stringify(v22Report,null,2)],{type:'application/json'});const u=URL.createObjectURL(blob);const a=document.createElement('a');a.href=u;a.download='APOGEE-v2.4-validation-report.json';a.click();URL.revokeObjectURL(u);});
  $('mode-observability').addEventListener('click',()=>switchMode('observability'));
  $('mode-joint-integrity').addEventListener('click',()=>switchMode('joint-integrity'));
  $('mode-joint').addEventListener('click',()=>switchMode('joint'));
  $('mode-integrated').addEventListener('click',()=>switchMode('integrated'));
  $('mode-epoch').addEventListener('click',()=>switchMode('epoch'));
  $('mode-robust').addEventListener('click',()=>switchMode('robust'));
  $('mode-validate').addEventListener('click',()=>switchMode('validate'));
  $('preset').addEventListener('change',()=>{
    const preset=app.presets?.[$('preset').value];
    if(!preset)return;
    Object.entries(preset).forEach(([key,val])=>{if($(key))$(key).value=val});
    $('velocity_factor').value=1;
    submitRequest('orbit');
  });
  $('reset-camera').addEventListener('click',()=>{app.yaw=.48;app.pitch=-.39;app.zoom=1;});
  $('zoom-in').addEventListener('click',()=>{app.zoom=Math.min(3.5,app.zoom*1.2)});
  $('zoom-out').addEventListener('click',()=>{app.zoom=Math.max(.32,app.zoom/1.2)});
  $('play-pause').addEventListener('click',()=>{app.playing=!app.playing;setText('play-pause',app.playing?'Ⅱ':'▶');});
  $('playback-speed').addEventListener('change',e=>app.speed=Number(e.target.value));
  $('scrubber').addEventListener('input',e=>{
    if(!app.data)return;
    app.playing=false;setText('play-pause','▶');
    app.simTime=Number(e.target.value)/1000*app.data.trajectory.time_s.at(-1);
    updateTelemetry();
  });
  $('export-json').addEventListener('click',exportJSON);
  $('export-csv').addEventListener('click',exportCSV);
  scene.addEventListener('pointerdown',e=>{
    app.dragging=true;app.pointerX=e.clientX;app.pointerY=e.clientY;scene.setPointerCapture(e.pointerId);
  });
  scene.addEventListener('pointermove',e=>{
    if(!app.dragging)return;
    app.yaw+=(e.clientX-app.pointerX)*.007;
    app.pitch=Math.min(1.52,Math.max(-1.52,app.pitch+(e.clientY-app.pointerY)*.007));
    app.pointerX=e.clientX;app.pointerY=e.clientY;
  });
  scene.addEventListener('pointerup',()=>app.dragging=false);
  scene.addEventListener('pointercancel',()=>app.dragging=false);
  scene.addEventListener('wheel',e=>{e.preventDefault();app.zoom=Math.min(3.5,Math.max(.32,app.zoom*(e.deltaY>0?.90:1.1)));},{passive:false});
  window.addEventListener('resize',drawDriftChart);
}
function tick(timestamp){
  const elapsed=Math.min(.1,(timestamp-app.lastFrame)/1000||0);app.lastFrame=timestamp;
  if(app.playing&&app.data){
    const duration=app.data.trajectory.time_s.at(-1);
    app.simTime+=elapsed*60*app.speed;
    if(app.simTime>duration)app.simTime=0;
    updateTelemetry();
  }
  drawScene();requestAnimationFrame(tick);
}
async function initialize(){
  registerEvents();
  registerConnectionUI();
  addBurn();
  const connected=await checkConnection();
  if(connected){
    try{
      const response=await apiFetch('/api/presets');
      app.presets=await response.json();
    }catch(_){/* Form defaults still work. */}
    await submitRequest('orbit',true);
  }else if(isStaticSite()){
    try {await loadDemoOrbit();} catch(e) {connectionUI(e.message,'error');}
  }else{
    await submitRequest('orbit',true);
  }
  requestAnimationFrame(tick);
}
initialize();
