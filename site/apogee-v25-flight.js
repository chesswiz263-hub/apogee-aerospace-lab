/* APOGEE v2.5 unified heliocentric scene: spacecraft and planet positions
   use the SAME AU/J2000-ecliptic projection functions as the Solar viewer.
   Lambert transfers are computed by the Render API, not invented by the browser. */
(() => {
  'use strict';
  if (window.APOGEE_V25_FLIGHT) return;
  let transfer = null;
  let satellite = null;
  let loading = false;
  let status;
  let elements;
  const DEFAULT_API = 'https://apogee-physics-api.onrender.com';
  const DEFAULT_DAYS = {Mercury:115,Venus:160,Earth:260,Mars:260,Jupiter:1050,Saturn:2100,Uranus:3900,Neptune:6200};
  const valid = ['Mercury','Venus','Earth','Mars','Jupiter','Saturn','Uranus','Neptune'];

  function showStatus(message, kind='info') {
    if (!status) return;
    status.textContent=message;
    status.dataset.kind=kind;
  }
  function baseUrl() {
    const box = document.getElementById('apiUrl');
    const val = box?.value?.trim() || DEFAULT_API;
    if (!/^https:\/\/[a-z0-9.-]+(?::\d+)?$/i.test(val) && !/^http:\/\/localhost(?::\d+)?$/.test(val)) {
      throw new Error('Set a valid HTTPS Render API origin');
    }
    return val.replace(/\/+$/,'');
  }
  const toIso = ms => new Date(ms).toISOString();
  async function launch(){
    if(loading)return;
    try {
      const from=elements.origin.value, to=elements.dest.value;
      if(from===to) throw Error('Choose a different destination planet');
      const days=Number(elements.days.value);
      if(!Number.isFinite(days) || days<20 || days>12000) throw Error('Flight duration must be 20–12,000 days');
      const departure=toIso(sim);
      const params=new URLSearchParams({origin:from,destination:to,departure,flight_days:String(days),samples:'210'});
      loading=true;elements.launch.disabled=true;
      showStatus('Calculating heliocentric departure and propagation…');
      const ctrl=new AbortController();
      const timer=setTimeout(()=>ctrl.abort(),30000);
      let result;
      try {
        const resp=await fetch(`${baseUrl()}/api/v25/interplanetary/transfer?${params}`,{signal:ctrl.signal});
        result=await resp.json();
        if(!resp.ok)throw Error(result.detail || `API HTTP ${resp.status}`);
      } finally {clearTimeout(timer);}
      if(!Array.isArray(result.trajectory) || result.trajectory.length<30 || !result.frame?.includes('heliocentric')){
        throw Error('Unexpected flight response; no valid heliocentric trajectory');
      }
      transfer=result;
      // Do not change any existing mission physics state or telemetry.
      showStatus(`${from} → ${to} · ${days.toFixed(0)} days · departure v∞ ${result.departure_v_infinity_km_s.toFixed(2)} km/s`, 'success');
    } catch (error) {
      showStatus(`Flight not launched: ${error.message}`, 'error');
    } finally {loading=false;elements.launch.disabled=false;}
  }

  async function trackIss(){
    try {
      // Orbital elements are only predictive near their fitted epoch. Select NOW.
      sim=Date.now();
      showStatus('Retrieving current ISS OMM and transforming TEME with Astropy…');
      const ctrl=new AbortController();
      const timer=setTimeout(()=>ctrl.abort(),20000);
      let response, data;
      try {
        response=await fetch(`${baseUrl()}/api/v25/satellites/25544/heliocentric?at=${encodeURIComponent(toIso(sim))}`,{signal:ctrl.signal});
        data=await response.json();
      } finally {clearTimeout(timer);}
      if(!response.ok)throw Error(data.detail || `HTTP ${response.status}`);
      if(!Array.isArray(data.position_au)||data.position_au.length!==3)throw Error('Invalid satellite state');
      satellite=data;
      showStatus(`ISS • OMM age ${data.element_age_days.toFixed(1)} days${data.stale_or_far_from_epoch?' ⚠ stale, not reliable':''} • snapshot near Earth`,'success');
    } catch(error) {showStatus('ISS: '+error.message,'error');}
  }

  function point(item){return {x:item[0],y:item[1],z:item[2]};}
  function drawTrail({ctx,project,visual}){
    if(!transfer)return;
    const trajectory=transfer.trajectory;
    ctx.save();ctx.beginPath();
    trajectory.forEach((s,i)=>{
      const p=project(visual(point(s.position_au)));
      if(i===0)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y);
    });
    ctx.strokeStyle='rgba(70,221,224,.95)';ctx.lineWidth=2.1;
    ctx.setLineDash([7,4]);ctx.stroke();ctx.setLineDash([]);
    ctx.restore();
  }
  function drawCraft({ctx,project,visual,sim:nowMs}){
    if(!transfer && !satellite)return;
    if(!transfer){drawSatellite({ctx,project,visual,sim:nowMs});return;}
    const dep=Date.parse(transfer.departure_utc);
    const days=(nowMs-dep)/86400000;
    const rows=transfer.trajectory;
    let xyz;
    if(days<0) {
      xyz=rows[0].position_au;
    } else if(days>=transfer.flight_days) {
      xyz=rows[rows.length-1].position_au;
    } else {
      let low=0, high=rows.length-1;
      while(high-low>1){const mid=(low+high)>>1; if(rows[mid].day<days)low=mid;else high=mid;}
      const t=(days-rows[low].day)/(rows[high].day-rows[low].day);
      xyz=rows[low].position_au.map((v,i)=>v*(1-t)+rows[high].position_au[i]*t);
    }
    const p=project(visual(point(xyz)));
    ctx.save();
    ctx.shadowBlur=17;ctx.shadowColor='#4aeaff';
    ctx.fillStyle='#ffffff';ctx.beginPath();ctx.arc(p.x,p.y,4.1,0,Math.PI*2);ctx.fill();
    ctx.shadowBlur=0;ctx.strokeStyle='#5de8f5';ctx.lineWidth=1.8;
    ctx.beginPath();ctx.arc(p.x,p.y,8.5,0,Math.PI*2);ctx.stroke();
    ctx.textAlign='left';ctx.font='600 11px system-ui';ctx.fillStyle='#ccffff';
    ctx.fillText('APOGEE · '+transfer.destination, p.x+13,p.y-9);
    ctx.restore();
    drawSatellite({ctx,project,visual,sim:nowMs});
  }
  function drawSatellite({ctx,project,visual,sim:nowMs}) {
    if(!satellite || Math.abs(nowMs-Date.parse(satellite.time_utc))>6*3600000)return;
    const p=project(visual(point(satellite.position_au)));
    ctx.save();ctx.strokeStyle='#ffc76d';ctx.lineWidth=1.7;ctx.beginPath();ctx.arc(p.x,p.y,13,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle='#ffe1ad';ctx.font='bold 10px system-ui';ctx.fillText('ISS · SGP4',p.x+17,p.y+6);ctx.restore();
  }

  function boot() {
    if(document.getElementById('v25FlightPanel')) return;
    const sheet=document.createElement('style');
    sheet.textContent=`
      #v25FlightPanel{position:fixed;top:12px;right:12px;z-index:12;max-width:min(290px,44vw);background:#061728eb;border:1px solid #35657f;border-radius:10px;padding:10px 12px;box-shadow:0 8px 24px #0007;font:11px system-ui;color:#d8f5ff}
      #v25FlightPanel .v25Title{color:#63cddd;font-size:10px;letter-spacing:.12em;font-weight:800;margin-bottom:7px}
      #v25FlightPanel .v25Form{display:grid;grid-template-columns:1fr 1fr;gap:6px}
      #v25FlightPanel label{display:block;color:#a5bfd0;font-size:9px}
      #v25FlightPanel select,#v25FlightPanel input,#v25FlightPanel button{width:100%;padding:5px 7px;border:1px solid #36576c;background:#102a40;color:#e4fbff;border-radius:5px;min-width:0}
      #v25FlightPanel button{grid-column:span 2;font-weight:700;cursor:pointer;background:#15506a}
      #v25FlightPanel button:disabled{opacity:.6;cursor:wait}
      #v25FlightPanel output{display:block;padding-top:7px;font-size:10px;line-height:1.4;color:#a7d4e1}
      #v25FlightPanel output[data-kind=error]{color:#ffb3aa}
      #v25FlightPanel output[data-kind=success]{color:#80ebc8}
      #v25FlightPanel .v25Note{color:#779db6;font-size:9px;padding-top:5px}
      @media(max-width:700px){#v25FlightPanel{top:60px;right:6px;padding:6px;max-width:46vw}.v25Note{display:none}}
    `;
    document.head.appendChild(sheet);
    const panel=document.createElement('section');
    panel.id='v25FlightPanel';
    panel.setAttribute('aria-label','Interplanetary spacecraft launch planner');
    panel.innerHTML=`<div class="v25Title">✦ INTERPLANETARY FLIGHT</div>
      <div class="v25Form">
      <label>Depart from<select id="v25Origin"></select></label>
      <label>Destination<select id="v25Dest"></select></label>
      <label style="grid-column:span 2">Flight duration · days<input id="v25Days" type="number" min="20" max="12000" step="1" value="260"></label>
      <button type="button" id="v25Launch">Launch spacecraft</button><button type="button" id="v25ISS">Track ISS (current)</button></div>
      <output id="v25FlightStatus" aria-live="polite">Select planets and launch. Spacecraft will follow the same heliocentric scene and clock.</output>
      <div class="v25Note">Lambert + Sun gravity · exploratory, not flight certified</div>`;
    document.body.appendChild(panel);
    elements={origin:panel.querySelector('#v25Origin'),dest:panel.querySelector('#v25Dest'),days:panel.querySelector('#v25Days'),launch:panel.querySelector('#v25Launch')};
    status=panel.querySelector('#v25FlightStatus');
    for(const name of valid){
      for(const select of [elements.origin,elements.dest]){
        const opt=document.createElement('option');opt.value=name;opt.textContent=name;select.appendChild(opt);
      }
    }
    elements.origin.value='Earth';elements.dest.value='Mars';
    elements.dest.addEventListener('change',()=>{elements.days.value=DEFAULT_DAYS[elements.dest.value]||260;});
    elements.launch.addEventListener('click',launch);
    panel.querySelector('#v25ISS').addEventListener('click',trackIss);
  }
  window.APOGEE_V25_FLIGHT={drawTrail,drawCraft,launch,trackIss,boot};
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',boot);
  else boot();
})();
