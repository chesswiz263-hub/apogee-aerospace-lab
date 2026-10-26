/* APOGEE v2.5 home: one mission viewport, one view selector.
 * The new spacecraft flight and planets are drawn on ONE canvas in the
 * Solar view; existing 26 mission modes continue untouched in Mission view.
 * Never intercepts fetch, POST requests, physics arrays, or CORS.
 */
(() => {
 'use strict';
 if(document.getElementById('apogee-v25-home-style'))return;
 const css=document.createElement('style');
 css.id='apogee-v25-home-style';
 css.textContent=`
 .apogee-v25-layer{position:absolute;overflow:hidden;z-index:45;background:#061426;border-radius:4px}
 .apogee-v25-layer iframe{width:100%;height:100%;border:0;display:block}
 .apogee-v25-tabs{position:absolute;top:10px;left:12px;z-index:65;display:flex;gap:5px;padding:4px;background:#071728e8;border:1px solid #2a536b;border-radius:8px}
 .apogee-v25-tabs button{border:1px solid transparent;border-radius:6px;color:#afcbd8;background:transparent;padding:6px 9px;cursor:pointer;font:600 11px system-ui}
 .apogee-v25-tabs button[aria-pressed=true]{background:#17506a;border-color:#417f95;color:#edfcff}
 .apogee-v25-tabs button:focus-visible{outline:2px solid #7fe7ff}
 `;
 document.head.appendChild(css);
 function findMissionCanvas(){
  return [...document.querySelectorAll('canvas')].filter(c=>!c.closest('.apogee-v25-layer'))
   .map(c=>({c,r:c.getBoundingClientRect()}))
   .filter(o=>o.r.width>300&&o.r.height>180)
   .sort((a,b)=>b.r.width*b.r.height-a.r.width*a.r.height)[0]?.c||null;
 }
 function install(){
  const original=findMissionCanvas();if(!original||!original.parentElement)return false;
  const container=original.parentElement;
  if(getComputedStyle(container).position==='static')container.style.position='relative';
  const layer=document.createElement('div');layer.className='apogee-v25-layer';layer.id='apogee-v25-integrated';
  const frame=document.createElement('iframe');frame.src='solar-system.html?embedded=1';frame.title='Solar System with interplanetary spacecraft missions';
  layer.append(frame);container.append(layer);
  const tabs=document.createElement('nav');tabs.className='apogee-v25-tabs';tabs.setAttribute('aria-label','APOGEE mission visualization');
  const solar=document.createElement('button'),mission=document.createElement('button');
  solar.type=mission.type='button';solar.textContent='☀ Solar System + Flights';mission.textContent='◇ Earth Mission';
  tabs.append(solar,mission);container.append(tabs);
  function layout(){const r=original.getBoundingClientRect(),p=container.getBoundingClientRect();if(r.width<60||r.height<60)return;
   for(const [prop,v] of Object.entries({left:r.left-p.left,top:r.top-p.top,width:r.width,height:r.height}))layer.style[prop]=`${v}px`;
   tabs.style.left=`${r.left-p.left+10}px`;tabs.style.top=`${r.top-p.top+9}px`;
  }
  function setSolar(value){layer.style.display=value?'block':'none';solar.setAttribute('aria-pressed',String(value));mission.setAttribute('aria-pressed',String(!value));layout();}
  solar.addEventListener('click',()=>setSolar(true));mission.addEventListener('click',()=>setSolar(false));
  if('ResizeObserver' in window)new ResizeObserver(layout).observe(original);
  window.addEventListener('resize',layout);
  setSolar(true);return true;
 }
 function boot(n=0){if(install())return;if(n<35)setTimeout(()=>boot(n+1),250);else console.warn('APOGEE v2.5: original mission canvas not found; mission functionality left alone.');}
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',()=>boot());else boot();
})();
