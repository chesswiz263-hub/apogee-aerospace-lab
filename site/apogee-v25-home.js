/* APOGEE v2.5 homepage bridge.
 * In-panel solar visualization; original mission canvas and controls remain alive.
 * Does NOT reproject spacecraft tracks into heliocentric coordinates.
 */
(() => {
  'use strict';
  if (document.getElementById('apogee-v25-home-style')) return;
  const css = document.createElement('style');
  css.id = 'apogee-v25-home-style';
  css.textContent = `
    .apogee-v25-layer { position:absolute; overflow:hidden; z-index:50; background:#040a15; border-radius:3px; }
    .apogee-v25-layer iframe { display:block; width:100%; height:100%; border:0; background:#040a15; }
    .apogee-v25-switch { position:absolute; top:12px; left:12px; display:flex; align-items:center; gap:6px; z-index:52;
      padding:5px; border:1px solid #355271; border-radius:10px; background:#081729e8; box-shadow:0 6px 22px #0009; }
    .apogee-v25-switch button { color:#a7c4dc; padding:7px 10px; cursor:pointer; border:1px solid transparent;
      border-radius:7px; background:transparent; font:600 11px system-ui, sans-serif; letter-spacing:.03em; }
    .apogee-v25-switch button[aria-pressed="true"] { color:#e5faff; border-color:#3a819f; background:#15364b; }
    .apogee-v25-switch button:focus-visible { outline:2px solid #79d8e9; }
    .apogee-v25-footnote { position:absolute; top:53px; left:14px; right:14px; z-index:52; color:#9cc8dc;
      text-shadow:0 1px 5px #000; font:10px system-ui, sans-serif; letter-spacing:.03em; pointer-events:none; }
    .apogee-v25-mini { position:absolute; bottom:61px; right:14px; z-index:52; background:#061426ed; border:1px solid #3e627a;
      border-radius:8px; padding:5px; width:min(24%,190px); min-width:125px; box-shadow:0 5px 22px #000a; cursor:pointer; }
    .apogee-v25-mini canvas { display:block; width:100%; height:auto; aspect-ratio:1.75; background:#061426; }
    .apogee-v25-mini span { display:block; font:10px system-ui,sans-serif; color:#b1d5e9; padding:4px 2px 2px; text-align:center; }
    @media(max-width:650px){.apogee-v25-mini{display:none}.apogee-v25-footnote{font-size:9px}}
  `;
  document.head.appendChild(css);

  function findMissionCanvas() {
    const all = [...document.querySelectorAll('canvas')];
    return all.filter(c => !c.closest('.apogee-v25-layer')).map(c => {
      const r=c.getBoundingClientRect();
      return {c,r,score:r.width*r.height*(r.left > innerWidth*.12 ? 1.15 : 1)};
    }).filter(o => o.r.width>=300 && o.r.height>=180).sort((a,b)=>b.score-a.score)[0]?.c || null;
  }

  function mount() {
    const original = findMissionCanvas();
    if (!original || !original.parentElement) return false;
    const parent=original.parentElement;
    if (getComputedStyle(parent).position==='static') parent.style.position='relative';

    const layer=document.createElement('div');
    layer.className='apogee-v25-layer';
    layer.id='apogee-v25-integrated';
    layer.setAttribute('aria-label','APOGEE in-panel Solar System visualization');
    const viewer=document.createElement('iframe');
    viewer.title='Solar System orbits and planetary positions';
    viewer.src='solar-system.html?embedded=1';
    viewer.loading='eager';
    layer.appendChild(viewer);

    const switcher=document.createElement('div');
    switcher.className='apogee-v25-switch';
    const solarBtn=document.createElement('button');
    solarBtn.textContent='☀ Solar System';
    solarBtn.type='button';
    const missionBtn=document.createElement('button');
    missionBtn.textContent='◇ Spacecraft / Mission';
    missionBtn.type='button';
    switcher.append(solarBtn,missionBtn);

    const footnote=document.createElement('div');
    footnote.className='apogee-v25-footnote';
    footnote.textContent='Planet positions: approximate JPL elements · Spacecraft trajectory remains Earth-centered in mission view';
    layer.appendChild(footnote);
    layer.appendChild(switcher);

    const mini=document.createElement('button');
    mini.type='button';
    mini.className='apogee-v25-mini';
    mini.title='Expand the original spacecraft visualization';
    const thumbnail=document.createElement('canvas');
    thumbnail.width=352; thumbnail.height=200;
    mini.append(thumbnail);
    const label=document.createElement('span');
    label.textContent='LIVE SPACECRAFT VIEW · expand';
    mini.append(label);
    layer.appendChild(mini);

    parent.appendChild(layer);
    const parentRect=()=>parent.getBoundingClientRect();
    function layout() {
      const r=original.getBoundingClientRect(), p=parentRect();
      if(r.width<50 || r.height<50) return;
      layer.style.left=`${r.left-p.left}px`;
      layer.style.top=`${r.top-p.top}px`;
      layer.style.width=`${r.width}px`;
      layer.style.height=`${r.height}px`;
    }
    let solar=true;
    function showSolar(show) {
      solar=!!show;
      // Hide the overlay, not the original canvas or its numerical renderer.
      layer.style.display=solar?'block':'none';
      solarBtn.setAttribute('aria-pressed',String(solar));
      missionBtn.setAttribute('aria-pressed',String(!solar));
      if(solar) layout();
    }
    solarBtn.addEventListener('click',()=>showSolar(true));
    missionBtn.addEventListener('click',()=>showSolar(false));
    mini.addEventListener('click',()=>showSolar(false));

    // Always-visible selector OUTSIDE the solar overlay, so mission view can return to Solar view.
    const bar=document.createElement('div');
    bar.id='apogee-v25-home-controls';
    bar.style.cssText='display:flex;align-items:center;justify-content:flex-end;gap:6px;padding:5px 0;';
    const solarTab=document.createElement('button');
    const missionTab=document.createElement('button');
    for(const b of [solarTab,missionTab]) b.style.cssText='background:#102b43;border:1px solid #2a607b;color:#c7e8f5;border-radius:6px;padding:6px 10px;cursor:pointer;font:600 11px system-ui,sans-serif';
    solarTab.textContent='☀ Solar System';
    missionTab.textContent='◇ Mission view';
    function setState(show) {showSolar(show); solarTab.setAttribute('aria-pressed',String(show));missionTab.setAttribute('aria-pressed',String(!show));solarTab.style.background=show?'#1c5368':'#102b43';missionTab.style.background=show?'#102b43':'#1c5368';}
    solarTab.onclick=()=>setState(true);
    missionTab.onclick=()=>setState(false);
    solarBtn.onclick=()=>setState(true);
    missionBtn.onclick=()=>setState(false);
    mini.onclick=()=>setState(false);
    bar.append(solarTab, missionTab);
    // Put the selector inside the current mission panel, close to visualization (not a floating site link).
    const visualPanel=parent.parentElement;
    if(visualPanel) visualPanel.insertBefore(bar,parent);
    else parent.insertBefore(bar,original);
    setState(true);

    if ('ResizeObserver' in window) new ResizeObserver(layout).observe(original);
    window.addEventListener('resize',layout);
    let last=0;
    function tick(now){
      if(now-last>350){
        last=now;
        layout();
        try {
          if(original.width && original.height) {
            const ctx=thumbnail.getContext('2d');
            ctx.clearRect(0,0,352,200);
            ctx.drawImage(original,0,0,352,200);
          }
        }catch(err){mini.style.display='none';}
      }
      requestAnimationFrame(tick);
    }
    requestAnimationFrame(tick);
    return true;
  }

  function boot(attempt) {
    if(mount()) return;
    if(attempt<25) setTimeout(()=>boot(attempt+1),300);
    else console.warn('APOGEE v2.5: mission canvas not found; existing mission view unchanged.');
  }
  if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',()=>boot(0));
  else boot(0);
})();
