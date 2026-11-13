// Port of b3798b8's CostmapPanel/mapPoint, using Scene's existing plain JS page.
window.createOutdoorCostmaps = function(container, tr) {
  const esc=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const ns='http://www.w3.org/2000/svg', keys=['global','local'], layers={};
  let active=false,follow=true,view={x:0,y:0,width:24},aspect=1.6,drag=null,clock=null;
  const controls=document.createElement('div');controls.className='od-cost-controls';
  const heading=document.createElement('strong');heading.textContent=tr('overlayCostmap');controls.append(heading);
  const enabled={global:true,local:true};
  keys.forEach(key=>{
    const label=document.createElement('label'),input=document.createElement('input');
    label.className='od-cost-'+key;input.type='checkbox';input.checked=true;
    input.onchange=()=>{enabled[key]=input.checked;draw();};
    label.append(input,document.createTextNode(tr(key+'Layer')));controls.append(label);
  });
  function button(text,action){const b=document.createElement('button');b.textContent=text;b.onclick=action;controls.append(b);return b;}
  const locate=button(tr('locateLynx'),()=>{follow=true;view.width=24;draw();});
  button(tr('wholeMap'),fit);
  const interaction=document.createElement('span');controls.append(interaction);
  const status=document.createElement('div');status.className='od-cost-status';status.setAttribute('role','status');
  const svg=document.createElementNS(ns,'svg');svg.classList.add('od-cost-canvas');svg.setAttribute('role','img');svg.setAttribute('aria-label',tr('overlayCostmap'));
  const footer=document.createElement('div');footer.className='od-cost-footer';footer.textContent=tr('overlayLegend');
  container.append(controls,status,svg,footer);
  const elapsed=layer=>(performance.now()-layer.received)/1000;
  const age=layer=>layer.data.age_s+elapsed(layer);
  function usable(layer){const d=layer.data;return d.map_from_frame&&(d.frame_id==='map'||d.map_from_frame.age_s+elapsed(layer)<3);}
  // Frame-local to map ENU; never silently treat odom as map.
  function mapPoint(d,p){const tf=d.map_from_frame,c=Math.cos(tf.yaw),s=Math.sin(tf.yaw);return{x:tf.x+c*p.x-s*p.y,y:tf.y+s*p.x+c*p.y};}
  function robotLayer(){return [layers.local,layers.global].find(l=>l?.data&&usable(l)&&age(l)<3&&l.data.robot&&l.data.robot.age_s+elapsed(l)<3);}
  function fit(){
    const layer=[layers.global,layers.local].find(l=>l?.data&&usable(l));if(!layer)return;
    const d=layer.data,o=mapPoint(d,d.origin),yaw=d.origin.yaw+d.map_from_frame.yaw,w=d.width*d.resolution,h=d.height*d.resolution;
    follow=false;view={x:o.x+(Math.cos(yaw)*w-Math.sin(yaw)*h)/2,y:-o.y-(Math.sin(yaw)*w+Math.cos(yaw)*h)/2,
      width:Math.max(Math.abs(Math.cos(yaw)*w)+Math.abs(Math.sin(yaw)*h),aspect*(Math.abs(Math.sin(yaw)*w)+Math.abs(Math.cos(yaw)*h)))*1.1};draw();
  }
  function draw(){
    if(!svg.clientWidth||!svg.clientHeight)return;
    aspect=svg.clientWidth/svg.clientHeight;
    const robotSource=robotLayer(),robot=robotSource?.data.robot,point=robot?mapPoint(robotSource.data,robot):null;
    if(follow&&point){view.x=point.x;view.y=-point.y;}
    interaction.textContent=tr(follow?'following':'freeBrowse')+' · '+tr('panZoom');locate.classList.toggle('primary',follow);
    const problems=[];
    keys.forEach(key=>{
      const layer=layers[key];if(!layer)return;
      if(layer.error)problems.push(tr(key+'Costmap')+': '+layer.error);
      if(layer.data){
        problems.push(tr(key+'Costmap')+' '+layer.data.resolution.toFixed(2)+' m / '+age(layer).toFixed(1)+' s'+(age(layer)>=3?' · '+tr('costmapStale'):'')+(!usable(layer)?' · '+tr('frameUnaligned'):''));
        if(layer.data.overlay_warnings?.length)problems.push(...layer.data.overlay_warnings.map(key=>tr(key)));
      }
    });
    status.textContent=(keys.some(key=>layers[key]?.data)?tr('mapOverlay'):tr('waitingCostmap'))+' · '+problems.join(' · ');
    svg.setAttribute('viewBox',`${view.x-view.width/2} ${view.y-view.width/aspect/2} ${view.width} ${view.width/aspect}`);
    let markup=`<defs><pattern id="od-cost-grid" width="1" height="1" patternUnits="userSpaceOnUse"><path d="M 1 0 L 0 0 0 1" fill="none" stroke="#57656b" stroke-width=".015"/></pattern></defs><rect x="${view.x-view.width/2}" y="${view.y-view.width/aspect/2}" width="${view.width}" height="${view.width/aspect}" fill="url(#od-cost-grid)"/>`;
    keys.forEach(key=>{
      const layer=layers[key];if(!layer?.data||!enabled[key]||!usable(layer))return;
      const d=layer.data,o=mapPoint(d,d.origin),yaw=d.origin.yaw+d.map_from_frame.yaw;
      markup+=`<g data-layer="${key}" transform="translate(${o.x} ${-o.y}) rotate(${-yaw*180/Math.PI}) scale(${d.resolution})" opacity="${age(layer)>=3?.3:1}"><image href="${esc(d.image)}" x="0" y="${-d.height}" width="${d.width}" height="${d.height}" style="image-rendering:pixelated"/><rect x="0" y="${-d.height}" width="${d.width}" height="${d.height}" fill="none" stroke="${key==='global'?'#bd79ff':'#22d3ee'}" stroke-width="1.5" vector-effect="non-scaling-stroke"/></g>`;
    });
    if(robotSource){
      const d=robotSource.data,points=(d.path||[]).map(p=>{const v=mapPoint(d,p);return `${v.x},${-v.y}`;}).join(' ');
      markup+=`<polyline data-overlay="path" points="${points}" fill="none" stroke="#ffdd75" stroke-width="2.5" vector-effect="non-scaling-stroke"/>`;
      markup+=`<g data-overlay="robot" transform="translate(${point.x} ${-point.y}) rotate(${-(robot.yaw+d.map_from_frame.yaw)*180/Math.PI})"><circle r=".3" fill="#1576e7" stroke="white" stroke-width="1.5" vector-effect="non-scaling-stroke"/><path d="M .55 0 L -.16 -.2 L -.16 .2 Z" fill="white"/></g>`;
    }
    svg.innerHTML=markup;
  }
  new ResizeObserver(draw).observe(svg);
  svg.addEventListener('pointerdown',e=>{if(e.button!==0)return;svg.setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,view:{...view}};follow=false;draw();});
  svg.addEventListener('pointermove',e=>{if(!drag)return;const ratio=drag.view.width/svg.clientWidth;view={...drag.view,x:drag.view.x-(e.clientX-drag.x)*ratio,y:drag.view.y-(e.clientY-drag.y)*ratio};draw();});
  svg.addEventListener('pointerup',()=>drag=null);svg.addEventListener('pointercancel',()=>drag=null);
  svg.addEventListener('wheel',e=>{
    e.preventDefault();follow=false;
    const box=svg.getBoundingClientRect(),dx=e.clientX-box.left-box.width/2,dy=e.clientY-box.top-box.height/2;
    const width=Math.max(2,Math.min(500,view.width*Math.exp(e.deltaY*.001)));
    view={width,x:view.x+dx*(view.width-width)/box.width,y:view.y+dy*(view.width-width)/box.width};draw();
  },{passive:false});
  keys.forEach(key=>{
    const layer=layers[key]={data:null,received:0,error:'',busy:false,timer:null,controller:null};
    layer.poll=async()=>{
      if(!active||document.hidden||layer.busy)return;
      layer.busy=true;layer.controller=new AbortController();const timeout=setTimeout(()=>layer.controller.abort(),2500);
      try{
        const response=await fetch('/api/outdoor/costmaps/'+key,{cache:'no-store',signal:layer.controller.signal});
        const data=await response.json();if(!response.ok)throw Error(data.detail||data.error||response.statusText);
        const bitmap=new Image();bitmap.src=data.image;await bitmap.decode();
        layer.data=data;layer.received=performance.now();layer.error='';draw();
      }catch(e){if(active&&!document.hidden){layer.error=tr('costmapUnavailable')+(e.name==='AbortError'?'':': '+e.message);draw();}}
      finally{clearTimeout(timeout);layer.busy=false;layer.controller=null;if(active&&!document.hidden)layer.timer=setTimeout(layer.poll,1000);}
    };
  });
  function update(){
    clearInterval(clock);
    keys.forEach(key=>{const l=layers[key];clearTimeout(l.timer);if(active&&!document.hidden)l.poll();else l.controller?.abort();});
    if(active&&!document.hidden){clock=setInterval(draw,250);draw();}
  }
  document.addEventListener('visibilitychange',update);
  return {setActive(value){if(value===active)return;active=value;container.hidden=!value;update();}};
};
