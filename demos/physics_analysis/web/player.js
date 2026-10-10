import {THREE, GLTFLoader} from '/vendor/three_viewer.js';
import {sample, upperBound, worldToThree} from './math.js';
import {addRoute} from './route.js';

const $=id=>document.getElementById(id), colors=['#44caff','#ffb369'];
const fmt=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
let dataset,panes=[],t=0,playing=false,lastWall=performance.now();
const sceneSource=await fetch('/scene.js').then(r=>{if(!r.ok)throw Error('Rumoca scene missing');return r.text();});

function line(points,color,dashed=false) {
  const geometry=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...worldToThree(p))));
  const material=dashed?new THREE.LineDashedMaterial({color,dashSize:1,gapSize:.6}):new THREE.LineBasicMaterial({color});
  const mesh=new THREE.Line(geometry,material);
  mesh.frustumCulled=false;
  if(dashed)mesh.computeLineDistances();
  return mesh;
}

async function makePane(index) {
  const node=document.createElement('section');node.className='pane';
  node.innerHTML='<div class="pane-top"><select aria-label="Comparison case"></select><span class="outcome"></span></div><div class="view"><div class="overlay"></div></div><div class="readout"></div><div class="metrics"></div>';
  $('panes').append(node);
  const select=node.querySelector('select'),view=node.querySelector('.view');
  dataset.cases.forEach((c,i)=>select.add(new Option(c.name,String(i))));
  const preferred=index===1?dataset.cases.findIndex(c=>c.metadata.scenario?.attack?.kind==='gps'):-1;
  select.value=String(preferred>=0?preferred:Math.min(index,dataset.cases.length-1));
  const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;
  renderer.shadowMap.type=THREE.PCFSoftShadowMap;renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1;
  view.append(renderer.domElement);
  const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(55,1,.05,1500);
  const cam={target:new THREE.Vector3(),dist:7,angle:-Math.PI/2,elev:.4};
  const ctx={},state={demoReplay:true};new Function('ctx',sceneSource)(ctx);
  await ctx.onInit({THREE,GLTFLoader,scene,camera,renderer,state,cam,frames:{}});
  const target=new THREE.Mesh(new THREE.TorusGeometry(.8,.025,8,40),new THREE.MeshBasicMaterial({color:colors[index]}));
  target.rotation.x = Math.PI/2;
  scene.add(target);
  const estimate=new THREE.Mesh(new THREE.SphereGeometry(.45,12,8),new THREE.MeshBasicMaterial({color:'#f3df63',wireframe:true}));
  scene.add(estimate);
  let trail,planned,estimatedTrail;
  const pane={index,node,view,renderer,scene,camera,cam,ctx,state,target,estimate};
  function choose() {
    pane.data=dataset.cases[Number(select.value)];
    planned?.dispose();
    for(const mesh of [trail,estimatedTrail])if(mesh){scene.remove(mesh);mesh.geometry.dispose();mesh.material.dispose();}
    trail=line(pane.data.frames.map(f=>f.p),colors[index]);scene.add(trail);pane.trail=trail;
    const spec=pane.data.metadata.scenario??{};
    planned=addRoute(THREE,scene,spec,colors[index]);
    // LineSegments breaks the estimated track at missing telemetry; never bridge gaps.
    const segments=[],segmentTimes=[];
    for(let i=1;i<pane.data.frames.length;i++){
      const a=pane.data.frames[i-1],b=pane.data.frames[i];
      if(a.estimated_p&&b.estimated_p&&b.t-a.t<=.5){segments.push(new THREE.Vector3(...worldToThree(a.estimated_p)),new THREE.Vector3(...worldToThree(b.estimated_p)));segmentTimes.push({t:b.t});}
    }
    estimatedTrail=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(segments),new THREE.LineBasicMaterial({color:'#f3df63',transparent:true,opacity:.6}));
    estimatedTrail.frustumCulled=false;scene.add(estimatedTrail);pane.estimatedTrail=estimatedTrail;pane.estimateTimes=segmentTimes;
    const bounds=new THREE.Box3().setFromPoints(pane.data.frames.map(f=>new THREE.Vector3(...worldToThree(f.p))));
    pane.center=bounds.getCenter(new THREE.Vector3());for(const [n,e] of spec.waypoints_ne_m??[])bounds.expandByPoint(new THREE.Vector3(e,spec.altitude_m,n));
    pane.center=bounds.getCenter(new THREE.Vector3());pane.span=Math.max(25,bounds.getSize(new THREE.Vector3()).length());
    const m=pane.data.metrics;
    node.querySelector('.outcome').textContent=m.completed===true?'FC MISSION + RTL COMPLETE':m.completed===false?'INCOMPLETE':'IMPORTED';
    node.querySelector('.metrics').textContent=`Waypoints: FC ${m.fc_waypoints_reached??'—'}/${m.planned_waypoints??'—'} · Truth dwell ${m.truth_waypoints_reached??'—'}/${m.planned_waypoints??'—'} · RTL home error ${fmt(m.rtl_home_error_m)} m · Analysis window: ${m.valid_window_s?.map(x=>fmt(x,1)).join('–')??'unavailable'} s · Heading P95 ${fmt(m.heading_error_deg.p95_abs)}° · Segment RMS ${fmt(m.segment_error_m.rms)} m · Altitude RMS ${fmt(m.altitude_error_m.rms)} m${m.contact_during_analysis_s!==null?' · Contact at '+fmt(m.contact_during_analysis_s)+' s':''}${m.mission_error?' · '+m.mission_error:''}`;
  }
  select.addEventListener('change',()=>{choose();render();});choose();
  let dragging=false,px=0,py=0;
  view.addEventListener('pointerdown',e=>{dragging=true;px=e.clientX;py=e.clientY;view.setPointerCapture(e.pointerId);});
  view.addEventListener('pointerup',()=>dragging=false);
  view.addEventListener('pointermove',e=>{if(!dragging)return;cam.angle-=(e.clientX-px)*.006;cam.elev=Math.max(.08,Math.min(1.5,cam.elev+(e.clientY-py)*.006));px=e.clientX;py=e.clientY;render();});
  view.addEventListener('wheel',e=>{e.preventDefault();cam.dist=Math.max(1,Math.min(80,cam.dist+e.deltaY*.01));render();},{passive:false});
  new ResizeObserver(()=>{const w=view.clientWidth,h=view.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();render();}).observe(view);
  return pane;
}

function drawPane(pane) {
  const {node,data,state,cam,camera,scene,ctx,renderer,target,trail,estimate}=pane;
  const f=sample(data.frames,t),mode=$('camera').value;
  state.quad.visible=Boolean(f);state.shadow.visible=Boolean(f);target.visible=Boolean(f?.target);
  const end=upperBound(data.frames,t),history=$('trail').value;
  const begin=history==='all'?0:upperBound(data.frames,t-Number(history));
  trail.geometry.setDrawRange(begin,Math.max(0,end-begin));
  const ee=upperBound(pane.estimateTimes,t),eb=history==='all'?0:upperBound(pane.estimateTimes,t-Number(history));
  pane.estimatedTrail.geometry.setDrawRange(2*eb,2*Math.max(0,ee-eb));
  estimate.visible=Boolean(f?.estimated_p);
  if(f?.estimated_p)estimate.position.set(...worldToThree(f.estimated_p));
  node.querySelector('.overlay').textContent=f?`${data.name} · ${f.attack_active?'ATTACK ACTIVE':'attack inactive'} · ${f.waypoint_index!=null?'WP '+(f.waypoint_index+1):'takeoff / RTL'}`:'No data at this simulation time';
  if(f){
    const signals={t:f.t,px:f.p[0],py:f.p[1],pz:f.p[2],...Object.fromEntries(f.q.map((x,i)=>[`q${i}`,x])),...Object.fromEntries(f.omega.map((x,i)=>[`omega_m${i+1}`,x]))};
    ctx.onFrame({THREE,scene,state,get:k=>signals[k],motors:signals,camera,cam,cameraMode:mode,frames:{}});
    if(f.target)target.position.set(...worldToThree(f.target));
    node.querySelector('.readout').textContent=`N ${fmt(f.p[0])} m   E ${fmt(-f.p[1])} m   Up ${fmt(f.p[2])} m\nHeading ${fmt(f.true_heading_deg,1)}°   Error ${fmt(f.heading_error_deg,1)}°\nMotors ${f.omega.map(x=>fmt(x,0)).join(' / ')} rad/s\nWind N/W/U ${f.wind.map(x=>fmt(x,1)).join(' / ')} m/s\nPosition estimate error ${fmt(f.position_estimate_error_m)} m\nMag Δ ${(f.mag_delta_gauss??[]).map(x=>fmt(x,3)).join(' / ')} G\nGPS offset N/E/U ${(f.gps_offset_neu??[]).map(x=>fmt(x,1)).join(' / ')} m\nGyro bias / false yaw ${fmt((f.gyro_bias_rps?.[2]??NaN)*180/Math.PI,1)} / ${fmt((f.gyro_false_rps?.[2]??NaN)*180/Math.PI,1)} °/s\nModel current proxy ${fmt(f.model_current_a)} A\nMeasured current ${fmt(f.current_a)} A   Voltage ${fmt(f.voltage_v)} V`;
  } else node.querySelector('.readout').textContent='Recording absent or a gap exceeds 0.5 s.\nNo extrapolated state.';
  if(mode!=='scene'){
    const c=pane.center,s=pane.span*cam.dist/8;
    camera.position.set(c.x+(mode==='top'?0:s*.5),c.y+s*(mode==='top'?1.05:.55),c.z+(mode==='top'?.01:s*.5));
    camera.lookAt(c);
    state.horizonHaze?.position.copy(camera.position);state.lowerSkyOccluder?.position.copy(camera.position);
  }
  renderer.render(scene,camera);
}

function chart(id,get,includeTarget=false) {
  const canvas=$(id),ctx=canvas.getContext('2d'),w=canvas.clientWidth,h=canvas.clientHeight,pad=42;
  const ratio=devicePixelRatio||1;canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);
  const start=Number($('time').min),end=Number($('time').max);
  let min=0,max=1;
  for(const p of panes)for(const f of p.data.frames){const v=get(f);if(Number.isFinite(v)){min=Math.min(min,v);max=Math.max(max,v);}}
  if(includeTarget)for(const p of panes)for(const c of p.data.commands)if(c.phase==='analysis'){min=Math.min(min,c.target_alt);max=Math.max(max,c.target_alt);}
  const span=Math.max(1,max-min);max+=span*.08;min-=span*.08;
  const x=s=>pad+(s-start)/(end-start||1)*(w-pad-12),y=v=>h-24-(v-min)/(max-min)*(h-40);
  ctx.clearRect(0,0,w,h);ctx.font='11px system-ui';ctx.fillStyle='#91a8bc';ctx.lineWidth=1;
  for(let i=0;i<=3;i++){const v=min+(max-min)*i/3;ctx.strokeStyle='#2c4153';ctx.beginPath();ctx.moveTo(pad,y(v));ctx.lineTo(w,y(v));ctx.stroke();ctx.fillText(v.toFixed(1),0,y(v)+5);}
  panes.forEach((p,i)=>{
    const attack=p.data.metadata.scenario?.attack;
    if(attack&&attack.kind!=='none'){ctx.fillStyle=i?'#ffb3690d':'#44caff0d';ctx.fillRect(x(attack.start_s),0,x(attack.start_s+attack.duration_s)-x(attack.start_s),h-24);}
    ctx.strokeStyle=colors[i];ctx.lineWidth=2.5;ctx.setLineDash([]);ctx.beginPath();let pen=false,previous=-Infinity;
    for(const f of p.data.frames){const v=get(f);if(!Number.isFinite(v)){pen=false;continue;}if(!pen||f.t-previous>.5)ctx.moveTo(x(f.t),y(v));else ctx.lineTo(x(f.t),y(v));pen=true;previous=f.t;}ctx.stroke();
    if(includeTarget){ctx.setLineDash([9,6]);ctx.beginPath();let pen=false;for(const c of p.data.commands){if(c.phase!=='analysis'){pen=false;continue;}if(!pen)ctx.moveTo(x(c.t),y(c.target_alt));else ctx.lineTo(x(c.t),y(c.target_alt));pen=true;}ctx.stroke();ctx.setLineDash([]);}
  });
  ctx.strokeStyle='#e4edf4';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(x(t),0);ctx.lineTo(x(t),h-20);ctx.stroke();
  ctx.fillStyle='#91a8bc';ctx.fillText(`${start.toFixed(0)} s`,pad,h-2);ctx.fillText(`${end.toFixed(0)} s`,w-75,h-2);
}

function render(){if(!panes.length)return;panes.forEach(drawPane);$('clock').textContent=`${t.toFixed(2)} s`;$('time').value=String(t);chart('altitude',f=>f.p[2],true);chart('heading',f=>f.analysis_valid?f.heading_error_deg:null);chart('position-error',f=>f.analysis_valid?f.position_estimate_error_m:null);chart('segment-error',f=>f.segment_error_m);}
function seek(value){t=Math.max(Number($('time').min),Math.min(Number($('time').max),value));render();}
window.renderFrame=seconds=>{playing=false;seek(seconds);};
$('time').addEventListener('input',e=>{playing=false;$('play').textContent='Play';seek(Number(e.target.value));});
$('play').addEventListener('click',()=>{playing=!playing;lastWall=performance.now();$('play').textContent=playing?'Pause':'Play';});
$('reset').addEventListener('click',()=>seek(Number($('time').min)));
$('camera').addEventListener('change',render);$('trail').addEventListener('change',render);
try {
  const response=await fetch('/api/replay',{cache:'no-store'});if(!response.ok)throw Error(await response.text());dataset=await response.json();
  if(dataset.schema_version!==1||!dataset.cases?.length)throw Error('Unsupported or empty replay');
  $('time').min=String(Math.min(...dataset.cases.map(c=>c.frames[0].t)));
  $('time').max=String(Math.max(...dataset.cases.map(c=>c.frames.at(-1).t)));
  t=Math.min(...dataset.cases.map(c=>c.metrics.valid_window_s?.[0]??c.frames[0].t));
  for(let i=0;i<Math.min(2,dataset.cases.length);i++)panes.push(await makePane(i));
  const source=decodeURIComponent(response.headers.get('X-Physics-Replay-Source')??'');
  $('notice').textContent=`${source?'Source: '+source+' · ':''}${dataset.cases.length} recorded case(s) · Shared simulation clock; select cases to compare · Solid: truth · Yellow: FC estimate · Numbered rings: planned waypoints · Shaded: attack window`;
  render();window.sceneReady=true;
} catch(error){$('notice').textContent=error.message;console.error(error);window.sceneReady=false;}
function animate(now){if(playing){const next=t+(now-lastWall)/1000*Number($('speed').value);seek(next);if(t>=Number($('time').max)){playing=false;$('play').textContent='Play';}}lastWall=now;requestAnimationFrame(animate);}
requestAnimationFrame(animate);
