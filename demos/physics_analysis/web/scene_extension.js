// Demo-only extension to the pinned Rumoca scene. Frame convention matches it:
// NWU position [N,W,U] -> Three.js [-W,U,N]. Never modify the submodule.
{
  const init = ctx.onInit, frame = ctx.onFrame;
  ctx.onInit = async function(api) {
    await init(api);
    const {THREE, scene, state:s} = api;
    s.demoTrailPositions = new Float32Array(4096*3);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(s.demoTrailPositions, 3));
    geometry.setDrawRange(0, 0);
    s.demoTrail = new THREE.Line(geometry, new THREE.LineBasicMaterial({color:0x19b5ff}));
    s.demoTrail.frustumCulled = false;
    scene.add(s.demoTrail);
    if (api.cam) api.cam.dist = 8;
    s.demoTrailCount = 0;
    s.demoTrailTime = -Infinity;
    if(!s.demoReplay) {
      const {addRoute}=await import('/web/route.js');
      s.demoLoadRoute=async()=>{
        const spec=await fetch('/api/scenario').then(r=>r.json());
        if(spec.waypoints_ne_m?.length)s.demoRoute=addRoute(THREE,scene,spec);
      };
      s.demoRouteFetchAt=-Infinity;
    }
    // Arming is not inferred from motor speed. Truth-only viewers hide this beacon.
    if (s.armingBeacon) { s.armingBeacon.marker.visible = false; s.armingBeacon.light.visible = false; }
  };
  ctx.onFrame = function(api) {
    frame(api);
    // Illustrative 3x mesh scale, identical in live and replay views.
    for (const i of [0,1,2,4,5,6,8,9,10]) api.state.quad.matrix.elements[i] *= 3;
    const s = api.state, t = api.get('t');
    if (!Number.isFinite(t)) return;
    // Deterministic, illustrative propeller phase; does not depend on render FPS.
    s.propGroups.forEach((p,i) => { p.group.rotation.y = p.spinSign*t*Math.min(Math.abs(api.get(`omega_m${i+1}`) ?? 0)/20, 25); });
    if (s.demoReplay) return;
    if(!s.demoRoute&&!s.demoRouteLoading&&performance.now()-s.demoRouteFetchAt>1000){
      s.demoRouteLoading=true;s.demoRouteFetchAt=performance.now();
      s.demoLoadRoute().catch(e=>console.warn('Route overlay:',e)).finally(()=>s.demoRouteLoading=false);
    }
    if (t < s.demoTrailTime) s.demoTrailCount = 0;
    if (t === s.demoTrailTime) return;
    s.demoTrailTime = t;
    if (s.demoTrailCount === 4096) {
      s.demoTrailPositions.copyWithin(0, 3);
      s.demoTrailCount--;
    }
    const offset = s.demoTrailCount++*3;
    s.demoTrailPositions.set([-api.get('py'), api.get('pz'), api.get('px')], offset);
    s.demoTrail.geometry.attributes.position.needsUpdate = true;
    s.demoTrail.geometry.setDrawRange(0, s.demoTrailCount);
  };
}
