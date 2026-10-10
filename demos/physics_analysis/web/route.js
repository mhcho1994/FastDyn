// Shared live/replay overlay. World NWU -> Three.js [E,U,N].
export function addRoute(THREE, scene, spec, color='#73c9df') {
  const group=new THREE.Group();scene.add(group);
  const points=spec?.waypoints_ne_m??[],alt=spec?.altitude_m??15;
  function path(ne,material) {
    const mesh=new THREE.Line(new THREE.BufferGeometry().setFromPoints(ne.map(([n,e])=>new THREE.Vector3(e,alt,n))),material);
    mesh.computeLineDistances();group.add(mesh);
  }
  if(points.length) {
    path([[0,0],...points],new THREE.LineDashedMaterial({color,dashSize:1,gapSize:.6}));
    path([points.at(-1),[0,0]],new THREE.LineDashedMaterial({color:'#8d9fb0',dashSize:2,gapSize:1}));
    [[0,0],...points].forEach(([n,e],i)=>{
      const canvas=document.createElement('canvas');canvas.width=128;canvas.height=64;
      const ctx=canvas.getContext('2d');ctx.fillStyle='#122536';ctx.fillRect(0,0,128,64);
      ctx.font='bold 36px system-ui';ctx.textAlign='center';ctx.fillStyle=i?color:'#ffffff';ctx.fillText(i?String(i):'HOME',64,46);
      const texture=new THREE.CanvasTexture(canvas);
      const label=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false}));
      label.position.set(e,alt+2,n);label.scale.set(4,2,1);group.add(label);
      const ring=new THREE.Mesh(new THREE.TorusGeometry(spec.acceptance_radius_m??2,.045,6,32),new THREE.MeshBasicMaterial({color:i?color:'#ffffff'}));
      ring.rotation.x=Math.PI/2;ring.position.set(e,alt,n);group.add(ring);
    });
  }
  return {group,dispose(){scene.remove(group);group.traverse(o=>{o.geometry?.dispose();o.material?.map?.dispose();o.material?.dispose();});}};
}
