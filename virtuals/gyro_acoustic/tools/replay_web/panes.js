// Rumoca scene integration follows the physics_analysis replay (FastDyn 74afbcc).
import {THREE, GLTFLoader} from '/vendor/three_viewer.js';
import {sample, upperBound, worldToThree} from './math.js';

export const colors = ['#54c9f2', '#ffb360'];
const fmt = (v, digits = 2) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const vector = (values, digits = 1) => values.map(v => fmt(v, digits)).join(' / ');

export async function prepareAssets() {
  // Share source bytes/textures between panes; GLTFLoader builds separate scene objects.
  // Consume the model response completely before handing it to Three's loading pipeline.
  THREE.Cache.enabled = true;
  const path = '/assets/models/drone.glb';
  const response = await fetch(path);
  if (!response.ok) throw Error('The Rumoca drone.glb asset could not be loaded.');
  THREE.Cache.add(path, await response.arrayBuffer());
}

export async function makePane(data, index, sceneSource, orbit, onChange, maxGap) {
  const node = document.createElement('section');
  node.className = 'pane';
  node.style.setProperty('--case-color', colors[index]);
  node.innerHTML = '<div class="pane-top"><strong></strong><span class="badge"></span></div>'
    + '<div class="view"><div class="overlay"></div></div>'
    + '<div class="readout"></div><div class="metrics"></div>';
  node.querySelector('strong').textContent = data.label;
  document.getElementById('panes').append(node);
  const view = node.querySelector('.view');
  const renderer = new THREE.WebGLRenderer({antialias: true, preserveDrawingBuffer: true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  const scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(55, 1, 0.05, 2000);
  const state = {}, ctx = {}, cam = {...orbit, target: new THREE.Vector3()};
  view.append(renderer.domElement);
  new Function('ctx', sceneSource)(ctx);
  await ctx.onInit({THREE, GLTFLoader, scene, camera, renderer, state, cam, frames: {}});
  // Neither arming state nor motor RPM is present in this trace.
  if (state.armingBeacon) {
    state.armingBeacon.marker.visible = false;
    state.armingBeacon.light.visible = false;
  }
  const segments = [], segmentTimes = [];
  for (let i = 1; i < data.frames.length; i++) {
    const a = data.frames[i - 1], b = data.frames[i];
    if (b.t - a.t > maxGap) continue;
    segments.push(new THREE.Vector3(...worldToThree(a.p)), new THREE.Vector3(...worldToThree(b.p)));
    segmentTimes.push({t: b.t});
  }
  const trail = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(segments),
    new THREE.LineBasicMaterial({color: colors[index]}));
  trail.frustumCulled = false;
  scene.add(trail);
  const home = new THREE.Mesh(new THREE.RingGeometry(0.8, 1, 40),
    new THREE.MeshBasicMaterial({color: colors[index], side: THREE.DoubleSide}));
  home.rotation.x = -Math.PI / 2;
  home.position.y = 0.03;
  scene.add(home);
  const fields = ['position', 'attitude', 'frequency', 'pressure', 'false-rate', 'pwm'];
  const labels = ['N / E / Up [m]', 'Roll / Pitch / Yaw [°]', 'Frequency [kHz]',
    'Pressure [Pa RMS]', 'False XYZ [rad/s]', 'PWM [µs]'];
  fields.forEach((key, i) => {
    const div = document.createElement('div'), label = document.createElement('span');
    const value = document.createElement('b');
    label.textContent = labels[i]; value.dataset.field = key;
    div.append(label, value); node.querySelector('.readout').append(div);
  });
  const m = data.metrics;
  node.querySelector('.metrics').textContent = `${m.source_samples.toLocaleString('en-US')} source samples · `
    + `Comparison window: 0–${m.window_s[1]} s · Peak false rate: ${fmt(m.max_false_rate_rad_s, 3)} rad/s · `
    + `Max horizontal distance: ${fmt(m.max_horizontal_distance_m)} m · `
    + `Early contact: ${m.early_contact_s === null ? 'none' : fmt(m.early_contact_s, 3) + ' s'}`;
  let dragging = false, previousX = 0, previousY = 0;
  view.addEventListener('pointerdown', event => {
    dragging = true; previousX = event.clientX; previousY = event.clientY;
    view.setPointerCapture(event.pointerId);
  });
  for (const kind of ['pointerup', 'pointercancel']) view.addEventListener(kind, () => dragging = false);
  view.addEventListener('pointermove', event => {
    if (!dragging) return;
    orbit.angle -= (event.clientX - previousX) * 0.006;
    orbit.elev = Math.max(0.08, Math.min(1.5, orbit.elev + (event.clientY - previousY) * 0.006));
    previousX = event.clientX; previousY = event.clientY; onChange();
  });
  view.addEventListener('wheel', event => {
    event.preventDefault();
    orbit.dist = Math.max(2, Math.min(80, orbit.dist * Math.exp(event.deltaY * 0.001)));
    onChange();
  }, {passive: false});
  new ResizeObserver(() => {
    renderer.setSize(view.clientWidth, view.clientHeight);
    camera.aspect = view.clientWidth / view.clientHeight;
    camera.updateProjectionMatrix(); onChange();
  }).observe(view);
  return {data, node, view, renderer, scene, camera, state, ctx, cam, trail, segmentTimes};
}

export function boundsFor(cases, range) {
  const bounds = new THREE.Box3();
  for (const data of cases) for (const frame of data.frames) {
    if (range[0] <= frame.t && frame.t <= range[1]) {
      bounds.expandByPoint(new THREE.Vector3(...worldToThree(frame.p)));
    }
  }
  bounds.expandByPoint(new THREE.Vector3());
  return {center: bounds.getCenter(new THREE.Vector3()), span: Math.max(20, bounds.getSize(new THREE.Vector3()).length())};
}

export function drawPane(pane, time, options) {
  const {data, node, renderer, scene, camera, state, ctx, cam, trail, segmentTimes} = pane;
  const frame = sample(data.frames, time, options.maxGap);
  Object.assign(cam, options.orbit);
  state.quad.visible = Boolean(frame);
  state.shadow.visible = Boolean(frame);
  const start = options.trail === 'all' ? options.range[0] : Math.max(options.range[0], time - Number(options.trail));
  const a = upperBound(segmentTimes, start), b = upperBound(segmentTimes, time);
  trail.geometry.setDrawRange(a * 2, Math.max(0, b - a) * 2);
  const badge = node.querySelector('.badge');
  const contact = data.metrics.early_contact_s !== null && time >= data.metrics.early_contact_s;
  const active = frame && frame.pressure_pa > 0;
  badge.className = `badge${contact ? ' contact' : active ? ' active' : ''}`;
  badge.textContent = !frame ? 'NO RECORDING' : contact ? 'AFTER EARLY GROUND CONTACT' : active ? 'ACOUSTIC ON' : 'ACOUSTIC OFF';
  if (frame) {
    const signals = {t: time, px: frame.p[0], py: frame.p[1], pz: frame.p[2],
      q0: frame.q[0], q1: frame.q[1], q2: frame.q[2], q3: frame.q[3]};
    ctx.onFrame({THREE, scene, state, get: key => signals[key], motors: {}, camera, cam,
      cameraMode: options.camera, frames: {}});
    // Match the original demo's visible mesh scale, retaining physical trajectory units.
    for (const i of [0, 1, 2, 4, 5, 6, 8, 9, 10]) state.quad.matrix.elements[i] *= 3;
    node.querySelector('.overlay').textContent = `FMU time ${fmt(time + data.sweep_start_sim_s, 3)} s\n`
      + `Sweep onset ${fmt(data.sweep_start_sim_s, 3)} s · ${data.source}`;
    const values = [vector([frame.p[0], -frame.p[1], frame.p[2]]), vector(frame.attitude_deg),
      fmt(frame.frequency_hz / 1000, 3), fmt(frame.pressure_pa, 2),
      vector(frame.false_rate_rad_s, 3), vector(frame.pwm_us, 0)];
    node.querySelectorAll('[data-field]').forEach((element, i) => element.textContent = values[i]);
  } else {
    node.querySelector('.overlay').textContent = 'No valid recording. Missing intervals are not interpolated.';
    node.querySelectorAll('[data-field]').forEach(element => element.textContent = '—');
  }
  if (options.camera !== 'scene') {
    const {center, span} = options.bounds;
    const radius = span * cam.dist / 8;
    const elev = options.camera === 'top' ? Math.PI / 2 - 0.001 : cam.elev;
    camera.position.set(center.x + radius * Math.sin(cam.angle) * Math.cos(elev),
      center.y + radius * Math.sin(elev), center.z + radius * Math.cos(cam.angle) * Math.cos(elev));
    camera.lookAt(center);
  }
  state.horizonHaze?.position.copy(camera.position);
  state.lowerSkyOccluder?.position.copy(camera.position);
  renderer.render(scene, camera);
}
