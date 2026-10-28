import {limits} from './math.js';
import {prepareAssets, makePane, drawPane, boundsFor} from './panes.js';
import {drawCharts, updateCursors} from './charts.js';

const $ = id => document.getElementById(id);
const orbit = {dist: 8, angle: -Math.PI / 2, elev: 0.4};
let dataset, panes = [], range = [0, 1], bounds, time = 0, playing = false;
let lastWall = performance.now(), ready = false;

function render() {
  if (!ready) return;
  const options = {orbit, bounds, range, camera: $('camera').value,
    trail: $('trail').value, maxGap: dataset.max_interpolation_gap_s};
  panes.forEach(pane => drawPane(pane, time, options));
  $('clock').textContent = `${time.toFixed(3)} s`;
  $('time').value = String(time);
  updateCursors(time, range);
}

function pause() { playing = false; $('play').textContent = 'Play'; }
function seek(value) { time = Math.max(range[0], Math.min(range[1], value)); render(); }

function charts() {
  if (ready) drawCharts(dataset.cases, range, Number($('axis').value), dataset.max_interpolation_gap_s);
}

function setRange() {
  if (!dataset) return;
  range = limits(dataset.cases, $('range').value === 'full');
  if (range[1] <= range[0]) throw Error('The two recordings have no shared time interval.');
  $('time').min = String(range[0]); $('time').max = String(range[1]);
  $('time-start').textContent = `${range[0].toFixed(2)} s`;
  $('time-end').textContent = `${range[1].toFixed(2)} s`;
  bounds = boundsFor(dataset.cases, range);
  for (const button of $('events').children) {
    button.disabled = Number(button.dataset.time) < range[0] || Number(button.dataset.time) > range[1];
  }
  charts(); seek(time);
}

function eventButton(label, value) {
  if (value === null) return;
  const button = document.createElement('button');
  button.textContent = label; button.dataset.time = String(value);
  button.addEventListener('click', () => { pause(); seek(value); });
  $('events').append(button);
}

$('play').addEventListener('click', () => {
  if (playing) pause();
  else {
    if (time >= range[1]) seek(range[0]);
    playing = true; $('play').textContent = 'Pause'; lastWall = performance.now();
  }
});
$('time').addEventListener('input', event => { pause(); seek(Number(event.target.value)); });
$('range').addEventListener('change', () => { pause(); setRange(); });
$('axis').addEventListener('change', charts);
$('camera').addEventListener('change', render);
$('trail').addEventListener('change', render);
window.addEventListener('resize', () => { charts(); render(); });
window.renderFrame = seconds => { pause(); seek(seconds); };

try {
  const [response, sceneResponse] = await Promise.all([
    fetch('/api/replay', {cache: 'no-store'}), fetch('/scene.js'),
  ]);
  if (!response.ok || !sceneResponse.ok) throw Error('Replay data or the Rumoca scene could not be loaded.');
  dataset = await response.json();
  if (dataset.schema !== 'fastdyn-acoustic-replay-v1' || dataset.cases?.length !== 2) {
    throw Error('Unsupported acoustic replay format.');
  }
  const source = await sceneResponse.text();
  await prepareAssets();
  for (let i = 0; i < dataset.cases.length; i++) {
    panes.push(await makePane(dataset.cases[i], i, source, orbit, render,
      dataset.max_interpolation_gap_s));
  }
  const exposure = dataset.cases[1];
  eventButton('Sweep onset', 0);
  eventButton('Resonance crossing', exposure.resonance_s);
  eventButton('Early ground contact', exposure.metrics.early_contact_s);
  eventButton('Sweep end', exposure.settings.duration_s);
  ready = true; setRange(); time = range[0]; render();
  const notes = dataset.cases.flatMap(c => c.notices.map(note => `${c.label}: ${note}`));
  $('notice').textContent = `Blue: ${dataset.cases[0].label} · Orange: ${dataset.cases[1].label} · `
    + `Replay: ${dataset.pose_sample_hz} Hz + key events · Metrics: all original samples`
    + (notes.length ? ` · ${notes.join(' · ')}` : ' · Dashed: PX4 estimate / gyro before injection');
  $('play').disabled = false; $('time').disabled = false;
  window.sceneReady = true;
} catch (error) {
  $('notice').textContent = error.message;
  console.error(error); window.sceneReady = false;
}

function animate(now) {
  if (playing && ready) {
    seek(time + (now - lastWall) / 1000 * Number($('speed').value));
    if (time >= range[1]) pause();
  }
  lastWall = now;
  requestAnimationFrame(animate);
}
document.addEventListener('visibilitychange', () => { lastWall = performance.now(); });
requestAnimationFrame(animate);
