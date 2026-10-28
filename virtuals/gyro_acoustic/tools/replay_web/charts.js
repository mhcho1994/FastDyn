import {colors} from './panes.js';

const definitions = [
  {id: 'altitude', value: f => f.p[2]},
  {id: 'distance', value: f => Math.hypot(f.p[0], f.p[1])},
  {id: 'attitude', value: (f, axis) => f.attitude_deg[axis], telemetry: 'attitude', angles: true},
  {id: 'gyro', value: (f, axis) => f.gyro_deg_s[axis], clean: (f, axis) => f.clean_gyro_deg_s[axis]},
  {id: 'false-rate', value: (f, axis) => f.false_rate_rad_s[axis]},
  {id: 'body-rate', value: (f, axis) => f.rate_deg_s[axis], telemetry: 'rate'},
];

function drawChart(spec, cases, range, axis, maxGap) {
  const canvas = document.querySelector(`#${spec.id} canvas`), ctx = canvas.getContext('2d');
  const width = canvas.clientWidth, height = canvas.clientHeight, ratio = devicePixelRatio || 1;
  canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  const series = [];
  cases.forEach((data, index) => {
    const add = (samples, get, dashed = false, gap = maxGap) => series.push({
      points: samples.filter(f => f.t >= range[0] && f.t <= range[1]).map(f => [f.t, get(f)]),
      color: colors[index], dashed, gap,
    });
    add(data.frames, f => spec.value(f, axis));
    if (spec.clean) add(data.frames, f => spec.clean(f, axis), true);
    if (spec.telemetry) add(data.telemetry[spec.telemetry] || [], f => f.value[axis], true, 0.75);
  });
  let low = 0, high = 0;
  for (const s of series) for (const [, value] of s.points) {
    if (Number.isFinite(value)) { low = Math.min(low, value); high = Math.max(high, value); }
  }
  const margin = Math.max(0.01, high - low) * 0.08;
  low -= margin; high += margin;
  const x = t => 54 + (t - range[0]) / (range[1] - range[0]) * (width - 66);
  const y = v => height - 24 - (v - low) / (high - low) * (height - 34);
  ctx.clearRect(0, 0, width, height);
  ctx.font = '10px system-ui'; ctx.lineWidth = 1;
  for (const data of cases.filter(c => c.settings.rms_pressure_pa > 0)) {
    ctx.fillStyle = '#dba05012';
    const start = Math.max(0, range[0]), end = Math.min(data.settings.duration_s, range[1]);
    if (end > start) ctx.fillRect(x(start), 8, x(end) - x(start), height - 32);
  }
  ctx.fillStyle = '#9cb2c4';
  for (let i = 0; i <= 3; i++) {
    const value = low + (high - low) * i / 3;
    ctx.strokeStyle = '#304452'; ctx.beginPath();
    ctx.moveTo(54, y(value)); ctx.lineTo(width - 12, y(value)); ctx.stroke();
    ctx.fillText(Math.abs(value) >= 1000 ? value.toFixed(0) : value.toFixed(1), 1, y(value) + 3);
  }
  for (const s of series) {
    ctx.strokeStyle = s.color; ctx.lineWidth = s.dashed ? 1.2 : 1.7;
    ctx.setLineDash(s.dashed ? [5, 4] : []);
    ctx.beginPath(); let previous = null;
    for (const point of s.points) {
      const [time, value] = point;
      if (!Number.isFinite(value)) { previous = null; continue; }
      const gap = !previous || time - previous[0] > s.gap
        || (spec.angles && Math.abs(value - previous[1]) > 180);
      if (gap) ctx.moveTo(x(time), y(value)); else ctx.lineTo(x(time), y(value));
      previous = point;
    }
    ctx.stroke();
  }
  const exposure = cases[1];
  for (const [time, color] of [[exposure.resonance_s, '#c1cbd0'],
    [exposure.metrics.early_contact_s, '#ee81b2']]) {
    if (time === null || time < range[0] || time > range[1]) continue;
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.setLineDash([2, 4]);
    ctx.beginPath(); ctx.moveTo(x(time), 8); ctx.lineTo(x(time), height - 24); ctx.stroke();
  }
  ctx.setLineDash([]); ctx.fillStyle = '#9cb2c4';
  for (let i = 0; i <= 4; i++) {
    const time = range[0] + i * (range[1] - range[0]) / 4;
    ctx.fillText(`${time.toFixed(1)} s`, Math.min(x(time), width - 42), height - 5);
  }
}

export function drawCharts(cases, range, axis, maxGap) {
  definitions.forEach(spec => drawChart(spec, cases, range, axis, maxGap));
}

export function updateCursors(time, range) {
  for (const graph of document.querySelectorAll('.graph')) {
    const x = 54 + (time - range[0]) / (range[1] - range[0]) * (graph.clientWidth - 66);
    graph.querySelector('.cursor').style.left = `${x}px`;
  }
}
