// Pose interpolation adapted from physics_analysis at FastDyn 74afbcc.
export const worldToThree = ([n, w, u]) => [-w, u, n];

export function slerp(a, b, fraction) {
  let dot = a.reduce((value, x, i) => value + x * b[i], 0);
  if (dot < 0) { b = b.map(x => -x); dot = -dot; }
  dot = Math.min(1, Math.max(-1, dot));
  let q;
  if (dot > 0.9995) q = a.map((x, i) => x + (b[i] - x) * fraction);
  else {
    const angle = Math.acos(dot), denominator = Math.sin(angle);
    q = a.map((x, i) => (x * Math.sin((1 - fraction) * angle)
      + b[i] * Math.sin(fraction * angle)) / denominator);
  }
  const norm = Math.hypot(...q);
  return q.map(x => x / norm);
}

export function upperBound(frames, time) {
  let lo = 0, hi = frames.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (frames[mid].t <= time) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

export function sample(frames, time, maxGap = 0.25) {
  if (!frames.length || time < frames[0].t || time > frames.at(-1).t) return null;
  const index = upperBound(frames, time) - 1;
  const a = frames[index], b = frames[Math.min(index + 1, frames.length - 1)];
  if (b.t - a.t > maxGap && time !== a.t) return null;
  const u = b.t > a.t ? (time - a.t) / (b.t - a.t) : 0;
  // Sensor values remain sampled values; only pose is interpolated for animation.
  return {...a, t: time, p: a.p.map((x, i) => x + u * (b.p[i] - x)), q: slerp(a.q, b.q, u)};
}

export function limits(cases, full = false) {
  const first = Math.max(...cases.map(c => c.frames[0].t));
  const last = Math.min(...cases.map(c => c.frames.at(-1).t));
  if (full) return [first, last];
  return [Math.max(first, -3), Math.min(last, Math.max(...cases.map(c => c.settings.duration_s)) + 10)];
}
