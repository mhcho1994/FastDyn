import test from 'node:test';
import assert from 'node:assert/strict';
import {worldToThree, slerp, sample, limits} from './math.js';

test('NWU positions use the Rumoca scene basis', () => {
  assert.deepEqual(worldToThree([1, -2, -3]), [2, -3, 1]);
});

test('antipodal quaternions interpolate without a full spin', () => {
  assert.deepEqual(slerp([1, 0, 0, 0], [-1, 0, 0, 0], .5), [1, 0, 0, 0]);
  const q = slerp([1, 0, 0, 0], [0, 0, 0, -1], .5);
  assert.ok(Math.abs(q[0] - Math.SQRT1_2) < 1e-12);
  assert.ok(Math.abs(q[3] + Math.SQRT1_2) < 1e-12);
});

test('gaps and timestamps outside recordings never invent a pose', () => {
  const frame = t => ({t, p: [t, 0, 10], q: [1, 0, 0, 0], pressure_pa: t});
  assert.equal(sample([frame(1), frame(2)], 1.5), null);
  assert.equal(sample([frame(1), frame(1.1)], 0), null);
  assert.equal(sample([frame(1), frame(1.1)], 2), null);
  assert.equal(sample([frame(1), frame(2)], 1).p[0], 1);
  const mid = sample([frame(1), frame(1.1)], 1.05);
  assert.equal(mid.p[0], 1.05);
  assert.equal(mid.pressure_pa, 1); // Sensor channels retain their recorded sample.
});

test('replay ranges use only the common recorded interval after onset alignment', () => {
  const a = {frames: [{t: -20}, {t: 90}], settings: {duration_s: 20}};
  const b = {frames: [{t: -25}, {t: 80}], settings: {duration_s: 20}};
  assert.deepEqual(limits([a, b]), [-3, 30]);
  assert.deepEqual(limits([a, b], true), [-20, 80]);
});
