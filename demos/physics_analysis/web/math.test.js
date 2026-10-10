import test from 'node:test';
import assert from 'node:assert/strict';
import {worldToThree,slerp,sample} from './math.js';
test('NWU positions match the Rumoca scene basis',()=>{
  assert.deepEqual(worldToThree([1,2,3]),[-2,3,1]);
});
test('shortest quaternion interpolation and normalization',()=>{
  assert.deepEqual(slerp([1,0,0,0],[-1,0,0,0],.5),[1,0,0,0]);
  const q=slerp([1,0,0,0],[0,0,0,1],.5);
  assert.ok(Math.abs(q[0]-Math.SQRT1_2)<1e-12);
  assert.ok(Math.abs(q[3]-Math.SQRT1_2)<1e-12);
});
test('shared time: no extrapolation, no motion across missing data',()=>{
  const f=t=>({t,p:[t,0,0],q:[1,0,0,0]});
  assert.equal(sample([f(1),f(1.1)],0),null);
  assert.equal(sample([f(1),f(1.1)],2),null);
  assert.equal(sample([f(1),f(3)],2),null);
  assert.equal(sample([f(1),f(1.1)],1.05).p[0],1.05);
});

test('estimated track stays separate from truth and disappears at telemetry gaps',()=>{
  const a={t:1,p:[0,0,15],q:[1,0,0,0],estimated_p:[12,8,15]};
  const b={...a,t:1.1,p:[1,0,15],estimated_p:[13,8,15]};
  const mid=sample([a,b],1.05);
  assert.deepEqual(mid.p,[.5,0,15]);
  assert.deepEqual(mid.estimated_p,[12.5,8,15]);
  assert.equal(sample([a,{...b,estimated_p:null}],1.05).estimated_p,null);
});
