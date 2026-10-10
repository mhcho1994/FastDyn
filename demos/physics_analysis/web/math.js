export const worldToThree = ([n,w,u]) => [-w,u,n];
export function slerp(a,b,u) {
  let dot = a.reduce((v,x,i)=>v+x*b[i],0);
  if (dot<0) { b=b.map(x=>-x); dot=-dot; }
  dot=Math.min(1,Math.max(-1,dot));
  let q;
  if(dot>.9995) q=a.map((x,i)=>x+(b[i]-x)*u);
  else { const th=Math.acos(dot), d=Math.sin(th); q=a.map((x,i)=>(x*Math.sin((1-u)*th)+b[i]*Math.sin(u*th))/d); }
  const n=Math.hypot(...q); return q.map(x=>x/n);
}
export function upperBound(frames,t) {
  let lo=0,hi=frames.length;
  while(lo<hi) { const mid=(lo+hi)>>>1; if(frames[mid].t<=t)lo=mid+1;else hi=mid; }
  return lo;
}
export function sample(frames,t) {
  if(!frames.length || t<frames[0].t || t>frames.at(-1).t)return null;
  const i=upperBound(frames,t)-1,a=frames[i],b=frames[Math.min(i+1,frames.length-1)];
  if (b.t-a.t>.5 && t!==a.t) return null; // missing data never becomes invented motion
  const u=b.t>a.t?(t-a.t)/(b.t-a.t):0;
  return {...a,t,estimated_p:a.estimated_p&&b.estimated_p?a.estimated_p.map((x,j)=>x+u*(b.estimated_p[j]-x)):null,p:a.p.map((x,j)=>x+u*(b.p[j]-x)),q:slerp(a.q,b.q,u)};
}
