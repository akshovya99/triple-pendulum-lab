const C = require('../src/core.js'); const assert = require('assert');
const P0 = { ...C.DEFAULTS, cf: 0, jd: 0 };
// Energy of the undamped, unforced system (T from the mass matrix, V from link COM heights)
function energy(pl, P) {
  const s = pl.s; pl.terms(s); let T = 0;
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) T += 0.5 * s[4 + i] * pl.M[i * 4 + j] * s[4 + j];
  const L = [P.L1, P.L2, P.L3], m = [P.m1, P.m2, P.m3]; let V = 0, y = 0;
  for (let i = 0; i < 3; i++) { V += m[i] * P.g * (y + 0.5 * L[i] * Math.cos(s[1 + i])); y += L[i] * Math.cos(s[1 + i]); }
  return T + V;
}
{ const pl = new C.Plant(P0), z = new Float64Array(4); pl.s.set([0, 0.3, -0.2, 0.5, 0, 0, 0, 0]);
  const e0 = energy(pl, P0); for (let i = 0; i < 200; i++) pl.substep(0, z);
  const d = Math.abs(energy(pl, P0) - e0); console.log('energy drift over 1 s:', d.toExponential(2)); assert(d < 0.02 * e0); }
// Horizontal momentum is conserved with no gravity, damping or external force
{ const P = { ...P0, g: 0 }, pl = new C.Plant(P), z = new Float64Array(4); pl.s.set([0, 0.3, -0.2, 0.5, 0.2, 1, -1, 0.5]);
  const mom = () => { pl.terms(pl.s); let p = 0; for (let j = 0; j < 4; j++) p += pl.M[j] * pl.s[4 + j]; return p; };
  const m0 = mom(); for (let i = 0; i < 400; i++) pl.substep(0, z);
  const d = Math.abs(mom() - m0); console.log('momentum drift:', d.toExponential(2)); assert(d < 1e-8); }
// Inelastic end-stop zeroes the cart velocity and clamps position
{ const pl = new C.Plant({ ...C.DEFAULTS }), z = new Float64Array(4); pl.s.set([2.399, 0, 0, 0, 3, 0, 0, 0]); pl.substep(0, z);
  assert.strictEqual(pl.s[0], C.TRACK); assert.strictEqual(pl.s[4], 0); assert(pl.hitStop); console.log('end-stop ok'); }
// The upright equilibrium is unstable without control
{ const env = new C.Env({ ...C.DEFAULTS }, C.makeRng(1)); env.reset(1); while (!env.done && !env.trunc) env.step(6);
  assert(env.done && env.steps < 100); console.log('uncontrolled pendulum falls after', env.steps, 'steps'); }
console.log('physics: pass');
