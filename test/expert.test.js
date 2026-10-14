const C = require('../src/core.js'); const assert = require('assert');
const P = { ...C.DEFAULTS }, rng = C.makeRng(7);
function run(noise, jitter, spread, n) {
  let ok = 0;
  for (let e = 0; e < n; e++) {
    const env = new C.Env(P, rng), ex = new C.ExpertPID(P); env.reset(spread);
    while (!env.done && !env.trunc) {
      let a = C.quantize(C.clamp((ex.act(env.plant.s) + noise * P.fmax * rng.gauss()) / P.fmax, -1, 1));
      if (jitter && rng() < jitter) a = C.clamp(a + rng.int(3) - 1, 0, 12);
      env.step(a);
    }
    if (env.trunc) ok++;
  }
  return ok / n;
}
const clean = run(0, 0, 1, 20), noisy = run(0.03, 0.05, 1.2, 40);
console.log('expert full-length rate  clean:', clean, ' noisy demos:', noisy);
assert(clean === 1 && noisy > 0.85);
console.log('expert: pass');
