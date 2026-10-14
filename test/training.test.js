// Smoke test of the whole pipeline: demos -> pretraining -> a short RL run. Outcomes vary by seed, so this only
// checks that every stage runs, stays finite and learns something measurable.
const C = require('../src/core.js'); const assert = require('assert');
const P = { ...C.DEFAULTS }, rng = C.makeRng(5), tr = new C.Trainer(P, rng);
tr.start(); assert(!tr.demoWarn);
while (tr.phase === 'demos') tr.run(1e9, 5);
assert.strictEqual(tr.demoEps, 60); console.log('demo transitions:', tr.agent.demo.size);
while (tr.phase === 'pretrain') tr.run(1e9, 50);
assert.strictEqual(tr.agent.updates, 1500);
const evalG = (n) => { const e = new C.Env(P, rng); let s = 0; for (let k = 0; k < n; k++) { e.reset(1); while (!e.done && !e.trunc) e.step(tr.agent.greedy(e.obs)); s += e.steps; } return s / n; };
const pre = evalG(20); console.log('mean greedy episode length after pretraining:', pre.toFixed(0));
const t0 = Date.now(); while (tr.rlSteps < 20000 && Date.now() - t0 < 60000) tr.run(1e9, 50);
console.log('RL steps', tr.rlSteps, 'avg train episode length', tr.avgLen.toFixed(0), 'loss', tr.lossEma.toFixed(4));
assert(Number.isFinite(tr.lossEma) && pre > 25);
console.log('training: pass');
