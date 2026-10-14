/* =====================================================================
   UI (browser only)
   ===================================================================== */
(function () {
  if (typeof document === 'undefined') return;
  const $ = (id) => document.getElementById(id);
  const reduceMotion = !!(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches);

  const C = {
    bg: '#0c1117', panel: '#121a23', line: '#243140', grid: '#1b2631', text: '#d5dde8', muted: '#8494a8',
    amber: '#f2b24a', teal: '#38d0c0', mag: '#e66fb8', ice: '#9fc2ff', warm: '#ffd98a'
  };
  const LINK_COLORS = [C.amber, C.teal, C.mag];
  const FONT = "'IBM Plex Sans', system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif";

  /* ---------- model instances ---------- */
  const P = Object.assign({}, DEFAULTS);
  const seed = (Math.random() * 4294967296) >>> 0;
  const trainer = new Trainer(P, makeRng(seed));
  const live = new Live(P, trainer.agent, makeRng(seed ^ 0x9e3779b9));
  const state = { paused: false, speed: 0.6, budget: 8, lrLog: -3 };
  trainer.agent.lr = Math.pow(10, state.lrLog);
  live.smooth = reduceMotion ? 1 : 0.3;      // reduced motion: no arrow easing

  /* ---------- small DOM helpers ---------- */
  function el(tag, cls, html) { const e = document.createElement(tag); if (cls) e.className = cls; if (html != null) e.innerHTML = html; return e; }
  function fmtInt(n) { return Math.round(n).toString().replace(/\B(?=(\d{3})+(?!\d))/g, ','); }
  function makeSlider(parent, o) {
    const wrap = el('div', 'ctl');
    const id = 'sl-' + o.id;
    wrap.innerHTML = '<div class="ctl-row"><label for="' + id + '">' + (o.swatch ? '<i class="sw" style="background:' + o.swatch + '"></i>' : '') + o.label +
      '</label><output id="' + id + '-out" for="' + id + '"></output></div>' +
      '<input type="range" id="' + id + '" min="' + o.min + '" max="' + o.max + '" step="' + o.step + '"' + (o.swatch ? ' style="--c:' + o.swatch + '"' : '') + '>';
    parent.appendChild(wrap);
    const input = wrap.querySelector('input'), out = wrap.querySelector('output');
    const api = {
      input,
      set(v) { input.value = v; out.textContent = o.fmt(+input.value); },
    };
    input.addEventListener('input', () => { out.textContent = o.fmt(+input.value); o.onInput(+input.value); });
    const release = () => { if (document.activeElement === input) input.blur(); };
    input.addEventListener('pointerup', release); input.addEventListener('change', release);
    api.set(o.value);
    return api;
  }

  /* ---------- sliders ---------- */
  const PHYS = [
    { k: 'g', label: 'Gravity', unit: ' m/s²', min: 1, max: 20, step: 0.01, d: 2 },
    { k: 'mc', label: 'Cart mass', unit: ' kg', min: 0.2, max: 5, step: 0.05, d: 2 },
    { k: 'L1', label: 'Link 1 length', unit: ' m', min: 0.15, max: 0.8, step: 0.01, d: 2, c: C.amber },
    { k: 'L2', label: 'Link 2 length', unit: ' m', min: 0.15, max: 0.8, step: 0.01, d: 2, c: C.teal },
    { k: 'L3', label: 'Link 3 length', unit: ' m', min: 0.15, max: 0.8, step: 0.01, d: 2, c: C.mag },
    { k: 'm1', label: 'Link 1 mass', unit: ' kg', min: 0.05, max: 1, step: 0.01, d: 2, c: C.amber },
    { k: 'm2', label: 'Link 2 mass', unit: ' kg', min: 0.05, max: 1, step: 0.01, d: 2, c: C.teal },
    { k: 'm3', label: 'Link 3 mass', unit: ' kg', min: 0.05, max: 1, step: 0.01, d: 2, c: C.mag },
    { k: 'cf', label: 'Cart friction', unit: ' N·s/m', min: 0, max: 2, step: 0.01, d: 2 },
    { k: 'jd', label: 'Joint damping', unit: ' N·m·s/rad', min: 0, max: 0.05, step: 0.001, d: 3 },
    { k: 'fmax', label: 'Force limit', unit: ' N', min: 5, max: 80, step: 1, d: 0 }
  ];
  const physHost = $('phys-sliders');
  const sliders = {};
  const physHint = $('phys-hint');
  function physicsChanged() {
    physHint.hidden = !trainer.physDiverged();
  }
  for (const s of PHYS) {
    sliders[s.k] = makeSlider(physHost, {
      id: s.k, label: s.label, min: s.min, max: s.max, step: s.step, value: P[s.k], swatch: s.c,
      fmt: (v) => v.toFixed(s.d) + s.unit,
      onInput: (v) => { P[s.k] = v; physicsChanged(); }
    });
  }
  const speedSlider = makeSlider($('speed-host'), {
    id: 'speed', label: 'Simulation speed', min: 0.1, max: 2, step: 0.05, value: state.speed,
    fmt: (v) => v.toFixed(2) + '×', onInput: (v) => { state.speed = v; }
  });
  $('btn-defaults').addEventListener('click', () => {
    for (const s of PHYS) { P[s.k] = DEFAULTS[s.k]; sliders[s.k].set(DEFAULTS[s.k]); }
    state.speed = 0.6; speedSlider.set(0.6);
    physicsChanged();
  });
  makeSlider($('rl-sliders'), {
    id: 'budget', label: 'Compute budget per frame', min: 1, max: 30, step: 1, value: state.budget,
    fmt: (v) => v + ' ms', onInput: (v) => { state.budget = v; }
  });
  makeSlider($('rl-sliders'), {
    id: 'lr', label: 'Learning rate', min: -4, max: -2.5, step: 0.01, value: state.lrLog,
    fmt: (v) => Math.pow(10, v).toExponential(1).replace('e-', '×10⁻').replace('e+', '×10'),
    onInput: (v) => { state.lrLog = v; trainer.agent.lr = Math.pow(10, v); }
  });

  /* ---------- readout ---------- */
  const RO = [
    ['x', 'Cart x', 'm'], ['xd', 'Cart ẋ', 'm/s'], ['u', 'Control u', 'N'],
    ['t1', 'θ1', '°', C.amber], ['t2', 'θ2', '°', C.teal], ['t3', 'θ3', '°', C.mag],
    ['w1', 'ω1', '°/s', C.amber], ['w2', 'ω2', '°/s', C.teal], ['w3', 'ω3', '°/s', C.mag],
    ['st', 'Upright streak', 's'], ['best', 'Best streak', 's']
  ];
  const roHost = $('readout'); const roEls = {};
  for (const [k, label, unit, col] of RO) {
    const d = el('div', 'ro', '<div class="k">' + (col ? '<i class="sw" style="background:' + col + '"></i>' : '') + label + '</div><div class="v"><span id="ro-' + k + '">–</span><small>' + unit + '</small></div>');
    roHost.appendChild(d); roEls[k] = d.querySelector('span');
  }
  const wrapDeg = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return (a - Math.PI) * 180 / Math.PI; };
  const sgn = (v, d) => (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(d);
  function updateReadout() {
    const s = live.plant.s, r = 180 / Math.PI;
    roEls.x.textContent = sgn(s[0], 2); roEls.xd.textContent = sgn(s[4], 2); roEls.u.textContent = sgn(live.plant.uApplied, 1);
    roEls.t1.textContent = sgn(wrapDeg(s[1]), 1); roEls.t2.textContent = sgn(wrapDeg(s[2]), 1); roEls.t3.textContent = sgn(wrapDeg(s[3]), 1);
    roEls.w1.textContent = sgn(s[5] * r, 0); roEls.w2.textContent = sgn(s[6] * r, 0); roEls.w3.textContent = sgn(s[7] * r, 0);
    roEls.st.textContent = live.streak.toFixed(1); roEls.best.textContent = live.best.toFixed(1);
  }

  /* ---------- stats ---------- */
  const STATS = [['phase', 'Phase', true], ['eps', 'ε (exploration)'], ['steps', 'Env steps'], ['eps_n', 'Episodes'], ['upd', 'Updates'],
    ['replay', 'Replay size'], ['ret', 'Last return'], ['avg', 'Avg episode length'], ['best', 'Best episode length'], ['loss', 'Loss']];
  const stHost = $('stats'); const stEls = {};
  for (const [k, label, wide] of STATS) {
    const d = el('div', 'stat' + (wide ? ' wide' : ''), '<span class="k">' + label + '</span><span class="v" id="st-' + k + '">–</span>');
    stHost.appendChild(d); stEls[k] = d.querySelector('.v');
  }
  let lastAnn = '';
  const btnTrain = $('btn-train'), btnPauseTrain = $('btn-pausetrain');
  function phaseText() {
    const t = trainer; let p;
    if (t.phase === 'idle') return 'Idle. Press Train.';
    if (t.phase === 'demos') p = 'Collecting demonstrations ' + t.demoEps + '/' + t.N_DEMO_EPS;
    else if (t.phase === 'pretrain') p = 'Pretraining ' + fmtInt(t.preDone) + '/' + fmtInt(t.PRE_UPDATES);
    else p = 'Reinforcement learning';
    return t.running ? p : 'Paused · ' + p;
  }
  function updateStats() {
    const t = trainer, ag = t.agent;
    stEls.phase.textContent = phaseText();
    stEls.eps.textContent = t.epsilon.toFixed(3);
    stEls.steps.textContent = fmtInt(t.envSteps);
    stEls.eps_n.textContent = fmtInt(t.episodes);
    stEls.upd.textContent = fmtInt(ag.updates);
    stEls.replay.textContent = fmtInt(ag.replay.size) + (ag.demo.size ? ' + ' + fmtInt(ag.demo.size) + ' demo' : '');
    stEls.ret.textContent = t.lastReturn === t.lastReturn ? t.lastReturn.toFixed(1) : '–';
    stEls.avg.textContent = t.lenWin.length ? t.avgLen.toFixed(0) + ' steps' : '–';
    stEls.best.textContent = t.bestLen ? t.bestLen + ' steps' : '–';
    stEls.loss.textContent = t.lossEma === t.lossEma ? t.lossEma.toFixed(4) : '–';
    btnTrain.textContent = t.phase === 'idle' ? 'Train' : 'Resume';
    btnTrain.disabled = t.running; btnPauseTrain.disabled = !t.running;
    $('status-phase').textContent = t.running ? (t.phase === 'rl' ? 'Learning' : (t.phase === 'demos' ? 'Collecting demos' : 'Pretraining')) : (t.phase === 'idle' ? 'Untrained' : 'Training paused');
    $('status-dot').className = 'dot' + (t.running ? ' on' : '');
    $('demo-warn').hidden = !t.demoWarn;
    physHint.hidden = !t.physDiverged();
    const ann = t.phase + (t.running ? '+' : '-');           // screen-reader announcement on phase changes only
    if (ann !== lastAnn) { lastAnn = ann; $('announce').textContent = 'Agent: ' + $('status-phase').textContent; }
  }

  /* ---------- controls ---------- */
  const demoToggle = $('tg-demos'), agentToggle = $('tg-agent');
  demoToggle.addEventListener('change', () => { trainer.useDemos = demoToggle.checked; });
  agentToggle.addEventListener('change', () => { live.agentOn = agentToggle.checked; });
  trainer.useDemos = demoToggle.checked; live.agentOn = agentToggle.checked;
  btnTrain.addEventListener('click', () => { trainer.useDemos = demoToggle.checked; trainer.start(); physicsChanged(); updateStats(); });
  btnPauseTrain.addEventListener('click', () => { trainer.pause(); updateStats(); });
  $('btn-resetenv').addEventListener('click', () => { trainer.resetEnv(); live.reset(); });
  $('btn-resetagent').addEventListener('click', () => { trainer.resetAgent(); physicsChanged(); updateStats(); });
  const btnPause = $('btn-pause');
  function setPaused(p) { state.paused = p; btnPause.textContent = p ? 'Play (Space)' : 'Pause (Space)'; btnPause.setAttribute('aria-pressed', p ? 'true' : 'false'); }
  btnPause.addEventListener('click', () => setPaused(!state.paused));
  $('btn-kick').addEventListener('click', () => live.kick());
  $('btn-reset').addEventListener('click', () => live.reset());

  /* ---------- canvases ---------- */
  function mk(id) { const cv = $(id); return { cv, ctx: cv.getContext('2d'), w: 0, h: 0 }; }
  const scene = mk('scene');
  const charts = { ang: mk('chart-angles'), rew: mk('chart-reward'), loss: mk('chart-loss') };
  function fit(o) {
    const r = o.cv.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2.5);
    const w = Math.max(0, Math.round(r.width)), h = Math.max(0, Math.round(r.height));
    o.cv.width = Math.max(1, Math.round(w * dpr)); o.cv.height = Math.max(1, Math.round(h * dpr));
    o.ctx.setTransform(dpr, 0, 0, dpr, 0, 0); o.w = w; o.h = h;
  }
  function fitAll() { fit(scene); for (const k in charts) fit(charts[k]); }
  if (window.ResizeObserver) {
    const ro = new ResizeObserver(() => fitAll());
    ro.observe(scene.cv); for (const k in charts) ro.observe(charts[k].cv);
  }
  window.addEventListener('resize', fitAll);
  fitAll();

  /* ---------- scene drawing ---------- */
  const view = { cx: 0, pivotY: 0, ppm: 100 };
  function arrow(g, x0, y0, x1, y1, color, lw) {
    const dx = x1 - x0, dy = y1 - y0, len = Math.hypot(dx, dy); if (len < 2) return;
    const ux = dx / len, uy = dy / len, hs = Math.min(9, len * 0.6);
    g.strokeStyle = color; g.fillStyle = color; g.lineWidth = lw; g.lineCap = 'round';
    g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1 - ux * hs * 0.6, y1 - uy * hs * 0.6); g.stroke();
    g.beginPath(); g.moveTo(x1, y1); g.lineTo(x1 - ux * hs - uy * hs * 0.5, y1 - uy * hs + ux * hs * 0.5);
    g.lineTo(x1 - ux * hs + uy * hs * 0.5, y1 - uy * hs - ux * hs * 0.5); g.closePath(); g.fill();
  }
  function rrect(g, x, y, w, h, r) {
    g.beginPath(); g.moveTo(x + r, y); g.lineTo(x + w - r, y); g.quadraticCurveTo(x + w, y, x + w, y + r);
    g.lineTo(x + w, y + h - r); g.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    g.lineTo(x + r, y + h); g.quadraticCurveTo(x, y + h, x, y + h - r); g.lineTo(x, y + r); g.quadraticCurveTo(x, y, x + r, y); g.closePath();
  }
  function drawScene() {
    const g = scene.ctx, w = scene.w, h = scene.h; if (w < 20 || h < 20) return;
    g.clearRect(0, 0, w, h);
    const s = live.plant.s, L = [P.L1, P.L2, P.L3];
    const trackY = h - 50;
    const ppm = Math.min(w / (2 * (TRACK + 0.7)), (trackY - 30) / (L[0] + L[1] + L[2] + 0.3));
    const cx = w / 2, pivotY = trackY - 0.18 * ppm;
    view.cx = cx; view.pivotY = pivotY; view.ppm = ppm;
    const X = (wx) => cx + wx * ppm, Y = (wy) => pivotY - wy * ppm;

    // faint measurement grid (every 1 m vertically, 0.5 m horizontally)
    g.strokeStyle = C.grid; g.lineWidth = 1; g.beginPath();
    for (let i = -5; i <= 5; i++) { const x = Math.round(X(i)) + 0.5; g.moveTo(x, 8); g.lineTo(x, trackY); }
    for (let yy = trackY - 0.5 * ppm; yy > 14; yy -= 0.5 * ppm) { const y = Math.round(yy) + 0.5; g.moveTo(0, y); g.lineTo(w, y); }
    g.stroke();

    // upright reference line through the pivot
    g.save(); g.setLineDash([2, 6]); g.strokeStyle = '#2c3d4f'; g.beginPath(); g.moveTo(X(s[0]), pivotY); g.lineTo(X(s[0]), Math.max(10, pivotY - (L[0] + L[1] + L[2] + 0.1) * ppm)); g.stroke(); g.restore();

    // rail + ruler
    const railL = X(-TRACK - 0.62), railR = X(TRACK + 0.62);
    g.fillStyle = '#1c2835'; rrect(g, railL, trackY, railR - railL, 6, 3); g.fill();
    g.strokeStyle = '#4a6076'; g.fillStyle = C.muted; g.font = '11px ' + FONT; g.textAlign = 'center'; g.lineWidth = 1;
    for (let i = -24; i <= 24; i++) {
      const x = Math.round(X(i * 0.1)) + 0.5, major = i % 10 === 0, mid = i % 5 === 0;
      g.beginPath(); g.moveTo(x, trackY + 8); g.lineTo(x, trackY + 8 + (major ? 11 : (mid ? 8 : 4))); g.stroke();
      if (major) g.fillText((i / 10 > 0 ? '+' : (i === 0 ? '' : '−')) + Math.abs(i / 10).toFixed(0), x, trackY + 34);
    }
    g.textAlign = 'right'; g.fillText('m', railR, trackY + 34);

    // end-stop bumpers (inner face where the cart edge touches at x = ±2.4)
    const touchL = s[0] <= -TRACK + 1e-6, touchR = s[0] >= TRACK - 1e-6;
    for (const sd of [-1, 1]) {
      const face = X(sd * (TRACK + 0.2)), bw = 0.12 * ppm, bh = 0.2 * ppm;
      const x0 = sd < 0 ? face - bw : face;
      g.fillStyle = '#26364a'; rrect(g, x0, trackY - bh + 6, bw, bh, 3); g.fill();
      const hit = sd < 0 ? touchL : touchR;
      g.fillStyle = hit ? C.amber : '#4f7a86';
      const pw = Math.max(3, bw * 0.38);
      rrect(g, sd < 0 ? face - pw : face, trackY - bh + 9, pw, bh - 6, 2); g.fill();
    }

    // cart
    const cartW = 0.4 * ppm, cartH = 0.12 * ppm, cxp = X(s[0]);
    const grad = g.createLinearGradient(0, pivotY, 0, pivotY + cartH);
    grad.addColorStop(0, '#34495e'); grad.addColorStop(1, '#22313f');
    g.fillStyle = grad; g.strokeStyle = '#5b738a'; g.lineWidth = 1.2;
    rrect(g, cxp - cartW / 2, pivotY, cartW, cartH, 4); g.fill(); g.stroke();
    g.fillStyle = '#16202a'; g.strokeStyle = '#5b738a';
    for (const sd of [-1, 1]) { g.beginPath(); g.arc(cxp + sd * 0.12 * ppm, trackY - 0.035 * ppm, 0.035 * ppm, 0, 6.2832); g.fill(); g.stroke(); }

    // links
    const pts = [[s[0], 0]]; let px = s[0], py = 0;
    for (let i = 0; i < 3; i++) { px += L[i] * Math.sin(s[1 + i]); py += L[i] * Math.cos(s[1 + i]); pts.push([px, py]); }

    // fading trail of the 3rd tip
    if (!reduceMotion) {
      const tr = live.trail, n = tr.length / 2;
      g.lineCap = 'round'; g.lineWidth = 2;
      for (let k = 1; k < n; k++) {
        g.strokeStyle = 'rgba(230,111,184,' + (0.55 * k / n).toFixed(3) + ')';
        g.beginPath(); g.moveTo(X(tr[2 * k - 2]), Y(tr[2 * k - 1])); g.lineTo(X(tr[2 * k]), Y(tr[2 * k + 1])); g.stroke();
      }
    }
    const masses = [P.m1, P.m2, P.m3];
    for (let i = 0; i < 3; i++) {
      const a = pts[i], b = pts[i + 1], lw = Math.min(14, 4 + 10 * masses[i]);
      g.lineCap = 'round';
      g.strokeStyle = C.bg; g.lineWidth = lw + 4; g.beginPath(); g.moveTo(X(a[0]), Y(a[1])); g.lineTo(X(b[0]), Y(b[1])); g.stroke();
      g.strokeStyle = LINK_COLORS[i]; g.lineWidth = lw; g.beginPath(); g.moveTo(X(a[0]), Y(a[1])); g.lineTo(X(b[0]), Y(b[1])); g.stroke();
    }
    for (let i = 0; i < 4; i++) {
      g.fillStyle = C.bg; g.strokeStyle = i === 0 ? '#8aa0b6' : LINK_COLORS[i - 1]; g.lineWidth = 2;
      g.beginPath(); g.arc(X(pts[i][0]), Y(pts[i][1]), i === 3 ? 4 : 4.5, 0, 6.2832); g.fill(); g.stroke();
    }

    // control-force arrow + label
    const u = live.uS, ay = pivotY + cartH / 2;
    const alen = clampN(u / P.fmax, -1, 1) * 1.0 * ppm, dirn = u >= 0 ? 1 : -1;
    const sx = cxp + dirn * cartW / 2;
    if (Math.abs(alen) > 2) arrow(g, sx, ay, sx + alen, ay, C.ice, 3);
    g.fillStyle = C.ice; g.font = '500 12px ' + FONT; g.textAlign = dirn > 0 ? 'left' : 'right';
    g.fillText('u = ' + sgn(u, 1) + ' N', sx + alen + dirn * 8, ay + 4);

    // drag rubber band + force arrow
    const d = live.drag;
    if (d.active) {
      const gx = X(d.px), gy = d.kind === 'cart' ? pivotY + cartH / 2 : Y(d.py), tx = X(d.tx), ty = Y(d.ty);
      g.save(); g.setLineDash([6, 5]); g.strokeStyle = '#e8eef6'; g.globalAlpha = 0.85; g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(gx, gy); g.lineTo(tx, ty); g.stroke(); g.restore();
      g.strokeStyle = '#e8eef6'; g.lineWidth = 1.5; g.beginPath(); g.arc(tx, ty, 6, 0, 6.2832); g.stroke();
      g.fillStyle = '#e8eef6'; g.beginPath(); g.arc(gx, gy, 3, 0, 6.2832); g.fill();
      const fm = Math.hypot(d.fx, d.fy), cap = d.kind === 'cart' ? 30 : 10;
      if (fm > 0.2) {
        const k = Math.min(1, fm / cap) * 0.7 * ppm;
        arrow(g, gx, gy, gx + d.fx / fm * k, gy - d.fy / fm * k, C.warm, 3);
        g.fillStyle = C.warm; g.font = '500 12px ' + FONT; g.textAlign = 'left';
        g.fillText('F = ' + fm.toFixed(1) + ' N', gx + d.fx / fm * k + 8, gy - d.fy / fm * k - 6);
      }
    }

    // HUD
    g.textAlign = 'left'; g.font = '500 12px ' + FONT;
    let msg, col = C.muted;
    if (state.paused) { msg = 'Paused'; col = C.ice; }
    else if (live.failed) { msg = 'Fell. Restarting…'; col = C.amber; }
    else if (live.agentOn) msg = 'Greedy agent, live';
    else msg = 'Agent off: passive pendulum';
    g.fillStyle = col; g.fillText(msg, 14, 22);
    g.fillStyle = C.muted; g.font = '12px ' + FONT; g.fillText('t = ' + live.simTime.toFixed(1) + ' s', 14, 40);
    g.textAlign = 'right'; g.fillText('track ±' + TRACK.toFixed(1) + ' m', w - 14, 22);
  }
  function clampN(v, a, b) { return v < a ? a : (v > b ? b : v); }

  /* ---------- charts ---------- */
  function drawChart(o, spec) {
    const g = o.ctx, w = o.w, h = o.h; if (w < 40 || h < 40) return;
    g.clearRect(0, 0, w, h);
    const padL = 46, padR = 12, padT = 28, padB = 22, iw = w - padL - padR, ih = h - padT - padB;
    g.font = '500 12px ' + FONT; g.textAlign = 'left'; g.fillStyle = C.text; g.fillText(spec.title, 14, 18);
    // legend
    g.font = '11px ' + FONT; g.textAlign = 'right'; let lx = w - 12;
    for (let i = spec.legend.length - 1; i >= 0; i--) {
      const l = spec.legend[i], tw = g.measureText(l.label).width;
      g.fillStyle = C.muted; g.fillText(l.label, lx, 17);
      g.fillStyle = l.color; g.fillRect(lx - tw - 14, 12, 9, 3);
      lx -= tw + 24;
    }
    const y2 = (v) => padT + ih * (1 - (v - spec.yMin) / (spec.yMax - spec.yMin));
    g.lineWidth = 1; g.textAlign = 'right'; g.font = '11px ' + FONT;
    for (let i = 0; i <= 4; i++) {
      const v = spec.yMax - (spec.yMax - spec.yMin) * i / 4, y = Math.round(padT + ih * i / 4) + 0.5;
      g.strokeStyle = C.grid; g.beginPath(); g.moveTo(padL, y); g.lineTo(w - padR, y); g.stroke();
      g.fillStyle = C.muted; g.fillText(spec.yFmt(v), padL - 7, y + 4);
    }
    if (spec.hlines) for (const hl of spec.hlines) {
      if (hl.y < spec.yMin || hl.y > spec.yMax) continue;
      g.save(); g.setLineDash(hl.dash || []); g.strokeStyle = hl.color; g.globalAlpha = hl.alpha || 1; g.beginPath(); g.moveTo(padL, y2(hl.y)); g.lineTo(w - padR, y2(hl.y)); g.stroke(); g.restore();
    }
    g.fillStyle = C.muted; g.textAlign = 'left'; g.fillText(spec.xLabels[0], padL, h - 6); g.textAlign = 'right'; g.fillText(spec.xLabels[1], w - padR, h - 6);
    if (!spec.series.length || !spec.series[0].n) {
      g.fillStyle = C.muted; g.textAlign = 'center'; g.font = '12px ' + FONT; g.fillText(spec.empty, padL + iw / 2, padT + ih / 2 + 4); return;
    }
    g.save(); g.beginPath(); g.rect(padL, padT - 2, iw, ih + 4); g.clip();
    for (const sr of spec.series) {
      const n = sr.n, tot = Math.max(2, spec.xCount || n);
      g.strokeStyle = sr.color; g.lineWidth = sr.width || 1.5; g.globalAlpha = sr.alpha || 1; g.lineJoin = 'round'; g.beginPath();
      for (let k = 0; k < n; k++) {
        const x = padL + iw * (tot - n + k) / (tot - 1), v = clampN(sr.ys[k], spec.yMin, spec.yMax), y = y2(v);
        if (k === 0) g.moveTo(x, y); else g.lineTo(x, y);
      }
      g.stroke(); g.globalAlpha = 1;
    }
    g.restore();
  }
  const angBuf = [new Float32Array(500), new Float32Array(500), new Float32Array(500)];
  // round axis bounds so that four equal divisions land on tidy tick values
  function niceBounds(lo, hi) {
    let span = Math.max(hi - lo, 1e-6), mag = Math.pow(10, Math.floor(Math.log10(span / 4)));
    for (let guard = 0; guard < 40; guard++) {
      for (const m of [1, 2, 2.5, 5, 10]) {
        const step = m * mag, a = Math.floor(lo / step) * step;
        if (a + 4 * step >= hi - 1e-9) return [a, a + 4 * step];
      }
      mag *= 10;
    }
    return [lo, hi];
  }
  function niceRange(m, steps) { for (const s of steps) if (m * 1.1 <= s) return s; return steps[steps.length - 1]; }
  function drawCharts() {
    // angle errors, last 10 s of simulated time
    const n = live.hN, len = live.HLEN; let m = 0;
    for (let k = 0; k < 3; k++) for (let i = 0; i < n; i++) {
      const v = live.hist[k][(live.hHead - n + i + len * 2) % len]; angBuf[k][i] = v; const a = Math.abs(v); if (a > m) m = a;
    }
    const yr = niceRange(Math.min(m, 40), [2, 5, 10, 20, 40]);
    drawChart(charts.ang, {
      title: 'Angle errors θ1–θ3', yMin: -yr, yMax: yr, yFmt: (v) => v.toFixed(yr < 5 ? 1 : 0) + '°', xLabels: ['−10 s', 'now'], xCount: 500,
      legend: [{ label: 'θ1', color: C.amber }, { label: 'θ2', color: C.teal }, { label: 'θ3', color: C.mag }],
      hlines: [{ y: 0, color: '#3a4d60' }, { y: 34.4, color: C.amber, dash: [4, 4], alpha: 0.5 }, { y: -34.4, color: C.amber, dash: [4, 4], alpha: 0.5 }],
      series: [0, 1, 2].map((k) => ({ ys: angBuf[k], n, color: LINK_COLORS[k], width: 1.6 })), empty: ''
    });
    // reward per episode
    const R = trainer.returns, nn = Math.min(R.length, 300), off = R.length - nn;
    const ys = new Float32Array(nn), av = new Float32Array(nn);
    let lo = 0, hi = 1;
    for (let i = 0; i < nn; i++) {
      ys[i] = R[off + i];
      let sum = 0, c = 0; for (let j = Math.max(0, off + i - 19); j <= off + i; j++) { sum += R[j]; c++; } av[i] = sum / c;
      if (i === 0) { lo = hi = ys[0]; } lo = Math.min(lo, ys[i]); hi = Math.max(hi, ys[i]);
    }
    const nb = niceBounds(Math.min(lo, 0), Math.max(hi, 1));
    drawChart(charts.rew, {
      title: 'Reward per episode', yMin: nb[0], yMax: nb[1], yFmt: (v) => v.toFixed(0), xLabels: [nn ? 'ep ' + (off + 1) : '', nn ? 'ep ' + R.length : ''], xCount: nn,
      legend: [{ label: 'return', color: '#5a7088' }, { label: '20-episode avg', color: C.teal }], hlines: [{ y: 0, color: '#3a4d60' }],
      series: nn ? [{ ys, n: nn, color: '#5a7088', width: 1, alpha: 0.9 }, { ys: av, n: nn, color: C.teal, width: 2 }] : [], empty: 'No episodes yet. Press Train.'
    });
    // DQN loss (smoothed)
    const lh = trainer.lossHist; let lm = 0; for (let i = 0; i < lh.length; i++) if (lh[i] > lm) lm = lh[i];
    drawChart(charts.loss, {
      title: 'DQN loss (smoothed)', yMin: 0, yMax: lm > 0 ? lm * 1.1 : 1, yFmt: (v) => (v >= 1 ? v.toFixed(1) : v.toFixed(3)), xLabels: ['', 'updates →'], xCount: lh.length,
      legend: [{ label: 'loss', color: C.amber }], series: lh.length ? [{ ys: lh, n: lh.length, color: C.amber, width: 1.8 }] : [], empty: 'No updates yet.'
    });
  }

  /* ---------- pointer interaction ---------- */
  function toWorld(e) {
    const r = scene.cv.getBoundingClientRect();
    return { x: (e.clientX - r.left - view.cx) / view.ppm, y: (view.pivotY - (e.clientY - r.top)) / view.ppm };
  }
  function hitTest(wp, touch) {
    const s = live.plant.s, L = [P.L1, P.L2, P.L3];
    // the cart body sits just below the pivot: grabbing there always means the cart
    if (Math.abs(wp.x - s[0]) < 0.26 && wp.y < 0.0 && wp.y > -0.26) return { kind: 'cart', link: 0, s: 0 };
    let best = null, bd = (touch ? 36 : 22) / view.ppm, px = s[0], py = 0;
    for (let i = 0; i < 3; i++) {
      const ex = px + L[i] * Math.sin(s[1 + i]), ey = py + L[i] * Math.cos(s[1 + i]);
      const t = clampN(((wp.x - px) * (ex - px) + (wp.y - py) * (ey - py)) / (L[i] * L[i]), 0, 1);
      const dist = Math.hypot(wp.x - (px + t * (ex - px)), wp.y - (py + t * (ey - py)));
      if (dist < bd) { bd = dist; best = { kind: 'link', link: i, s: t * L[i] }; }
      px = ex; py = ey;
    }
    return best;
  }
  scene.cv.addEventListener('pointerdown', (e) => {
    const wp = toWorld(e), hit = hitTest(wp, e.pointerType === 'touch'); if (!hit) return;
    e.preventDefault();
    try { scene.cv.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
    Object.assign(live.drag, { active: true, kind: hit.kind, link: hit.link, s: hit.s, tx: wp.x, ty: wp.y, fx: 0, fy: 0 });
    scene.cv.style.cursor = 'grabbing';
  });
  scene.cv.addEventListener('pointermove', (e) => {
    const wp = toWorld(e);
    if (live.drag.active) { live.drag.tx = wp.x; live.drag.ty = wp.y; }
    else scene.cv.style.cursor = hitTest(wp, false) ? 'grab' : 'default';
  });
  const endDrag = () => { live.drag.active = false; scene.cv.style.cursor = 'default'; };
  scene.cv.addEventListener('pointerup', endDrag);
  scene.cv.addEventListener('pointercancel', endDrag);
  scene.cv.addEventListener('lostpointercapture', endDrag);

  /* ---------- keyboard ---------- */
  const keys = { l: false, r: false };
  const setPush = () => { live.push = (keys.r ? 1 : 0) - (keys.l ? 1 : 0); };
  function typing(t) { return t && (t.tagName === 'INPUT' && t.type !== 'checkbox' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT'); }
  window.addEventListener('keydown', (e) => {
    if (typing(e.target) || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.code === 'ArrowLeft' || e.code === 'KeyA') { keys.l = true; setPush(); e.preventDefault(); }
    else if (e.code === 'ArrowRight' || e.code === 'KeyD') { keys.r = true; setPush(); e.preventDefault(); }
    else if (e.code === 'Space') { e.preventDefault(); if (!e.repeat) setPaused(!state.paused); }
    else if (e.code === 'KeyR') { if (!e.repeat) live.reset(); }
  });
  window.addEventListener('keyup', (e) => {
    if (e.code === 'ArrowLeft' || e.code === 'KeyA') { keys.l = false; setPush(); }
    else if (e.code === 'ArrowRight' || e.code === 'KeyD') { keys.r = false; setPush(); }
    else if (e.code === 'Space' && !typing(e.target)) e.preventDefault();
  });
  window.addEventListener('blur', () => { keys.l = keys.r = false; setPush(); });

  /* ---------- main loop ---------- */
  let last = 0, tChart = 0, tRead = 0, tStat = 0;
  function frame(now) {
    const dtReal = last ? Math.min(0.05, (now - last) / 1000) : 0.016; last = now;
    if (!state.paused) {
      live.advance(dtReal * state.speed);
      if (live.failed && !live.drag.active) {
        live.failReal += dtReal;
        if (live.failReal > 1.4) live.reset();       // auto-restart ~1.4 s after the fall
      }
    }
    trainer.run(state.budget);                        // time-sliced background training
    drawScene();
    if (now - tRead > 90) { tRead = now; updateReadout(); }
    if (now - tChart > 90) { tChart = now; drawCharts(); }
    if (now - tStat > 200) { tStat = now; updateStats(); }
    requestAnimationFrame(frame);
  }
  // Hidden tabs stop requestAnimationFrame, which also pauses training; resume cleanly on return.
  document.addEventListener('visibilitychange', () => { last = 0; });
  updateStats(); updateReadout();
  requestAnimationFrame(frame);
  window.__lab = { trainer, live, P, state };   // handy for debugging in the console
})();
