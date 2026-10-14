/* =====================================================================
   CORE (DOM-free): physics, environment, expert, DQN, trainer, live sim
   ===================================================================== */
const CTRL_DT = 0.02;          // 50 Hz zero-order-hold control period
const SUB_DT = 0.005;          // RK4 sub-step
const SUBSTEPS = 4;            // 4 x 5 ms = 20 ms
const TRACK = 2.4;             // cart travel limit (m)
const FAIL_ANGLE = 0.6;        // rad
const MAX_EP_STEPS = 500;
const N_STATE = 8, N_ACT = 13, HID = 64;

// 13 force levels as fractions of Fmax, denser near zero: sign(k)*(k/6)^2
const ACT_FRAC = [];
for (let k = -6; k <= 6; k++) ACT_FRAC.push(Math.sign(k) * k * k / 36);

const DEFAULTS = {
  g: 9.81, mc: 1.0,
  L1: 0.40, L2: 0.40, L3: 0.40,
  m1: 0.20, m2: 0.20, m3: 0.20,
  cf: 0.10,      // cart viscous friction  (N·s/m)
  jd: 0.002,     // joint viscous damping  (N·m·s/rad)
  fmax: 40       // actuator force limit   (N)
};

function makeRng(seed) {
  let a = seed >>> 0;
  const r = () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  r.gauss = () => { let u = 0; while (u === 0) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); };
  r.int = (n) => Math.floor(r() * n);
  return r;
}

function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* Observation: x, xdot, th1..th3, w1..w3, scaled to O(1) and clipped. */
const OBS_SCALE = [1.5, 0.8, 0.12, 0.12, 0.12, 0.8, 0.8, 0.8];   // x, xd, th1-3, w1-3
function encode(s, out) {
  const S = OBS_SCALE;
  out[0] = clamp(s[0] / S[0], -4, 4);
  out[1] = clamp(s[4] / S[1], -4, 4);
  out[2] = clamp(s[1] / S[2], -4, 4);
  out[3] = clamp(s[2] / S[3], -4, 4);
  out[4] = clamp(s[3] / S[4], -4, 4);
  out[5] = clamp(s[5] / S[5], -4, 4);
  out[6] = clamp(s[6] / S[6], -4, 4);
  out[7] = clamp(s[7] / S[7], -4, 4);
}

function quantize(frac) {
  let best = 0, bd = 1e9;
  for (let i = 0; i < N_ACT; i++) { const d = Math.abs(ACT_FRAC[i] - frac); if (d < bd) { bd = d; best = i; } }
  return best;
}

/* =====================================================================
   PLANT: cart + 3 hinged uniform rods  (Lagrangian mechanics)

   Generalised coordinates  q = [x, th1, th2, th3]   (th_i absolute, 0 = up)
   State                    s = [q, qdot]            (8 numbers)

   Centre of mass of link i (a_j = L_j for j < i, a_i = L_i/2):
       cx_i = x + sum_j a_j sin th_j        cy_i = sum_j a_j cos th_j
   COM Jacobian  J_i = d(cx,cy)/dq :
       dcx/dx = 1,  dcx/dth_j =  a_j cos th_j,   dcy/dth_j = -a_j sin th_j   (j <= i)

   Kinetic energy  T = 1/2 Mc xdot^2 + sum_i [ 1/2 m_i |J_i qdot|^2 + 1/2 I_i th_i_dot^2 ],
                      I_i = m_i L_i^2 / 12   (uniform rod about its COM)
   Potential       V = sum_i m_i g cy_i

   Lagrange's equations   d/dt(dT/dqdot) - dT/dq + dV/dq = Q   give
       M(q) qddot + h(q,qdot) + G(q) = Q
   with
       M = diag(Mc,0,0,0) + sum_i m_i J_i^T J_i + diag(0, I1, I2, I3)
       h = sum_i m_i J_i^T a0_i        (Coriolis + centripetal; a0_i = Jdot_i qdot,
                                        a0x = -sum a_j sin th_j w_j^2, a0y = -sum a_j cos th_j w_j^2)
       G = sum_i m_i g (dcy_i/dq)      (gravity;  G_thj = -sum_i m_i g a_j sin th_j)
   Generalised forces
       Q_x  = u + ext_x - cf * xdot                              (actuator, push, friction)
       Q_th = ext_th - jd * [ (w_j - w_{j-1}) - (w_{j+1} - w_j) ]  (viscous joint damping,
                                                                   w_0 = 0 is the cart-link joint)
   Each step solves the SPD 4x4 system  M qddot = Q - h - G  by Cholesky.
   ===================================================================== */
class Plant {
  constructor(P) {
    this.P = P;
    this.s = new Float64Array(8);
    this.M = new Float64Array(16); this.h = new Float64Array(4); this.G = new Float64Array(4);
    this.Lc = new Float64Array(16); this.y = new Float64Array(4);
    this.rhs = new Float64Array(4); this.acc = new Float64Array(4);
    this.Q = new Float64Array(4); this.zeroExt = new Float64Array(4);
    this.k1 = new Float64Array(8); this.k2 = new Float64Array(8);
    this.k3 = new Float64Array(8); this.k4 = new Float64Array(8);
    this.tmp = new Float64Array(8); this.ex = new Float64Array(4); this.z = new Float64Array(4);
    this.jx = new Float64Array(4); this.jy = new Float64Array(4);
    this.sn = new Float64Array(3); this.cs = new Float64Array(3);
    this.Lv = new Float64Array(3); this.mv = new Float64Array(3);
    this.hitStop = false;
    this.uApplied = 0;
  }

  // Fill M, h, G for state s
  terms(s) {
    const P = this.P, M = this.M, h = this.h, G = this.G, jx = this.jx, jy = this.jy;
    const sn = this.sn, cs = this.cs, Lv = this.Lv, mv = this.mv, g = P.g;
    Lv[0] = P.L1; Lv[1] = P.L2; Lv[2] = P.L3; mv[0] = P.m1; mv[1] = P.m2; mv[2] = P.m3;
    for (let j = 0; j < 3; j++) { sn[j] = Math.sin(s[1 + j]); cs[j] = Math.cos(s[1 + j]); }
    M.fill(0); h.fill(0); G.fill(0);
    M[0] = P.mc;
    for (let i = 0; i < 3; i++) {
      jx[0] = 1; jy[0] = 0;
      let ax = 0, ay = 0;
      for (let j = 0; j < 3; j++) {
        const a = j < i ? Lv[j] : (j === i ? 0.5 * Lv[i] : 0);
        const w = s[5 + j];
        jx[j + 1] = a * cs[j]; jy[j + 1] = -a * sn[j];
        ax -= a * sn[j] * w * w; ay -= a * cs[j] * w * w;
      }
      const mi = mv[i];
      for (let r = 0; r < 4; r++) {
        h[r] += mi * (jx[r] * ax + jy[r] * ay);
        G[r] += mi * g * jy[r];
        for (let c = 0; c < 4; c++) M[r * 4 + c] += mi * (jx[r] * jx[c] + jy[r] * jy[c]);
      }
      M[(i + 1) * 5] += mi * Lv[i] * Lv[i] / 12;
    }
  }

  // Cholesky solve of the SPD 4x4 system A x = b
  solve4(A, b, x) {
    const l = this.Lc, y = this.y;
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j <= i; j++) {
        let sum = A[i * 4 + j];
        for (let k = 0; k < j; k++) sum -= l[i * 4 + k] * l[j * 4 + k];
        l[i * 4 + j] = i === j ? Math.sqrt(sum) : sum / l[j * 4 + j];
      }
    }
    for (let i = 0; i < 4; i++) {
      let sum = b[i];
      for (let k = 0; k < i; k++) sum -= l[i * 4 + k] * y[k];
      y[i] = sum / l[i * 4 + i];
    }
    for (let i = 3; i >= 0; i--) {
      let sum = y[i];
      for (let k = i + 1; k < 4; k++) sum -= l[k * 4 + i] * x[k];
      x[i] = sum / l[i * 4 + i];
    }
  }

  // qddot = M^-1 (Q - h - G - damping).  Q = generalised forces (4).
  accel(s, Q, out) {
    const P = this.P, rhs = this.rhs;
    this.terms(s);
    rhs[0] = Q[0] - this.h[0] - this.G[0] - P.cf * s[4];
    const w0 = s[5], w1 = s[6], w2 = s[7], jd = P.jd;
    const r0 = w0, r1 = w1 - w0, r2 = w2 - w1;     // relative joint rates
    rhs[1] = Q[1] - this.h[1] - this.G[1] - jd * (r0 - r1);
    rhs[2] = Q[2] - this.h[2] - this.G[2] - jd * (r1 - r2);
    rhs[3] = Q[3] - this.h[3] - this.G[3] - jd * r2;
    this.solve4(this.M, rhs, out);
  }

  deriv(s, Q, d) {
    this.accel(s, Q, this.acc);
    d[0] = s[4]; d[1] = s[5]; d[2] = s[6]; d[3] = s[7];
    d[4] = this.acc[0]; d[5] = this.acc[1]; d[6] = this.acc[2]; d[7] = this.acc[3];
  }

  rk4(Q, h) {
    const s = this.s, k1 = this.k1, k2 = this.k2, k3 = this.k3, k4 = this.k4, t = this.tmp;
    this.deriv(s, Q, k1);
    for (let i = 0; i < 8; i++) t[i] = s[i] + 0.5 * h * k1[i];
    this.deriv(t, Q, k2);
    for (let i = 0; i < 8; i++) t[i] = s[i] + 0.5 * h * k2[i];
    this.deriv(t, Q, k3);
    for (let i = 0; i < 8; i++) t[i] = s[i] + h * k3[i];
    this.deriv(t, Q, k4);
    for (let i = 0; i < 8; i++) s[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }

  // One 5 ms RK4 sub-step with force u held (zero-order hold) and extra generalised forces ext.
  substep(u, ext) {
    const P = this.P, Q = this.Q, s = this.s;
    u = clamp(u, -P.fmax, P.fmax);
    this.uApplied = u;
    Q[0] = u + ext[0]; Q[1] = ext[1]; Q[2] = ext[2]; Q[3] = ext[3];
    this.rk4(Q, SUB_DT);
    // Inelastic end-stops: an impulse along x that zeroes the cart velocity, distributed
    // through the mass matrix:  M dqdot = lambda e_x,  dxdot = -xdot.
    if ((s[0] >= TRACK && s[4] > 0) || (s[0] <= -TRACK && s[4] < 0)) {
      this.terms(s);
      this.ex[0] = 1; this.ex[1] = 0; this.ex[2] = 0; this.ex[3] = 0;
      this.solve4(this.M, this.ex, this.z);
      const lam = -s[4] / this.z[0];
      for (let i = 0; i < 4; i++) s[4 + i] += lam * this.z[i];
      s[4] = 0;
    }
    if (s[0] >= TRACK) { s[0] = TRACK; this.hitStop = true; }
    else if (s[0] <= -TRACK) { s[0] = -TRACK; this.hitStop = true; }
  }

  // One 20 ms control period.
  step(u) { for (let k = 0; k < SUBSTEPS; k++) this.substep(u, this.zeroExt); }

  reset(rng, spread) {
    const s = this.s; s.fill(0);
    const u = (a) => (rng() * 2 - 1) * a * spread;
    s[0] = u(0.1);
    s[1] = u(0.04); s[2] = u(0.04); s[3] = u(0.04);
    s[4] = u(0.05); s[5] = u(0.05); s[6] = u(0.05); s[7] = u(0.05);
    this.hitStop = false;
  }
}

/* =====================================================================
   ENVIRONMENT (headless)
   ===================================================================== */
class Env {
  constructor(P, rng) {
    this.P = P; this.rng = rng;
    this.plant = new Plant(P);
    this.obs = new Float32Array(8);
    this.reset(1);
  }
  reset(spread) {
    this.plant.reset(this.rng, spread == null ? 1 : spread);
    this.steps = 0; this.ret = 0; this.r = 0; this.done = false; this.trunc = false;
    encode(this.plant.s, this.obs);
  }
  step(a) {
    const P = this.P, pl = this.plant;
    pl.hitStop = false;
    const u = ACT_FRAC[a] * P.fmax;
    pl.step(u);
    this.steps++;
    const s = pl.s;
    const fail = Math.abs(s[1]) > FAIL_ANGLE || Math.abs(s[2]) > FAIL_ANGLE ||
                 Math.abs(s[3]) > FAIL_ANGLE || pl.hitStop || !(Math.abs(s[1]) < 1e3);
    if (fail) { this.r = -10; this.done = true; }
    else {
      const ang = s[1] * s[1] + s[2] * s[2] + s[3] * s[3];
      const om = s[5] * s[5] + s[6] * s[6] + s[7] * s[7];
      const xn = s[0] / TRACK, un = u / P.fmax;
      // +1 alive bonus - angle error - cart offset - control effort - small angular-rate term
      this.r = Math.max(-3, 1 - 2.0 * ang - 0.3 * xn * xn - 0.05 * un * un - 0.01 * om);
      this.done = false;
    }
    this.trunc = !this.done && this.steps >= MAX_EP_STEPS;
    this.ret += this.r;
    encode(s, this.obs);
  }
}

/* =====================================================================
   EXPERT (internal only): LQR-seeded PID, designed from the live physics
   ===================================================================== */
function matMul(A, B, n) {
  const C = new Float64Array(n * n);
  for (let i = 0; i < n; i++) for (let k = 0; k < n; k++) {
    const a = A[i * n + k]; if (a === 0) continue;
    for (let j = 0; j < n; j++) C[i * n + j] += a * B[k * n + j];
  }
  return C;
}
function expmFlat(A, n) {
  let nrm = 0;
  for (let i = 0; i < n; i++) { let r = 0; for (let j = 0; j < n; j++) r += Math.abs(A[i * n + j]); nrm = Math.max(nrm, r); }
  const sq = Math.max(0, Math.ceil(Math.log2(nrm || 1)) + 1), sc = Math.pow(2, -sq);
  const As = A.map(v => v * sc);
  let E = new Float64Array(n * n), T = new Float64Array(n * n);
  for (let i = 0; i < n; i++) { E[i * n + i] = 1; T[i * n + i] = 1; }
  for (let k = 1; k <= 18; k++) {
    T = matMul(T, As, n); for (let i = 0; i < n * n; i++) { T[i] /= k; E[i] += T[i]; }
  }
  for (let i = 0; i < sq; i++) E = matMul(E, E, n);
  return E;
}

function invFlat(A, n) {   // Gauss-Jordan with partial pivoting
  const M = Float64Array.from(A), I = new Float64Array(n * n);
  for (let i = 0; i < n; i++) I[i * n + i] = 1;
  for (let c = 0; c < n; c++) {
    let p = c, best = Math.abs(M[c * n + c]);
    for (let r = c + 1; r < n; r++) { const v = Math.abs(M[r * n + c]); if (v > best) { best = v; p = r; } }
    if (p !== c) for (let j = 0; j < n; j++) {
      let t = M[c * n + j]; M[c * n + j] = M[p * n + j]; M[p * n + j] = t;
      t = I[c * n + j]; I[c * n + j] = I[p * n + j]; I[p * n + j] = t;
    }
    const d = 1 / M[c * n + c];
    for (let j = 0; j < n; j++) { M[c * n + j] *= d; I[c * n + j] *= d; }
    for (let r = 0; r < n; r++) if (r !== c) {
      const f = M[r * n + c]; if (f === 0) continue;
      for (let j = 0; j < n; j++) { M[r * n + j] -= f * M[c * n + j]; I[r * n + j] -= f * I[c * n + j]; }
    }
  }
  return I;
}

// Linearise at upright, discretise with ZOH at the control period, solve the DARE by iteration.
function designLQR(P, qdIn, Rin) {
  const pl = new Plant(P), n = 8, eps = 1e-6;
  const A = new Float64Array(64), B = new Float64Array(8);
  const s = new Float64Array(8), Q = new Float64Array(4), a1 = new Float64Array(4), a2 = new Float64Array(4);
  for (let i = 0; i < 4; i++) A[i * 8 + 4 + i] = 1;
  for (let j = 0; j < 8; j++) {
    s.fill(0); Q.fill(0);
    s[j] = eps; pl.accel(s, Q, a1); s[j] = -eps; pl.accel(s, Q, a2);
    for (let i = 0; i < 4; i++) A[(4 + i) * 8 + j] = (a1[i] - a2[i]) / (2 * eps);
  }
  s.fill(0); Q.fill(0);
  Q[0] = eps; pl.accel(s, Q, a1); Q[0] = -eps; pl.accel(s, Q, a2);
  for (let i = 0; i < 4; i++) B[4 + i] = (a1[i] - a2[i]) / (2 * eps);
  const Aug = new Float64Array(81);
  for (let i = 0; i < 8; i++) { for (let j = 0; j < 8; j++) Aug[i * 9 + j] = A[i * 8 + j] * CTRL_DT; Aug[i * 9 + 8] = B[i] * CTRL_DT; }
  const E = expmFlat(Aug, 9);
  const Ad = new Float64Array(64), Bd = new Float64Array(8);
  for (let i = 0; i < 8; i++) { for (let j = 0; j < 8; j++) Ad[i * 8 + j] = E[i * 9 + j]; Bd[i] = E[i * 9 + 8]; }
  // state order [x, th1, th2, th3, xd, w1, w2, w3]
  const qd = qdIn || [5, 20, 20, 20, 1, 1, 1, 1], R = Rin || 1;
  // Discrete algebraic Riccati equation by the structure-preserving doubling algorithm (robust for
  // badly conditioned, unstable plants):  A_{k+1}=A(I+GH)^-1 A, G_{k+1}=G+A(I+GH)^-1 G A^T,
  // H_{k+1}=H+A^T H (I+GH)^-1 A,   G = B R^-1 B^T,  H0 = Q;  H -> X.
  const n8 = 8;
  let Ak = Float64Array.from(Ad), Gk = new Float64Array(64), Hk = new Float64Array(64);
  for (let i = 0; i < 8; i++) { Hk[i * 9] = qd[i]; for (let j = 0; j < 8; j++) Gk[i * 8 + j] = Bd[i] * Bd[j] / R; }
  const transp = (M) => { const T = new Float64Array(64); for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) T[j * 8 + i] = M[i * 8 + j]; return T; };
  let iters = 0;
  for (let it = 0; it < 80; it++) {
    iters = it;
    const W = matMul(Gk, Hk, n8); for (let i = 0; i < 8; i++) W[i * 9] += 1;
    const Wi = invFlat(W, n8);
    const AW = matMul(Ak, Wi, n8);
    const Gn = matMul(matMul(AW, Gk, n8), transp(Ak), n8);
    const Hn = matMul(matMul(transp(Ak), matMul(Hk, Wi, n8), n8), Ak, n8);
    const An = matMul(AW, Ak, n8);
    let diff = 0, mag = 0;
    for (let i = 0; i < 64; i++) { Gk[i] += Gn[i]; const h = Hk[i] + Hn[i]; diff += Math.abs(Hn[i]); mag += Math.abs(h); Hk[i] = h; }
    Ak = An;
    if (diff < 1e-12 * mag) break;
  }
  const Pm = Hk, K = new Float64Array(8);
  const PB = new Float64Array(8);
  for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) PB[i] += Pm[i * 8 + j] * Bd[j];
  let S = R; for (let i = 0; i < 8; i++) S += Bd[i] * PB[i];
  for (let j = 0; j < 8; j++) { let v = 0; for (let i = 0; i < 8; i++) v += Ad[i * 8 + j] * PB[i]; K[j] = v / S; }
  K.iters = iters;
  K.Ad = Ad; K.Bd = Bd; K.A = A; K.B = B;
  return K;
}

class ExpertPID {
  constructor(P, qd, R) {
    this.P = P;
    this.K = designLQR(P, qd, R);
    this.Ki = 0.05 * this.K[0];   // light integral action on cart position (same sign as the P gain)
    this.I = 0;
  }
  reset() { this.I = 0; }
  // returns commanded force (N); s = plant state [x,th1..3,xd,w1..3]
  act(s) {
    const K = this.K;
    this.I = clamp(this.I + s[0] * CTRL_DT, -0.5, 0.5);
    let u = 0;
    for (let i = 0; i < 8; i++) u -= K[i] * s[i];
    return u - this.Ki * this.I;
  }
}

/* =====================================================================
   Q-NETWORK: 8 -> 64 -> 64 -> 13, ReLU, hand-written backprop + Adam
   ===================================================================== */
class QNet {
  constructor(rng) {
    this.rng = rng;
    const n1 = HID * N_STATE, n2 = HID * HID, n3 = N_ACT * HID;
    this.sizes = [n1, HID, n2, HID, n3, N_ACT];
    this.n = this.sizes.reduce((a, b) => a + b, 0);
    this.p = new Float32Array(this.n); this.g = new Float32Array(this.n);
    this.m = new Float32Array(this.n); this.v = new Float32Array(this.n);
    this.w = this.views(this.p); this.gw = this.views(this.g);
    this.t = 0;
    this.dz1 = new Float32Array(HID); this.dz2 = new Float32Array(HID);
    this.init();
  }
  views(base) {
    let o = 0; const sz = this.sizes;
    const v = (l) => { const r = base.subarray(o, o + l); o += l; return r; };
    return { W1: v(sz[0]), b1: v(sz[1]), W2: v(sz[2]), b2: v(sz[3]), W3: v(sz[4]), b3: v(sz[5]) };
  }
  init() {
    const r = this.rng, w = this.w;
    const fill = (a, std) => { for (let i = 0; i < a.length; i++) a[i] = r.gauss() * std; };
    fill(w.W1, Math.sqrt(2 / N_STATE)); fill(w.W2, Math.sqrt(2 / HID)); fill(w.W3, 0.1 * Math.sqrt(1 / HID));
    w.b1.fill(0); w.b2.fill(0); w.b3.fill(0);
    this.m.fill(0); this.v.fill(0); this.t = 0;
  }
  copyFrom(o) { this.p.set(o.p); }
  soft(o, tau) { const p = this.p, q = o.p; for (let i = 0; i < p.length; i++) p[i] += tau * (q[i] - p[i]); }
  fwd(X, B, h1, h2, out) {
    const { W1, b1, W2, b2, W3, b3 } = this.w;
    for (let n = 0; n < B; n++) {
      const xo = n * N_STATE, o1 = n * HID, oo = n * N_ACT;
      for (let j = 0; j < HID; j++) {
        let s = b1[j]; const wo = j * N_STATE;
        for (let i = 0; i < N_STATE; i++) s += W1[wo + i] * X[xo + i];
        h1[o1 + j] = s > 0 ? s : 0;
      }
      for (let j = 0; j < HID; j++) {
        let s = b2[j]; const wo = j * HID;
        for (let i = 0; i < HID; i++) s += W2[wo + i] * h1[o1 + i];
        h2[o1 + j] = s > 0 ? s : 0;
      }
      for (let k = 0; k < N_ACT; k++) {
        let s = b3[k]; const wo = k * HID;
        for (let i = 0; i < HID; i++) s += W3[wo + i] * h2[o1 + i];
        out[oo + k] = s;
      }
    }
  }
  // Accumulates dLoss/dparams into this.g given dOut = dLoss/dQ (B x 13)
  bwd(X, B, h1, h2, dOut) {
    this.g.fill(0);
    const { W2, W3 } = this.w, G = this.gw, dz1 = this.dz1, dz2 = this.dz2;
    for (let n = 0; n < B; n++) {
      const xo = n * N_STATE, o1 = n * HID, oo = n * N_ACT;
      dz2.fill(0);
      for (let k = 0; k < N_ACT; k++) {
        const d = dOut[oo + k]; if (d === 0) continue;
        G.b3[k] += d;
        const wo = k * HID;
        for (let i = 0; i < HID; i++) { G.W3[wo + i] += d * h2[o1 + i]; dz2[i] += d * W3[wo + i]; }
      }
      for (let i = 0; i < HID; i++) if (!(h2[o1 + i] > 0)) dz2[i] = 0;
      dz1.fill(0);
      for (let j = 0; j < HID; j++) {
        const d = dz2[j]; if (d === 0) continue;
        G.b2[j] += d;
        const wo = j * HID;
        for (let i = 0; i < HID; i++) { G.W2[wo + i] += d * h1[o1 + i]; dz1[i] += d * W2[wo + i]; }
      }
      for (let i = 0; i < HID; i++) if (!(h1[o1 + i] > 0)) dz1[i] = 0;
      for (let j = 0; j < HID; j++) {
        const d = dz1[j]; if (d === 0) continue;
        G.b1[j] += d;
        const wo = j * N_STATE;
        for (let i = 0; i < N_STATE; i++) G.W1[wo + i] += d * X[xo + i];
      }
    }
  }
  adam(lr, clipNorm) {
    const g = this.g, p = this.p, m = this.m, v = this.v;
    let gn = 0; for (let i = 0; i < g.length; i++) gn += g[i] * g[i];
    gn = Math.sqrt(gn);
    const sc = gn > clipNorm ? clipNorm / gn : 1;
    this.t++;
    const b1 = 0.9, b2 = 0.999, c1 = 1 - Math.pow(b1, this.t), c2 = 1 - Math.pow(b2, this.t);
    for (let i = 0; i < g.length; i++) {
      const gi = g[i] * sc;
      m[i] = b1 * m[i] + (1 - b1) * gi;
      v[i] = b2 * v[i] + (1 - b2) * gi * gi;
      p[i] -= lr * (m[i] / c1) / (Math.sqrt(v[i] / c2) + 1e-8);
    }
  }
}

/* =====================================================================
   REPLAY BUFFER
   ===================================================================== */
class Buffer {
  constructor(cap) {
    this.cap = cap; this.size = 0; this.ptr = 0;
    this.S = new Float32Array(cap * N_STATE); this.S2 = new Float32Array(cap * N_STATE);
    this.A = new Uint8Array(cap); this.AE = new Uint8Array(cap);
    this.R = new Float32Array(cap); this.D = new Float32Array(cap);
  }
  clear() { this.size = 0; this.ptr = 0; }
  push(s, a, ae, r, s2, d) {
    const i = this.ptr;
    this.S.set(s, i * N_STATE); this.S2.set(s2, i * N_STATE);
    this.A[i] = a; this.AE[i] = ae; this.R[i] = r; this.D[i] = d;
    this.ptr = (i + 1) % this.cap; if (this.size < this.cap) this.size++;
  }
}

/* =====================================================================
   DOUBLE DQN with Huber loss, soft target updates, replay and a
   large-margin demonstration loss (DQfD)
   ===================================================================== */
class DQN {
  constructor(rng) {
    this.rng = rng;
    this.gamma = 0.98; this.tau = 0.005; this.batch = 64;
    this.rscale = 0.1;     // rewards are scaled for the TD targets only
    this.margin = 0.05;    // large-margin loss margin (scaled units)
    this.lr = 1e-3;
    this.online = new QNet(rng); this.target = new QNet(rng); this.target.copyFrom(this.online);
    this.replay = new Buffer(100000); this.demo = new Buffer(40000);
    const B = this.batch;
    this.X = new Float32Array(B * N_STATE); this.X2 = new Float32Array(B * N_STATE);
    this.A = new Uint8Array(B); this.AE = new Uint8Array(B); this.R = new Float32Array(B);
    this.D = new Float32Array(B); this.isDemo = new Uint8Array(B);
    this.h1a = new Float32Array(B * HID); this.h2a = new Float32Array(B * HID); this.qa = new Float32Array(B * N_ACT);
    this.h1b = new Float32Array(B * HID); this.h2b = new Float32Array(B * HID); this.qb = new Float32Array(B * N_ACT);
    this.h1c = new Float32Array(B * HID); this.h2c = new Float32Array(B * HID); this.qc = new Float32Array(B * N_ACT);
    this.dOut = new Float32Array(B * N_ACT);
    this.s1 = new Float32Array(HID); this.s2 = new Float32Array(HID); this.sq = new Float32Array(N_ACT);
    this.updates = 0;
  }
  reset() {
    this.online.init(); this.target.copyFrom(this.online);
    this.replay.clear(); this.demo.clear(); this.updates = 0;
  }
  greedy(obs) {
    this.online.fwd(obs, 1, this.s1, this.s2, this.sq);
    let b = 0, bv = this.sq[0];
    for (let k = 1; k < N_ACT; k++) if (this.sq[k] > bv) { bv = this.sq[k]; b = k; }
    return b;
  }
  /* One gradient step.  demoOnly: sample the whole batch from demos (pretraining).
     Otherwise nDemoMix demo samples are mixed with replay samples. */
  update(demoOnly, nDemoMix, lam, lrScale) {
    const B = this.batch, rng = this.rng, rb = this.replay, db = this.demo;
    let nd = 0;
    if (demoOnly) nd = B; else if (nDemoMix > 0 && db.size > 0) nd = Math.min(nDemoMix, B);
    if (!demoOnly && rb.size === 0) return NaN;
    for (let n = 0; n < B; n++) {
      const fromDemo = n < nd;
      const buf = fromDemo ? db : rb;
      const i = rng.int(buf.size);
      this.X.set(buf.S.subarray(i * N_STATE, i * N_STATE + N_STATE), n * N_STATE);
      this.X2.set(buf.S2.subarray(i * N_STATE, i * N_STATE + N_STATE), n * N_STATE);
      this.A[n] = buf.A[i]; this.AE[n] = buf.AE[i]; this.R[n] = buf.R[i]; this.D[n] = buf.D[i];
      this.isDemo[n] = fromDemo ? 1 : 0;
    }
    // forward passes
    this.online.fwd(this.X, B, this.h1a, this.h2a, this.qa);
    this.online.fwd(this.X2, B, this.h1b, this.h2b, this.qb);   // Double DQN: action selection
    this.target.fwd(this.X2, B, this.h1c, this.h2c, this.qc);    // Double DQN: action evaluation
    const dOut = this.dOut; dOut.fill(0);
    let loss = 0;
    for (let n = 0; n < B; n++) {
      const o = n * N_ACT;
      let ab = 0, bv = this.qb[o];
      for (let k = 1; k < N_ACT; k++) if (this.qb[o + k] > bv) { bv = this.qb[o + k]; ab = k; }
      const y = this.R[n] * this.rscale + this.gamma * (1 - this.D[n]) * this.qc[o + ab];
      const a = this.A[n];
      const td = this.qa[o + a] - y, at = Math.abs(td);
      loss += at <= 1 ? 0.5 * td * td : at - 0.5;                      // Huber (delta = 1)
      dOut[o + a] += clamp(td, -1, 1) / B;
      if (this.isDemo[n] && lam > 0) {                                 // large-margin loss
        const ae = this.AE[n];
        let am = 0, mv = -1e9;
        for (let k = 0; k < N_ACT; k++) {
          const v = this.qa[o + k] + (k === ae ? 0 : this.margin);
          if (v > mv) { mv = v; am = k; }
        }
        loss += lam * (mv - this.qa[o + ae]);
        if (am !== ae) { dOut[o + am] += lam / B; dOut[o + ae] -= lam / B; }
      }
    }
    this.online.bwd(this.X, B, this.h1a, this.h2a, dOut);
    this.online.adam(this.lr * (lrScale || 1), 10);
    this.target.soft(this.online, this.tau);
    this.updates++;
    return loss / B;
  }
}

/* =====================================================================
   TRAINER: time-sliced headless training (demos -> pretrain -> RL)
   ===================================================================== */
const nowMs = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

class Trainer {
  constructor(P, rng) {
    this.P = P; this.rng = rng;
    this.env = new Env(P, rng);
    this.agent = new DQN(rng);
    this.useDemos = true; this.running = false; this.phase = 'idle';
    this.N_DEMO_EPS = 60; this.PRE_UPDATES = 1500;
    this.epsDecay = 30000; this.eps0 = 1; this.eps1 = 0.02; this.warmup = 500;
    this.s0 = new Float32Array(8);
    this.expert = null; this.demoNew = true; this.physSnap = null; this.demoWarn = false;
    this.clearStats();
  }
  clearStats() {
    this.envSteps = 0; this.rlSteps = 0; this.episodes = 0; this.demoEps = 0; this.demoSteps = 0; this.preDone = 0;
    this.lastReturn = NaN; this.bestLen = 0; this.lenWin = []; this.lossEma = NaN;
    this.returns = []; this.lossHist = []; this.lossAcc = 0; this.lossN = 0; this.lastLen = 0;
    this.demoFails = 0;
  }
  get epsilon() {
    if (this.phase === 'idle') return this.useDemos ? 0.15 : 1;
    const t = Math.min(1, this.rlSteps / this.epsDecay);
    return this.eps0 + (this.eps1 - this.eps0) * t;
  }
  get avgLen() { const w = this.lenWin; if (!w.length) return NaN; let s = 0; for (const v of w) s += v; return s / w.length; }
  start() {
    if (this.phase === 'idle') {
      this.eps0 = this.useDemos ? 0.15 : 1;
      this.physSnap = Object.assign({}, this.P);      // physics the agent starts learning under
      this.demoWarn = false;
      if (this.useDemos && this.agent.demo.size === 0) {
        this.expert = new ExpertPID(this.P);
        this.demoWarn = !this.checkExpert();
        this.phase = 'demos'; this.demoNew = true; this.demoEps = 0;
      } else this.phase = 'rl';
      this.env.reset(1);
    }
    this.running = true;
  }
  // Silent sanity check: can the demonstrator balance the current physics at all?
  checkExpert() {
    const e = new Env(this.P, this.rng); let ok = 0;
    for (let k = 0; k < 3; k++) {
      e.reset(1); this.expert.reset();
      while (!e.done && !e.trunc) e.step(quantize(clamp(this.expert.act(e.plant.s) / this.P.fmax, -1, 1)));
      if (e.trunc) ok++;
    }
    this.expert.reset();
    return ok >= 2;
  }
  // True when the physics sliders differ from what training started under
  physDiverged() {
    if (!this.physSnap) return false;
    for (const k in this.physSnap) if (this.physSnap[k] !== this.P[k]) return true;
    return false;
  }
  pause() { this.running = false; }
  resetEnv() { this.env.reset(1); this.demoNew = true; }
  resetAgent() {
    this.running = false; this.phase = 'idle'; this.agent.reset(); this.clearStats(); this.physSnap = null; this.demoWarn = false; this.env.reset(1); this.demoNew = true;
  }
  run(budgetMs, maxUnits) {
    if (!this.running) return;
    const t0 = nowMs(); let units = 0;
    maxUnits = maxUnits || Infinity;
    while (this.running && nowMs() - t0 < budgetMs && units < maxUnits) {
      units++;
      if (this.phase === 'demos') { for (let i = 0; i < 8 && this.phase === 'demos'; i++) this.demoStep(); }
      else if (this.phase === 'pretrain') {
        // pretraining: large-margin + TD on demos only, learning rate decays 3x -> 0.3x of the slider value
        const f = this.preDone / this.PRE_UPDATES;
        const l = this.agent.update(true, 0, 1.0, 3 * Math.pow(0.1, f)); this.noteLoss(l); this.preDone++;
        if (this.preDone >= this.PRE_UPDATES) {
          this.phase = 'rl'; this.env.reset(1);
          // the loss curve restarts here: pretraining loss includes the margin term, RL loss does not
          this.lossHist = []; this.lossEma = NaN; this.lossAcc = 0; this.lossN = 0;
        }
      } else this.rlChunk();
    }
  }
  demoStep() {
    const env = this.env, P = this.P, rng = this.rng;
    if (this.demoNew) { env.reset(1.2); this.expert.reset(); this.demoNew = false; }
    const uE = this.expert.act(env.plant.s);
    const aE = quantize(clamp(uE / P.fmax, -1, 1));
    let a = quantize(clamp((uE + 0.03 * P.fmax * rng.gauss()) / P.fmax, -1, 1));
    if (rng() < 0.05) a = clamp(a + rng.int(3) - 1, 0, N_ACT - 1);
    this.s0.set(env.obs);
    env.step(a);
    this.agent.demo.push(this.s0, a, aE, env.r, env.obs, env.done ? 1 : 0);
    this.envSteps++; this.demoSteps++;
    if (env.done || env.trunc) {
      this.demoEps++; if (env.done) this.demoFails++;
      this.demoNew = true;
      if (this.demoEps >= this.N_DEMO_EPS) { this.phase = 'pretrain'; this.preDone = 0; }
    }
  }
  noteLoss(l) {
    if (!(l === l)) return;
    this.lossEma = this.lossEma === this.lossEma ? 0.98 * this.lossEma + 0.02 * l : l;
    this.lossAcc += this.lossEma; this.lossN++;
    if (this.lossN >= 5) {
      this.lossHist.push(this.lossAcc / this.lossN); this.lossAcc = 0; this.lossN = 0;
      if (this.lossHist.length > 600) this.lossHist.shift();      // rolling window of the last 600 points
    }
  }
  rlChunk() {
    const env = this.env, ag = this.agent, rng = this.rng;
    for (let k = 0; k < 4; k++) {
      this.s0.set(env.obs);
      let a;
      if (rng() < this.epsilon) {
        a = rng() < 0.5 ? rng.int(N_ACT) : clamp(ag.greedy(this.s0) + rng.int(5) - 2, 0, N_ACT - 1);
      } else a = ag.greedy(this.s0);
      env.step(a);
      ag.replay.push(this.s0, a, a, env.r, env.obs, env.done ? 1 : 0);
      this.envSteps++; this.rlSteps++;
      if (env.done || env.trunc) {
        this.returns.push(env.ret); if (this.returns.length > 3000) this.returns.shift();
        this.lastReturn = env.ret; this.lastLen = env.steps; this.episodes++;
        this.lenWin.push(env.steps); if (this.lenWin.length > 20) this.lenWin.shift();
        if (env.steps > this.bestLen) this.bestLen = env.steps;
        env.reset(1);
      }
    }
    if (ag.replay.size >= this.warmup) {
      // demo margin loss fades over the first 40k RL steps (to 20% of its weight) so RL can improve on the demonstrator
      const lam = 0.5 * Math.max(0.2, 1 - this.rlSteps / 40000);   // fades to a small floor, never fully off
      const l = ag.update(false, this.useDemos ? 16 : 0, lam); this.noteLoss(l);
    }
  }
}

/* =====================================================================
   LIVE SIMULATION: the visible pendulum, always driven by the greedy agent
   ===================================================================== */
class Live {
  constructor(P, agent, rng) {
    this.P = P; this.agent = agent; this.rng = rng;
    this.plant = new Plant(P);
    this.obs = new Float32Array(8); this.ext = new Float64Array(4);
    this.HLEN = 500; this.hist = [new Float32Array(500), new Float32Array(500), new Float32Array(500)];
    this.hN = 0; this.hHead = 0;
    this.drag = { active: false, kind: 'link', link: 0, s: 0, tx: 0, ty: 0, fx: 0, fy: 0, px: 0, py: 0 };
    this.push = 0; this.agentOn = true; this.best = 0; this.smooth = 0.3;
    this.trail = [];
    this.reset();
  }
  reset() {
    this.plant.reset(this.rng, 1); this.plant.hitStop = false;
    this.sub = 0; this.u = 0; this.uS = 0; this.acc = 0;
    this.failed = false; this.failReal = 0; this.streak = 0; this.simTime = 0;
    this.trail.length = 0;
  }
  kick() {
    const s = this.plant.s, r = () => (this.rng() < 0.5 ? -1 : 1) * (0.5 + 0.5 * this.rng());
    s[4] += 0.5 * r(); s[5] += 0.5 * r(); s[6] += 0.7 * r(); s[7] += 0.9 * r();
  }
  advance(dt) {
    this.acc = Math.min(this.acc + dt, 0.2);
    while (this.acc >= SUB_DT) {
      this.acc -= SUB_DT;
      if (this.sub === 0) this.decide();
      this.ext.fill(0);
      if (this.push) this.ext[0] += this.push * 0.5 * this.P.fmax;
      this.applyDrag();
      this.plant.substep(this.u, this.ext);
      if (++this.sub === SUBSTEPS) { this.sub = 0; this.tick(); }
    }
  }
  decide() {
    if (this.failed || !this.agentOn) { this.u = 0; return; }
    encode(this.plant.s, this.obs);
    this.u = ACT_FRAC[this.agent.greedy(this.obs)] * this.P.fmax;
  }
  // Spring-damper on the grabbed point, mapped to generalised forces through its Jacobian:  Q = J^T f
  applyDrag() {
    const d = this.drag; if (!d.active) return;
    const s = this.plant.s, P = this.P, ext = this.ext;
    if (d.kind === 'cart') {
      let fx = 60 * (d.tx - s[0]) - 14 * s[4];
      fx = clamp(fx, -30, 30);
      ext[0] += fx; d.fx = fx; d.fy = 0; d.px = s[0]; d.py = 0;
      return;
    }
    const L = [P.L1, P.L2, P.L3], i = d.link;
    let px = s[0], py = 0, vx = s[4], vy = 0;
    const jx = [1, 0, 0, 0], jy = [0, 0, 0, 0];
    for (let j = 0; j <= i; j++) {
      const a = j < i ? L[j] : d.s, sn = Math.sin(s[1 + j]), cs = Math.cos(s[1 + j]);
      px += a * sn; py += a * cs;
      jx[j + 1] = a * cs; jy[j + 1] = -a * sn;
      vx += a * cs * s[5 + j]; vy -= a * sn * s[5 + j];
    }
    let fx = 35 * (d.tx - px) - 3.5 * vx, fy = 35 * (d.ty - py) - 3.5 * vy;
    const fm = Math.hypot(fx, fy);
    if (fm > 10) { fx *= 10 / fm; fy *= 10 / fm; }
    for (let r = 0; r < 4; r++) ext[r] += jx[r] * fx + jy[r] * fy;
    d.fx = fx; d.fy = fy; d.px = px; d.py = py;
  }
  tick() {
    const s = this.plant.s, pl = this.plant;
    this.simTime += CTRL_DT;
    this.uS += (pl.uApplied - this.uS) * this.smooth;
    const wrap = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };
    const hi = this.hHead;
    for (let k = 0; k < 3; k++) this.hist[k][hi] = wrap(s[1 + k]) * 180 / Math.PI;
    this.hHead = (hi + 1) % this.HLEN; if (this.hN < this.HLEN) this.hN++;
    const fail = Math.abs(wrap(s[1])) > FAIL_ANGLE || Math.abs(wrap(s[2])) > FAIL_ANGLE ||
                 Math.abs(wrap(s[3])) > FAIL_ANGLE || pl.hitStop;
    pl.hitStop = false;
    if (fail && !this.failed) { this.failed = true; this.failReal = 0; }
    const up = !this.failed && Math.abs(s[1]) < 0.25 && Math.abs(s[2]) < 0.25 && Math.abs(s[3]) < 0.25;
    if (up) { this.streak += CTRL_DT; if (this.streak > this.best) this.best = this.streak; } else this.streak = 0;
    // fading trail of the third tip (world coordinates)
    if (this.simTime * 50 % 2 < 1) {
      const P = this.P;
      const x = s[0] + P.L1 * Math.sin(s[1]) + P.L2 * Math.sin(s[2]) + P.L3 * Math.sin(s[3]);
      const y = P.L1 * Math.cos(s[1]) + P.L2 * Math.cos(s[2]) + P.L3 * Math.cos(s[3]);
      this.trail.push(x, y); if (this.trail.length > 160) this.trail.splice(0, 2);
    }
  }
}

const Core = { OBS_SCALE, CTRL_DT, SUB_DT, SUBSTEPS, TRACK, FAIL_ANGLE, MAX_EP_STEPS, ACT_FRAC, DEFAULTS, makeRng, Plant, Env, ExpertPID,
  designLQR, QNet, Buffer, DQN, Trainer, Live, encode, quantize, clamp };
if (typeof module !== 'undefined' && module.exports) module.exports = Core;
