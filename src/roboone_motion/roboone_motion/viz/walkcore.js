// 歩行計画エンジン walk_core の JS 版 (ブラウザシミュレータ用)。
//
// **roboone_motion/walk_core/engine.py の機械移植で、Python 版が仕様の原本。**
// ロジックを変えるときは必ず Python/C++/JS の 3 つを揃え、
// src/roboone_walk_core/tools/compare_walk_engines.py で数値一致を確認すること。
//
// node で直接実行すると照合用の CSV を吐く:
//   node walkcore.js <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005]
'use strict';

const WALK_STATES = ['IDLE', 'START', 'STEP', 'STOP', 'ESTOP'];

function walkDefaultParams() {
  return {
    z_c: 0.16, gravity: 9.81, t_step: 0.40, foot_spacing: 0.08,
    swing_height: 0.02, swing_lock_phase: 0.70,
    td_overdrive: 0.004, td_speed_max: 0.10,
    v_max: [0.15, 0.08], a_max: [0.15, 0.08],
    step_clamp_x: 0.04, step_clamp_out: 0.045, step_clamp_in: 0.015,
    start_pushoff_max: 0.15, k_dcm: 1.0, cmd_timeout: 0.5, loop_hz: 200.0,
    v_start_eps: 0.005, v_stop_eps: 0.010, settle_eps: 0.002,
    stop_outside_eps: 0.005,
  };
}

function walkOmega(p) { return Math.sqrt(p.gravity / p.z_c); }
function walkEwt(p) { return Math.exp(walkOmega(p) * p.t_step); }

const _clampv = (v, lo, hi) => v < lo ? lo : (v > hi ? hi : v);
const _quintic = (tau) => {
  tau = _clampv(tau, 0.0, 1.0);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
};

class WalkEngineJS {
  constructor(params) {
    this.p = params || walkDefaultParams();
    this.reset();
  }

  reset() {
    const w2 = this.p.foot_spacing / 2.0;
    this.t = 0.0;
    this.state = 'IDLE';
    this.stepIdx = 0;
    this.v = [0.0, 0.0];
    this.foot = { 1: [0.0, +w2], '-1': [0.0, -w2] };
    this.sup = 1;
    this.xi = [0.0, 0.0];
    this.com = [0.0, 0.0];
    this.zmp = [0.0, 0.0];
    this.phase = 0.0;
    this.tLocal = 0.0;
    this.xiIni = [0.0, 0.0];
    this.swingR0 = [0.0, 0.0];
    this.swingZ = 0.0;
    this.pNom = null;
    this.pLand = null;
    this.bNext = null;
    this.xiEos = null;
    this.clampBox = null;
    this.locked = false;
    this.stopping = false;
    this.stopPrep = false;
    this.steps = [];
  }

  _shapeCmd(vx, vy, dt) {
    const p = this.p;
    vx = _clampv(vx, -p.v_max[0], p.v_max[0]);
    vy = _clampv(vy, -p.v_max[1], p.v_max[1]);
    const s = Math.hypot(vx / p.v_max[0], vy / p.v_max[1]);
    if (s > 1.0) { vx /= s; vy /= s; }
    const vin = [vx, vy];
    for (let k = 0; k < 2; k++) {
      this.v[k] += _clampv(vin[k] - this.v[k], -p.a_max[k] * dt, p.a_max[k] * dt);
    }
  }

  // 戻り値 [pNom, bHere, bNext] (engine.py _step_params 参照)
  _stepParams() {
    const p = this.p;
    const lx = this.v[0] * p.t_step;
    const ly = this.v[1] * p.t_step;
    const w = p.foot_spacing;
    const sNext = -this.sup;
    const ps = this.foot[this.sup];
    const pNom = [ps[0] + lx, ps[1] + sNext * w + ly];
    const ewt = walkEwt(p);
    const denom = ewt * ewt - 1.0;
    const lFirst = [lx, sNext * w + ly];
    const lSecond = [lx, this.sup * w + ly];
    const bHere = [0, 0], bNext = [0, 0];
    for (let k = 0; k < 2; k++) {
      bHere[k] = (lFirst[k] * ewt + lSecond[k]) / denom;
      bNext[k] = (lSecond[k] * ewt + lFirst[k]) / denom;
    }
    return [pNom, bHere, bNext];
  }

  _clampLanding(raw, pNom) {
    const p = this.p;
    const sNext = -this.sup;
    const xmin = pNom[0] - p.step_clamp_x;
    const xmax = pNom[0] + p.step_clamp_x;
    let ymin, ymax;
    if (sNext === 1) {
      ymin = pNom[1] - p.step_clamp_in;
      ymax = pNom[1] + p.step_clamp_out;
    } else {
      ymin = pNom[1] - p.step_clamp_out;
      ymax = pNom[1] + p.step_clamp_in;
    }
    this.clampBox = [xmin, xmax, ymin, ymax];
    return [_clampv(raw[0], xmin, xmax), _clampv(raw[1], ymin, ymax)];
  }

  _predictXiEos() {
    const p = this.p;
    const e = Math.exp(walkOmega(p) * (p.t_step - this.tLocal));
    const ps = this.foot[this.sup];
    return [ps[0] + (this.xi[0] - ps[0]) * e, ps[1] + (this.xi[1] - ps[1]) * e];
  }

  _updateLanding() {
    const p = this.p;
    const [pNom, , b] = this._stepParams();
    const xiEos = this._predictXiEos();
    const raw = [0, 0];
    for (let k = 0; k < 2; k++) {
      raw[k] = pNom[k] + p.k_dcm * (xiEos[k] - (pNom[k] + b[k]));
    }
    this.pNom = pNom;
    this.bNext = b;
    this.xiEos = xiEos;
    this.pLand = this._clampLanding(raw, pNom);
  }

  _updatePrepLanding() {          // 停止準備歩 (式 21)
    const p = this.p;
    const sNext = -this.sup;
    const ps = this.foot[this.sup];
    const pNom = [ps[0], ps[1] + sNext * p.foot_spacing];
    const xiEos = this._predictXiEos();
    const bStop = [0.0, this.sup * (p.foot_spacing / 2.0) / walkEwt(p)];
    const raw = [xiEos[0] - bStop[0], xiEos[1] - bStop[1]];
    this.pNom = pNom;
    this.bNext = bStop;
    this.xiEos = xiEos;
    this.pLand = this._clampLanding(raw, pNom);
  }

  _updateStopLanding() {          // 停止の最後の歩
    const p = this.p;
    const sNext = -this.sup;
    const ps = this.foot[this.sup];
    const pNom = [ps[0], ps[1] + sNext * p.foot_spacing];
    const xiEos = this._predictXiEos();
    const raw = [2.0 * xiEos[0] - ps[0], 2.0 * xiEos[1] - ps[1]];
    this.pNom = pNom;
    this.bNext = null;
    this.xiEos = xiEos;
    this.pLand = this._clampLanding(raw, pNom);
  }

  _enterStep() {
    const p = this.p;
    this.state = 'STEP';
    this.stepIdx += 1;
    this.phase = 0.0;
    this.tLocal = 0.0;
    this.locked = false;
    this.zmp = this.foot[this.sup].slice();
    this.xiIni = this.xi.slice();
    const swing = -this.sup;
    this.swingR0 = this.foot[swing].slice();
    this.swingZ = 0.0;
    const vSmall = Math.hypot(this.v[0], this.v[1]) < p.v_stop_eps;
    let mode;
    if (this.stopPrep) {
      mode = 'stop';
      this.stopping = true;
      this.stopPrep = false;
      this._updateStopLanding();
      this.locked = true;
    } else if (vSmall) {
      mode = 'prep';
      this.stopping = false;
      this.stopPrep = true;
      this._updatePrepLanding();
      this.locked = true;
    } else {
      mode = 'walk';
      this.stopping = false;
      this._updateLanding();
    }
    this.steps.push({
      i: this.stepIdx, t0: this.t, sup: this.sup, mode,
      v: this.v.slice(), psup: this.foot[this.sup].slice(),
      pnom: null, pland: null, b: null, clamped: false, t1: null,
    });
  }

  _finishStepRecord() {
    if (!this.steps.length) return;
    const r = this.steps[this.steps.length - 1];
    r.pnom = this.pNom.slice();
    r.pland = this.pLand.slice();
    r.b = this.bNext ? this.bNext.slice() : null;
    const box = this.clampBox;
    r.clamped = !!box && (
      this.pLand[0] === box[0] || this.pLand[0] === box[1] ||
      this.pLand[1] === box[2] || this.pLand[1] === box[3]);
    r.t1 = this.t;
  }

  _swingPos(dt) {
    const p = this.p;
    const tau = _clampv(this.phase, 0.0, 1.0);
    const s = _quintic(tau);
    const x = this.swingR0[0] + s * (this.pLand[0] - this.swingR0[0]);
    const y = this.swingR0[1] + s * (this.pLand[1] - this.swingR0[1]);
    let z;
    if (tau < 0.45) {
      z = p.swing_height * _quintic(tau / 0.45);
      this.swingZ = z;
    } else {
      const u = (tau - 0.45) / 0.55;
      const zRef = p.swing_height * (1.0 - _quintic(u)) - p.td_overdrive * _quintic(u);
      z = Math.max(zRef, this.swingZ - p.td_speed_max * dt);
      this.swingZ = z;
    }
    return [x, y, z];
  }

  _advanceDcm(dt) {
    const w = walkOmega(this.p);
    const e = Math.exp(w * this.tLocal);
    for (let k = 0; k < 2; k++) {
      this.xi[k] = this.zmp[k] + (this.xiIni[k] - this.zmp[k]) * e;
      this.com[k] += w * (this.xi[k] - this.com[k]) * dt;
    }
  }

  update(vxCmd, vyCmd, dt, estop) {
    this.t += dt;
    this._shapeCmd(vxCmd, vyCmd, dt);
    if (estop) this.state = 'ESTOP';
    if (this.state === 'ESTOP') return this._outputs();
    if (this.state === 'IDLE') this._tickIdle();
    else if (this.state === 'START') this._tickStart(dt);
    else if (this.state === 'STEP') this._tickStep(dt);
    else if (this.state === 'STOP') this._tickStop(dt);
    return this._outputs();
  }

  _tickIdle() {
    const mid = this._midpoint();
    this.xi = mid.slice();
    this.com = mid.slice();
    this.zmp = mid.slice();
    this.pNom = this.pLand = this.bNext = this.xiEos = null;
    this.clampBox = null;
    this.stopping = false;
    if (Math.hypot(this.v[0], this.v[1]) >= this.p.v_start_eps) this._enterStart();
  }

  _enterStart() {
    const vy = this.v[1];
    if (Math.abs(vy) > 1e-6) this.sup = vy > 0 ? -1 : 1;   // 遊脚 = 進行方向側
    else this.sup = 1;
    this.state = 'START';
    this.tLocal = 0.0;
    this.phase = 0.0;
    this.xiIni = this.xi.slice();
    this.zmp = this.foot[-this.sup].slice();               // ZMP は押し出し足
    this.stopping = false;
    this.stopPrep = false;
  }

  _tickStart(dt) {
    const p = this.p;
    this.tLocal += dt;
    this.phase = this.tLocal / p.start_pushoff_max;
    this._advanceDcm(dt);
    const [pNom, bHere, bNext] = this._stepParams();
    this.pNom = pNom;
    this.bNext = bNext;
    this.pLand = null;
    this.xiEos = null;
    this.clampBox = null;
    const targetY = this.foot[this.sup][1] + bHere[1];
    if (this.sup * (this.xi[1] - targetY) >= 0.0) {
      // 交差時刻を閉形式で解き ξ を交差点に置く (離散化誤差の増幅対策)
      const zy = this.zmp[1];
      const y0 = this.xiIni[1];
      if (Math.abs(y0 - zy) > 1e-12 && (targetY - zy) / (y0 - zy) > 0.0) {
        const eStar = (targetY - zy) / (y0 - zy);
        this.xi[0] = this.zmp[0] + (this.xiIni[0] - this.zmp[0]) * eStar;
        this.xi[1] = targetY;
      }
      this._enterStep();
    } else if (this.tLocal > p.start_pushoff_max) {
      this.state = 'STOP';
    } else if (Math.hypot(this.v[0], this.v[1]) < p.v_stop_eps) {
      this.state = 'STOP';
    }
  }

  _tickStep(dt) {
    const p = this.p;
    this.tLocal = Math.min(this.tLocal + dt, p.t_step);
    this.phase = this.tLocal / p.t_step;
    this._advanceDcm(dt);
    if (!this.locked) {
      if (this.phase < p.swing_lock_phase) this._updateLanding();
      else this.locked = true;
    }
    const swing = -this.sup;
    const sw = this._swingPos(dt);
    this.foot[swing][0] = sw[0];
    this.foot[swing][1] = sw[1];
    if (this.tLocal >= p.t_step) this._land(swing);
  }

  _land(swing) {
    this._finishStepRecord();
    this.foot[swing][0] = this.pLand[0];
    this.foot[swing][1] = this.pLand[1];
    this.swingZ = 0.0;
    if (this.stopping) {
      this.state = 'STOP';
      this.pNom = this.pLand = this.bNext = this.xiEos = null;
      this.clampBox = null;
    } else {
      this.sup = swing;
      this._enterStep();
    }
  }

  _tickStop(dt) {
    const p = this.p;
    const [proj, dist] = this._projectBetweenFeet(this.xi);
    if (dist > p.stop_outside_eps) {
      this.sup = this._nearerFoot(this.xi);
      this.v = [0.0, 0.0];
      this._enterStep();
      return;
    }
    this.zmp = proj;
    this.xiIni = this.xi.slice();
    this.tLocal = 0.0;
    this._advanceDcm(dt);
    this.phase = 0.0;
    if (Math.abs(this.xi[0] - this.com[0]) < p.settle_eps &&
        Math.abs(this.xi[1] - this.com[1]) < p.settle_eps) {
      this.state = 'IDLE';
    }
  }

  _midpoint() {
    return [(this.foot[1][0] + this.foot[-1][0]) / 2.0,
            (this.foot[1][1] + this.foot[-1][1]) / 2.0];
  }

  _nearerFoot(pt) {
    const dl = Math.hypot(pt[0] - this.foot[1][0], pt[1] - this.foot[1][1]);
    const dr = Math.hypot(pt[0] - this.foot[-1][0], pt[1] - this.foot[-1][1]);
    return dl <= dr ? 1 : -1;
  }

  _projectBetweenFeet(pt) {
    const a = this.foot[1], b = this.foot[-1];
    const abx = b[0] - a[0], aby = b[1] - a[1];
    const den = abx * abx + aby * aby;
    const u = den === 0 ? 0.0 :
      _clampv(((pt[0] - a[0]) * abx + (pt[1] - a[1]) * aby) / den, 0.0, 1.0);
    const proj = [a[0] + u * abx, a[1] + u * aby];
    return [proj, Math.hypot(pt[0] - proj[0], pt[1] - proj[1])];
  }

  _outputs() {
    const inStep = this.state === 'STEP';
    const swing = -this.sup;
    return {
      t: this.t, state: this.state, stepIdx: this.stepIdx, phase: this.phase,
      support: inStep ? this.sup : 0,
      v: this.v.slice(), xi: this.xi.slice(), com: this.com.slice(),
      zmp: this.zmp.slice(),
      leftFoot: [this.foot[1][0], this.foot[1][1],
                 (inStep && swing === 1) ? this.swingZ : 0.0],
      rightFoot: [this.foot[-1][0], this.foot[-1][1],
                  (inStep && swing === -1) ? this.swingZ : 0.0],
      pelvis: [this.com[0], this.com[1], this.p.z_c],
      pNom: this.pNom ? this.pNom.slice() : null,
      pLand: this.pLand ? this.pLand.slice() : null,
      bNext: this.bNext ? this.bNext.slice() : null,
      xiEos: this.xiEos ? this.xiEos.slice() : null,
      clampBox: this.clampBox ? this.clampBox.slice() : null,
      locked: this.locked,
      stopping: this.stopping || this.stopPrep,
    };
  }
}

// ---------------------------------------------------------------------------
// node 直接実行: 照合用 CSV (walk_dump.cpp と同じプロファイル・列)
// ---------------------------------------------------------------------------
if (typeof process !== 'undefined' && typeof require !== 'undefined' &&
    require.main === module) {
  const argv = process.argv.slice(2);
  if (argv.length < 2) {
    console.error('usage: node walkcore.js <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005]');
    process.exit(2);
  }
  const vx = +argv[0], vy = +argv[1];
  const tWalk = argv.length > 2 ? +argv[2] : 4.5;
  const tEnd = argv.length > 3 ? +argv[3] : 8.0;
  const dt = argv.length > 4 ? +argv[4] : 0.005;
  const stateCode = { IDLE: 0, START: 1, STEP: 2, STOP: 3, ESTOP: 4 };
  const e = new WalkEngineJS();
  const lines = ['t,st,ph,sup,vx,vy,xix,xiy,comx,comy,zx,zy,lfx,lfy,lfz,rfx,rfy,rfz'];
  const n = Math.round(tEnd / dt);
  const g = (x) => {
    // %.17g 相当 (往復可能な最短表現)
    const s = String(x);
    return s;
  };
  for (let i = 0; i < n; i++) {
    const t = i * dt;
    const on = t >= 0.5 && t < tWalk;
    const o = e.update(on ? vx : 0.0, on ? vy : 0.0, dt, false);
    lines.push([o.t, stateCode[o.state], o.phase, o.support, o.v[0], o.v[1],
                o.xi[0], o.xi[1], o.com[0], o.com[1], o.zmp[0], o.zmp[1],
                o.leftFoot[0], o.leftFoot[1], o.leftFoot[2],
                o.rightFoot[0], o.rightFoot[1], o.rightFoot[2]].map(g).join(','));
  }
  console.log(lines.join('\n'));
}
