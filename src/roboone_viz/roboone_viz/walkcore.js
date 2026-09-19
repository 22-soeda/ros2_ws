// 歩行計画エンジン walk_core の JS 版 (ブラウザシミュレータ用)。
//
// **roboone_walk_ref/walk_core/engine.py の機械移植で、Python 版が仕様の原本。**
// ロジックを変えるときは必ず Python/C++/JS の 3 つを揃え、
// src/roboone_walk_core/tools/compare_walk_engines.py で数値一致を確認すること。
//
// node で直接実行すると照合用の CSV を吐く:
//   node walkcore.js <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005] [key=value ...]
// key=value は既定パラメータの上書き (walk_dump.cpp の kKeys と同じ並び)。
'use strict';

const WALK_STATES = ['IDLE', 'START', 'STEP', 'STOP', 'ESTOP'];

function walkDefaultParams() {
  return {
    z_c: 0.261, gravity: 9.81, t_step: 0.60, foot_spacing: 0.1786,
    swing_height: 0.05, swing_lock_phase: 0.70,
    td_overdrive: 0.004, td_speed_max: 0.20,
    swing_ratio: 1.0, ds_time: 0.0,
    v_max: [0.10, 0.04], a_max: [0.06, 0.03],
    step_clamp_x: 0.04, step_clamp_out: 0.045, step_clamp_in: 0.020,
    start_pushoff_max: 0.15, k_dcm: 1.0, cmd_timeout: 0.5, loop_hz: 200.0,
    v_start_eps: 0.005, v_stop_eps: 0.010, settle_eps: 0.002,
    stop_outside_eps: 0.005,
  };
}

function walkOmega(p) { return Math.sqrt(p.gravity / p.z_c); }
function walkEwt(p) { return Math.exp(walkOmega(p) * p.t_step); }

// 両足支持の定数 [E_d, κ, E, K] (params.py の ds_consts と同じ)。
//   E_d = e^{ωTd},  κ = (E_d - 1)/(ωTd) (Td → 0 で 1),  E = E_d e^{ωTs},  K = κ e^{ωTs}
function walkDsConsts(p) {
  const es = walkEwt(p);
  const wt = walkOmega(p) * p.ds_time;
  const ed = Math.exp(wt);
  const kappa = wt > 0.0 ? (ed - 1.0) / wt : 1.0;
  return [ed, kappa, ed * es, kappa * es];
}

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
    // 両足支持 (ds_time > 0)。STEP の頭と、STOP の頭 (最後の両足支持) で使う
    this.inDs = false;
    this.dsT = 0.0;            // 両足支持の経過時間
    this.dsFrom = [0.0, 0.0];  // ZMP の出発点
    this.dsRate = [0.0, 0.0];  // ZMP の速度 ṗ
    this.startMid = [0.0, 0.0];  // 歩き出しの両足の中点
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

  // ds_time > 0 版。戻り値 [pNom, cNext, cAfter] (engine.py _step_params_ds 参照)。
  // cNext は歩の終わりの ξ の名目のずれ (いまの支持足 p_i から見た c_{i+1})、
  // cAfter はその次の歩の終わりの名目 (p_{i+1} から見た c_{i+2})。
  _stepParamsDs() {
    const p = this.p;
    const lx = this.v[0] * p.t_step;
    const ly = this.v[1] * p.t_step;
    const w = p.foot_spacing;
    const sNext = -this.sup;
    const ps = this.foot[this.sup];
    const pNom = [ps[0] + lx, ps[1] + sNext * w + ly];
    const [, , e, k] = walkDsConsts(p);
    const denom = e * e - 1.0;
    const lFirst = [lx, sNext * w + ly];    // p_i -> p_{i+1}
    const lSecond = [lx, this.sup * w + ly];  // p_{i+1} -> p_{i+2}
    const cNext = [0, 0], cAfter = [0, 0];
    for (let i = 0; i < 2; i++) {
      cNext[i] = k * (e * lFirst[i] + lSecond[i]) / denom;
      cAfter[i] = k * (e * lSecond[i] + lFirst[i]) / denom;
    }
    return [pNom, cNext, cAfter];
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
    const ps = this.foot[this.sup];
    if (this.inDs) {
      // 両足支持の残りを閉形式で進めてから、単脚支持 Ts を丸ごと進める
      const w = walkOmega(p);
      const eRem = Math.exp(w * (p.ds_time - this.dsT));
      const ewt = walkEwt(p);
      const out = [0, 0];
      for (let k = 0; k < 2; k++) {
        const v = this.dsRate[k] / w;
        const relD = v + (this.xi[k] - this.zmp[k] - v) * eRem;   // ξ(Td) - p_i
        out[k] = ps[k] + relD * ewt;
      }
      return out;
    }
    const e = Math.exp(walkOmega(p) * (p.t_step - this.tLocal));
    return [ps[0] + (this.xi[0] - ps[0]) * e, ps[1] + (this.xi[1] - ps[1]) * e];
  }

  _updateLanding() {
    const p = this.p;
    if (p.ds_time > 0.0) {
      // 次の歩の終わりで定常解に戻る着地点。ずれの吸収に要るずらしは e^{ωTd}/κ 倍
      const [pNom, cNext] = this._stepParamsDs();
      const xiEos = this._predictXiEos();
      const [ed, kappa] = walkDsConsts(p);
      const sp = this.foot[this.sup];
      const g = p.k_dcm * ed / kappa;
      const raw = [0, 0];
      for (let k = 0; k < 2; k++) raw[k] = pNom[k] + g * (xiEos[k] - sp[k] - cNext[k]);
      this.pNom = pNom;
      this.bNext = cNext;
      this.xiEos = xiEos;
      this.pLand = this._clampLanding(raw, pNom);
      return;
    }
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
    if (p.ds_time > 0.0) {
      // 最後の歩 (支持足 p_{N-1}) の終わりに ξ が p_{N-1} から
      // d = κ/(2 e^{ωTd}) (p_N - p_{N-1}) にいれば、最後の両足支持で ZMP を中点へ
      // 移したときに ξ も中点で止まる。その d に着くよう p_{N-1} を選ぶ
      const [ed, kappa, e, k] = walkDsConsts(p);
      const d = [0.0, kappa / (2.0 * ed) * this.sup * p.foot_spacing];
      const c = [xiEos[0] - ps[0], xiEos[1] - ps[1]];
      const raw = [0, 0];
      for (let i = 0; i < 2; i++) raw[i] = ps[i] + (e * c[i] - d[i]) / k;
      this.pNom = pNom;
      this.bNext = d;
      this.xiEos = xiEos;
      this.pLand = this._clampLanding(raw, pNom);
      return;
    }
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
    let raw;
    if (p.ds_time > 0.0) {
      // 最後の両足支持で中点 m へ移す ZMP に対し、ξ が m で止まる条件
      // ξ_eos - p_{N-1} = κ/e^{ωTd} (m - p_{N-1}) を p_N について解く
      const [ed, kappa] = walkDsConsts(p);
      const g = 2.0 * ed / kappa;
      raw = [ps[0] + g * (xiEos[0] - ps[0]), ps[1] + g * (xiEos[1] - ps[1])];
    } else {
      raw = [2.0 * xiEos[0] - ps[0], 2.0 * xiEos[1] - ps[1]];
    }
    this.pNom = pNom;
    this.bNext = null;
    this.xiEos = xiEos;
    this.pLand = this._clampLanding(raw, pNom);
  }

  // ds_time > 0 なら、ZMP を dsFrom (省略時はいまの ZMP = 前の支持足) から
  // 新しい支持足へ移す両足支持から始める
  _enterStep(dsFrom) {
    const p = this.p;
    this.state = 'STEP';
    this.stepIdx += 1;
    this.phase = 0.0;
    this.tLocal = 0.0;
    this.locked = false;
    this.inDs = p.ds_time > 0.0;
    if (this.inDs) {
      this._startDs(dsFrom === undefined ? this.zmp : dsFrom, this.foot[this.sup]);
    } else {
      this.zmp = this.foot[this.sup].slice();
    }
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
    const tau = _clampv(this.phase / p.swing_ratio, 0.0, 1.0);
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

  // ZMP を frm から to へ ds_time で直線に移す両足支持を始める
  _startDs(frm, to) {
    const p = this.p;
    this.inDs = true;
    this.dsT = 0.0;
    this.phase = 0.0;
    this.dsFrom = frm.slice();
    this.dsRate = [(to[0] - frm[0]) / p.ds_time, (to[1] - frm[1]) / p.ds_time];
    this.zmp = frm.slice();
    this.xiIni = this.xi.slice();
  }

  // 両足支持の 1 周期。ξ は始点からの閉形式、重心はオイラー積分。
  // 終わり (dsT = Td) は閉形式でちょうどに取る。戻り値: 両足支持が終わったか
  _advanceDs(dt) {
    const p = this.p;
    const w = walkOmega(p);
    this.dsT = Math.min(this.dsT + dt, p.ds_time);
    const e = Math.exp(w * this.dsT);
    for (let k = 0; k < 2; k++) {
      const v = this.dsRate[k] / w;
      this.zmp[k] = this.dsFrom[k] + this.dsRate[k] * this.dsT;
      this.xi[k] = this.zmp[k] + v + (this.xiIni[k] - this.dsFrom[k] - v) * e;
      this.com[k] += w * (this.xi[k] - this.com[k]) * dt;
    }
    return this.dsT >= p.ds_time;
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
    this.startMid = this._midpoint();
    this.zmp = this.foot[-this.sup].slice();               // ZMP は押し出し足
    this.stopping = false;
    this.stopPrep = false;
  }

  _tickStart(dt) {
    const p = this.p;
    this.tLocal += dt;
    this.phase = this.tLocal / p.start_pushoff_max;
    this._advanceDcm(dt);
    let targetY;
    if (p.ds_time > 0.0) {
      // 最初の歩は中点 m から支持足への両足支持で始まる。その歩の終わりで定常解に
      // 乗るには、両足支持の頭で ξ が m から c_1 = (c_2 + K (p_1 - m))/E にいればよい
      const [pNom, cNext] = this._stepParamsDs();
      const [, , e, k] = walkDsConsts(p);
      const m = this.startMid;
      const ps = this.foot[this.sup];
      this.pNom = pNom;
      this.bNext = cNext;
      targetY = m[1] + (cNext[1] + k * (ps[1] - m[1])) / e;
    } else {
      const [pNom, bHere, bNext] = this._stepParams();
      this.pNom = pNom;
      this.bNext = bNext;
      targetY = this.foot[this.sup][1] + bHere[1];
    }
    this.pLand = null;
    this.xiEos = null;
    this.clampBox = null;
    if (this.sup * (this.xi[1] - targetY) >= 0.0) {
      // 交差時刻を閉形式で解き ξ を交差点に置く (離散化誤差の増幅対策)
      const zy = this.zmp[1];
      const y0 = this.xiIni[1];
      if (Math.abs(y0 - zy) > 1e-12 && (targetY - zy) / (y0 - zy) > 0.0) {
        const eStar = (targetY - zy) / (y0 - zy);
        this.xi[0] = this.zmp[0] + (this.xiIni[0] - this.zmp[0]) * eStar;
        this.xi[1] = targetY;
      }
      this._enterStep(p.ds_time > 0.0 ? this.startMid : undefined);
    } else if (this.tLocal > p.start_pushoff_max) {
      this.state = 'STOP';
    } else if (Math.hypot(this.v[0], this.v[1]) < p.v_stop_eps) {
      this.state = 'STOP';
    }
  }

  _tickStep(dt) {
    const p = this.p;
    if (this.inDs) {
      // 両足支持: 足は動かさず ZMP だけ移す。着地点は先に更新しておく
      // (ξ の予測は両足支持の残りを含む)。位相は単脚支持の中で測る
      this.phase = 0.0;
      const done = this._advanceDs(dt);
      if (!this.locked) this._updateLanding();
      if (done) {
        this.inDs = false;
        this.zmp = this.foot[this.sup].slice();
        this.xiIni = this.xi.slice();
        this.tLocal = 0.0;
      }
      return;
    }
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
      if (this.p.ds_time > 0.0) {
        // 最後の両足支持: ZMP を支持足から両足の中点へ移す
        this._startDs(this.foot[this.sup], this._midpoint());
      }
    } else {
      this.sup = swing;
      this._enterStep();
    }
  }

  _tickStop(dt) {
    const p = this.p;
    if (this.inDs) {
      this.phase = 0.0;
      if (this._advanceDs(dt)) this.inDs = false;
      return;
    }
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
    const inDs = this.inDs && (this.state === 'STEP' || this.state === 'STOP');
    const swing = -this.sup;
    return {
      t: this.t, state: this.state, stepIdx: this.stepIdx, phase: this.phase,
      support: (inStep && !inDs) ? this.sup : 0,
      doubleSupport: inDs,
      dsElapsed: inDs ? this.dsT : 0.0,
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
    console.error(
      'usage: node walkcore.js <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005] [key=value ...]');
    process.exit(2);
  }
  const vx = +argv[0], vy = +argv[1];
  // 位置引数は '=' を含まないものだけ数える (walk_dump.cpp と同じ扱い)
  const params = walkDefaultParams();
  const KEYS = ['ds_time', 't_step', 'foot_spacing', 'swing_height', 'k_dcm',
                'a_max_x', 'a_max_y'];
  const pos = [4.5, 8.0, 0.005];
  let npos = 0;
  for (const a of argv.slice(2)) {
    const i = a.indexOf('=');
    if (i < 0) {
      if (npos < 3) pos[npos++] = +a;
      continue;
    }
    const key = a.slice(0, i), val = +a.slice(i + 1);
    if (!KEYS.includes(key)) {
      console.error(`unknown key: ${a}`);
      process.exit(2);
    }
    if (key === 'a_max_x') params.a_max[0] = val;
    else if (key === 'a_max_y') params.a_max[1] = val;
    else params[key] = val;
  }
  const tWalk = pos[0], tEnd = pos[1], dt = pos[2];
  const stateCode = { IDLE: 0, START: 1, STEP: 2, STOP: 3, ESTOP: 4 };
  const e = new WalkEngineJS(params);
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
