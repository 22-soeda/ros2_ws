// 準静的歩行の試作 (可視化専用) の JS 版。
//
// **実機の歩行エンジン (roboone_walk_core) にも、仕様原本の walk_core にも入っていない。**
// quasistatic.py の QuasiStaticWalker の移植で、設計と式の説明はそちらの docstring にある。
// ロジックを変えるときは両方を揃えること。walkcore.js (3 実装照合の対象) とは別物。
//
//   SHIFT  両足支持のまま、重心を次の支持足の上へ 5 次多項式で移す
//   SWING  重心を止めたまま、反対の足を振り出す
//   STOP   足を揃えたあと、重心を両足の中点へ戻す
//
// 出力は WalkEngineJS._outputs() と同じ形なので、画面側はそのまま描ける。
'use strict';

const QS_K_QUINTIC = 10.0 / Math.sqrt(3.0);

function qsDefaultParams() {
  return { zmp_tol: 0.005, t_swing: 0.8, t_shift_min: 0.3, com_inset: 0.0, swing_height: null };
}

function _qsS(u) {
  u = u < 0.0 ? 0.0 : (u > 1.0 ? 1.0 : u);
  return [
    u * u * u * (10.0 + u * (-15.0 + 6.0 * u)),
    30.0 * u * u * (1.0 + u * (-2.0 + u)),
    60.0 * u * (1.0 + u * (-3.0 + 2.0 * u)),
  ];
}

class QuasiStaticWalkerJS {
  constructor(params, qs) {
    this.p = params;
    this.q = Object.assign(qsDefaultParams(), qs || {});
    this.hSw = (this.q.swing_height == null) ? this.p.swing_height : this.q.swing_height;
    this.march = false;          // MarchWalkEngineJS と同じく、true の間は指令ゼロでも歩き続ける
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
    this.com = [0.0, 0.0];
    this.comv = [0.0, 0.0];
    this.coma = [0.0, 0.0];
    this.phase = 0.0;
    this.tLocal = 0.0;
    this.dur = 0.0;
    this.c0 = [0.0, 0.0];
    this.c1 = [0.0, 0.0];
    this.swingR0 = [0.0, 0.0];
    this.swingZ = 0.0;
    this.pLand = null;
    this.mode = 'walk';
    this.steps = [];
  }

  _omega2() { return this.p.gravity / this.p.z_c; }

  _midpoint() {
    const a = this.foot[1], b = this.foot[-1];
    return [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
  }

  _moving(eps) { return this.march || Math.hypot(this.v[0], this.v[1]) >= eps; }

  _feetAligned() {
    const a = this.foot[1], b = this.foot[-1];
    return Math.abs(a[0] - b[0]) < 1e-9 &&
           Math.abs((a[1] - b[1]) - this.p.foot_spacing) < 1e-9;
  }

  _overSupport() {
    const f = this.foot[this.sup];
    return [f[0], f[1] - this.sup * this.q.com_inset];
  }

  _shapeCmd(vx, vy, dt) {           // walk_core と同じ (飽和・楕円・a_max)
    const p = this.p;
    const cl = (v, lo, hi) => v < lo ? lo : (v > hi ? hi : v);
    vx = cl(vx, -p.v_max[0], p.v_max[0]);
    vy = cl(vy, -p.v_max[1], p.v_max[1]);
    const s = Math.hypot(vx / p.v_max[0], vy / p.v_max[1]);
    if (s > 1.0) { vx /= s; vy /= s; }
    const vin = [vx, vy];
    for (let k = 0; k < 2; k++) {
      this.v[k] += cl(vin[k] - this.v[k], -p.a_max[k] * dt, p.a_max[k] * dt);
    }
  }

  update(vxCmd, vyCmd, dt, estop) {
    this.t += dt;
    this._shapeCmd(vxCmd, vyCmd, dt);
    if (estop) this.state = 'ESTOP';
    if (this.state === 'ESTOP') return this._outputs();
    if (this.state === 'IDLE') this._tickIdle();
    else if (this.state === 'SHIFT' || this.state === 'STOP') this._tickShift(dt);
    else if (this.state === 'SWING') this._tickSwing(dt);
    return this._outputs();
  }

  _tickIdle() {
    this.com = this._midpoint();
    this.comv = [0.0, 0.0];
    this.coma = [0.0, 0.0];
    this.phase = 0.0;
    if (this._moving(this.p.v_start_eps)) {
      const vy = this.v[1];
      if (Math.abs(vy) > 1e-6) this.sup = vy > 0 ? -1 : 1;   // 進行方向側の足から踏み出す
      else this.sup = 1;
      this._startShift(this._overSupport(), 'SHIFT');
    }
  }

  _startShift(target, state) {
    this.state = state;
    this.c0 = this.com.slice();
    this.c1 = [target[0], target[1]];
    const d = Math.hypot(this.c1[0] - this.c0[0], this.c1[1] - this.c0[1]);
    const aLim = this._omega2() * this.q.zmp_tol;
    this.dur = Math.max(this.q.t_shift_min, Math.sqrt(QS_K_QUINTIC * d / aLim));
    this.tLocal = 0.0;
    this.phase = 0.0;
  }

  _tickShift(dt) {
    this.tLocal = Math.min(this.tLocal + dt, this.dur);
    const u = this.tLocal / this.dur;
    const [s, ds, dds] = _qsS(u);
    for (let k = 0; k < 2; k++) {
      const dc = this.c1[k] - this.c0[k];
      this.com[k] = this.c0[k] + dc * s;
      this.comv[k] = dc * ds / this.dur;
      this.coma[k] = dc * dds / (this.dur * this.dur);
    }
    this.phase = u;
    if (this.tLocal < this.dur) return;
    this.com = this.c1.slice();
    this.comv = [0.0, 0.0];
    this.coma = [0.0, 0.0];
    if (this.state === 'STOP') {
      this.state = 'IDLE';
      this.phase = 0.0;
    } else if (!this._moving(this.p.v_stop_eps) && this._feetAligned()) {
      this._startShift(this._midpoint(), 'STOP');
    } else {
      this._startSwing();
    }
  }

  _startSwing() {
    const p = this.p;
    this.state = 'SWING';
    this.stepIdx += 1;
    this.tLocal = 0.0;
    this.dur = this.q.t_swing;
    this.phase = 0.0;
    const swing = -this.sup;
    this.swingR0 = this.foot[swing].slice();
    this.swingZ = 0.0;
    const ps = this.foot[this.sup];
    if (this._moving(p.v_stop_eps)) {
      this.mode = 'walk';
      this.pLand = [ps[0] + this.v[0] * p.t_step,
                    ps[1] + swing * p.foot_spacing + this.v[1] * p.t_step];
    } else {
      this.mode = 'stop';
      this.pLand = [ps[0], ps[1] + swing * p.foot_spacing];
    }
    this.steps.push({
      i: this.stepIdx, t0: this.t, sup: this.sup, mode: this.mode,
      v: this.v.slice(), psup: ps.slice(),
      pnom: this.pLand.slice(), pland: this.pLand.slice(), b: null, clamped: false, t1: null,
    });
  }

  _tickSwing(dt) {
    const p = this.p;
    this.tLocal = Math.min(this.tLocal + dt, this.dur);
    this.phase = this.tLocal / this.dur;
    const tau = this.phase;
    const s = _qsS(tau)[0];
    const swing = -this.sup;
    const f = this.foot[swing];
    f[0] = this.swingR0[0] + s * (this.pLand[0] - this.swingR0[0]);
    f[1] = this.swingR0[1] + s * (this.pLand[1] - this.swingR0[1]);
    if (tau < 0.45) {
      this.swingZ = this.hSw * _qsS(tau / 0.45)[0];
    } else {
      const u = (tau - 0.45) / 0.55;
      const su = _qsS(u)[0];
      const zRef = this.hSw * (1.0 - su) - p.td_overdrive * su;
      this.swingZ = Math.max(zRef, this.swingZ - p.td_speed_max * dt);
    }
    if (this.tLocal < this.dur) return;
    f[0] = this.pLand[0];
    f[1] = this.pLand[1];
    this.swingZ = 0.0;
    this.steps[this.steps.length - 1].t1 = this.t;
    if (this.mode === 'stop' || (!this._moving(p.v_stop_eps) && this._feetAligned())) {
      this.pLand = null;
      this._startShift(this._midpoint(), 'STOP');
    } else {
      this.sup = swing;
      this.pLand = null;
      this._startShift(this._overSupport(), 'SHIFT');
    }
  }

  _outputs() {
    const w = Math.sqrt(this._omega2());
    const swinging = this.state === 'SWING';
    const swing = -this.sup;
    const lf = this.foot[1], rf = this.foot[-1];
    return {
      t: this.t, state: this.state, stepIdx: this.stepIdx, phase: this.phase,
      support: swinging ? this.sup : 0,
      v: this.v.slice(),
      xi: [this.com[0] + this.comv[0] / w, this.com[1] + this.comv[1] / w],
      com: this.com.slice(),
      zmp: [this.com[0] - this.coma[0] / (w * w), this.com[1] - this.coma[1] / (w * w)],
      leftFoot: [lf[0], lf[1], (swinging && swing === 1) ? this.swingZ : 0.0],
      rightFoot: [rf[0], rf[1], (swinging && swing === -1) ? this.swingZ : 0.0],
      pelvis: [this.com[0], this.com[1], this.p.z_c],
      pNom: swinging ? this.pLand.slice() : null,
      pLand: swinging ? this.pLand.slice() : null,
      bNext: null, xiEos: null, clampBox: null,
      locked: swinging,
      stopping: (swinging && this.mode === 'stop') || this.state === 'STOP',
    };
  }
}

// node で直接読み込んだとき (照合用) にだけ公開する
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { QuasiStaticWalkerJS, qsDefaultParams };
}
