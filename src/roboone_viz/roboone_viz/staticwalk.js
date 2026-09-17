// 静歩行の計画エンジン (static_walk) の JS 版。ブラウザのライブ操縦シミュレータ用。
//
// **仕様の原本は roboone_walk_ref/static_walk/engine.py。** 設計と式の説明はそちらの
// docstring にある。C++ 版 (roboone_walk_core/static_walk_engine.hpp) と合わせて 3 実装で、
// ロジックを変えるときは揃えて roboone_walk_core/tools/compare_walk_engines.py で照合する。
//
//   SHIFT  両足支持のまま、重心を次の支持足の上へ 5 次多項式で移す
//   SWING  重心を止めたまま、反対の足を振り出す
//   STOP   足を揃えたあと、重心を両足の中点へ戻す
//
// 出力は WalkEngineJS._outputs() と同じ形なので、画面側はそのまま描ける。
// 足踏み (指令ゼロでも歩き続ける) は可視化だけの包みで、template.html の
// MarchStaticWalkEngineJS が _moving() を差し替える。
'use strict';

const SW_K_QUINTIC = 10.0 / Math.sqrt(3.0);
const SW_SWING_RISE = 0.45;

// static_walk/params.py の既定値 (= config/static_gait.yaml) と同じ
function staticDefaultParams() {
  return {
    z_c: 0.261, gravity: 9.81, foot_spacing: 0.140,
    sole_length: 0.118, sole_width: 0.074,
    zmp_tol: 0.010, t_shift_min: 0.3, com_offset_y: 0.0,
    t_swing: 0.6, swing_height: 0.025, td_overdrive: 0.004, td_speed_max: 0.20,
    stride_time: 0.60, v_max: [0.10, 0.04], a_max: [0.06, 0.03],
    v_start_eps: 0.005, v_stop_eps: 0.010, loop_hz: 200.0,
  };
}

function _swClamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

function _swS(u) {
  u = _swClamp(u, 0.0, 1.0);
  return [
    u * u * u * (10.0 + u * (-15.0 + 6.0 * u)),
    30.0 * u * u * (1.0 + u * (-2.0 + u)),
    60.0 * u * (1.0 + u * (-3.0 + 2.0 * u)),
  ];
}

class StaticWalkEngineJS {
  constructor(params) {
    this.p = Object.assign(staticDefaultParams(), params || {});
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

  _omega() { return Math.sqrt(this.p.gravity / this.p.z_c); }

  _shiftTime(dist) {
    const w = this._omega();
    const aLim = w * w * this.p.zmp_tol;
    return Math.max(this.p.t_shift_min, Math.sqrt(SW_K_QUINTIC * dist / aLim));
  }

  _swingHeightAt(tau, zPrev, dt) {
    const p = this.p;
    if (tau < SW_SWING_RISE) return p.swing_height * _swS(tau / SW_SWING_RISE)[0];
    const su = _swS((tau - SW_SWING_RISE) / (1.0 - SW_SWING_RISE))[0];
    const zRef = p.swing_height * (1.0 - su) - p.td_overdrive * su;
    return Math.max(zRef, zPrev - p.td_speed_max * dt);
  }

  _midpoint() {
    const a = this.foot[1], b = this.foot[-1];
    return [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
  }

  _moving(eps) { return Math.hypot(this.v[0], this.v[1]) >= eps; }

  _feetAligned() {
    const a = this.foot[1], b = this.foot[-1];
    return Math.abs(a[0] - b[0]) < 1e-9 &&
           Math.abs((a[1] - b[1]) - this.p.foot_spacing) < 1e-9;
  }

  _overSupport() {
    const f = this.foot[this.sup];
    return [f[0], f[1] + this.sup * this.p.com_offset_y];
  }

  _shapeCmd(vx, vy, dt) {           // walk_core と同じ (飽和・楕円・a_max)
    const p = this.p;
    vx = _swClamp(vx, -p.v_max[0], p.v_max[0]);
    vy = _swClamp(vy, -p.v_max[1], p.v_max[1]);
    const s = Math.hypot(vx / p.v_max[0], vy / p.v_max[1]);
    if (s > 1.0) { vx /= s; vy /= s; }
    const vin = [vx, vy];
    for (let k = 0; k < 2; k++) {
      this.v[k] += _swClamp(vin[k] - this.v[k], -p.a_max[k] * dt, p.a_max[k] * dt);
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
    this.dur = this._shiftTime(d);
    this.tLocal = 0.0;
    this.phase = 0.0;
  }

  _tickShift(dt) {
    this.tLocal = Math.min(this.tLocal + dt, this.dur);
    const u = this.tLocal / this.dur;
    const [s, ds, dds] = _swS(u);
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
    this.dur = p.t_swing;
    this.phase = 0.0;
    const swing = -this.sup;
    this.swingR0 = this.foot[swing].slice();
    this.swingZ = 0.0;
    const ps = this.foot[this.sup];
    if (this._moving(p.v_stop_eps)) {
      this.mode = 'walk';
      this.pLand = [ps[0] + this.v[0] * p.stride_time,
                    ps[1] + swing * p.foot_spacing + this.v[1] * p.stride_time];
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
    this.tLocal = Math.min(this.tLocal + dt, this.dur);
    this.phase = this.tLocal / this.dur;
    const tau = this.phase;
    const s = _swS(tau)[0];
    const swing = -this.sup;
    const f = this.foot[swing];
    f[0] = this.swingR0[0] + s * (this.pLand[0] - this.swingR0[0]);
    f[1] = this.swingR0[1] + s * (this.pLand[1] - this.swingR0[1]);
    this.swingZ = this._swingHeightAt(tau, this.swingZ, dt);
    if (this.tLocal < this.dur) return;
    f[0] = this.pLand[0];
    f[1] = this.pLand[1];
    this.swingZ = 0.0;
    this.steps[this.steps.length - 1].t1 = this.t;
    this.pLand = null;
    if (this.mode === 'stop' || (!this._moving(this.p.v_stop_eps) && this._feetAligned())) {
      this._startShift(this._midpoint(), 'STOP');
    } else {
      this.sup = swing;
      this._startShift(this._overSupport(), 'SHIFT');
    }
  }

  _outputs() {
    const w = this._omega();
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

// ---------------------------------------------------------------------------
// node 直接実行: 照合用 CSV (compare_walk_engines.py --engine static と同じプロファイル・列)
//   node staticwalk.js <vx> <vy> [t_walk=9.5] [t_end=20.0] [dt=0.005] [params_json]
// ---------------------------------------------------------------------------
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { StaticWalkEngineJS, staticDefaultParams };
}
if (typeof process !== 'undefined' && typeof require !== 'undefined' &&
    require.main === module) {
  const argv = process.argv.slice(2);
  if (argv.length < 2) {
    console.error('usage: node staticwalk.js <vx> <vy> [t_walk] [t_end] [dt] [params_json]');
    process.exit(2);
  }
  const vx = +argv[0], vy = +argv[1];
  const tWalk = argv.length > 2 ? +argv[2] : 9.5;
  const tEnd = argv.length > 3 ? +argv[3] : 20.0;
  const dt = argv.length > 4 ? +argv[4] : 0.005;
  const params = argv.length > 5 ? JSON.parse(argv[5]) : {};
  const stateCode = { IDLE: 0, STOP: 3, ESTOP: 4, SHIFT: 5, SWING: 6 };
  const e = new StaticWalkEngineJS(params);
  const lines = ['t,st,ph,sup,vx,vy,xix,xiy,comx,comy,zx,zy,lfx,lfy,lfz,rfx,rfy,rfz'];
  const n = Math.round(tEnd / dt);
  for (let i = 0; i < n; i++) {
    const t = i * dt;
    const on = t >= 0.5 && t < tWalk;
    const o = e.update(on ? vx : 0.0, on ? vy : 0.0, dt, false);
    lines.push([o.t, stateCode[o.state], o.phase, o.support, o.v[0], o.v[1],
                o.xi[0], o.xi[1], o.com[0], o.com[1], o.zmp[0], o.zmp[1],
                o.leftFoot[0], o.leftFoot[1], o.leftFoot[2],
                o.rightFoot[0], o.rightFoot[1], o.rightFoot[2]].map(String).join(','));
  }
  console.log(lines.join('\n'));
}
