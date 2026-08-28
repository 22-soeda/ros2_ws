# -*- coding: utf-8 -*-
"""止まらないための小移動。docs/behavior_planning.pdf §4.9。

規則 10.3(d) は「倒れていなくても 3 秒以上停止、または 10 秒以上前後左右に
移動しない」とスタンディング（スリップ相当）扱いにする。SEARCH の旋回だけ、
RETREAT の待ち、ENGAGE の技の合間は、どれも前後左右には動いていないので、
そのまま置くと 10 秒を超えうる。

そこで最後に並進してから T_ka 秒経ったら、0.05 m の横移動を 1 回入れる。
向きは左右交互で、縁に近い側へは出さない。

文書との差
----------

文書はこれに加えて「歩行指令が零でも歩行エンジンを足踏み (STEP, v=0) の
ままにする」と書いているが、これは行動層だけでは実現できない。歩行ノートの
§4.3 では v = 0 は STOP → IDLE への遷移そのもので、/cmd_walk に零を出すと
足を揃えて止まるのが約束になっている。「零だが足踏みは続ける」を表す口が
/cmd_walk に無い。

ここでは小移動だけで規則を満たす形にしてある（横移動が入れば 3 秒の停止も
10 秒の無移動も成立しない）。足踏みの維持が要るなら motion 側に
「march in place」の口を足すのが筋で、README の申し送りに挙げてある。
"""

import math

from .ring import distance_to_edge


class KeepAlive:
    """停止が続いたら横移動を 1 回差し込む。

    判定に使うのは行動が決めた指令（この重ね合わせの入力）であって、出力では
    ない。出力で見ると自分が入れた横移動でタイマが戻り、周期が狂う。
    """

    def __init__(self, match, robot, tune):
        self.m = match
        self.r = robot
        self.t = tune
        self.reset()

    def reset(self):
        self.t_since_move = 0.0    # [s] 最後に並進してからの経過
        self.remaining = 0.0       # [s] 差し込み中の残り時間
        self.side = 1.0            # +1 が左。1 回ごとに反転する

    @property
    def speed(self):
        """横移動の速さ。0.05 m を 0.6 s ほどで動く。"""
        return self.r.v_y_max

    def overlay(self, vx, vy, pose, dt):
        """状態が決めた (vx, vy) に小移動を重ねる。返り値は重ねた後の指令。"""
        if math.hypot(vx, vy) > self.t.move_eps:
            # 行動そのものが動いている。差し込みは要らないし、途中でも打ち切る
            self.t_since_move = 0.0
            self.remaining = 0.0
            return vx, vy

        self.t_since_move += dt

        if self.remaining <= 0.0 and self.t_since_move >= self.m.keepalive_period:
            side = self._pick_side(pose)
            if side is None:
                # 左右とも縁が近い。EDGE が中央へ戻すのでそちらに任せる
                return vx, vy
            self.side = side
            self.remaining = self.m.keepalive_shift / max(self.speed, 1e-3)

        if self.remaining > 0.0:
            self.remaining -= dt
            if self.remaining <= 0.0:
                self.t_since_move = 0.0
                self.side = -self.side      # 次は反対側へ
            return vx, vy + self.side * self.speed

        return vx, vy

    def hold(self, dt):
        """指令を出せない周期のタイマだけを進める。

        技の再生中と WAIT / SELF_DOWN で呼ぶ。規則の「10 秒以上前後左右に
        移動しない」は技を再生している間も進むので、カウンタは止めない
        （§4.9 の 3 つ目）。差し込みの途中だったなら取り消す。
        """
        self.t_since_move += dt
        self.remaining = 0.0

    def _pick_side(self, pose):
        """出てよい側を選ぶ。既定は前回の反対側、縁が近ければ逆へ。

        pose が無い（リング座標系を持っていない）ときは交互のまま出す。
        """
        if pose is None:
            return self.side
        shift = self.m.keepalive_shift
        for side in (self.side, -self.side):
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            # 機体の y 方向 (左) は世界では (-sinψ, cosψ)
            nx = pose.x + side * (-s) * shift
            ny = pose.y + side * c * shift
            if distance_to_edge(self.m, nx, ny) >= pose.d_margin:
                return side
        return None
