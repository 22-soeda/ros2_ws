# 脚 IK: 大腿・下腿リンクの x 成分を入れた導出

`脚IK導出.tex` の続き。同文書は $\bm p_3, \bm p_4$ が $z$ 成分しか持たない前提で
閉形式解を出しているが、CAD 実測でこれらに $x$ 成分が付くことがわかった。
本メモはその場合に式を組み直したもので、実装は

- `src/roboone_kinematics/include/roboone_kinematics/leg_kinematics.hpp`（C++・本番用）
- `scripts/leg_ik.py`（Python・参照実装）

にある。式番号 (X-n) は両方のコード中のコメントと対応する。

## 何が変わるか

| | `脚IK導出.tex` | 本メモ |
|---|---|---|
| $\bm p_3$ | $(0, b, -\ell_3)$ | $(a_3, b, -\ell_3)$ |
| $\bm p_4$ | $(0, 0, -\ell_4)$ | $(a_4, 0, -\ell_4)$ |
| $\theta_5, \theta_4$ の解き方 | $X = A\cos\theta_5$ の 2 次方程式 (IK-7) | $\theta_5$ の**線形式** (X-7) |
| 三角関数の呼び出し | atan2 x5, arccos x1 | atan2 x6, asin x1, arccos x1 |

**文書 §6 は「$a_3 \ne 0$ は吸収できない」としているが、これは (IK-2) の
$X, Y$ 分離が崩れて 2 次方程式の道が閉じる、という意味では正しい。
別の経路（下の [5]）を通れば閉形式は保たれる。** 反復解法は要らない。

なお $\bm p_5$ の $y$ 成分（A3 の破れ）については文書 §6 のとおりで、
こちらは今も閉形式にならない。CAD で $\delta = 0$ を確認すること。

## 導出

### [1] $x$ は和だけが効く

$x$ は膝軸 (J4) の方向で、$R_x$ はこれを動かさない。よって $o_4$ を膝軸に沿って
$\delta$ 滑らせると $\bm p_3 \to \bm p_3 + (\delta,0,0)$、$\bm p_4 \to \bm p_4 - (\delta,0,0)$ となり、
FK の $\bm p_3 + R_4(\bm p_4 + \cdots)$ では $+\delta$ と $R_4(\delta,0,0) = (\delta,0,0)$ が打ち消し合う。

$$a := a_3 + a_4$$

だけが幾何に効く。$a_3, a_4$ の分け方は結果に影響しない（selftest で確認済み）。
CAD の実測値をそのまま両方に入れてよい。

### [2] $y$ オフセットの吸収は従来どおり

文書 §6 の $\ell_3' = \sqrt{\ell_3^2 + b^2}$、$\varphi = \operatorname{atan2}(b, \ell_3)$、$\theta_4' = \theta_4 - \varphi$ で吸収できる。
$x$ 成分は $R_x$ で不変なのでこの吸収と干渉しない。

$$\bm a := R_4^{\mathsf T}\bm p_3 + \bm p_4 = (a,\ -B,\ -A), \qquad
A = \ell_3'\cos\theta_4' + \ell_4,\quad B = \ell_3'\sin\theta_4' \tag{X-1}$$

### [3] 遠位の展開

$$\bm w := R_5^{\mathsf T}\bm a + \bm p_5 = (a c_5 + A s_5,\ \ -B,\ \ a s_5 - A c_5 - \ell_5) \tag{X-2}$$

$V := a s_5 - A c_5 - \ell_5$ と置くと、$\bm r = R_6^{\mathsf T}\bm w$ の成分は

$$r_x = a c_5 + A s_5 \tag{X-3}$$
$$r_y = -B c_6 + V s_6 \tag{X-4}$$
$$r_z = \ \ B s_6 + V c_6 \tag{X-5}$$

$a = 0$ なら $V = -(A c_5 + \ell_5)$ で文書の (IK-2)〜(IK-4) に戻る。

### [4] 長さの式

$r_y^2 + r_z^2 = B^2 + V^2$（回転で長さが保たれる）。$r_x^2 + V^2$ を展開すると
交差項 $2aA c_5 s_5$ が消えて

$$|\bm r|^2 = a^2 + A^2 + B^2 + \ell_5^2 + 2\ell_5(A c_5 - a s_5)$$

$A^2 + B^2 = \ell_3'^2 - \ell_4^2 + 2\ell_4 A$（文書 (26)）を入れて

$$K := |\bm r|^2 - a^2 - \ell_3'^2 + \ell_4^2 - \ell_5^2 = 2\ell_4 A + 2\ell_5(A c_5 - a s_5) \tag{X-6}$$

$K$ は既知。$a = 0$ なら文書 (IK-5) に一致する。

### [5] ここが分かれ目 — 2 次式ではなく線形式

$a \ne 0$ だと (X-3) が $r_x = a c_5 + A s_5$ となり、$X = A c_5$、$Y = A s_5$ の分離が崩れる。
文書 §6 が指摘するとおり $X$ の 2 次方程式にはならない。

そこで **$A$ を消す向きに進む**。(X-6) を $A$ について解くと

$$A = \frac{K + 2\ell_5 a s_5}{2(\ell_4 + \ell_5 c_5)} \tag{X-8}$$

（分母は $\ell_4 > \ell_5$ なので 0 にならない。）これを (X-3) に入れて分母を払うと、
$A$ の 2 次項がちょうど打ち消し合って $c_5^2$ と $s_5^2$ が $c_5^2 + s_5^2 = 1$ にまとまり、

$$K\,s_5 + 2(a\ell_4 - \ell_5 r_x)\,c_5 = 2(\ell_4 r_x - a\ell_5) \tag{X-7}$$

という $\theta_5$ の**線形式**が残る。$P s_5 + Q c_5 = C$ の形は
$R_m = \sqrt{P^2+Q^2}$、$\psi = \operatorname{atan2}(Q,P)$ として $R_m \sin(\theta_5 + \psi) = C$ なので

$$\theta_5 = \arcsin\!\left(\frac{C}{R_m}\right) - \psi
\quad\text{または}\quad
\pi - \arcsin\!\left(\frac{C}{R_m}\right) - \psi$$

の 2 根。掛けた因子 $2(\ell_4 + \ell_5 c_5)$ は 0 にならないので偽根は出ない。

### [6] 残り

$\theta_5$ が出れば (X-8) で $A$ が戻り、以降は文書どおり。

$$\cos\theta_4' = \frac{A - \ell_4}{\ell_3'},\qquad \theta_4 = \sigma\arccos(\cdot) + \varphi,\qquad B = \ell_3'\sin\theta_4'$$

(X-4)(X-5) を $(c_6, s_6)$ について解いて

$$\theta_6 = \operatorname{atan2}\bigl(V r_y + B r_z,\ \ V r_z - B r_y\bigr) \tag{X-9}$$

$a=0$ で $V = -(A c_5 + \ell_5)$ を入れると文書 (IK-10) に一致する。
股 3 軸は文書のまま $M := R R_{456}^{\mathsf T} = R_{123}$ から (IK-12)〜(IK-14)。

## 分岐と到達不能

| | 文書 | 本メモ |
|---|---|---|
| 足首の枝 | (IK-7') の $+$ 根 = $\cos\theta_5 > 0$ | 2 根のうち $A>0$ かつ膝が範囲内、次に $\cos\theta_5$ が大きい方 |
| 膝の枝 | $\sigma = \pm 1$ | 同じ |

到達不能は 3 種類を区別して返す（`IkStatus`）。

| 状態 | 条件 | 意味 |
|---|---|---|
| `AnkleOutOfRange` | $\lvert C\rvert > R_m$ | (X-7) を満たす $\theta_5$ が無い。$x$ 方向に遠すぎる |
| `KneeOutOfRange` | $\lvert(A-\ell_4)/\ell_3'\rvert > 1$ | 脚長に対して遠すぎる / 近すぎる |
| `NoBranch` | 両根とも $A \le 0$ | 足先が股中心に近すぎる。丸めても意味がない |

`clamp=true` なら `NoBranch` 以外は最寄り姿勢を書き戻す。

## 検算

`ros2 run roboone_kinematics leg_selftest`（C++）と `python3 scripts/leg_ik.py`（Python）。

- 可動域内 20,000 姿勢で $\mathrm{FK}(\mathrm{IK}(\mathrm{FK}(\theta)))$: 位置の最大誤差 $8.5\times10^{-14}$ mm、姿勢 $6.4\times10^{-16}$
- $a_3+a_4$ を保ったまま 4 通りに分け直しても姿勢は不変（[1] の確認）
- $a=0$ で (IK-2) と (X-9)↔(IK-10) が一致、$\ell_5=0$ で $\cos\theta_4$ が余弦定理に一致
- `AXIS_FLIP` 全 64 通りで幾何が不変
- `python3 scripts/crosscheck_cpp.py` で C++ と Python を 7 パラメータ組 x 5,000 姿勢で突き合わせ

## 速度

Raspberry Pi 5 実測（`-O2`）。

| | Python | C++ |
|---|---|---|
| `fk` | 37.4 us | 0.221 us |
| `ik` | 27.5 us | 0.473 us |

両脚 `fk`+`ik` で 1.4 us。200 Hz ループ（5 ms）の 0.03 %。
