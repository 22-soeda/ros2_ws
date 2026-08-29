# -*- coding: utf-8 -*-
"""teleop の調整項目 (ROS パラメータ) の一覧表。

**人が手で触る値は、ここに並んでいるものが全部。** 名前・意味・単位・既定値・許される
範囲を 1 か所に集め、teleop_node.py はこの表を読んで宣言と検査をする。表にない名前は
config に書いても効かないので、起動時に警告が出る。

この表そのものは ROS に依存しない (rclpy を使うのは declare_all / read_all の中だけ)。
だから config を編集したあと、ROS を通さずにこれで検査できる:

    python3 -m roboone_teleop.params --check config/ps5_dualsense.yaml
    python3 -m roboone_teleop.params --md      # 項目一覧を Markdown の表で出す

ROS 環境なら ``ros2 run roboone_teleop teleop_params`` でも同じ。

値の効く順番 (後のものが勝つ):

    1. この表の default              … config を渡さずに起動したときの値
    2. パッケージの config/*.yaml    … launch の既定。普段はこれが効いている
    3. launch の overrides:=<yaml>    … 自分用の差分ファイル (変えたいキーだけ書く)
    4. ros2 param set (走らせたまま)   … 試すとき。ノードを再起動すると消える

数値は int でも float でも受け付ける (``scale.x: 1`` と書いても 1.0 として読む)。
"""

import argparse
from dataclasses import dataclass
import difflib
import sys
from typing import Any, Optional

from .bindings import Binding, parse_motion_bindings


@dataclass(frozen=True)
class Tunable:
    """調整項目 1 つ。"""

    name: str           #: ROS パラメータ名 (config では '.' がネストになる)
    default: Any        #: config を渡さないときの値
    doc: str            #: 意味。1 行で
    group: str          #: 表で並べるときの見出し
    unit: str = ''      #: 単位。無ければ空
    kind: str = 'float'  #: 'float' | 'int' | 'bool' | 'str' | 'path' (空でもよい str) | 'str_list'
    low: Optional[float] = None   #: 数値の下限 (含む)。None なら無し
    high: Optional[float] = None  #: 数値の上限 (含む)。None なら無し


_G_SAFETY = '周期・安全'
_G_STICK = 'スティック'
_G_BUTTON = 'ボタン'
_G_HOME = 'ホームポジション / その場保持'
_G_AUTO = '自律動作'
_G_MOTION = '技'
_G_LINK = '無線テスト'
_G_UI = '状態表示 (ui ノードへ)'

#: 調整項目の全部。並びは config/ps5_dualsense.yaml と同じにしてある。
TUNABLES = (
    # --- 周期・安全 ---------------------------------------------------------
    Tunable('rate_hz', 20.0, '/cmd_walk の送信周期。motion の cmd_timeout (0.5 s) より十分速く',
            _G_SAFETY, 'Hz', low=5.0, high=100.0),
    Tunable('joy_timeout', 0.5, '/joy がこれだけ途切れたら脱力をラッチ。大きくすると電波断に気付くのが遅れる',
            _G_SAFETY, 's', low=0.1, high=5.0),
    Tunable('deadzone', 0.12, 'スティックの遊び (0..1)。触っていないのに歩き出すなら上げる',
            _G_SAFETY, '', low=0.0, high=0.9),
    # --- スティック ---------------------------------------------------------
    Tunable('axes.walk_x', 1, '前後 (linear.x) に使う軸番号。joy_probe で確かめる',
            _G_STICK, '', kind='int', low=0, high=31),
    Tunable('axes.walk_y', 0, '左右の並行移動 (linear.y) に使う軸番号',
            _G_STICK, '', kind='int', low=0, high=31),
    Tunable('axes.walk_yaw', 2, '旋回 (angular.z) に使う軸番号。今の motion は使わない',
            _G_STICK, '', kind='int', low=0, high=31),
    Tunable('invert.walk_x', False, '前後の符号を反転。joy ノードが ROS 規約へ反転済みなので普段は false',
            _G_STICK, '', kind='bool'),
    Tunable('invert.walk_y', False, '左右の符号を反転', _G_STICK, '', kind='bool'),
    Tunable('invert.walk_yaw', False, '旋回の符号を反転', _G_STICK, '', kind='bool'),
    Tunable('scale.x', 0.06, 'スティック全倒しの前後速度。walk_core の v_max (gait.yaml) で頭打ち',
            _G_STICK, 'm/s', low=0.0, high=0.5),
    Tunable('scale.y', 0.03, 'スティック全倒しの左右速度。v_max の y で頭打ち',
            _G_STICK, 'm/s', low=0.0, high=0.5),
    Tunable('scale.yaw', 0.40, 'スティック全倒しの旋回速度', _G_STICK, 'rad/s', low=0.0, high=3.0),
    Tunable('accel.x', 0.15, '前後指令の変化率の上限。gait.yaml の a_max より大きくしても motion 側で削られる',
            _G_STICK, 'm/s²', low=0.01, high=10.0),
    Tunable('accel.y', 0.10, '左右指令の変化率の上限', _G_STICK, 'm/s²', low=0.01, high=10.0),
    Tunable('accel.yaw', 1.50, '旋回指令の変化率の上限', _G_STICK, 'rad/s²', low=0.01, high=30.0),
    # --- ボタン -------------------------------------------------------------
    Tunable('buttons.deadman', 'b10', '押している間だけ歩行・技が通る (R1)', _G_BUTTON, '', kind='str'),
    Tunable('buttons.relax', 'b9', '押した瞬間に脱力 (L1)', _G_BUTTON, '', kind='str'),
    Tunable('buttons.home', 'b6', 'home_hold 秒の長押しでホームポジション → トルクオン (Options)',
            _G_BUTTON, '', kind='str'),
    Tunable('buttons.link_test', 'b4', '押している間ブザー + LED で電波の疎通を示す (Create)',
            _G_BUTTON, '', kind='str'),
    Tunable('buttons.autonomy', 'b11', 'autonomy_hold 秒の長押しで自律動作へ (十字キー 上)',
            _G_BUTTON, '', kind='str'),
    Tunable('buttons.hold', 'b7',
            'hold_hold 秒の長押しで、今の姿勢のままトルクを入れる (L3)。転倒 → 脱力 → 起き上がりの経路用',
            _G_BUTTON, '', kind='str'),
    # --- ホームポジション ---------------------------------------------------
    Tunable('home_hold', 1.0, '長押し時間。誤発動防止なので短くしない', _G_HOME, 's', low=0.0, high=10.0),
    Tunable('home_motion', 'home', '先に /cmd_motion へ送る技名', _G_HOME, '', kind='str'),
    Tunable('home_torque_delay', 0.1, 'home / hold を送ってから /estop false (トルクオン) までの間',
            _G_HOME, 's', low=0.0, high=5.0),
    Tunable('hold_hold', 1.0, 'その場保持で武装する長押し時間', _G_HOME, 's', low=0.0, high=10.0),
    Tunable('hold_motion', 'hold',
            'その場保持で /cmd_motion へ送る名前。motion ノードが特別扱いする (motions.yaml には書かない)',
            _G_HOME, '', kind='str'),
    # --- 自律動作 -----------------------------------------------------------
    Tunable('autonomy_hold', 1.0, '自律動作に入る長押し時間', _G_AUTO, 's', low=0.0, high=10.0),
    Tunable('autonomy.stop_on_joy_loss', True,
            '電波が切れたら自律も止めて脱力する。false にすると非常停止の手が無くなる',
            _G_AUTO, '', kind='bool'),
    # --- 技 -----------------------------------------------------------------
    Tunable('motion_bindings',
            ['b1:punch_r', 'b2:punch_l', 'b3:getup_front', 'b0:getup_back',
             'b13:turn_l', 'b14:turn_r', 'b12:squat'],
            '"<割り当て>:<技名>" の並び。技名は motions.yaml と一致させる',
            _G_MOTION, '', kind='str_list'),
    Tunable('motion_interrupts', ['getup_front', 'getup_back'],
            'デッドマン不要で、押すと自律動作を止める技', _G_MOTION, '', kind='str_list'),
    Tunable('motion_requires_deadman', True, '上記以外の技はデッドマンを押している間だけ通す',
            _G_MOTION, '', kind='bool'),
    Tunable('motion_cooldown', 0.5, '同じ技を続けて送らない間隔', _G_MOTION, 's', low=0.0, high=5.0),
    Tunable('motions_yaml', '',
            '技名の照合に使う motions.yaml の場所。空なら roboone_motion_node の share から探す',
            _G_MOTION, '', kind='path'),
    # --- 無線テスト ---------------------------------------------------------
    Tunable('link_test.buzzer', 'beep', 'ui ノードのブザープリセット名 (beep / ack / error)',
            _G_LINK, '', kind='str'),
    Tunable('link_test.buzzer_hz', 15.0, 'ブザーを撃ち直す周期', _G_LINK, 'Hz', low=1.0, high=50.0),
    Tunable('link_test.stale', 0.15, '/joy がこれだけ途切れたら即鳴り止む (joy_timeout を待たない)',
            _G_LINK, 's', low=0.02, high=2.0),
    # --- 状態表示 -----------------------------------------------------------
    Tunable('ui.enable', True, 'OLED / LED / ブザーへの状態表示を出すか', _G_UI, '', kind='bool'),
    Tunable('ui.pattern.nolink', 'dark', '/joy 未受信の LED プリセット', _G_UI, '', kind='str'),
    Tunable('ui.pattern.relax', 'estop', '脱力中の LED プリセット', _G_UI, '', kind='str'),
    Tunable('ui.pattern.unarmed', 'warn', '再武装待ちの LED プリセット', _G_UI, '', kind='str'),
    Tunable('ui.pattern.manual', 'ready', '武装済み (歩ける) の LED プリセット', _G_UI, '', kind='str'),
    Tunable('ui.pattern.auto', 'auto', '自律動作中の LED プリセット', _G_UI, '', kind='str'),
    Tunable('ui.pattern.link', 'link', '無線テスト中の LED プリセット', _G_UI, '', kind='str'),
    Tunable('ui.buzzer.relax', 'error', '脱力に落ちたときの音', _G_UI, '', kind='str'),
    Tunable('ui.buzzer.auto', 'ack', '自律動作に入ったときの音', _G_UI, '', kind='str'),
    Tunable('ui.buzzer.manual', 'beep', '武装した (歩ける状態になった) ときの音', _G_UI, '', kind='str'),
)

#: ROS が勝手に足すパラメータ。config に無くても、あっても、関知しない。
_ROS_BUILTIN_PREFIXES = ('use_sim_time', 'qos_overrides.', 'start_type_description_service')


def by_name():
    """名前 -> Tunable の辞書。"""
    return {t.name: t for t in TUNABLES}


def groups():
    """表の見出しを、出てくる順に。"""
    seen = []
    for t in TUNABLES:
        if t.group not in seen:
            seen.append(t.group)
    return seen


# ------------------------------------------------------------------ 値の検査
def coerce(t: Tunable, value):
    """設定ファイルや ros2 param set から来た値を、その項目の型に直す。駄目なら ValueError。"""
    if t.kind == 'float':
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'数値 (例 {t.default!r}) を書くこと')
        return float(value)
    if t.kind == 'int':
        if isinstance(value, bool) or not isinstance(value, int):
            if isinstance(value, float) and value.is_integer():
                return int(value)
            raise ValueError(f'整数 (例 {t.default!r}) を書くこと')
        return int(value)
    if t.kind == 'bool':
        if not isinstance(value, bool):
            raise ValueError('true か false を書くこと (引用符なし)')
        return value
    if t.kind == 'str':
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'文字列 (例 "{t.default}") を書くこと')
        return value
    if t.kind == 'path':
        if not isinstance(value, str):
            raise ValueError('パスの文字列 (空でもよい) を書くこと')
        return value
    if t.kind == 'str_list':
        if isinstance(value, str):
            value = [value]
        if not isinstance(value, (list, tuple)) or not all(isinstance(v, str) for v in value):
            raise ValueError('文字列のリスト (例 ["b1:punch_r", "b2:punch_l"]) を書くこと')
        return list(value)
    raise ValueError(f'kind が不正: {t.kind}')


def validate(t: Tunable, value) -> Optional[str]:
    """その値を受け付けてよいか。駄目なら理由の文字列、よければ None。"""
    try:
        v = coerce(t, value)
    except ValueError as e:
        return str(e)
    if t.kind in ('float', 'int'):
        if t.low is not None and v < t.low:
            return f'{t.low} 以上にすること'
        if t.high is not None and v > t.high:
            return f'{t.high} 以下にすること'
    try:
        if t.name.startswith('buttons.'):
            Binding(v)
        elif t.name == 'motion_bindings':
            parse_motion_bindings(v)
    except ValueError as e:
        return str(e)
    return None


def unknown_keys(names):
    """設定に書かれているが、この表に無い名前。ROS 組み込みのものは除く。"""
    known = by_name()
    out = []
    for n in names:
        if n in known or n.startswith(_ROS_BUILTIN_PREFIXES):
            continue
        out.append(n)
    return out


def suggest(name: str) -> Optional[str]:
    """打ち間違いに対して、いちばん近い項目名を 1 つ返す。"""
    hits = difflib.get_close_matches(name, [t.name for t in TUNABLES], n=1, cutoff=0.6)
    return hits[0] if hits else None


# ---------------------------------------------------------- 技名の照合
def load_motion_names(path: str):
    """motions.yaml に定義されている技名の集合。"""
    import yaml
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    motions = data.get('motions') or {}
    if not isinstance(motions, dict):
        raise ValueError('motions: の下が辞書になっていない')
    return set(motions.keys())


def bound_motion_names(bindings):
    """motion_bindings の "<割り当て>:<技名>" から技名だけを (順序を保って) 取り出す。"""
    out = []
    for item in bindings:
        name = str(item).split(':', 1)[1].strip() if ':' in str(item) else ''
        if name and name not in out:
            out.append(name)
    return out


def missing_motions(used, defined, builtin=()):
    """割り当てた技名のうち motions.yaml に無いもの。motion ノードが特別扱いする名前は除く。"""
    out = []
    for name in used:
        if name in defined or name in builtin or name in out:
            continue
        out.append(name)
    return out


# ------------------------------------------------------------ config の読み込み
def flatten(d: dict, prefix: str = '') -> dict:
    """ネストした dict を 'a.b.c' 形式の 1 段に潰す。"""
    out = {}
    for k, v in d.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def load_param_file(path: str) -> dict:
    """ROS 2 のパラメータ YAML を読んで {名前: 値} にする。

    ``/**:`` でも ``/teleop:`` でも ``teleop:`` でもよい。``ros__parameters:`` の下を拾う。
    """
    import yaml
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    found = {}

    def walk(node):
        if not isinstance(node, dict):
            return
        rp = node.get('ros__parameters')
        if isinstance(rp, dict):
            found.update(flatten(rp))
            return
        for v in node.values():
            walk(v)

    walk(data)
    if not found and isinstance(data, dict) and 'ros__parameters' not in data:
        found = flatten(data)   # ros__parameters 無しの素の dict も許す
    return found


def check_file(path: str, motions_yaml: Optional[str] = None):
    """設定ファイルを検査して (エラーの一覧, 補足の一覧) を返す。エラーが空なら合格。

    motions_yaml を渡すと、割り当てた技名がそこに定義されているかも見る
    (「割り当て済み ≠ 動く」を黙って通さないため)。
    """
    errors, notes = [], []
    try:
        values = load_param_file(path)
    except Exception as e:      # 読めない理由をそのまま見せる
        return [f'読めない: {e}'], notes
    known = by_name()
    for n in unknown_keys(values):
        hint = suggest(n)
        msg = f'知らないキー "{n}" (効かない)'
        if hint:
            msg += f' — "{hint}" の打ち間違い?'
        errors.append(msg)
    for n, v in values.items():
        t = known.get(n)
        if t is None:
            continue
        err = validate(t, v)
        if err:
            errors.append(f'{n} = {v!r}: {err}')
    missing = [t.name for t in TUNABLES if t.name not in values]
    if missing:
        notes.append('config に無い項目 (既定値が使われる): ' + ', '.join(missing))
    if motions_yaml:
        merged = {t.name: t.default for t in TUNABLES}
        merged.update({k: v for k, v in values.items() if k in known})
        try:
            defined = load_motion_names(motions_yaml)
        except Exception as e:      # 読めない理由をそのまま見せる
            errors.append(f'motions.yaml を読めない ({motions_yaml}): {e}')
        else:
            used = bound_motion_names(merged['motion_bindings'])
            builtin = (merged['home_motion'], merged['hold_motion'])
            for name in missing_motions(used, defined, builtin):
                errors.append(f'技 "{name}" は motions.yaml に無い (押しても何も起きない)')
            notes.append(f'技名の照合: {len(used)} 件を {motions_yaml} と突き合わせた')
    return errors, notes


# ------------------------------------------------------------------ 表の出力
def _fmt(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(str(v) for v in value) + ']'
    if isinstance(value, str):
        return f'"{value}"'
    return repr(value)


def _range(t: Tunable) -> str:
    if t.kind not in ('float', 'int'):
        return ''
    lo = '' if t.low is None else f'{t.low:g}'
    hi = '' if t.high is None else f'{t.high:g}'
    return f'{lo} 〜 {hi}' if (lo or hi) else ''


def markdown_table() -> str:
    """項目一覧を Markdown の表 (見出しごと) で返す。docs に貼るためのもの。"""
    lines = []
    for g in groups():
        lines.append(f'#### {g}')
        lines.append('')
        lines.append('| 名前 | 既定 | 単位 | 範囲 | 意味 |')
        # 区切りのダッシュの数が列幅の比になる (pandoc)。意味の列を広く取る
        lines.append('|------|-----|---|-----|----------------|')
        for t in TUNABLES:
            if t.group != g:
                continue
            lines.append(
                f'| `{t.name}` | `{_fmt(t.default)}` | {t.unit} | {_range(t)} | {t.doc} |')
        lines.append('')
    return '\n'.join(lines)


def plain_table() -> str:
    """端末向けの一覧。"""
    lines = []
    w = max(len(t.name) for t in TUNABLES)
    for g in groups():
        lines.append(f'[{g}]')
        for t in TUNABLES:
            if t.group == g:
                unit = f' [{t.unit}]' if t.unit else ''
                rng = f'  ({_range(t)})' if _range(t) else ''
                lines.append(f'  {t.name:<{w}}  = {_fmt(t.default)}{unit}{rng}')
                lines.append(f'  {"":<{w}}    {t.doc}')
        lines.append('')
    return '\n'.join(lines)


# ------------------------------------------------------------ ROS との接点
def declare_all(node):
    """全項目をノードに宣言する。説明は ``ros2 param describe`` で読める。"""
    from rcl_interfaces.msg import ParameterDescriptor
    for t in TUNABLES:
        text = t.doc + (f' [{t.unit}]' if t.unit else '')
        rng = _range(t)
        if rng:
            text += f' (範囲 {rng})'
        # 数値は int / float どちらで書かれても受ける (dynamic_typing)。読むときに直す。
        desc = ParameterDescriptor(description=text,
                                   dynamic_typing=t.kind in ('float', 'int'))
        node.declare_parameter(t.name, t.default, desc)
        # 型は rclpy が見るが範囲は見ないので、ここで止める (黙って範囲外で動かない)
        value = node.get_parameter(t.name).value
        err = validate(t, value)
        if err:
            raise ValueError(
                f'設定の {t.name} = {value!r}: {err} '
                '(ros2 run roboone_teleop teleop_params --check <yaml> で起動前に確かめられる)')


def read_all(node) -> dict:
    """宣言済みの値を全部読んで {名前: 型を直した値} にする。"""
    return {t.name: coerce(t, node.get_parameter(t.name).value) for t in TUNABLES}


# ------------------------------------------------------------------ コマンド
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog='teleop_params',
        description='teleop の調整項目の一覧と、config YAML の検査 (ROS 不要)')
    ap.add_argument('--check', metavar='YAML', nargs='+',
                    help='config を検査する (知らないキー・型・範囲)')
    ap.add_argument('--motions', metavar='YAML',
                    help='--check のとき、技名をこの motions.yaml と照合する')
    ap.add_argument('--md', action='store_true', help='一覧を Markdown の表で出す')
    args = ap.parse_args(argv)

    if args.check:
        rc = 0
        for path in args.check:
            errors, notes = check_file(path, args.motions)
            print(f'== {path}')
            for e in errors:
                print(f'  NG  {e}')
            for n in notes:
                print(f'  --  {n}')
            if errors:
                rc = 1
            else:
                print('  OK  問題なし')
        return rc
    print(markdown_table() if args.md else plain_table())
    return 0


if __name__ == '__main__':
    sys.exit(main())
