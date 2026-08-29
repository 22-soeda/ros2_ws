# -*- coding: utf-8 -*-
"""調整項目の表 (params.py) と config の整合を見る。ROS 無しで走る。

見張るのは「人が config を触ったときに黙って壊れる」経路:
  * config に書いた名前の打ち間違い (ROS は宣言に無いキーを黙って捨てる)
  * 型・範囲の外れた値
  * 表と config の食い違い (config に全部の項目が並んでいること)
"""

import pathlib

import pytest
from roboone_teleop import params

CONFIG = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'ps5_dualsense.yaml'
# 隣のパッケージの motions.yaml (ws ごとチェックアウトしていれば見える)
MOTIONS = (pathlib.Path(__file__).resolve().parents[2] / 'roboone_motion_node' / 'config'
           / 'motions.yaml')


def test_names_unique():
    names = [t.name for t in params.TUNABLES]
    assert len(names) == len(set(names))


def test_defaults_are_valid():
    for t in params.TUNABLES:
        assert params.validate(t, t.default) is None, t.name


def test_config_lists_every_tunable_and_nothing_else():
    """設定ファイルは調整項目の一覧そのもの。人が code を読まずに済むように全部並べておく。"""
    values = params.load_param_file(str(CONFIG))
    assert set(values) == {t.name for t in params.TUNABLES}


def test_config_values_are_valid():
    errors, _ = params.check_file(str(CONFIG))
    assert errors == []


def test_check_file_reports_typo_with_suggestion(tmp_path):
    p = tmp_path / 'bad.yaml'
    p.write_text('/**:\n  ros__parameters:\n    scal:\n      x: 0.1\n', encoding='utf-8')
    errors, _ = params.check_file(str(p))
    assert len(errors) == 1
    assert 'scal.x' in errors[0] and 'scale.x' in errors[0]


def test_check_file_reports_out_of_range(tmp_path):
    p = tmp_path / 'bad.yaml'
    p.write_text('/**:\n  ros__parameters:\n    deadzone: 1.5\n', encoding='utf-8')
    errors, _ = params.check_file(str(p))
    assert any('deadzone' in e for e in errors)


def test_check_file_accepts_node_name_form(tmp_path):
    p = tmp_path / 'ok.yaml'
    p.write_text('/teleop:\n  ros__parameters:\n    scale:\n      x: 0.08\n', encoding='utf-8')
    errors, notes = params.check_file(str(p))
    assert errors == []
    assert notes and 'config に無い項目' in notes[0]


def test_int_is_accepted_for_float():
    t = params.by_name()['scale.x']
    assert params.validate(t, 0) is None
    assert params.coerce(t, 0) == 0.0 and isinstance(params.coerce(t, 0), float)


def test_bool_and_str_are_rejected_for_numbers():
    t = params.by_name()['scale.x']
    assert params.validate(t, True) is not None
    assert params.validate(t, '0.1') is not None


def test_range_is_enforced():
    t = params.by_name()['joy_timeout']
    assert params.validate(t, 0.0) is not None
    assert params.validate(t, 100.0) is not None
    assert params.validate(t, 0.5) is None


def test_binding_strings_are_checked():
    assert params.validate(params.by_name()['buttons.deadman'], 'x5') is not None
    assert params.validate(params.by_name()['buttons.deadman'], 'a7-') is None
    assert params.validate(params.by_name()['motion_bindings'], ['b1punch']) is not None
    assert params.validate(params.by_name()['motion_bindings'], ['b1:punch_r']) is None


def test_unknown_keys_ignore_ros_builtins():
    assert params.unknown_keys(['use_sim_time', 'qos_overrides./cmd_walk.publisher.depth']) == []
    assert params.unknown_keys(['scale.x', 'scal.x']) == ['scal.x']


def test_markdown_table_lists_everything():
    md = params.markdown_table()
    for t in params.TUNABLES:
        assert f'`{t.name}`' in md


def test_cli_check_returns_nonzero_on_error(tmp_path, capsys):
    p = tmp_path / 'bad.yaml'
    p.write_text('/**:\n  ros__parameters:\n    rate_hz: "fast"\n', encoding='utf-8')
    assert params.main(['--check', str(p)]) == 1
    assert 'rate_hz' in capsys.readouterr().out


def test_cli_check_passes_shipped_config(capsys):
    assert params.main(['--check', str(CONFIG)]) == 0
    assert 'OK' in capsys.readouterr().out


@pytest.mark.parametrize('kind', ['float', 'int', 'bool', 'str', 'str_list'])
def test_every_kind_has_a_default_of_its_own_type(kind):
    for t in params.TUNABLES:
        if t.kind == kind:
            assert params.validate(t, t.default) is None


# ------------------------------------------------------------ 技名の照合
def test_missing_motions_skips_builtin_and_dedups():
    used = ['punch_r', 'home', 'hold', 'turn_l', 'turn_l']
    assert params.missing_motions(used, {'punch_r'}, builtin=('home', 'hold')) == ['turn_l']


def test_bound_motion_names_keeps_order_and_dedups():
    assert params.bound_motion_names(['b1:punch_r', 'b2:punch_l', 'b3:punch_r', 'bad']) == \
        ['punch_r', 'punch_l']


def test_load_motion_names_reads_shipped_file():
    if not MOTIONS.exists():
        pytest.skip('motions.yaml が隣に無い (パッケージ単体のチェックアウト)')
    names = params.load_motion_names(str(MOTIONS))
    assert {'punch_r', 'getup_front', 'getup_back'} <= names


@pytest.mark.xfail(strict=True, reason='turn_l / turn_r は割り当てだけあって motions.yaml に無い '
                   '(docs/無線操縦_不足項目レビュー.md §2.2)。作ったらこの印を外す')
def test_shipped_bindings_are_all_defined_in_motions_yaml():
    """「割り当て済み ≠ 動く」を黙って通さない。motions.yaml が揃ったら XPASS で気付く。"""
    if not MOTIONS.exists():
        pytest.skip('motions.yaml が隣に無い (パッケージ単体のチェックアウト)')
    errors, _ = params.check_file(str(CONFIG), motions_yaml=str(MOTIONS))
    assert errors == []


def test_cli_check_with_motions_reports_undefined(tmp_path, capsys):
    m = tmp_path / 'motions.yaml'
    m.write_text('motions:\n  punch_r: {}\n', encoding='utf-8')
    c = tmp_path / 'cfg.yaml'
    c.write_text('/**:\n  ros__parameters:\n    motion_bindings: ["b1:punch_r", "b2:kick"]\n',
                 encoding='utf-8')
    assert params.main(['--check', str(c), '--motions', str(m)]) == 1
    out = capsys.readouterr().out
    assert 'NG' in out and 'kick' in out
