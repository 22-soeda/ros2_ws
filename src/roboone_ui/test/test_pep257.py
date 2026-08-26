# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # 除外する理由:
    #   D400/D415 … docstring 冒頭行が '.' 等で終わることを要求する。このパッケージの
    #               docstring は日本語で文末が '。' になるため原理的に通らない。
    #               ws の他パッケージ (feetech_servo) も日本語コメントで統一されている。
    #   D213      … 「要約は 2 行目から」。ament が既定で無視している D212 (「要約は
    #               1 行目から」) と排他の規約で、このコードは D212 側の書き方に揃えてある。
    rc = main(argv=['.', 'test', '--add-ignore', 'D400,D415,D213'])
    assert rc == 0, 'Found code style errors / warnings'
