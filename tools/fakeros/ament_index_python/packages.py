# -*- coding: utf-8 -*-
"""ament_index_python.packages の最小版。share ディレクトリの代わりに src/<pkg> を返す。

config/ や launch/ は setup.py の data_files でそのまま share へ入るので、ソースツリーの
パッケージディレクトリを返せば同じ相対パスで辿れる。
"""

import pathlib
import re

_WS = pathlib.Path(__file__).resolve().parents[3]


class PackageNotFoundError(KeyError):
    pass


def get_package_share_directory(package_name):
    for pkg_xml in (_WS / 'src').glob('*/package.xml'):
        text = pkg_xml.read_text(encoding='utf-8', errors='replace')
        m = re.search(r'<name>\s*([^<\s]+)\s*</name>', text)
        if m and m.group(1) == package_name:
            return str(pkg_xml.parent)
    raise PackageNotFoundError(package_name)


def get_package_prefix(package_name):
    return get_package_share_directory(package_name)
