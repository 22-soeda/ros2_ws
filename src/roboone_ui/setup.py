from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'roboone_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 絵は share に置く。起動時に全部読んで前変換するので、ここに入れ忘れると
        # /ui/oled/image が全部「未知の画像名」になる。
        (os.path.join('share', package_name, 'images'), glob('images/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='soedayu.030622@gmail.com',
    description='OLED(SSD1331)/RGB LED/圧電サウンダーで機体状態を提示する UI ノード',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui_node = roboone_ui.ui_node:main'
        ],
    },
)
