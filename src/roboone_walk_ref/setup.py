from glob import glob

from setuptools import find_packages, setup

package_name = 'roboone_walk_ref'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # motion ノード (roboone_motion) が share から読む。
        # gait.yaml = 歩行の設定 / home_pose.yaml = ホーム姿勢 (足裏の位置姿勢)。
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soedayu',
    maintainer_email='soedayu.030622@gmail.com',
    description='歩行計画 walk_core の仕様原本 (Python) と正準の設定 YAML',
    license='MIT',
    tests_require=['pytest'],
)
