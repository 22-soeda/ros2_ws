from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'roboone_behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='soedayu.030622@gmail.com',
    description='自律動作の行動層 behavior ノード（/opponent, /ring_edge → /cmd_walk, /cmd_motion）',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'behavior = roboone_behavior.behavior_node:main',
        ],
    },
)
