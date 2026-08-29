from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'roboone_teleop'

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
    description='PS5 (DualSense) コントローラによる無線操縦 teleop ノード',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = roboone_teleop.teleop_node:main',
            'joy_probe = roboone_teleop.joy_probe:main',
            'teleop_params = roboone_teleop.params:main',
        ],
    },
)
