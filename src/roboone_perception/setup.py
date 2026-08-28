from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'roboone_perception'

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
    description='敵機検出 opponent_detector ノード（depth + IMU → /opponent, /ring_edge）',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'opponent_detector = roboone_perception.opponent_detector_node:main',
            'detector_bench = roboone_perception.detector_bench:main',
        ],
    },
)
