from setuptools import find_packages, setup

package_name = 'roboone_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        'roboone_viz': ['template.html', 'walkcore.js', 'staticwalk.js',
                        'knee3d.html', 'leg3d.html', 'legs3d.html'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soedayu',
    maintainer_email='soedayu.030622@gmail.com',
    description='歩行・脚・膝の可視化ツール (ROS ノードは持たない)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walk_viz = roboone_viz.gen_walk_viz:main',
        ],
    },
)
