from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'node_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='View robot in RViz',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_diff_drive = node_pkg.fake_diff_drive:main',
            'odom_frame_publisher = node_pkg.odom_frame_publisher:main',
            'odom_tf_broadcaster = node_pkg.odom_tf_broadcaster:main',
            'my_teleop_node = node_pkg.teleop_node:main',
        ],
    },
)
