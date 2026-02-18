from setuptools import find_packages, setup
from glob import glob

package_name = 'view_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apostolos-ubuntu-pc',
    maintainer_email='apostolos-ubuntu-pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_diff_drive = view_robot_pkg.fake_diff_drive:main',
            'odom_frame_publisher = view_robot_pkg.odom_frame_publisher:main',
            'odom_tf_broadcaster = view_robot_pkg.odom_tf_broadcaster:main',
            'my_teleop_node = view_robot_pkg.teleop_node:main',
        ],
    },
)
