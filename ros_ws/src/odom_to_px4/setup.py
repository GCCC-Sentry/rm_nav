from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odom_to_px4'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='adam@todo.todo',
    description='Bridge node: subscribes to nav_msgs/Odometry and publishes px4_msgs/VehicleOdometry for PX4 v1.15',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'odom_to_px4_node = odom_to_px4.odom_to_px4_node:main',
        ],
    },
)
