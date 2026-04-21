from setuptools import setup
from glob import glob
import os

package_name = 'adam_2026uc_decision_v1'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PolarBear',
    maintainer_email='user@todo.todo',
    description='ADAM 2026UC 哨兵决策节点 v1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alliance_decision_node = adam_2026uc_decision_v1.alliance_decision_node:main',
        ],
    },
)
