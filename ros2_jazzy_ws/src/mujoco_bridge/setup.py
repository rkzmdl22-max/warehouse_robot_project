from setuptools import setup
import os
from glob import glob

package_name = 'mujoco_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse_team',
    maintainer_email='student@koreait.ac.kr',
    description='MuJoCo simulation bridge for ROS 2 Jazzy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = mujoco_bridge.bridge_node:main',
        ],
    },
)
