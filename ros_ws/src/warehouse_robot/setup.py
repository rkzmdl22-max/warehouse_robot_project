from setuptools import setup
from glob import glob
import os

package_name = 'warehouse_robot'

data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
    (
        os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py'),
    ),
    (
        os.path.join('share', package_name, 'worlds'),
        glob('worlds/*'),
    ),
    (
        os.path.join('share', package_name, 'urdf'),
        glob('urdf/*.urdf'),
    ),
    (
        os.path.join('share', package_name, 'rviz'),
        glob('rviz/*'),
    ),
    (
        os.path.join('share', package_name, 'maps'),
        glob('maps/*'),
    ),
]

# models 아래의 모든 파일을 재귀적으로 설치
for root, dirs, files in os.walk('models'):
    for file in files:
        src = os.path.join(root, file)
        relative_dir = os.path.dirname(src)

        install_dir = os.path.join(
            'share',
            package_name,
            relative_dir
        )

        data_files.append((install_dir, [src]))

# config 아래의 모든 파일을 재귀적으로 설치
for root, dirs, files in os.walk('config'):
    for file in files:
        src = os.path.join(root, file)
        relative_dir = os.path.dirname(src)

        install_dir = os.path.join(
            'share',
            package_name,
            relative_dir
        )

        data_files.append((install_dir, [src]))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Warehouse Robot Simulation',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'fr3_mujoco_adapter = warehouse_robot.fr3_mujoco_adapter:main',
            'mission_manager = warehouse_robot.mission_manager:main',
            'fleet_manager = warehouse_robot.fleet_manager:main',
            'bottle_detector = warehouse_robot.bottle_detector:main',
            'side_grasp_verifier = warehouse_robot.side_grasp_verifier:main',
            'tron1_mission_fsm = warehouse_robot.tron1_mission_fsm:main',
            'tron1_mujoco_vision = warehouse_robot.tron1_mujoco_vision:main',
            'ur5e_g14_executor = warehouse_robot.ur5e_g14_executor:main',
        ],
    },
)
