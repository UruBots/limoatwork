from setuptools import find_packages, setup
from glob import glob

package_name = 'limo_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts',
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='solverbot',
    maintainer_email='igordacosta.sl010@gmail.com',
    description='Mission orchestration + docking for LIMO (Nav2 + LiDAR)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mission_ws = limo_mission.mission_ws:main',
            'mission_manager = limo_mission.mission_manager:main',
            'dock_server = limo_mission.dock_server:main',
            'wall_docking_node = limo_mission.wall_docking_node:main',
            'pick_and_return_example = limo_mission.pick_and_return_example:main',
        ],
    },
)
