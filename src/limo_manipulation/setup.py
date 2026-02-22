import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'limo_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UruBots',
    maintainer_email='team@urubots.com',
    description='Manipulation stack for LIMO + OpenManipulator-X (RoboCup @Work)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manipulation_manager  = limo_manipulation.manipulation_manager:main',
            'object_detector       = limo_manipulation.object_detector:main',
            'apriltag_sim_publisher = limo_manipulation.apriltag_sim_publisher:main',
            'detection_visualizer  = limo_manipulation.detection_visualizer:main',
        ],
    },
)
