from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_ugv_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autonomy Team',
    maintainer_email='dev@defence-autonomy.local',
    description='Autonomous patrol mission executive and GPS-denial health supervisor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_mission_node = autonomous_ugv_mission.patrol_mission_node:main',
            'gps_denied_monitor = autonomous_ugv_mission.gps_denied_monitor:main',
            'trajectory_logger = autonomous_ugv_mission.trajectory_logger:main',
        ],
    },
)
