import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amiga_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_gps_waypoint_logger = amiga_navigation.joy_gps_waypoint_logger:main',
            'logged_waypoint_follower = amiga_navigation.logged_waypoint_follower:main',
            'gps_to_waypoint_service = amiga_navigation.gps_to_waypoint:main',
        ],
    },
)
