from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'josh_intro_to_ros'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joshuatang',
    maintainer_email='joshuatang2007@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['publisher = josh_intro_to_ros.publisher:main',
# 'subscriber = josh_intro_to_ros.subscriber:main',
'bluerov2_sensors = josh_intro_to_ros.bluerov2_sensors:main',
'arm = josh_intro_to_ros.arm:main',
'movement = josh_intro_to_ros.movement:main',
'depth = josh_intro_to_ros.depth:main',
'depth_control = josh_intro_to_ros.depth_control:main',
'heading_control = josh_intro_to_ros.heading_control:main',
'lane_subscriber = josh_intro_to_ros.lane_subscriber:main',
'tag_subscriber = josh_intro_to_ros.tag_subscriber:main',
'lights_controller = josh_intro_to_ros.lights_controller:main', 
'controller = josh_intro_to_ros.controller:main', 
'message = josh_intro_to_ros.message:main'
        ],
    },
)
