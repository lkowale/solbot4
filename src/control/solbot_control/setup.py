from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'solbot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='lkowale@gmail.com',
    description='Control nodes for solbot robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = solbot_control.drive:main',
            'imu_bridge = solbot_control.imu_bridge:main',
            'imu_visualizer = solbot_control.imu_visualizer:main',
            'lift = solbot_control.lift:main',
            'mqtt_op = solbot_control.mqtt_op:main',
            'planter = solbot_control.planter:main',
            'steering = solbot_control.steering:main',
        ],
    },
)
