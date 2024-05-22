from setuptools import setup
from glob import glob
import os

package_name = 'my_serial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YJS',
    maintainer_email='yjs010604@naver.com',
    description='ROS2 package to read serial data from ESP32 and publish to a topic',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'serial_reader = my_serial_pkg.serial_reader:main',
        ],
    },
)

