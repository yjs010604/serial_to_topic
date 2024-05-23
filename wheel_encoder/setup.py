from setuptools import setup
import os
from glob import glob

package_name = 'wheel_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # 런치 파일 설치 경로 설정
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to read wheel encoder data and publish to a topic',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'wheel_encoder_node = wheel_encoder.wheel_encoder_node:main',
        ],
    },
)

