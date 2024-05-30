from setuptools import setup
import os

package_name = 'encoder_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/encoder_reader_launch.py'  # 여기에 포함할 파일을 명시적으로 지정
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to read serial data and publish to topics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_reader_node = encoder_reader.encoder_reader_node:main',
        ],
    },
)
