import os
from glob import glob
from setuptools import setup

package_name = 'ros2_maestro'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 node for controlling Pololu Maestro servos from /face_motors topic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maestro_node = ros2_maestro.maestro_node:main',
        ],
    },
)
