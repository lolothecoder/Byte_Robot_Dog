from setuptools import find_packages, setup
from glob import glob

package_name = 'byte_leg_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Loic Finette',
    maintainer_email='loicfinette@gmail.com',
    description='Inverse kinematics and Xbox joystick teleop for the byte_leg simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = byte_leg_control.ik_node:main',
            'joy_teleop = byte_leg_control.joy_teleop:main',
        ],
    },
)
