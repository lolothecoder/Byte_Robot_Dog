from setuptools import find_packages, setup
from glob import glob

package_name = 'byte_leg_hardware'

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
    description='CAN bridge to the real byte_leg ODrive motors.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_relay = byte_leg_hardware.can_relay:main',
        ],
    },
)
