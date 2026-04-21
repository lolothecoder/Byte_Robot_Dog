from setuptools import setup
from glob import glob

package_name = 'byte_leg_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Loic Finette',
    maintainer_email='loicfinette@gmail.com',
    description='Top-level launch for the byte_leg Gazebo simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
