import os
from glob import glob
from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install the resource file for ament index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install the package.xml file
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Install URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email',
    description='Example drone control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = drone_control.drone_controller:main',
        ],
    },
)
