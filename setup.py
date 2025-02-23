import os
from glob import glob
from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # This line installs the package.xml
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # This line installs the package.xml into share/<package_name>
        ('share/' + package_name, ['package.xml']),

        # **Important**: Install all .launch.py files from your 'launch' folder
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
