import os
from glob import glob
from setuptools import setup

package_name = 'octo_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='watonomous',
    maintainer_email='watonomous@uwaterloo.ca',
    description='OctoMap generation for 3D mapping',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'octo_map_node = octo_map.octo_map_node:main'
        ],
    },
)
