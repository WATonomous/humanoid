import os
from glob    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyzhou, aryanafrouzi',
    maintainer_email='e23zhou@watonomous.ca, aafrouzi@watonomous.ca',
    description='Sample aggregator node for data collection',
    license='Apache2.0',
    tests_require=['pytest'], glob
from setuptools import setup

package_name = 'aggregator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyzhou, aryanafrouzi',
    maintainer_email='e23zhou@watonomous.ca, aryanafrouzi@swaprobotics.com',
    description='Sample aggregator node for data collection',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aggregator_node = aggregator.aggregator_node:main'
        ],
    },
)
