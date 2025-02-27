from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_swarm_controller_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include config files if any
        (os.path.join('share', package_name, 'config'),
         glob('config/*.json')),
        # Include Trajectories YAML files
        (os.path.join('share', package_name, 'config', 'Trajectories'),
         glob('config/Trajectories/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorge',
    maintainer_email='jafs2@kth.se',
    description='PX4 Swarm Controller Python implementation',
    license='Apache-2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'weighted_topology_controller = px4_swarm_controller_python.swarm_controllers.weighted_topology_controller.weighted_topology_controller:main',
            'weighted_topology_neighbors = px4_swarm_controller_python.swarm_controllers.weighted_topology_controller.weighted_topology_neighbors:main',
            'simulation_node = px4_swarm_controller_python.simulation_node:main',
            'reference_signal_publisher = px4_swarm_controller_python.reference_signal:main',
            # Add more nodes here
        ],
    },
)