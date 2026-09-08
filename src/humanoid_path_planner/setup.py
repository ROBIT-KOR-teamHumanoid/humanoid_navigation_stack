from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'humanoid_path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz'),
        ),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robit',
    maintainer_email='leokim0503@kw.ac.kr',
    description='Visibility-graph path planner for the ROBIT humanoid',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planning = '
            'humanoid_path_planner.ros2_adapter.path_planning_node:main',
        ],
    },
)
