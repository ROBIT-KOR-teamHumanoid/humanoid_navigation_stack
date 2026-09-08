from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'field_coordinate_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robit',
    maintainer_email='leokim0503@naver.com',
    description='Coordinate adapter for the ROBIT humanoid soccer field',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'field_coordinate_adapter = field_coordinate_adapter.node:main',
        ],
    },
)
