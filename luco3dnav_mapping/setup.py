import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'luco3dnav_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lkh-ubuntu',
    maintainer_email='lkh060320@gmail.com',
    description='Luco3DNav map layers: octomap voxel / 2D occupancy / 2.5D elevation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'elevation_mapper = luco3dnav_mapping.elevation_mapper:main',
            'save_maps = luco3dnav_mapping.save_maps:main',
            'remove_dynamic = luco3dnav_mapping.remove_dynamic:main',
        ],
    },
)
