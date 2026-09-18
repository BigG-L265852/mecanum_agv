import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mecanum_agv'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='big-g',
    maintainer_email='gijs.v.lankvelt@gmail.com',
    description='Mecanum-wheel AGV drive control, odometry, and SLAM bringup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_drive_node = mecanum_agv.mecanum_drive_node:main',
            'map_to_png = mecanum_agv.map_to_png:main',
            'map_to_pointcloud = mecanum_agv.map_to_pointcloud:main',
            'map_viewer = mecanum_agv.map_viewer:main',
        ],
    },
)
