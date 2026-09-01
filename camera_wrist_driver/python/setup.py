from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_wrist_driver_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='ryan.liu@x-humanoid.com',
    description='Wrist RealSense D405 camera RGB/depth ROS2 publisher (left/right)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_wrist_driver_node = camera_wrist_driver.camera_wrist_driver_node:main',
        ],
    },
)
