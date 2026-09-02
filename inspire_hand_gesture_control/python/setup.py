from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspire_hand_gesture_control_py'

setup(
    name=package_name,
    version='0.0.1',
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
    description='因时灵巧手手势控制节点 - 实现OK手势和剪刀石头布功能 (Python)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_gesture_control_node = inspire_hand_gesture_control.hand_gesture_control:main',
        ],
    },
)
