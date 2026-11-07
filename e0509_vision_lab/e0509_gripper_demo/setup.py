import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'e0509_gripper_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airoboticshj',
    maintainer_email='airoboticshj@gmail.com',
    description='Demo nodes that drive the RH-P12-RN-A gripper on a Doosan E0509',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_demo = e0509_gripper_demo.gripper_demo_node:main',
            'ex01_gripper_sequence = e0509_gripper_demo.gripper_demo_node:main',
            'ex02_move_joint = e0509_gripper_demo.ex02_move_joint:main',
            'ex03_move_line = e0509_gripper_demo.ex03_move_line:main',
            'ex04_pick_and_place = e0509_gripper_demo.ex04_pick_and_place:main',
            'ex05_state_monitor = e0509_gripper_demo.ex05_state_monitor:main',
        ],
    },
)
