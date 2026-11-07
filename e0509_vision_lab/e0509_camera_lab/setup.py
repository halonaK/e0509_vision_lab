import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'e0509_camera_lab'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airoboticshj',
    maintainer_email='airoboticshj@gmail.com',
    description='Eye-to-hand camera lab for Doosan E0509 (virtual RealSense, calibration, vision pick and place)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'virtual_camera = e0509_camera_lab.virtual_camera_node:main',
            'calib_tf_publisher = e0509_camera_lab.calib_tf_publisher:main',
            'click_to_base = e0509_camera_lab.click_to_base_node:main',
            'eye_to_hand_calib = e0509_camera_lab.eye_to_hand_calib_node:main',
        ],
    },
)
