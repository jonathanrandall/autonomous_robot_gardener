from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_sync_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='me@me.com',
    description='ROS2 package for synchronizing and processing webcam and ToF compressed images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_sync_node = image_sync_processor.image_sync_node:main',
        ],
    },
)
