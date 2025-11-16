from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pan_tilt_track'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/launch', ['launch/person_tracker.launch.py']),
        ('share/' + package_name + '/launch', ['launch/leaf_tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='me@me.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_tracker = pan_tilt_track.person_tracker:main',
            'leaf_tracker = pan_tilt_track.leaf_tracker:main',
            'leaf_track_status = pan_tilt_track.leaf_track_status:main',
        ],
    },
)
