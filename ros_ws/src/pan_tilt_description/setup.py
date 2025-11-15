import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pan_tilt_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name,'urdf', 'meshes', 'stls'), glob('urdf/meshes/stls/*.stl')),
        (os.path.join('share', package_name,'urdf', 'meshes', 'stls'), glob('urdf/meshes/stls/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='me@me.com',
    description='Pan tilt mechanism description with joystick teleop control for ROS2 Jazzy and Gazebo Harmonic',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pan_tilt_teleop = pan_tilt_description.pan_tilt_teleop:main',
        ],
    },
)
