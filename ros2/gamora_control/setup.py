from setuptools import setup
import os
from glob import glob

package_name = 'gamora_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdu7rahman',
    maintainer_email='mohammedabdulr.1@northeastern.edu',
    description='Motion nodes for the gamora arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_pose = gamora_control.move_to_pose:main',
            'joint_state_demo = gamora_control.joint_state_demo:main',
            'ik_pose_follower = gamora_control.ik_pose_follower:main',
        ],
    },
)
