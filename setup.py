from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lh_demo_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='alfchr21@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = lh_demo_commander.commander:main',
            "keypoint_estimator = lh_demo_commander.keypoint_estimator:main",
            "emulate_keypoints = lh_demo_commander.emulate_keypoints:main",
            "keypoint_subscriber = lh_demo_commander.keypoint_subscriber:main",
        ],
    },
)
