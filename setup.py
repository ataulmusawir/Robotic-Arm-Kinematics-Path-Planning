from setuptools import setup
import os
from glob import glob

package_name = 'arm_kinematics'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ata Ul Musawir',
    maintainer_email='ata.musawir@giki.edu.pk',
    description='Project 1 6-DOF Kinematics Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_path_planner = arm_kinematics.ik_path_planner:main',
        ],
    },
)
