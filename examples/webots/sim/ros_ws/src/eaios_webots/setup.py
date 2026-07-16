from setuptools import setup
from glob import glob
import os

package_name = 'eaios_webots'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),

    ('share/' + package_name + '/resource', [
        'resource/tiago_webots.urdf',
        'resource/ros2_control.yml',
    ]),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robonix',
    maintainer_email='dev@robonix',
    description='Webots + ros2_control launch for Tiago-style worlds',
    license='MulanPSL-2.0',
    tests_require=['pytest'],
)