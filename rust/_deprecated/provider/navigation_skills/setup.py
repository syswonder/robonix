from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'navigation_skills_provider'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['rbnx_manifest.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='demo@demo.demo',
    description='Navigation skills package providing wandering and move_to_object skills',
    license='MulanPSL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wandering_skill = navigation_skills_provider.wandering_skill:main',
            'move_to_object_skill = navigation_skills_provider.move_to_object_skill:main',
        ],
    },
)
