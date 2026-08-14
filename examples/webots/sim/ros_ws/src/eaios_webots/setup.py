# SPDX-License-Identifier: MulanPSL-2.0

from glob import glob
from pathlib import Path

from setuptools import setup

package_name = 'eaios_webots'


def packaged_resource_files(*root_names):
    """Preserve generated resource subdirectories in the ROS share tree."""
    entries = []
    for root_name in root_names:
        root = Path('resource') / root_name
        if not root.is_dir():
            continue
        directories = [
            root,
            *sorted(path for path in root.rglob('*') if path.is_dir()),
        ]
        for directory in directories:
            files = sorted(str(path) for path in directory.iterdir() if path.is_file())
            if files:
                entries.append((str(Path('share') / package_name / directory), files))
    return entries

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),

    ('share/' + package_name + '/resource', [
        'resource/tiago_webots.urdf',
        'resource/tiago_full_webots.urdf',
        'resource/ros2_control.yml',
        'resource/TIAGO_VISUALS_LICENSE-APACHE-2.0.txt',
        'resource/TIAGO_VISUALS_SOURCES.md',
        'resource/tiago_visuals.manifest.json',
    ]),
] + packaged_resource_files('meshes', 'textures')

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
