# SPDX-License-Identifier: MulanPSL-2.0
# Setup Script for Demo Service Provider Package
#
# Setup script for demo_service_provider ROS2 package

from setuptools import find_packages, setup

package_name = 'demo_service_provider'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'python-dotenv',
        'openai>=1.0.0',
        'cv-bridge',
        'numpy',
        'Pillow',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='demo@demo.demo',
    description='Demo service provider package with semantic_map and task_plan services',
    license='MulanPSL-2.0',
    entry_points={
        'console_scripts': [
            'semantic_map_service = demo_service_provider.semantic_map_service:main',
            'task_plan_service = demo_service_provider.task_plan_service:main',
        ],
    },
)

