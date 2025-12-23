from setuptools import find_packages, setup

package_name = 'demo_rgb_provider'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Demo RGB camera package that outputs random color images',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rgb_publisher = demo_rgb_provider.rgb_publisher:main',
            'grasp_move = demo_rgb_provider.grasp_move:main',
            'pick_skill = demo_rgb_provider.pick_skill:main',
            'update_map_skill = demo_rgb_provider.update_map_skill:main',
        ],
    },
)

