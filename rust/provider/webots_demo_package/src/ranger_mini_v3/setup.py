from setuptools import find_packages, setup

package_name = 'ranger_mini_v3'

data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/ranger_mini_v3_webots.urdf',
    'resource/ros2_control.yml',
    'resource/default.rviz',
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/default.wbt',
    'worlds/test1.wbt',
    'worlds/RangerMiniV3.proto',
    'worlds/ranger_base.dae',
    'worlds/steering_wheel.dae',
    'worlds/wheel_v3.dae',
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='hvisor',
    maintainer_email='hvisor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
            'camera_info_publisher = ranger_mini_v3.camera_info_publisher:main',
        ],
    }
)
