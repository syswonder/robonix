from setuptools import setup

package_name = 'eaios_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/office.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/tiago_webots.urdf','resource/ros2_control.yml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robonix',
    maintainer_email='dev@robonix',
    description='Webots + ros2_control launch for Tiago-style office world',
    license='MulanPSL-2.0',
    tests_require=['pytest'],
)
