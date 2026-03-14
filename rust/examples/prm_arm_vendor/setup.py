from setuptools import find_packages, setup

package_name = "prm_arm_vendor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "grpcio"],
    zip_safe=True,
    maintainer="robonix",
    maintainer_email="wheatfox17@icloud.com",
    description="Example arm vendor: prm::arm and prm::gripper",
    license="MulanPSL-2.0",
    entry_points={
        "console_scripts": [
            "arm_node = prm_arm_vendor.arm_node:main",
        ],
    },
)
