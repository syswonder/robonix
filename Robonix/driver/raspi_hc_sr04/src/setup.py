# SPDX-License-Identifier: MIT
# Copyright (c) 2025 whetfox
from setuptools import find_packages, setup
import os

package_name = "raspi_hc_sr04"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), ["sensors.yml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheatfox",
    maintainer_email="wheatfox17@icloud.com",
    description="HC-SR04 ultrasonic sensor driver for Raspberry Pi",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hc_sr04_node = raspi_hc_sr04.node:main",
        ],
    },
)
