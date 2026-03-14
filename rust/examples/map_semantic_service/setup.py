from setuptools import find_packages, setup

package_name = "map_semantic_service"

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
    description="Semantic map query service",
    license="MulanPSL-2.0",
    entry_points={
        "console_scripts": [
            "semantic_server = map_semantic_service.semantic_server:main",
        ],
    },
)
