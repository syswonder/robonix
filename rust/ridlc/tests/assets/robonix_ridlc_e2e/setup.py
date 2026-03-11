from setuptools import find_packages, setup


package_name = "robonix_ridlc_e2e"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robonix",
    maintainer_email="wheatfox17@icloud.com",
    description="End-to-end test nodes for RIDL-generated interfaces.",
    license="MulanPSL-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "combined_runtime = robonix_ridlc_e2e.combined_runtime:main",
            "query_server = robonix_ridlc_e2e.query_server:main",
            "query_client = robonix_ridlc_e2e.query_client:main",
            "runtime_probe = robonix_ridlc_e2e.runtime_probe:main",
            "stream_publisher = robonix_ridlc_e2e.stream_publisher:main",
            "stream_subscriber = robonix_ridlc_e2e.stream_subscriber:main",
            "command_server = robonix_ridlc_e2e.command_server:main",
            "command_client = robonix_ridlc_e2e.command_client:main",
        ],
    },
)
