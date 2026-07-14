from setuptools import find_packages, setup


package_name = "pico_teleop_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pico_teleop.launch.py"]),
        ("share/" + package_name + "/config", ["config/default.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="PICO ROS 2 to Isaac Lab bridge",
    license="BSD-3-Clause",
    entry_points={"console_scripts": ["bridge = pico_teleop_bridge.node:main"]},
)

