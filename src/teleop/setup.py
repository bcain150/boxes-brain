from setuptools import find_packages, setup

package_name = "teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "boxes_interfaces", "boxes_utils", "evdev"],
    zip_safe=True,
    maintainer="bcain",
    maintainer_email="bcain150@gmail.com",
    description="Implements communication with xbox controller over Game Input Protocol using xone kernel driver",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["xbox_control_interface = teleop.xbox_interface_node:main"],
    },
)
