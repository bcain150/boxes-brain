from setuptools import find_packages, setup
import os
from glob import glob

package_name = "entry_point"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'),
         glob("launch/*.launch.py"))
    ],
    install_requires=["setuptools", "speed", "teleop"],
    zip_safe=True,
    maintainer="bcain",
    maintainer_email="bcain150@gmail.com",
    description="Entry point and launch for boxes the robot",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
