#!/usr/bin/env python3

from setuptools import find_packages, setup


setup(
    name="fruc_ros_utils",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    entry_points={
        "console_scripts": [
            "bagutils=bag.bagbridge:main",
            "ros1utils=bag.ros1utils:main",
            "ros2utils=bag.ros2utils:main",
        ],
    },
)
