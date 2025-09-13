#!/usr/bin/env python3
"""
Setup script for ROS2 Robot Framework Library
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read the README file
readme_file = Path(__file__).parent / "README.md"
long_description = (
    readme_file.read_text(encoding="utf-8") if readme_file.exists() else ""
)

# Read requirements
requirements_file = Path(__file__).parent / "requirements.txt"
requirements = []
if requirements_file.exists():
    requirements = requirements_file.read_text(encoding="utf-8").strip().split("\n")
    requirements = [
        req.strip() for req in requirements if req.strip() and not req.startswith("#")
    ]

setup(
    name="ros2-robotframework",
    version="0.1.0",
    author="ROS2 Robot Framework Team",
    author_email="",
    description="A comprehensive Robot Framework library for ROS2 CLI operations",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/your-org/ros2-robotframework",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Software Development :: Testing",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "black>=21.0",
            "flake8>=3.8",
            "mypy>=0.800",
        ],
        "nav2": [
            "nav2-msgs",
            "nav2-common",
        ],
        "behaviour-tree": [
            "behaviortree-cpp-v3",
        ],
    },
    entry_points={
        "robotframework_libraries": [
            "ROS2ClientLibrary = ros2_client.ros2_client:ROS2ClientLibrary",
        ],
    },
    include_package_data=True,
    zip_safe=False,
)
