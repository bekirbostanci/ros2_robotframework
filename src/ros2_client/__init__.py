"""
ROS2 Robot Framework Library

A comprehensive Robot Framework library for interacting with ROS2 using both CLI and native operations.
"""

from .cli_client import ROS2CLIClient
from .native_client import ROS2NativeClient
from .ros2_client import ROS2ClientLibrary
from .utils import ROS2BaseClient, ROS2CLIUtils

__version__ = "0.1.0"
__all__ = [
    "ROS2ClientLibrary",  # ROS2 client (recommended)
    "ROS2CLIClient",  # CLI-only client
    "ROS2NativeClient",  # Native-only client
    "ROS2BaseClient",  # Base class
    "ROS2CLIUtils",  # CLI utilities
]
