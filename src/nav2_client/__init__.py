"""
Navigation2 Robot Framework Library

A comprehensive Robot Framework library for interacting with Navigation2 stack.
Provides keywords for navigation, path planning, localization, and costmap operations.
"""

from .nav2_client import Nav2ClientLibrary
from .cli_client import Nav2CLIClient
from .native_client import Nav2NativeClient
from .utils import Nav2BaseClient, Pose, NavigationResult

__version__ = "0.1.0"
__all__ = [
    "Nav2ClientLibrary",  # Navigation2 client (recommended)
    "Nav2CLIClient",  # CLI-only client
    "Nav2NativeClient",  # Native-only client
    "Nav2BaseClient",  # Base class
    "Pose",  # Pose data class
    "NavigationResult",  # Navigation result data class
]
