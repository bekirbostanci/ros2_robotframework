"""
Common utilities and base classes for ROS2 Robot Framework Library
"""

import subprocess
import time
from typing import Any, Dict, List, Optional

from robot.api import logger
from robot.api.deco import keyword


class ROS2BaseClient:
    """Base class with common utilities for ROS2 operations."""

    ROBOT_LIBRARY_SCOPE = "GLOBAL"
    ROBOT_LIBRARY_DOC_FORMAT = "ROBOT"

    def __init__(self, timeout: float = 10.0):
        """Initialize base client with common settings."""
        self.timeout = timeout
        self._ros2_executable = self._find_ros2_executable()
        self._initialized = False

    def _find_ros2_executable(self) -> str:
        """Find the ROS2 executable path."""
        try:
            result = subprocess.run(
                ["which", "ros2"], capture_output=True, text=True, timeout=5.0
            )
            if result.returncode == 0:
                return result.stdout.strip()
            else:
                return "ros2"  # Fallback to assuming it's in PATH
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return "ros2"  # Fallback to assuming it's in PATH

    def _run_ros2_command(
        self,
        command: List[str],
        timeout: Optional[float] = None,
        capture_output: bool = True,
    ) -> subprocess.CompletedProcess:
        """Run a ROS2 command and return the result."""
        full_command = [self._ros2_executable] + command
        timeout_value = timeout or self.timeout

        logger.info(f"Running ROS2 command: {' '.join(full_command)}")

        try:
            result = subprocess.run(
                full_command,
                capture_output=capture_output,
                text=True,
                timeout=timeout_value,
                check=False,
            )

            logger.debug(f"Command return code: {result.returncode}")
            if result.stdout:
                logger.debug(f"Command stdout: {result.stdout}")
            if result.stderr:
                logger.debug(f"Command stderr: {result.stderr}")

            return result

        except subprocess.TimeoutExpired:
            logger.error(
                f"ROS2 command timed out after {timeout_value}s: {' '.join(full_command)}"
            )
            raise
        except Exception as e:
            logger.error(f"Failed to run ROS2 command: {e}")
            raise


class ROS2CLIUtils(ROS2BaseClient):
    """CLI-based ROS2 operations using subprocess calls."""

    def __init__(self, timeout: float = 10.0):
        """Initialize CLI utils."""
        super().__init__(timeout)
        self._initialized = True

    # ============================================================================
    # TOPIC OPERATIONS - REMOVED
    # ============================================================================
    # Topic operations have been moved to native_client.py for better performance
    # and reliability. Use the native implementations instead.

    # ============================================================================
    # SERVICE OPERATIONS
    # ============================================================================

    @keyword
    def list_services(self, timeout: Optional[float] = None) -> List[str]:
        """
        List all available ROS2 services.

        Args:
            timeout: Override default timeout for this operation

        Returns:
            List of service names

        Example:
            | ${services}= | List Services |
            | Should Contain | ${services} | /add_two_ints |
        """
        result = self._run_ros2_command(["service", "list"], timeout=timeout)

        if result.returncode != 0:
            raise RuntimeError(f"Failed to list services: {result.stderr}")

        services = [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]
        logger.info(f"Found {len(services)} services: {services}")
        return services

    # ============================================================================
    # NODE OPERATIONS
    # ============================================================================

    @keyword
    def list_nodes(self, timeout: Optional[float] = None) -> List[str]:
        """
        List all running ROS2 nodes.

        Args:
            timeout: Override default timeout for this operation

        Returns:
            List of node names

        Example:
            | ${nodes}= | List Nodes |
            | Should Contain | ${nodes} | /talker |
        """
        result = self._run_ros2_command(["node", "list"], timeout=timeout)

        if result.returncode != 0:
            raise RuntimeError(f"Failed to list nodes: {result.stderr}")

        nodes = [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]
        logger.info(f"Found {len(nodes)} nodes: {nodes}")
        return nodes

    @keyword
    def get_node_info(
        self, node_name: str, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Get detailed information about a specific node.

        Args:
            node_name: Name of the node to get info for
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing node information (namespace, topics, services, etc.)

        Example:
            | ${info}= | Get Node Info | /talker |
            | Should Contain | ${info}[topics] | /chatter |
        """
        result = self._run_ros2_command(["node", "info", node_name], timeout=timeout)

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to get node info for '{node_name}': {result.stderr}"
            )

        info = {"name": node_name}
        lines = result.stdout.strip().split("\n")
        current_section = None

        for line in lines:
            original_line = line
            line = line.strip()

            if line.startswith("Node name:"):
                info["name"] = line.split(":", 1)[1].strip()
            elif line.startswith("Node namespace:"):
                info["namespace"] = line.split(":", 1)[1].strip()
            elif line.startswith("Node namespace: /"):
                info["namespace"] = line.split(":", 1)[1].strip()
            elif line.startswith("Node namespace: (no namespace)"):
                info["namespace"] = "/"
            elif line.startswith("Subscribers:"):
                current_section = "subscribers"
                info["subscribers"] = []
            elif line.startswith("Publishers:"):
                current_section = "publishers"
                info["publishers"] = []
            elif line.startswith("Service Servers:"):
                current_section = "service_servers"
                info["service_servers"] = []
            elif line.startswith("Service Clients:"):
                current_section = "service_clients"
                info["service_clients"] = []
            elif line.startswith("Action Servers:"):
                current_section = "action_servers"
                info["action_servers"] = []
            elif line.startswith("Action Clients:"):
                current_section = "action_clients"
                info["action_clients"] = []
            elif line.startswith("Parameters:"):
                current_section = "parameters"
                info["parameters"] = []
            elif original_line.startswith("  ") and line:  # Indented lines with content
                if current_section and line:
                    # Extract topic/service name and type
                    if ": " in line:
                        name, msg_type = line.split(": ", 1)
                        info[current_section].append(
                            {"name": name.strip(), "type": msg_type.strip()}
                        )
                    else:
                        info[current_section].append(line.strip())

        logger.info(f"Node info for '{node_name}': {info}")
        return info

    @keyword
    def node_exists(self, node_name: str, timeout: Optional[float] = None) -> bool:
        """
        Check if a node exists.

        Args:
            node_name: Name of the node to check
            timeout: Override default timeout for this operation

        Returns:
            True if node exists, False otherwise

        Example:
            | ${exists}= | Node Exists | /talker |
            | Should Be True | ${exists} |
        """
        try:
            nodes = self.list_nodes(timeout=timeout)
            exists = node_name in nodes
            logger.info(f"Node '{node_name}' exists: {exists}")
            return exists
        except Exception as e:
            logger.error(f"Error checking if node '{node_name}' exists: {e}")
            return False

    @keyword
    def wait_for_node(
        self, node_name: str, timeout: float = 30.0, check_interval: float = 1.0
    ) -> bool:
        """
        Wait for a node to become available.

        Args:
            node_name: Name of the node to wait for
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds

        Returns:
            True if node becomes available within timeout

        Example:
            | ${available}= | Wait For Node | /talker | timeout=10.0 |
            | Should Be True | ${available} |
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.node_exists(node_name, timeout=check_interval):
                logger.info(
                    f"Node '{node_name}' became available after {time.time() - start_time:.2f}s"
                )
                return True
            time.sleep(check_interval)

        logger.warn(f"Node '{node_name}' did not become available within {timeout}s")
        return False
