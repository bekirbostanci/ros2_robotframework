"""
Main ROS2 client that combines CLI and native operations
"""

import time
from typing import List, Dict, Any, Optional, Union
from robot.api.deco import keyword
from robot.api import logger

from .utils import ROS2BaseClient
from .cli_client import ROS2CLIClient
from .native_client import ROS2NativeClient


class ROS2ClientLibrary(ROS2BaseClient):
    """
    Main ROS2 client that automatically chooses between CLI and native operations.
    
    This is the primary client that users should use. It provides a unified interface
    that automatically uses the most appropriate method (CLI or native) for each operation.
    """
    
    def __init__(self, timeout: float = 10.0, node_name: str = "robotframework_ros2"):
        """
        Initialize main client.
        
        Args:
            timeout: Default timeout for operations
            node_name: Name for the native ROS2 node
        """
        super().__init__(timeout)
        
        # Initialize both clients
        self.cli_client = ROS2CLIClient(timeout)
        
        # Initialize native client
        self.native_client = ROS2NativeClient(timeout, node_name)
        logger.info("Main client initialized with both CLI and native support")
    
    # ============================================================================
    # TOPIC OPERATIONS (Smart Selection)
    # ============================================================================
    
    @keyword
    def list_topics(self, timeout: Optional[float] = None) -> List[str]:
        """List all available ROS2 topics (always uses CLI)."""
        return self.cli_client.list_topics(timeout)
    
    @keyword
    def get_topic_info(self, topic_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Get detailed information about a topic (always uses CLI)."""
        return self.cli_client.get_topic_info(topic_name, timeout)
    
    @keyword
    def create_publisher(self, topic_name: str, message_type: str = "std_msgs/msg/String") -> str:
        """
        Create a publisher (uses native if available, otherwise CLI).
        
        Args:
            topic_name: Name of the topic
            message_type: Type of the message
            
        Returns:
            Publisher ID or topic name
            
        Example:
            | ${publisher}= | Create Publisher | /chatter | std_msgs/msg/String |
            | Publish Message | ${publisher} | Hello World |
        """
        return self.native_client.create_publisher(topic_name, message_type)
    
    @keyword
    def publish_message(self, publisher_id: str, data: Any) -> bool:
        """
        Publish a message (uses native if available, otherwise CLI).
        
        Args:
            publisher_id: Publisher ID or topic name
            data: Message data
            
        Returns:
            True if successful
            
        Example:
            | ${publisher}= | Create Publisher | /chatter |
            | Publish Message | ${publisher} | Hello World |
        """
        return self.native_client.publish_message(publisher_id, data)
    
    @keyword
    def create_subscriber(self, topic_name: str, message_type: str = "std_msgs/msg/String") -> str:
        """
        Create a subscriber (uses native if available, otherwise CLI).
        
        Args:
            topic_name: Name of the topic
            message_type: Type of the message
            
        Returns:
            Subscriber ID or topic name
            
        Example:
            | ${subscriber}= | Create Subscriber | /chatter | std_msgs/msg/String |
            | ${message}= | Get Latest Message | /chatter |
        """
        return self.native_client.create_subscriber(topic_name, message_type)
    
    @keyword
    def get_latest_message(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Get the latest message (uses native if available, otherwise CLI).
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            Latest message data or None
            
        Example:
            | ${subscriber}= | Create Subscriber | /chatter |
            | ${message}= | Get Latest Message | /chatter |
        """
        return self.native_client.get_latest_message(topic_name)
    
    @keyword
    def echo_topic(self, topic_name: str, count: int = 1, timeout: Optional[float] = None) -> List[str]:
        """Echo messages from a topic (always uses CLI)."""
        return self.cli_client.echo_topic(topic_name, count, timeout)
    
    @keyword
    def topic_exists(self, topic_name: str, timeout: Optional[float] = None) -> bool:
        """Check if a topic exists (always uses CLI)."""
        return self.cli_client.topic_exists(topic_name, timeout)
    
    @keyword
    def wait_for_topic(self, topic_name: str, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """Wait for a topic to become available (always uses CLI)."""
        return self.cli_client.wait_for_topic(topic_name, timeout, check_interval)

    # ============================================================================
    # SERVICE OPERATIONS (Smart Selection)
    # ============================================================================
    
    @keyword
    def get_action_list(self) -> List[str]:
        """List all available actions (always uses CLI)."""
        return self.cli_client.get_action_list()

    @keyword
    def list_services(self, timeout: Optional[float] = None) -> List[str]:
        """List all available services (always uses CLI)."""
        return self.cli_client.list_services(timeout)
    
    @keyword
    def get_service_info(self, service_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Get detailed information about a service (always uses CLI)."""
        return self.cli_client.get_service_info(service_name, timeout)
    
    @keyword
    def call_service(self, service_name: str, service_type: str, request_data: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Call a service (always uses CLI for now)."""
        return self.cli_client.call_service(service_name, service_type, request_data, timeout)
    
    @keyword
    def service_exists(self, service_name: str, timeout: Optional[float] = None) -> bool:
        """Check if a service exists (always uses CLI)."""
        return self.cli_client.service_exists(service_name, timeout)
    
    @keyword
    def wait_for_service(self, service_name: str, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """Wait for a service to become available (always uses CLI)."""
        return self.cli_client.wait_for_service(service_name, timeout, check_interval)

    # ============================================================================
    # NODE OPERATIONS (Always CLI)
    # ============================================================================
    
    @keyword
    def kill_process_by_name(self, process_name: str) -> bool:
        """Kill a process by name (always uses CLI)."""
        return self.cli_client.pkill_process(process_name)

    @keyword
    def has_running_nodes(self, timeout: float = 30.0) -> bool:
        """Check if there are any nodes running."""
        return self.cli_client.has_running_nodes(timeout)

    @keyword
    def list_nodes(self, timeout: Optional[float] = None) -> List[str]:
        """List all running nodes (always uses CLI)."""
        return self.cli_client.list_nodes(timeout)
    
    @keyword
    def get_node_info(self, node_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Get node information (always uses CLI)."""
        return self.cli_client.get_node_info(node_name, timeout)
    
    @keyword
    def node_exists(self, node_name: str, timeout: Optional[float] = None) -> bool:
        """Check if a node exists (always uses CLI)."""
        return self.cli_client.node_exists(node_name, timeout)
    
    @keyword
    def wait_for_node(self, node_name: str, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """Wait for a node to become available (always uses CLI)."""
        return self.cli_client.wait_for_node(node_name, timeout, check_interval)

    # ============================================================================
    # PARAMETER OPERATIONS (Native Only)
    # ============================================================================
    
    @keyword
    def list_parameters(self) -> List[str]:
        """List all parameters for the native node (native only)."""
        if self.native_client:
            return self.native_client.list_parameters()
        else:
            logger.warn("Native client not available, cannot list parameters")
            return []
            
    @keyword
    def get_parameter(self, parameter_name: str, default_value: Any = None) -> Any:
        """Get a parameter from the native node (native only)."""
        if self.native_client:
            return self.native_client.get_parameter(parameter_name, default_value)
        else:
            logger.warn("Native client not available, cannot get parameter")
            return default_value
    
    @keyword
    def set_parameter(self, parameter_name: str, value: Union[str, int, float, bool]) -> bool:
        """Set a parameter on the native node (native only)."""
        if self.native_client:
            return self.native_client.set_parameter(parameter_name, value)
        else:
            logger.warn("Native client not available, cannot set parameter")
            return False
    
    @keyword
    def parameter_exists(self, parameter_name: str) -> bool:
        """Check if a parameter exists on the native node (native only)."""
        if self.native_client:
            return self.native_client.parameter_exists(parameter_name)
        else:
            logger.warn("Native client not available, cannot check parameter existence")
            return False
    
    @keyword
    def get_all_parameters(self) -> Dict[str, Any]:
        """Get all parameters from the native node (native only)."""
        if self.native_client:
            return self.native_client.get_all_parameters()
        else:
            logger.warn("Native client not available, cannot get all parameters")
            return {}

    # ============================================================================
    # LAUNCH OPERATIONS (Always CLI)
    # ============================================================================
    
    @keyword
    def launch_file(self, launch_file_path: str, arguments: Optional[Dict[str, str]] = None, timeout: Optional[float] = None):
        """Launch a launch file (always uses CLI)."""
        return self.cli_client.launch_file(launch_file_path, arguments, timeout)
    
    @keyword
    def launch_package(self, package_name: str, launch_file_name: str, arguments: Optional[Dict[str, str]] = None, timeout: Optional[float] = None):
        """Launch a package (always uses CLI)."""
        return self.cli_client.launch_package(package_name, launch_file_name, arguments, timeout)
    
    @keyword
    def find_launch_files(self, package_name: str, timeout: Optional[float] = None) -> List[str]:
        """Find launch files in a package (always uses CLI)."""
        return self.cli_client.find_launch_files(package_name, timeout)
    
    @keyword
    def wait_for_launch_completion(self, process, timeout: float = 30.0) -> bool:
        """Wait for launch completion (always uses CLI)."""
        return self.cli_client.wait_for_launch_completion(process, timeout)
    
    @keyword
    def terminate_launch_process(self, process, force: bool = False) -> bool:
        """Terminate launch process (always uses CLI)."""
        return self.cli_client.terminate_launch_process(process, force)

    # ============================================================================
    # RUN OPERATIONS (Always CLI)
    # ============================================================================
    
    @keyword
    def run_node(self, package_name: str, executable_name: str, arguments: Optional[List[str]] = None, timeout: Optional[float] = None):
        """Run a node (always uses CLI)."""
        return self.cli_client.run_node(package_name, executable_name, arguments, timeout)
    
    @keyword
    def run_node_with_remap(self, package_name: str, executable_name: str, remaps: Optional[Dict[str, str]] = None, arguments: Optional[List[str]] = None, timeout: Optional[float] = None):
        """Run a node with remapping (always uses CLI)."""
        return self.cli_client.run_node_with_remap(package_name, executable_name, remaps, arguments, timeout)
    
    @keyword
    def find_executables(self, package_name: str, timeout: Optional[float] = None) -> List[str]:
        """Find executables in a package (always uses CLI)."""
        return self.cli_client.find_executables(package_name, timeout)
    
    @keyword
    def wait_for_node_completion(self, process, timeout: float = 30.0) -> bool:
        """Wait for node completion (always uses CLI)."""
        return self.cli_client.wait_for_node_completion(process, timeout)
    
    @keyword
    def terminate_node_process(self, process, force: bool = False) -> bool:
        """Terminate node process (always uses CLI)."""
        return self.cli_client.terminate_node_process(process, force)
    
    @keyword
    def get_process_output(self, process, timeout: float = 1.0) -> Dict[str, str]:
        """Get process output (always uses CLI)."""
        return self.cli_client.get_process_output(process, timeout)
    
    @keyword
    def is_process_running(self, process) -> bool:
        """Check if process is running (always uses CLI)."""
        return self.cli_client.is_process_running(process)
    
    @keyword
    def shutdown_process(self, process_name: str, force: bool = False) -> bool:
        """Shutdown a process by name (always uses CLI)."""
        return self.cli_client.shutdown_process(process_name, force)

    # ============================================================================
    # NATIVE-SPECIFIC OPERATIONS
    # ============================================================================
    
    @keyword
    def get_all_messages(self, topic_name: str) -> List[Dict[str, Any]]:
        """Get all buffered messages from a subscribed topic (native only)."""
        if self.native_client:
            return self.native_client.get_all_messages(topic_name)
        else:
            logger.warn("Native client not available, cannot get all messages")
            return []
    
    @keyword
    def clear_message_buffer(self, topic_name: str) -> bool:
        """Clear message buffer for a topic (native only)."""
        if self.native_client:
            return self.native_client.clear_message_buffer(topic_name)
        else:
            logger.warn("Native client not available, cannot clear message buffer")
            return False
    
    @keyword
    def wait_for_message(self, topic_name: str, timeout: float = 10.0, check_interval: float = 0.1) -> Optional[Dict[str, Any]]:
        """Wait for a message on a subscribed topic (native only)."""
        if self.native_client:
            return self.native_client.wait_for_message(topic_name, timeout, check_interval)
        else:
            logger.warn("Native client not available, cannot wait for message")
            return None
    
    
    @keyword
    def declare_parameter(self, parameter_name: str, default_value: Any) -> bool:
        """Declare a parameter with default value on the native node (native only)."""
        if self.native_client:
            return self.native_client.declare_parameter(parameter_name, default_value)
        else:
            logger.warn("Native client not available, cannot declare parameter")
            return False

    @keyword
    def get_transform(self, target_frame: str, source_frame: str, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """Get transform between two frames using tf2 (native only)."""
        if self.native_client:
            return self.native_client.get_tf(target_frame, source_frame, timeout)
        else:
            logger.warn("Native client not available, cannot get transform")
            return None

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================
    
    @keyword
    def cleanup(self):
        """Clean up all resources."""
        if self.native_client:
            self.native_client.cleanup()
        logger.info("Main client cleanup completed")
    
    @keyword
    def get_client_info(self) -> Dict[str, Any]:
        """Get information about the current client configuration."""
        info = {
            'native_available': self.native_client is not None,
            'timeout': self.timeout,
            'ros2_executable': self._ros2_executable
        }
        
        if self.native_client:
            info['native_info'] = self.native_client.get_client_info()
        
        return info
    


    @keyword
    def is_within_tolerance(
        self,
        transform_data: dict[str, object],
        tolerance: float,
        target_x: float,
        target_y: float,
        target_z: float = 0.0,
    ) -> bool:
        """
        Check if a position (from transform data) is within a given tolerance of a target point.

        Args:
            transform_data: Transform data dictionary from get_transform method.
            target_x: Target X coordinate.
            target_y: Target Y coordinate.
            target_z: Target Z coordinate (default: 0.0).
            tolerance: Maximum allowed distance from target.

        Returns:
            True if position is within tolerance of target, False otherwise.

        Example:
            | ${transform}= | Get Transform | map | base_link | duration=1.0 |
            | ${within}= | Is Within Tolerance | ${transform} | 2.0 | 3.0 | 0.0 | 0.5 |
            | Should Be True | ${within} |
        """
        try:
            if "translation" not in transform_data:
                raise ValueError("Transform data must contain 'translation' field")

            current_pos = transform_data["translation"]
            current_x = current_pos["x"]
            current_y = current_pos["y"]
            current_z = current_pos.get("z", 0.0)

            from math import sqrt

            distance = sqrt(
                (target_x - current_x) ** 2
                + (target_y - current_y) ** 2
                + (target_z - current_z) ** 2
            )

            within = distance <= tolerance

            logger.info(
                f"Position tolerance check: within={within}, distance={distance:.3f}m, tolerance={tolerance}m"
            )
            logger.info(
                f"Current position: ({current_x:.3f}, {current_y:.3f}, {current_z:.3f})"
            )
            logger.info(
                f"Target position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
            )

            return within

        except Exception as e:
            logger.error(f"Failed to check position tolerance: {e}")
            raise