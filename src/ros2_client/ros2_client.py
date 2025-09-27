"""
Main ROS2 client that combines CLI and native operations for RobotFramework testing.

This module provides a unified interface for ROS2 operations in RobotFramework tests,
combining the power of CLI commands for discovery and monitoring with native ROS2
operations for real-time communication.

Key Features:
- Topic operations (publish, subscribe, monitor, discover)
- Service operations (call, monitor, discover)
- Action operations (send goals, monitor, discover)
- Node operations (list, info, monitor)
- Parameter operations (get, set, list, monitor)
- Launch operations (launch files, packages)
- Run operations (execute nodes, manage processes)
- Transform operations (tf2 support)
- Utility operations (cleanup, info, validation)
"""

from typing import Any, Dict, List, Optional, Union

from robot.api import logger
from robot.api.deco import keyword

from .cli_client import ROS2CLIClient
from .native_client import ROS2NativeClient
from .utils import ROS2BaseClient


class ROS2ClientLibrary(ROS2BaseClient):
    """
    Main ROS2 client that combines CLI and native operations for RobotFramework testing.

    This is the primary client that users should use. It provides a unified interface
    that intelligently combines:
    - Native operations for all topic operations, real-time communication, and performance-critical tasks
    - CLI operations for system management, launch operations, and process management

    The client automatically selects the best approach for each operation:
    - Native operations for all topic operations, publishers, subscribers, and service clients (better performance)
    - CLI operations for system management, launch operations, and process management (more reliable)

    Example:
        | Library | ROS2ClientLibrary | timeout=10.0 | node_name=test_robot |
        | ${topics}= | List Topics |
        | ${publisher}= | Create Publisher | /chatter | std_msgs/msg/String |
        | Publish Message | ${publisher} | Hello World |
    """

    def __init__(self, timeout: float = 10.0, node_name: str = "robotframework_ros2"):
        """
        Initialize the main ROS2 client.

        Args:
            timeout: Default timeout for operations in seconds
            node_name: Name for the native ROS2 node (must be unique)

        Example:
            | Library | ROS2ClientLibrary | timeout=15.0 | node_name=my_test_node |
        """
        super().__init__(timeout)

        # Initialize both clients
        self.cli_client = ROS2CLIClient(timeout)
        self.native_client = ROS2NativeClient(timeout, node_name)

        logger.info(
            f"ROS2ClientLibrary initialized with timeout={timeout}s, node_name='{node_name}'"
        )
        logger.info(
            "Available operations: topics, services, actions, nodes, parameters, launch, run, transforms"
        )

    @keyword
    def ros_doctor(self, timeout: float = 5.0) -> str:
        """
        Run the ROS2 doctor tool.

        Args:
            timeout: Timeout for the doctor tool

        Returns:
            Output of the doctor tool
        """
        return self.cli_client.ros_doctor(timeout)

    # ============================================================================
    # TOPIC OPERATIONS
    # ============================================================================
    # Topic operations provide comprehensive functionality for ROS2 topic communication
    # including discovery, monitoring, publishing, subscribing, and performance analysis.

    # --- Discovery and Information ---
    @keyword
    def list_topics(self, timeout: Optional[float] = None) -> List[str]:
        """
        List all available ROS2 topics.

        Uses native ROS2 operations for better performance.

        Args:
            timeout: Override default timeout for this operation

        Returns:
            List of topic names

        Example:
            | ${topics}= | List Topics |
            | Should Contain | ${topics} | /chatter |
        """
        return self.native_client.list_topics(timeout)

    @keyword
    def get_topic_info(
        self, topic_name: str, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Get detailed information about a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing topic information (type, publishers, subscribers)

        Example:
            | ${info}= | Get Topic Info | /chatter |
            | Log | Topic type: ${info['type']} |
        """
        return self.native_client.get_topic_info(topic_name, timeout)

    @keyword
    def get_topic_type(self, topic_name: str, timeout: Optional[float] = None) -> str:
        """
        Get the message type for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            timeout: Override default timeout for this operation

        Returns:
            Message type string (e.g., 'std_msgs/msg/String')

        Example:
            | ${type}= | Get Topic Type | /chatter |
            | Should Be Equal | ${type} | std_msgs/msg/String |
        """
        return self.native_client.get_topic_type(topic_name, timeout)

    @keyword
    def find_topics_by_type(
        self, message_type: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Find topics that use a specific message type.

        Uses native ROS2 operations for better performance.

        Args:
            message_type: Message type to search for (e.g., 'std_msgs/msg/String')
            timeout: Override default timeout for this operation

        Returns:
            List of topic names using the specified message type

        Example:
            | ${string_topics}= | Find Topics By Type | std_msgs/msg/String |
            | Should Contain | ${string_topics} | /chatter |
        """
        return self.native_client.find_topics_by_type(message_type, timeout)

    @keyword
    def topic_exists(self, topic_name: str, timeout: Optional[float] = None) -> bool:
        """
        Check if a topic exists.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic to check
            timeout: Override default timeout for this operation

        Returns:
            True if topic exists, False otherwise

        Example:
            | ${exists}= | Topic Exists | /chatter |
            | Should Be True | ${exists} |
        """
        return self.native_client.topic_exists(topic_name, timeout)

    @keyword
    def wait_for_topic(
        self, topic_name: str, timeout: float = 30.0, check_interval: float = 1.0
    ) -> bool:
        """
        Wait for a topic to become available.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic to wait for
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds

        Returns:
            True if topic becomes available, False if timeout

        Example:
            | ${available}= | Wait For Topic | /chatter | timeout=10.0 |
            | Should Be True | ${available} |
        """
        return self.native_client.wait_for_topic(topic_name, timeout, check_interval)

    # --- Publishing and Subscribing (Native Operations) ---
    @keyword
    def create_publisher(self, topic_name: str, message_type: str) -> str:
        """
        Create a publisher for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            message_type: Type of the message (e.g., 'std_msgs/msg/String')

        Returns:
            Publisher ID for use with publish_message

        Example:
            | ${publisher}= | Create Publisher | /chatter | std_msgs/msg/String |
            | Publish Message | ${publisher} | Hello World |
        """
        return self.native_client.create_publisher(topic_name, message_type)

    @keyword
    def publish_message(self, publisher_id: str, data: Any) -> bool:
        """
        Publish a message using a created publisher.

        Uses native ROS2 operations for better performance.

        Args:
            publisher_id: Publisher ID from create_publisher
            data: Message data to publish

        Returns:
            True if successful

        Example:
            | ${publisher}= | Create Publisher | /chatter | std_msgs/msg/String |
            | ${success}= | Publish Message | ${publisher} | Hello World |
            | Should Be True | ${success} |
        """
        return self.native_client.publish_message(publisher_id, data)

    @keyword
    def create_subscriber(self, topic_name: str, message_type: str) -> str:
        """
        Create a subscriber for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            message_type: Type of the message (e.g., 'std_msgs/msg/String')

        Returns:
            Subscriber ID for use with message retrieval

        Example:
            | ${subscriber}= | Create Subscriber | /chatter | std_msgs/msg/String |
            | ${message}= | Get Latest Message | /chatter |
        """
        return self.native_client.create_subscriber(topic_name, message_type)

    @keyword
    def get_latest_message(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Get the latest message from a subscribed topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic

        Returns:
            Latest message data or None if no message available

        Example:
            | ${subscriber}= | Create Subscriber | /chatter | std_msgs/msg/String |
            | ${message}= | Get Latest Message | /chatter |
            | Should Not Be None | ${message} |
        """
        return self.native_client.get_latest_message(topic_name)

    # --- Monitoring and Analysis (Native Operations) ---
    @keyword
    def echo_topic(
        self, topic_name: str, count: int = 1, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Echo messages from a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            count: Number of messages to capture (default: 1)
            timeout: Override default timeout for this operation

        Returns:
            List of message strings

        Example:
            | ${messages}= | Echo Topic | /chatter | count=5 |
            | Length Should Be | ${messages} | 5 |
        """
        return self.native_client.echo_topic(topic_name, count, timeout)

    @keyword
    def get_topic_frequency(self, topic_name: str, timeout: float = 10.0) -> float:
        """
        Get the message frequency for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            timeout: Time to measure frequency in seconds

        Returns:
            Average message frequency in Hz

        Example:
            | ${freq}= | Get Topic Frequency | /chatter | timeout=5.0 |
            | Should Be True | ${freq} > 0 |
        """
        return self.native_client.get_topic_frequency(topic_name, timeout)

    @keyword
    def get_topic_bandwidth(self, topic_name: str, timeout: float = 10.0) -> float:
        """
        Get the bandwidth usage for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            timeout: Time to measure bandwidth in seconds

        Returns:
            Average bandwidth in bytes per second

        Example:
            | ${bw}= | Get Topic Bandwidth | /chatter | timeout=5.0 |
            | Log | Bandwidth: ${bw} bytes/sec |
        """
        return self.native_client.get_topic_bandwidth(topic_name, timeout)

    @keyword
    def get_topic_delay(self, topic_name: str, timeout: float = 10.0) -> float:
        """
        Get the message delay for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            timeout: Time to measure delay in seconds

        Returns:
            Average message delay in seconds

        Example:
            | ${delay}= | Get Topic Delay | /chatter | timeout=5.0 |
            | Should Be True | ${delay} >= 0 |
        """
        return self.native_client.get_topic_delay(topic_name, timeout)

    # ============================================================================
    # ACTION OPERATIONS
    # ============================================================================
    # Action operations provide functionality for ROS2 action communication
    # including discovery, goal sending, and monitoring.

    # --- Discovery and Information ---
    @keyword
    def get_action_list(self) -> List[str]:
        """
        List all available ROS2 actions.

        Uses CLI command: ros2 action list

        Returns:
            List of action names

        Example:
            | ${actions}= | Get Action List |
            | Should Contain | ${actions} | /execute_action |
        """
        return self.cli_client.get_action_list()

    @keyword
    def get_action_info(
        self, action_name: str, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Get detailed information about an action.

        Uses CLI command: ros2 action info

        Args:
            action_name: Name of the action
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing action information (type, servers, clients)

        Example:
            | ${info}= | Get Action Info | /execute_action |
            | Log | Action type: ${info['type']} |
        """
        return self.cli_client.get_action_info(action_name, timeout)

    @keyword
    def get_action_type(self, action_name: str, timeout: Optional[float] = None) -> str:
        """
        Get the action type for an action.

        Uses CLI command: ros2 action type

        Args:
            action_name: Name of the action
            timeout: Override default timeout for this operation

        Returns:
            Action type string (e.g., 'pyrobosim_msgs/action/ExecuteTaskAction')

        Example:
            | ${type}= | Get Action Type | /execute_action |
            | Should Be Equal | ${type} | pyrobosim_msgs/action/ExecuteTaskAction |
        """
        result = self.cli_client._run_ros2_command(
            ["action", "type", action_name], timeout=timeout
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to get action type for '{action_name}': {result.stderr}"
            )
        return result.stdout.strip()

    @keyword
    def action_exists(self, action_name: str, timeout: Optional[float] = None) -> bool:
        """
        Check if an action exists.

        Args:
            action_name: Name of the action to check
            timeout: Override default timeout for this operation

        Returns:
            True if action exists, False otherwise

        Example:
            | ${exists}= | Action Exists | /execute_action |
            | Should Be True | ${exists} |
        """
        return self.cli_client.action_exists(action_name, timeout)

    # --- Goal Execution ---
    @keyword
    def send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_data: str,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Send a goal to a ROS2 action server and wait for completion.

        Uses CLI command: ros2 action send_goal

        Args:
            action_name: Name of the action (e.g., '/execute_action')
            action_type: Type of the action (e.g., 'pyrobosim_msgs/action/ExecuteTaskAction')
            goal_data: Goal data as a JSON string
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing the result of the action

        Example:
            | ${result}= | Send Action Goal | /execute_action | pyrobosim_msgs/action/ExecuteTaskAction | '{"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}' |
            | Should Be Equal | ${result['status']} | SUCCEEDED |
        """
        return self.cli_client.send_action_goal(
            action_name, action_type, goal_data, timeout
        )

    @keyword
    def async_send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_data: str,
        timeout: Optional[float] = None,
    ) -> None:
        """
        Send a goal to a ROS2 action server asynchronously (fire and forget).

        This function starts the action goal but doesn't wait for completion.
        It returns immediately after sending the goal.

        Args:
            action_name: Name of the action (e.g., '/execute_action')
            action_type: Type of the action (e.g., 'pyrobosim_msgs/action/ExecuteTaskAction')
            goal_data: Goal data as a JSON string
            timeout: Override default timeout for this operation (not used in fire-and-forget mode)

        Returns:
            None

        Example:
            | Async Send Action Goal | /execute_action | pyrobosim_msgs/action/ExecuteTaskAction | '{"action": {"robot": "robot0", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}' |
        """
        # Delegate to CLI client for async action goal implementation
        self.cli_client.async_send_action_goal(
            action_name, action_type, goal_data, timeout
        )

    # ============================================================================
    # SERVICE OPERATIONS
    # ============================================================================
    # Service operations provide functionality for ROS2 service communication
    # including discovery, calling services, and monitoring.

    # --- Discovery and Information ---
    @keyword
    def list_services(self, timeout: Optional[float] = None) -> List[str]:
        """
        List all available ROS2 services.

        Uses CLI command: ros2 service list

        Args:
            timeout: Override default timeout for this operation

        Returns:
            List of service names

        Example:
            | ${services}= | List Services |
            | Should Contain | ${services} | /add_two_ints |
        """
        return self.cli_client.list_services(timeout)

    @keyword
    def get_service_info(
        self, service_name: str, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Get detailed information about a service via CLI.

        Uses CLI command: ros2 service info

        Args:
            service_name: Name of the service
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing service information (type, nodes)

        Example:
            | ${info}= | Get Service Info | /add_two_ints |
            | Log | Service type: ${info['type']} |
        """
        return self.cli_client.get_service_info(service_name, timeout)

    @keyword
    def get_service_type(
        self, service_name: str, timeout: Optional[float] = None
    ) -> str:
        """
        Get the service type for a service.

        Uses CLI command: ros2 service type

        Args:
            service_name: Name of the service
            timeout: Override default timeout for this operation

        Returns:
            Service type string (e.g., 'example_interfaces/srv/AddTwoInts')

        Example:
            | ${type}= | Get Service Type | /add_two_ints |
            | Should Be Equal | ${type} | example_interfaces/srv/AddTwoInts |
        """
        result = self.cli_client._run_ros2_command(
            ["service", "type", service_name], timeout=timeout
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to get service type for '{service_name}': {result.stderr}"
            )
        return result.stdout.strip()

    @keyword
    def find_services_by_type(
        self, service_type: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Find services that use a specific service type.

        Uses CLI command: ros2 service find

        Args:
            service_type: Service type to search for (e.g., 'example_interfaces/srv/AddTwoInts')
            timeout: Override default timeout for this operation

        Returns:
            List of service names using the specified service type

        Example:
            | ${add_services}= | Find Services By Type | example_interfaces/srv/AddTwoInts |
            | Should Contain | ${add_services} | /add_two_ints |
        """
        result = self.cli_client._run_ros2_command(
            ["service", "find", service_type], timeout=timeout
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to find services by type '{service_type}': {result.stderr}"
            )
        return [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]

    # --- Service Communication (Native Operations) ---
    @keyword
    def create_service_client(
        self,
        service_name: str,
        service_type: str,
    ) -> str:
        """
        Create a native ROS2 service client.

        Uses native ROS2 operations for better performance and reliability.

        Args:
            service_name: Name of the service (e.g., '/add_two_ints')
            service_type: Type of the service (e.g., 'example_interfaces/srv/AddTwoInts')

        Returns:
            Client ID that can be used with call_service and service_available

        Example:
            | ${client_id}= | Create Service Client | /add_two_ints | example_interfaces/srv/AddTwoInts |
            | ${result}= | Call Service | ${client_id} | {"a": 5, "b": 3} |
        """
        return self.native_client.create_service_client(service_name, service_type)

    @keyword
    def call_service(
        self,
        client_id: str,
        request_data: Any = None,
        timeout: float = 10.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Call a service using a native service client.

        Uses native ROS2 operations for better performance and reliability.

        Args:
            client_id: ID of the service client (from create_service_client)
            request_data: Request data to send to the service
            timeout: Timeout for the service call

        Returns:
            Service response data or None if failed

        Example:
            | ${client_id}= | Create Service Client | /add_two_ints | example_interfaces/srv/AddTwoInts |
            | ${result}= | Call Service | ${client_id} | {"a": 5, "b": 3} |
            | Should Be Equal | ${result['sum']} | 8 |
        """
        return self.native_client.call_service(client_id, request_data, timeout)

    @keyword
    def call_service_cli(
        self,
        service_name: str,
        service_type: str,
        request_data: Any = None,
        timeout: float = 10.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Call a service using a CLI service client.

        Args:
            service_name: Name of the service
            service_type: Type of the service
            request_data: Request data to send to the service
            timeout: Timeout for the service call

        Returns:
            Service response data or None if failed
        """
        return self.cli_client.call_service(
            service_name, service_type, request_data, timeout
        )

    @keyword
    def service_available(self, client_id: str, timeout: float = 1.0) -> bool:
        """
        Check if a service is available using native client.

        Args:
            client_id: ID of the service client (from create_service_client)
            timeout: Timeout for checking availability

        Returns:
            True if service is available, False otherwise

        Example:
            | ${client_id}= | Create Service Client | /add_two_ints | example_interfaces/srv/AddTwoInts |
            | ${available}= | Service Available | ${client_id} |
            | Should Be True | ${available} |
        """
        return self.native_client.service_available(client_id, timeout)

    @keyword
    def create_service_server(
        self,
        service_name: str,
        service_type: str,
        callback_function: Optional[Any] = None,
    ) -> str:
        """
        Create a native ROS2 service server.

        Uses native ROS2 operations for better performance and reliability.

        Args:
            service_name: Name of the service (e.g., '/add_two_ints')
            service_type: Type of the service (e.g., 'example_interfaces/srv/AddTwoInts')
            callback_function: Optional callback function to handle service requests

        Returns:
            Server ID for managing the service server

        Example:
            | ${server_id}= | Create Service Server | /add_two_ints | example_interfaces/srv/AddTwoInts |
        """
        return self.native_client.create_service_server(
            service_name, service_type, callback_function
        )

    @keyword
    def get_service_info(self) -> Dict[str, Any]:
        """
        Get information about native service clients and servers.

        Returns:
            Dictionary containing information about all created service clients and servers

        Example:
            | ${info}= | Get Service Info |
            | Log | Service clients: ${info['clients']} |
        """
        return self.native_client.get_service_info()

    # ============================================================================
    # NODE OPERATIONS
    # ============================================================================
    # Node operations provide functionality for ROS2 node discovery and monitoring.
    # All operations use CLI commands for reliable system-wide node management.

    @keyword
    def list_nodes(self, timeout: Optional[float] = None) -> List[str]:
        """
        List all running ROS2 nodes.

        Uses CLI command: ros2 node list

        Args:
            timeout: Override default timeout for this operation

        Returns:
            List of node names

        Example:
            | ${nodes}= | List Nodes |
            | Should Contain | ${nodes} | /talker |
        """
        return self.cli_client.list_nodes(timeout)

    @keyword
    def get_node_info(
        self, node_name: str, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Get detailed information about a node.

        Uses CLI command: ros2 node info

        Args:
            node_name: Name of the node
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing node information (topics, services, actions)

        Example:
            | ${info}= | Get Node Info | /talker |
            | Log | Node topics: ${info['topics']} |
        """
        return self.cli_client.get_node_info(node_name, timeout)

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
        return self.cli_client.node_exists(node_name, timeout)

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
            True if node becomes available, False if timeout

        Example:
            | ${available}= | Wait For Node | /talker | timeout=10.0 |
            | Should Be True | ${available} |
        """
        return self.cli_client.wait_for_node(node_name, timeout, check_interval)

    @keyword
    def has_running_nodes(self, timeout: float = 30.0) -> bool:
        """
        Check if there are any nodes running.

        Args:
            timeout: Timeout for checking node availability

        Returns:
            True if any nodes are running, False otherwise

        Example:
            | ${running}= | Has Running Nodes | timeout=5.0 |
            | Should Be True | ${running} |
        """
        return self.cli_client.has_running_nodes(timeout)

    # ============================================================================
    # PARAMETER OPERATIONS
    # ============================================================================
    # Parameter operations provide functionality for ROS2 parameter management.
    # Includes both native operations (for the test node) and CLI operations (for any node).

    # --- Native Node Parameters (Native Operations) ---
    @keyword
    def list_parameters(self) -> List[str]:
        """
        List all parameters for the native test node.

        Uses native ROS2 operations for the test node only.

        Returns:
            List of parameter names for the native node

        Example:
            | ${params}= | List Parameters |
            | Log | Native node parameters: ${params} |
        """
        if self.native_client:
            return self.native_client.list_parameters()
        else:
            logger.warn("Native client not available, cannot list parameters")
            return []

    @keyword
    def get_parameter(self, parameter_name: str, default_value: Any = None) -> Any:
        """
        Get a parameter from the native test node.

        Uses native ROS2 operations for the test node only.

        Args:
            parameter_name: Name of the parameter
            default_value: Default value if parameter doesn't exist

        Returns:
            Parameter value or default value

        Example:
            | ${value}= | Get Parameter | my_param | default=10 |
            | Should Be Equal | ${value} | 10 |
        """
        if self.native_client:
            return self.native_client.get_parameter(parameter_name, default_value)
        else:
            logger.warn("Native client not available, cannot get parameter")
            return default_value

    @keyword
    def set_parameter(
        self, parameter_name: str, value: Union[str, int, float, bool]
    ) -> bool:
        """
        Set a parameter on the native test node.

        Uses native ROS2 operations for the test node only.

        Args:
            parameter_name: Name of the parameter
            value: Value to set

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Set Parameter | my_param | 42 |
            | Should Be True | ${success} |
        """
        if self.native_client:
            return self.native_client.set_parameter(parameter_name, value)
        else:
            logger.warn("Native client not available, cannot set parameter")
            return False

    @keyword
    def parameter_exists(self, parameter_name: str) -> bool:
        """
        Check if a parameter exists on the native test node.

        Uses native ROS2 operations for the test node only.

        Args:
            parameter_name: Name of the parameter to check

        Returns:
            True if parameter exists, False otherwise

        Example:
            | ${exists}= | Parameter Exists | my_param |
            | Should Be True | ${exists} |
        """
        if self.native_client:
            return self.native_client.parameter_exists(parameter_name)
        else:
            logger.warn("Native client not available, cannot check parameter existence")
            return False

    @keyword
    def get_all_parameters(self) -> Dict[str, Any]:
        """
        Get all parameters from the native test node.

        Uses native ROS2 operations for the test node only.

        Returns:
            Dictionary of all parameter names and values

        Example:
            | ${all_params}= | Get All Parameters |
            | Log | All parameters: ${all_params} |
        """
        if self.native_client:
            return self.native_client.get_all_parameters()
        else:
            logger.warn("Native client not available, cannot get all parameters")
            return {}

    # --- Any Node Parameters (CLI Operations) ---
    @keyword
    def list_node_parameters(
        self, node_name: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        List parameters for any node via CLI.

        Uses CLI command: ros2 param list

        Args:
            node_name: Name of the node
            timeout: Override default timeout for this operation

        Returns:
            List of parameter names for the specified node

        Example:
            | ${params}= | List Node Parameters | /my_node |
            | Should Contain | ${params} | use_sim_time |
        """
        result = self.cli_client._run_ros2_command(
            ["param", "list", node_name], timeout=timeout
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to list parameters for node '{node_name}': {result.stderr}"
            )
        return [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]

    @keyword
    def get_node_parameter(
        self, node_name: str, param_name: str, timeout: Optional[float] = None
    ) -> Any:
        """
        Get a parameter from any node via CLI.

        Uses CLI command: ros2 param get

        Args:
            node_name: Name of the node
            param_name: Name of the parameter
            timeout: Override default timeout for this operation

        Returns:
            Parameter value

        Example:
            | ${value}= | Get Node Parameter | /my_node | use_sim_time |
            | Should Be Equal | ${value} | False |
        """
        result = self.cli_client._run_ros2_command(
            ["param", "get", node_name, param_name], timeout=timeout
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to get parameter '{param_name}' from node '{node_name}': {result.stderr}"
            )

        value_text = result.stdout.strip()
        # Try to parse the value
        try:
            if value_text.lower() in ["true", "false"]:
                return value_text.lower() == "true"
            elif value_text.isdigit():
                return int(value_text)
            elif value_text.replace(".", "").replace("-", "").isdigit():
                return float(value_text)
            else:
                return value_text
        except Exception:
            return value_text

    @keyword
    def set_node_parameter(
        self,
        node_name: str,
        param_name: str,
        value: Any,
        timeout: Optional[float] = None,
    ) -> bool:
        """
        Set a parameter on any node via CLI.

        Uses CLI command: ros2 param set

        Args:
            node_name: Name of the node
            param_name: Name of the parameter
            value: Value to set
            timeout: Override default timeout for this operation

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Set Node Parameter | /my_node | use_sim_time | True |
            | Should Be True | ${success} |
        """
        # Convert value to string representation
        if isinstance(value, bool):
            value_str = str(value).lower()
        else:
            value_str = str(value)

        result = self.cli_client._run_ros2_command(
            ["param", "set", node_name, param_name, value_str], timeout=timeout
        )
        if result.returncode != 0:
            logger.error(
                f"Failed to set parameter '{param_name}' on node '{node_name}': {result.stderr}"
            )
            return False
        return True

    @keyword
    def dump_node_parameters(
        self,
        node_name: str,
        output_file: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> str:
        """
        Dump all parameters from a node to YAML format.

        Uses CLI command: ros2 param dump

        Args:
            node_name: Name of the node
            output_file: Optional output file path (if None, returns YAML string)
            timeout: Override default timeout for this operation

        Returns:
            YAML string of parameters or file path if output_file specified

        Example:
            | ${yaml}= | Dump Node Parameters | /my_node |
            | Log | Parameters YAML: ${yaml} |
        """
        command = ["param", "dump", node_name]
        if output_file:
            command.extend(["--output-dir", output_file])

        result = self.cli_client._run_ros2_command(command, timeout=timeout)
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to dump parameters for node '{node_name}': {result.stderr}"
            )

        if output_file:
            return output_file
        else:
            return result.stdout.strip()

    # ============================================================================
    # LAUNCH OPERATIONS
    # ============================================================================
    # Launch operations provide functionality for starting ROS2 launch files and packages.
    # All operations use CLI commands for reliable system management.

    @keyword
    def launch_file(
        self,
        launch_file_path: str,
        arguments: Optional[Dict[str, str]] = None,
        timeout: Optional[float] = None,
    ):
        """
        Launch a ROS2 launch file.

        Uses CLI command: ros2 launch

        Args:
            launch_file_path: Path to the launch file
            arguments: Optional launch arguments as key-value pairs
            timeout: Override default timeout for this operation

        Returns:
            Launch process object

        Example:
            | ${process}= | Launch File | /path/to/launch_file.launch.py | arguments={"use_sim_time": "true"} |
        """
        return self.cli_client.launch_file(launch_file_path, arguments, timeout)

    @keyword
    def launch_package(
        self,
        package_name: str,
        launch_file_name: str,
        arguments: Optional[str] = None,
    ):
        """
        Launch a ROS2 package launch file.

        Uses CLI command: ros2 launch

        Args:
            package_name: Name of the ROS2 package
            launch_file_name: Name of the launch file within the package
            arguments: Optional launch arguments as key-value pairs

        Returns:
            Launch process object

        Example:
            | ${process}= | Launch Package | nav2_bringup | tb3_simulation_launch.py | arguments={"use_sim_time": "true"} |
        """
        return self.cli_client.launch_package(package_name, launch_file_name, arguments)

    @keyword
    def find_launch_files(
        self, package_name: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Find launch files in a ROS2 package.

        Args:
            package_name: Name of the ROS2 package
            timeout: Override default timeout for this operation

        Returns:
            List of launch file names in the package

        Example:
            | ${files}= | Find Launch Files | nav2_bringup |
            | Should Contain | ${files} | tb3_simulation_launch.py |
        """
        return self.cli_client.find_launch_files(package_name, timeout)

    @keyword
    def wait_for_launch_completion(self, process, timeout: float = 30.0) -> bool:
        """
        Wait for a launch process to complete.

        Args:
            process: Launch process object
            timeout: Maximum time to wait in seconds

        Returns:
            True if process completed, False if timeout

        Example:
            | ${completed}= | Wait For Launch Completion | ${process} | timeout=60.0 |
            | Should Be True | ${completed} |
        """
        return self.cli_client.wait_for_launch_completion(process, timeout)

    @keyword
    def terminate_launch_process(self, process, force: bool = False) -> bool:
        """
        Terminate a launch process.

        Args:
            process: Launch process object
            force: Whether to force termination

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Terminate Launch Process | ${process} | force=True |
            | Should Be True | ${success} |
        """
        return self.cli_client.terminate_launch_process(process, force)

    # ============================================================================
    # RUN OPERATIONS
    # ============================================================================
    # Run operations provide functionality for executing ROS2 nodes and managing processes.
    # All operations use CLI commands for reliable system management.

    @keyword
    def run_node(
        self,
        package_name: str,
        executable_name: str,
        arguments: Optional[List[str]] = None,
        setup_script: Optional[str] = None,
    ):
        """
        Run a ROS2 node.

        Uses CLI command: ros2 run

        Args:
            package_name: Name of the ROS2 package
            executable_name: Name of the executable
            arguments: Optional command line arguments
            setup_script: Optional setup script to source before running

        Returns:
            Process object

        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker | arguments=["--ros-args", "-p", "use_sim_time:=true"] |
        """
        return self.cli_client.run_node(
            package_name, executable_name, arguments, setup_script
        )

    @keyword
    def run_node_with_remap(
        self,
        package_name: str,
        executable_name: str,
        remaps: Optional[Dict[str, str]] = None,
        arguments: Optional[List[str]] = None,
        setup_script: Optional[str] = None,
        timeout: Optional[float] = None,
    ):
        """
        Run a ROS2 node with topic remapping.

        Uses CLI command: ros2 run with remapping

        Args:
            package_name: Name of the ROS2 package
            executable_name: Name of the executable
            remaps: Dictionary of topic remappings (old_topic -> new_topic)
            arguments: Optional command line arguments
            setup_script: Optional setup script to source before running
            timeout: Override default timeout for this operation

        Returns:
            Process object

        Example:
            | ${process}= | Run Node With Remap | demo_nodes_cpp | talker | remaps={"chatter": "my_chatter"} |
        """
        return self.cli_client.run_node_with_remap(
            package_name, executable_name, remaps, arguments, setup_script
        )

    @keyword
    def find_executables(
        self, package_name: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Find executables in a ROS2 package.

        Args:
            package_name: Name of the ROS2 package
            timeout: Override default timeout for this operation

        Returns:
            List of executable names in the package

        Example:
            | ${executables}= | Find Executables | demo_nodes_cpp |
            | Should Contain | ${executables} | talker |
        """
        return self.cli_client.find_executables(package_name, timeout)

    # --- Process Management ---
    @keyword
    def wait_for_node_completion(self, process, timeout: float = 30.0) -> bool:
        """
        Wait for a node process to complete.

        Args:
            process: Process object
            timeout: Maximum time to wait in seconds

        Returns:
            True if process completed, False if timeout

        Example:
            | ${completed}= | Wait For Node Completion | ${process} | timeout=60.0 |
            | Should Be True | ${completed} |
        """
        return self.cli_client.wait_for_node_completion(process, timeout)

    @keyword
    def terminate_node_process(self, process, force: bool = False) -> bool:
        """
        Terminate a node process.

        Args:
            process: Process object
            force: Whether to force termination

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Terminate Node Process | ${process} | force=True |
            | Should Be True | ${success} |
        """
        return self.cli_client.terminate_node_process(process, force)

    @keyword
    def get_process_output(self, process, timeout: float = 1.0) -> Dict[str, str]:
        """
        Get output from a process.

        Args:
            process: Process object
            timeout: Timeout for reading output

        Returns:
            Dictionary with 'stdout' and 'stderr' keys

        Example:
            | ${output}= | Get Process Output | ${process} |
            | Log | stdout: ${output['stdout']} |
        """
        return self.cli_client.get_process_output(process, timeout)

    @keyword
    def is_process_running(self, process) -> bool:
        """
        Check if a process is running.

        Args:
            process: Process object

        Returns:
            True if process is running, False otherwise

        Example:
            | ${running}= | Is Process Running | ${process} |
            | Should Be True | ${running} |
        """
        return self.cli_client.is_process_running(process)

    @keyword
    def shutdown_process(self, process_name: str, force: bool = False) -> bool:
        """
        Shutdown a process by name.

        Args:
            process_name: Name of the process to shutdown
            force: Whether to force shutdown

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Shutdown Process | talker | force=True |
            | Should Be True | ${success} |
        """
        return self.cli_client.shutdown_process(process_name, force)

    @keyword
    def kill_process_by_name(self, process_name: str) -> bool:
        """
        Kill a process by name.

        Args:
            process_name: Name of the process to kill

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Kill Process By Name | talker |
            | Should Be True | ${success} |
        """
        return self.cli_client.pkill_process(process_name)

    # ============================================================================
    # NATIVE-SPECIFIC OPERATIONS
    # ============================================================================
    # Native-specific operations provide advanced functionality using native ROS2 operations.
    # These operations require the native client and provide enhanced performance and features.

    # --- Advanced Message Operations ---
    @keyword
    def get_all_messages(self, topic_name: str) -> List[Dict[str, Any]]:
        """
        Get all buffered messages from a subscribed topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic

        Returns:
            List of all buffered messages

        Example:
            | ${messages}= | Get All Messages | /chatter |
            | Length Should Be | ${messages} | 5 |
        """
        if self.native_client:
            return self.native_client.get_all_messages(topic_name)
        else:
            logger.warn("Native client not available, cannot get all messages")
            return []

    @keyword
    def clear_message_buffer(self, topic_name: str) -> bool:
        """
        Clear message buffer for a topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Clear Message Buffer | /chatter |
            | Should Be True | ${success} |
        """
        if self.native_client:
            return self.native_client.clear_message_buffer(topic_name)
        else:
            logger.warn("Native client not available, cannot clear message buffer")
            return False

    @keyword
    def wait_for_message(
        self, topic_name: str, timeout: float = 10.0, check_interval: float = 0.1
    ) -> Optional[Dict[str, Any]]:
        """
        Wait for a message on a subscribed topic.

        Uses native ROS2 operations for better performance.

        Args:
            topic_name: Name of the topic
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds

        Returns:
            Message data or None if timeout

        Example:
            | ${message}= | Wait For Message | /chatter | timeout=5.0 |
            | Should Not Be None | ${message} |
        """
        if self.native_client:
            return self.native_client.wait_for_message(
                topic_name, timeout, check_interval
            )
        else:
            logger.warn("Native client not available, cannot wait for message")
            return None

    # --- Advanced Parameter Operations ---
    @keyword
    def declare_parameter(self, parameter_name: str, default_value: Any) -> bool:
        """
        Declare a parameter with default value on the native node.

        Uses native ROS2 operations for better performance.

        Args:
            parameter_name: Name of the parameter
            default_value: Default value for the parameter

        Returns:
            True if successful, False otherwise

        Example:
            | ${success}= | Declare Parameter | my_param | 42 |
            | Should Be True | ${success} |
        """
        if self.native_client:
            return self.native_client.declare_parameter(parameter_name, default_value)
        else:
            logger.warn("Native client not available, cannot declare parameter")
            return False

    # ============================================================================
    # TRANSFORM OPERATIONS
    # ============================================================================
    # Transform operations provide functionality for ROS2 tf2 transform management.
    # These operations use native ROS2 operations for real-time transform queries.

    @keyword
    def get_transform(
        self, target_frame: str, source_frame: str, timeout: float = 5.0
    ) -> Optional[Dict[str, Any]]:
        """
        Get transform between two frames using tf2.

        Uses native ROS2 operations for real-time transform queries.

        Args:
            target_frame: Target frame name
            source_frame: Source frame name
            timeout: Maximum time to wait for transform in seconds

        Returns:
            Transform data dictionary with translation and rotation, or None if failed

        Example:
            | ${transform}= | Get Transform | map | base_link | timeout=1.0 |
            | Should Not Be None | ${transform} |
            | Log | Position: ${transform['translation']} |
        """
        if self.native_client:
            return self.native_client.get_tf(target_frame, source_frame, timeout)
        else:
            logger.warn("Native client not available, cannot get transform")
            return None

    @keyword
    def get_transform_at_time(
        self,
        target_frame: str,
        source_frame: str,
        time_stamp: float,
        timeout: float = 5.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Get transform between two frames at a specific time using tf2.

        Uses native ROS2 operations for real-time transform queries.

        Args:
            target_frame: Target frame name
            source_frame: Source frame name
            time_stamp: Time stamp for the transform
            timeout: Maximum time to wait for transform in seconds

        Returns:
            Transform data dictionary with translation and rotation, or None if failed

        Example:
            | ${transform}= | Get Transform At Time | map | base_link | 1234567890.0 | timeout=1.0 |
            | Should Not Be None | ${transform} |
        """
        if self.native_client:
            return self.native_client.get_tf_at_time(
                target_frame, source_frame, time_stamp, timeout
            )
        else:
            logger.warn("Native client not available, cannot get transform at time")
            return None

    @keyword
    def can_transform(
        self, target_frame: str, source_frame: str, timeout: float = 5.0
    ) -> bool:
        """
        Check if a transform is available between two frames.

        Uses native ROS2 operations for real-time transform queries.

        Args:
            target_frame: Target frame name
            source_frame: Source frame name
            timeout: Maximum time to wait for transform in seconds

        Returns:
            True if transform is available, False otherwise

        Example:
            | ${available}= | Can Transform | map | base_link | timeout=1.0 |
            | Should Be True | ${available} |
        """
        if self.native_client:
            return self.native_client.can_transform(target_frame, source_frame, timeout)
        else:
            logger.warn(
                "Native client not available, cannot check transform availability"
            )
            return False

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================
    # Utility methods provide helper functionality for testing, cleanup, and validation.

    @keyword
    def cleanup(self):
        """
        Clean up all resources and connections.

        This method should be called at the end of test suites to properly
        clean up ROS2 connections and resources.

        Example:
            | Cleanup |
        """
        if self.native_client:
            self.native_client.cleanup()
        logger.info("ROS2ClientLibrary cleanup completed")

    @keyword
    def get_client_info(self) -> Dict[str, Any]:
        """
        Get information about the current client configuration.

        Returns:
            Dictionary containing client configuration information

        Example:
            | ${info}= | Get Client Info |
            | Log | Native available: ${info['native_available']} |
            | Log | Timeout: ${info['timeout']} |
        """
        info = {
            "native_available": self.native_client is not None,
            "timeout": self.timeout,
            "ros2_executable": self._ros2_executable,
        }

        if self.native_client:
            info["native_info"] = self.native_client.get_client_info()

        return info

    # --- Validation and Testing Helpers ---
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

        This is a utility method for navigation and positioning tests.

        Args:
            transform_data: Transform data dictionary from get_transform method
            tolerance: Maximum allowed distance from target in meters
            target_x: Target X coordinate
            target_y: Target Y coordinate
            target_z: Target Z coordinate (default: 0.0)

        Returns:
            True if position is within tolerance of target, False otherwise

        Example:
            | ${transform}= | Get Transform | map | base_link | timeout=1.0 |
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
