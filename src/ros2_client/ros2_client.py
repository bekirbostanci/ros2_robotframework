"""
ROS2 CLI Robot Framework Library

A comprehensive Robot Framework library for interacting with ROS2 command-line interface.
Provides keywords for topic, service, node, and parameter operations.
"""

import subprocess
import json
import time
import os
from typing import List, Dict, Any, Optional, Union
from pathlib import Path
from robot.api.deco import keyword
from robot.api import logger


class ROS2ClientLibrary:
    """
    Robot Framework library for ROS2 CLI operations.
    
    This library provides keywords to interact with ROS2 command-line tools
    for testing and automation purposes.
    """
    
    ROBOT_LIBRARY_SCOPE = 'GLOBAL'
    ROBOT_LIBRARY_DOC_FORMAT = 'ROBOT'
    
    def __init__(self, timeout: float = 10.0):
        """
        Initialize the ROS2 Client Library.
        
        Args:
            timeout: Default timeout for ROS2 CLI operations in seconds
        """
        self.timeout = timeout
        self._ros2_executable = self._find_ros2_executable()
        
    def _find_ros2_executable(self) -> str:
        """Find the ROS2 executable path."""
        try:
            result = subprocess.run(
                ['which', 'ros2'], 
                capture_output=True, 
                text=True, 
                timeout=5.0
            )
            if result.returncode == 0:
                return result.stdout.strip()
            else:
                return 'ros2'  # Fallback to assuming it's in PATH
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return 'ros2'  # Fallback to assuming it's in PATH
    
    def _run_ros2_command(
        self, 
        command: List[str], 
        timeout: Optional[float] = None,
        capture_output: bool = True
    ) -> subprocess.CompletedProcess:
        """
        Run a ROS2 command and return the result.
        
        Args:
            command: List of command arguments (without 'ros2')
            timeout: Override default timeout
            capture_output: Whether to capture stdout/stderr
            
        Returns:
            CompletedProcess object with return code and output
            
        Raises:
            subprocess.TimeoutExpired: If command times out
            subprocess.CalledProcessError: If command fails
        """
        full_command = [self._ros2_executable] + command
        timeout_value = timeout or self.timeout
        
        logger.info(f"Running ROS2 command: {' '.join(full_command)}")
        
        try:
            result = subprocess.run(
                full_command,
                capture_output=capture_output,
                text=True,
                timeout=timeout_value,
                check=False  # Don't raise exception on non-zero return code
            )
            
            logger.debug(f"Command return code: {result.returncode}")
            if result.stdout:
                logger.debug(f"Command stdout: {result.stdout}")
            if result.stderr:
                logger.debug(f"Command stderr: {result.stderr}")
                
            return result
            
        except subprocess.TimeoutExpired as e:
            logger.error(f"ROS2 command timed out after {timeout_value}s: {' '.join(full_command)}")
            raise
        except Exception as e:
            logger.error(f"Failed to run ROS2 command: {e}")
            raise

    # ============================================================================
    # TOPIC OPERATIONS
    # ============================================================================
    
    @keyword
    def list_topics(self, timeout: Optional[float] = None) -> List[str]:
        """
        List all available ROS2 topics.
        
        Args:
            timeout: Override default timeout for this operation
            
        Returns:
            List of topic names
            
        Example:
            | ${topics}= | List Topics |
            | Should Contain | ${topics} | /chatter |
        """
        result = self._run_ros2_command(['topic', 'list'], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to list topics: {result.stderr}")
        
        topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        logger.info(f"Found {len(topics)} topics: {topics}")
        return topics
    
    @keyword
    def get_topic_info(self, topic_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get detailed information about a specific topic.
        
        Args:
            topic_name: Name of the topic to get info for
            timeout: Override default timeout for this operation
            
        Returns:
            Dictionary containing topic information (type, publishers, subscribers)
            
        Example:
            | ${info}= | Get Topic Info | /chatter |
            | Should Be Equal | ${info}[type] | std_msgs/msg/String |
        """
        result = self._run_ros2_command(['topic', 'info', topic_name], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to get topic info for '{topic_name}': {result.stderr}")
        
        info = {'name': topic_name}
        lines = result.stdout.strip().split('\n')
        
        for line in lines:
            line = line.strip()
            if line.startswith('Type:'):
                info['type'] = line.split(':', 1)[1].strip()
            elif line.startswith('Publisher count:'):
                info['publisher_count'] = int(line.split(':', 1)[1].strip())
            elif line.startswith('Subscriber count:'):
                info['subscriber_count'] = int(line.split(':', 1)[1].strip())
            elif line.startswith('Publisher:'):
                if 'publishers' not in info:
                    info['publishers'] = []
                info['publishers'].append(line.split(':', 1)[1].strip())
            elif line.startswith('Subscriber:'):
                if 'subscribers' not in info:
                    info['subscribers'] = []
                info['subscribers'].append(line.split(':', 1)[1].strip())
        
        logger.info(f"Topic info for '{topic_name}': {info}")
        return info
    
    @keyword
    def echo_topic(self, topic_name: str, count: int = 1, timeout: Optional[float] = None) -> List[str]:
        """
        Echo messages from a topic.
        
        Args:
            topic_name: Name of the topic to echo
            count: Number of messages to capture (default: 1)
            timeout: Override default timeout for this operation
            
        Returns:
            List of message strings received
            
        Example:
            | ${messages}= | Echo Topic | /chatter | count=3 |
            | Length Should Be | ${messages} | 3 |
        """
        command = ['topic', 'echo', topic_name, '--once'] if count == 1 else ['topic', 'echo', topic_name]
        
        # For multiple messages, we need to handle this differently
        if count > 1:
            messages = []
            for i in range(count):
                result = self._run_ros2_command(['topic', 'echo', topic_name, '--once'], timeout=timeout)
                if result.returncode == 0:
                    messages.append(result.stdout.strip())
                else:
                    logger.warn(f"Failed to get message {i+1} from topic '{topic_name}': {result.stderr}")
            return messages
        else:
            result = self._run_ros2_command(command, timeout=timeout)
            
            if result.returncode != 0:
                raise RuntimeError(f"Failed to echo topic '{topic_name}': {result.stderr}")
            
            message = result.stdout.strip()
            logger.info(f"Echoed message from '{topic_name}': {message}")
            return [message] if message else []
    
    @keyword
    def publish_topic(self, topic_name: str, message_type: str, data: str, timeout: Optional[float] = None) -> bool:
        """
        Publish a message to a topic.
        
        Args:
            topic_name: Name of the topic to publish to
            message_type: Type of the message (e.g., 'std_msgs/msg/String')
            data: Message data as a string
            timeout: Override default timeout for this operation
            
        Returns:
            True if publish was successful
            
        Example:
            | ${success}= | Publish Topic | /chatter | std_msgs/msg/String | "Hello World" |
            | Should Be True | ${success} |
        """
        result = self._run_ros2_command(
            ['topic', 'pub', '--once', topic_name, message_type, data], 
            timeout=timeout
        )
        
        if result.returncode != 0:
            logger.error(f"Failed to publish to topic '{topic_name}': {result.stderr}")
            return False
        
        logger.info(f"Successfully published to topic '{topic_name}': {data}")
        return True
    
    @keyword
    def topic_exists(self, topic_name: str, timeout: Optional[float] = None) -> bool:
        """
        Check if a topic exists.
        
        Args:
            topic_name: Name of the topic to check
            timeout: Override default timeout for this operation
            
        Returns:
            True if topic exists, False otherwise
            
        Example:
            | ${exists}= | Topic Exists | /chatter |
            | Should Be True | ${exists} |
        """
        try:
            topics = self.list_topics(timeout=timeout)
            exists = topic_name in topics
            logger.info(f"Topic '{topic_name}' exists: {exists}")
            return exists
        except Exception as e:
            logger.error(f"Error checking if topic '{topic_name}' exists: {e}")
            return False
    
    @keyword
    def wait_for_topic(self, topic_name: str, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """
        Wait for a topic to become available.
        
        Args:
            topic_name: Name of the topic to wait for
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            True if topic becomes available within timeout
            
        Example:
            | ${available}= | Wait For Topic | /chatter | timeout=10.0 |
            | Should Be True | ${available} |
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.topic_exists(topic_name, timeout=check_interval):
                logger.info(f"Topic '{topic_name}' became available after {time.time() - start_time:.2f}s")
                return True
            time.sleep(check_interval)
        
        logger.warn(f"Topic '{topic_name}' did not become available within {timeout}s")
        return False

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
        result = self._run_ros2_command(['service', 'list'], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to list services: {result.stderr}")
        
        services = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        logger.info(f"Found {len(services)} services: {services}")
        return services
    
    @keyword
    def get_service_info(self, service_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get detailed information about a specific service.
        
        Args:
            service_name: Name of the service to get info for
            timeout: Override default timeout for this operation
            
        Returns:
            Dictionary containing service information (type, clients, servers)
            
        Example:
            | ${info}= | Get Service Info | /add_two_ints |
            | Should Be Equal | ${info}[type] | example_interfaces/srv/AddTwoInts |
        """
        result = self._run_ros2_command(['service', 'info', service_name], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to get service info for '{service_name}': {result.stderr}")
        
        info = {'name': service_name}
        lines = result.stdout.strip().split('\n')
        
        for line in lines:
            line = line.strip()
            if line.startswith('Type:'):
                info['type'] = line.split(':', 1)[1].strip()
            elif line.startswith('Client count:'):
                info['client_count'] = int(line.split(':', 1)[1].strip())
            elif line.startswith('Server count:'):
                info['server_count'] = int(line.split(':', 1)[1].strip())
            elif line.startswith('Client:'):
                if 'clients' not in info:
                    info['clients'] = []
                info['clients'].append(line.split(':', 1)[1].strip())
            elif line.startswith('Server:'):
                if 'servers' not in info:
                    info['servers'] = []
                info['servers'].append(line.split(':', 1)[1].strip())
        
        logger.info(f"Service info for '{service_name}': {info}")
        return info
    
    @keyword
    def call_service(self, service_name: str, service_type: str, request_data: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Call a ROS2 service with request data.
        
        Args:
            service_name: Name of the service to call
            service_type: Type of the service (e.g., 'example_interfaces/srv/AddTwoInts')
            request_data: Request data as a string (e.g., 'a: 5, b: 3')
            timeout: Override default timeout for this operation
            
        Returns:
            Dictionary containing the service response
            
        Example:
            | ${response}= | Call Service | /add_two_ints | example_interfaces/srv/AddTwoInts | "a: 5, b: 3" |
            | Should Be Equal | ${response}[sum] | 8 |
        """
        result = self._run_ros2_command(
            ['service', 'call', service_name, service_type, request_data], 
            timeout=timeout
        )
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to call service '{service_name}': {result.stderr}")
        
        # Parse the response - this is a simplified parser
        # In a real implementation, you might want more sophisticated parsing
        response_text = result.stdout.strip()
        logger.info(f"Service call response for '{service_name}': {response_text}")
        
        # Try to parse as JSON-like structure
        try:
            # Simple parsing for common response formats
            response = {}
            lines = response_text.split('\n')
            for line in lines:
                line = line.strip()
                if ':' in line and not line.startswith('requester:'):
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    # Try to convert to appropriate type
                    try:
                        if value.isdigit():
                            response[key] = int(value)
                        elif value.replace('.', '').isdigit():
                            response[key] = float(value)
                        else:
                            response[key] = value
                    except ValueError:
                        response[key] = value
            return response
        except Exception:
            # If parsing fails, return the raw response
            return {'raw_response': response_text}
    
    @keyword
    def service_exists(self, service_name: str, timeout: Optional[float] = None) -> bool:
        """
        Check if a service exists.
        
        Args:
            service_name: Name of the service to check
            timeout: Override default timeout for this operation
            
        Returns:
            True if service exists, False otherwise
            
        Example:
            | ${exists}= | Service Exists | /add_two_ints |
            | Should Be True | ${exists} |
        """
        try:
            services = self.list_services(timeout=timeout)
            exists = service_name in services
            logger.info(f"Service '{service_name}' exists: {exists}")
            return exists
        except Exception as e:
            logger.error(f"Error checking if service '{service_name}' exists: {e}")
            return False
    
    @keyword
    def wait_for_service(self, service_name: str, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """
        Wait for a service to become available.
        
        Args:
            service_name: Name of the service to wait for
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            True if service becomes available within timeout
            
        Example:
            | ${available}= | Wait For Service | /add_two_ints | timeout=10.0 |
            | Should Be True | ${available} |
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.service_exists(service_name, timeout=check_interval):
                logger.info(f"Service '{service_name}' became available after {time.time() - start_time:.2f}s")
                return True
            time.sleep(check_interval)
        
        logger.warn(f"Service '{service_name}' did not become available within {timeout}s")
        return False
    
    @keyword
    def get_service_type(self, service_name: str, timeout: Optional[float] = None) -> str:
        """
        Get the type of a service.
        
        Args:
            service_name: Name of the service
            timeout: Override default timeout for this operation
            
        Returns:
            Service type string
            
        Example:
            | ${type}= | Get Service Type | /add_two_ints |
            | Should Be Equal | ${type} | example_interfaces/srv/AddTwoInts |
        """
        info = self.get_service_info(service_name, timeout=timeout)
        return info.get('type', '')

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
        result = self._run_ros2_command(['node', 'list'], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to list nodes: {result.stderr}")
        
        nodes = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        logger.info(f"Found {len(nodes)} nodes: {nodes}")
        return nodes
    
    @keyword
    def get_node_info(self, node_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
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
        result = self._run_ros2_command(['node', 'info', node_name], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to get node info for '{node_name}': {result.stderr}")
        
        info = {'name': node_name}
        lines = result.stdout.strip().split('\n')
        current_section = None
        
        for line in lines:
            original_line = line
            line = line.strip()
            
            if line.startswith('Node name:'):
                info['name'] = line.split(':', 1)[1].strip()
            elif line.startswith('Node namespace:'):
                info['namespace'] = line.split(':', 1)[1].strip()
            elif line.startswith('Node namespace: /'):
                info['namespace'] = line.split(':', 1)[1].strip()
            elif line.startswith('Node namespace: (no namespace)'):
                info['namespace'] = '/'
            elif line.startswith('Subscribers:'):
                current_section = 'subscribers'
                info['subscribers'] = []
            elif line.startswith('Publishers:'):
                current_section = 'publishers'
                info['publishers'] = []
            elif line.startswith('Service Servers:'):
                current_section = 'service_servers'
                info['service_servers'] = []
            elif line.startswith('Service Clients:'):
                current_section = 'service_clients'
                info['service_clients'] = []
            elif line.startswith('Action Servers:'):
                current_section = 'action_servers'
                info['action_servers'] = []
            elif line.startswith('Action Clients:'):
                current_section = 'action_clients'
                info['action_clients'] = []
            elif line.startswith('Parameters:'):
                current_section = 'parameters'
                info['parameters'] = []
            elif original_line.startswith('  ') and line:  # Indented lines with content
                if current_section and line:
                    # Extract topic/service name and type
                    if ': ' in line:
                        name, msg_type = line.split(': ', 1)
                        info[current_section].append({
                            'name': name.strip(),
                            'type': msg_type.strip()
                        })
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
    def wait_for_node(self, node_name: str, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
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
                logger.info(f"Node '{node_name}' became available after {time.time() - start_time:.2f}s")
                return True
            time.sleep(check_interval)
        
        logger.warn(f"Node '{node_name}' did not become available within {timeout}s")
        return False

    # ============================================================================
    # PARAMETER OPERATIONS
    # ============================================================================
    
    @keyword
    def list_parameters(self, node_name: str, timeout: Optional[float] = None) -> List[str]:
        """
        List all parameters for a specific node.
        
        Args:
            node_name: Name of the node to list parameters for
            timeout: Override default timeout for this operation
            
        Returns:
            List of parameter names
            
        Example:
            | ${params}= | List Parameters | /my_node |
            | Should Contain | ${params} | my_param |
        """
        result = self._run_ros2_command(['param', 'list', node_name], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to list parameters for node '{node_name}': {result.stderr}")
        
        params = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        logger.info(f"Found {len(params)} parameters for node '{node_name}': {params}")
        return params
    
    @keyword
    def get_parameter(self, node_name: str, parameter_name: str, timeout: Optional[float] = None) -> Any:
        """
        Get the value of a specific parameter.
        
        Args:
            node_name: Name of the node
            parameter_name: Name of the parameter
            timeout: Override default timeout for this operation
            
        Returns:
            Parameter value (string, int, float, bool, or list)
            
        Example:
            | ${value}= | Get Parameter | /my_node | my_param |
            | Should Be Equal | ${value} | 42 |
        """
        result = self._run_ros2_command(['param', 'get', node_name, parameter_name], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to get parameter '{parameter_name}' from node '{node_name}': {result.stderr}")
        
        value_text = result.stdout.strip()
        logger.info(f"Parameter '{parameter_name}' from node '{node_name}': {value_text}")
        
        # Try to parse the value
        try:
            # Handle different parameter types
            if value_text.lower() in ['true', 'false']:
                return value_text.lower() == 'true'
            elif value_text.isdigit():
                return int(value_text)
            elif value_text.replace('.', '').replace('-', '').isdigit():
                return float(value_text)
            elif value_text.startswith('[') and value_text.endswith(']'):
                # Simple list parsing - this could be enhanced
                return value_text
            else:
                return value_text
        except Exception:
            return value_text
    
    @keyword
    def set_parameter(self, node_name: str, parameter_name: str, value: Union[str, int, float, bool], timeout: Optional[float] = None) -> bool:
        """
        Set the value of a specific parameter.
        
        Args:
            node_name: Name of the node
            parameter_name: Name of the parameter
            value: Value to set (will be converted to string)
            timeout: Override default timeout for this operation
            
        Returns:
            True if parameter was set successfully
            
        Example:
            | ${success}= | Set Parameter | /my_node | my_param | 42 |
            | Should Be True | ${success} |
        """
        # Convert value to string representation
        if isinstance(value, bool):
            value_str = str(value).lower()
        else:
            value_str = str(value)
        
        result = self._run_ros2_command(
            ['param', 'set', node_name, parameter_name, value_str], 
            timeout=timeout
        )
        
        if result.returncode != 0:
            logger.error(f"Failed to set parameter '{parameter_name}' on node '{node_name}': {result.stderr}")
            return False
        
        logger.info(f"Successfully set parameter '{parameter_name}' on node '{node_name}' to: {value_str}")
        return True
    
    @keyword
    def parameter_exists(self, node_name: str, parameter_name: str, timeout: Optional[float] = None) -> bool:
        """
        Check if a parameter exists on a node.
        
        Args:
            node_name: Name of the node
            parameter_name: Name of the parameter to check
            timeout: Override default timeout for this operation
            
        Returns:
            True if parameter exists, False otherwise
            
        Example:
            | ${exists}= | Parameter Exists | /my_node | my_param |
            | Should Be True | ${exists} |
        """
        try:
            params = self.list_parameters(node_name, timeout=timeout)
            exists = parameter_name in params
            logger.info(f"Parameter '{parameter_name}' exists on node '{node_name}': {exists}")
            return exists
        except Exception as e:
            logger.error(f"Error checking if parameter '{parameter_name}' exists on node '{node_name}': {e}")
            return False
    
    @keyword
    def get_all_parameters(self, node_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get all parameters and their values for a specific node.
        
        Args:
            node_name: Name of the node
            timeout: Override default timeout for this operation
            
        Returns:
            Dictionary mapping parameter names to their values
            
        Example:
            | ${params}= | Get All Parameters | /my_node |
            | Should Be Equal | ${params}[my_param] | 42 |
        """
        param_names = self.list_parameters(node_name, timeout=timeout)
        all_params = {}
        
        for param_name in param_names:
            try:
                value = self.get_parameter(node_name, param_name, timeout=timeout)
                all_params[param_name] = value
            except Exception as e:
                logger.warn(f"Failed to get value for parameter '{param_name}': {e}")
                all_params[param_name] = None
        
        logger.info(f"Retrieved {len(all_params)} parameters for node '{node_name}'")
        return all_params

    # ============================================================================
    # LAUNCH OPERATIONS
    # ============================================================================
    
    @keyword
    def launch_file(self, launch_file_path: str, arguments: Optional[Dict[str, str]] = None, timeout: Optional[float] = None) -> subprocess.Popen:
        """
        Launch a ROS2 launch file.
        
        Args:
            launch_file_path: Path to the launch file (can be relative or absolute)
            arguments: Dictionary of launch arguments (key=value pairs)
            timeout: Override default timeout for this operation
            
        Returns:
            Popen process object for the launched process
            
        Example:
            | ${process}= | Launch File | /path/to/my_launch.launch.py |
            | ${process}= | Launch File | my_package launch/my_launch.launch.py | arguments={'arg1': 'value1'} |
        """
        command = ['launch', launch_file_path]
        
        # Add arguments if provided
        if arguments:
            for key, value in arguments.items():
                command.append(f'{key}:={value}')
        
        # Run the launch command in the background
        full_command = [self._ros2_executable] + command
        timeout_value = timeout or self.timeout
        
        logger.info(f"Launching ROS2 file: {' '.join(full_command)}")
        
        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=None if os.name == 'nt' else os.setsid  # Create new process group
            )
            
            logger.info(f"Launched process with PID: {process.pid}")
            
            # Give the process a moment to start
            time.sleep(0.5)
            
            # Check if process is still running
            if process.poll() is not None:
                # Process has already terminated, get the output
                stdout, stderr = process.communicate()
                logger.error(f"Launch process terminated immediately. Return code: {process.returncode}")
                logger.error(f"STDOUT: {stdout}")
                logger.error(f"STDERR: {stderr}")
                raise RuntimeError(f"Launch process failed to start: {stderr}")
            
            return process
            
        except Exception as e:
            logger.error(f"Failed to launch file '{launch_file_path}': {e}")
            raise
    
    @keyword
    def launch_package(self, package_name: str, launch_file_name: str, arguments: Optional[Dict[str, str]] = None, timeout: Optional[float] = None) -> subprocess.Popen:
        """
        Launch a ROS2 launch file from a package.
        
        Args:
            package_name: Name of the ROS2 package
            launch_file_name: Name of the launch file within the package
            arguments: Dictionary of launch arguments (key=value pairs)
            timeout: Override default timeout for this operation
            
        Returns:
            Popen process object for the launched process
            
        Example:
            | ${process}= | Launch Package | my_package | my_launch.launch.py |
            | ${process}= | Launch Package | nav2_bringup | tb3_simulation_launch.py | arguments={'use_sim_time': 'True'} |
        """
        # For package launch, we need to pass package and file separately
        command = ['launch', package_name, launch_file_name]
        
        # Add arguments if provided
        if arguments:
            for key, value in arguments.items():
                command.append(f'{key}:={value}')
        
        # Run the launch command in the background
        full_command = [self._ros2_executable] + command
        timeout_value = timeout or self.timeout
        
        logger.info(f"Launching ROS2 package: {' '.join(full_command)}")
        
        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=None if os.name == 'nt' else os.setsid  # Create new process group
            )
            
            logger.info(f"Launched process with PID: {process.pid}")
            
            # Give the process a moment to start
            time.sleep(0.5)
            
            # Check if process is still running
            if process.poll() is not None:
                # Process has already terminated, get the output
                stdout, stderr = process.communicate()
                logger.error(f"Launch process terminated immediately. Return code: {process.returncode}")
                logger.error(f"STDOUT: {stdout}")
                logger.error(f"STDERR: {stderr}")
                raise RuntimeError(f"Launch process failed to start: {stderr}")
            
            return process
            
        except Exception as e:
            logger.error(f"Failed to launch package '{package_name}' file '{launch_file_name}': {e}")
            raise
    
    @keyword
    def find_launch_files(self, package_name: str, timeout: Optional[float] = None) -> List[str]:
        """
        Find all launch files in a ROS2 package.
        
        Args:
            package_name: Name of the ROS2 package
            timeout: Override default timeout for this operation
            
        Returns:
            List of launch file names found in the package
            
        Example:
            | ${launch_files}= | Find Launch Files | nav2_bringup |
            | Should Contain | ${launch_files} | tb3_simulation_launch.py |
        """
        result = self._run_ros2_command(['pkg', 'list', '--packages'], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to list packages: {result.stderr}")
        
        # Check if package exists
        packages = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        if package_name not in packages:
            raise RuntimeError(f"Package '{package_name}' not found")
        
        # Find launch files using find command
        try:
            find_result = subprocess.run(
                ['find', f'/opt/ros/*/share/{package_name}/launch', '-name', '*.launch.py', '-o', '-name', '*.launch'],
                capture_output=True,
                text=True,
                timeout=timeout or self.timeout
            )
            
            if find_result.returncode == 0:
                launch_files = []
                for line in find_result.stdout.strip().split('\n'):
                    if line.strip():
                        # Extract just the filename
                        filename = Path(line.strip()).name
                        launch_files.append(filename)
                
                logger.info(f"Found {len(launch_files)} launch files in package '{package_name}': {launch_files}")
                return launch_files
            else:
                logger.warn(f"Could not find launch files for package '{package_name}'")
                return []
                
        except Exception as e:
            logger.warn(f"Error finding launch files for package '{package_name}': {e}")
            return []
    
    @keyword
    def wait_for_launch_completion(self, process: subprocess.Popen, timeout: float = 30.0) -> bool:
        """
        Wait for a launched process to complete.
        
        Args:
            process: Popen process object returned by launch_file or launch_package
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if process completed within timeout, False otherwise
            
        Example:
            | ${process}= | Launch File | my_launch.launch.py |
            | ${completed}= | Wait For Launch Completion | ${process} | timeout=60.0 |
        """
        try:
            process.wait(timeout=timeout)
            logger.info(f"Launch process completed with return code: {process.returncode}")
            return True
        except subprocess.TimeoutExpired:
            logger.warn(f"Launch process did not complete within {timeout}s")
            return False
        except Exception as e:
            logger.error(f"Error waiting for launch completion: {e}")
            return False
    
    @keyword
    def terminate_launch_process(self, process: subprocess.Popen, force: bool = False) -> bool:
        """
        Terminate a launched process.
        
        Args:
            process: Popen process object to terminate
            force: If True, use SIGKILL instead of SIGTERM
            
        Returns:
            True if process was terminated successfully
            
        Example:
            | ${process}= | Launch File | my_launch.launch.py |
            | ${terminated}= | Terminate Launch Process | ${process} |
            | Should Be True | ${terminated} |
        """
        try:
            if process.poll() is not None:
                logger.info(f"Launch process with PID {process.pid} has already terminated")
                return True
            
            if force:
                if os.name == 'nt':
                    process.kill()
                else:
                    # Kill the entire process group
                    os.killpg(os.getpgid(process.pid), 9)
                logger.info(f"Force killed launch process with PID: {process.pid}")
            else:
                if os.name == 'nt':
                    process.terminate()
                else:
                    # Terminate the entire process group
                    os.killpg(os.getpgid(process.pid), 15)
                logger.info(f"Terminated launch process with PID: {process.pid}")
            
            # Wait a bit for graceful termination
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                if not force:
                    if os.name == 'nt':
                        process.kill()
                    else:
                        os.killpg(os.getpgid(process.pid), 9)
                    logger.info(f"Force killed launch process after timeout")
            
            return True
        except Exception as e:
            logger.error(f"Failed to terminate launch process: {e}")
            return False

    # ============================================================================
    # RUN OPERATIONS
    # ============================================================================
    
    @keyword
    def run_node(self, package_name: str, executable_name: str, arguments: Optional[List[str]] = None, timeout: Optional[float] = None) -> subprocess.Popen:
        """
        Run a ROS2 node directly.
        
        Args:
            package_name: Name of the ROS2 package containing the node
            executable_name: Name of the executable/node
            arguments: List of command-line arguments for the node
            timeout: Override default timeout for this operation
            
        Returns:
            Popen process object for the running node
            
        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker |
            | ${process}= | Run Node | nav2_controller | controller_server | arguments=['--ros-args', '-p', 'use_sim_time:=True'] |
        """
        command = ['run', package_name, executable_name]
        
        # Add arguments if provided
        if arguments:
            command.extend(arguments)
        
        # Run the node in the background
        full_command = [self._ros2_executable] + command
        timeout_value = timeout or self.timeout
        
        logger.info(f"Running ROS2 node: {' '.join(full_command)}")
        
        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            logger.info(f"Started node process with PID: {process.pid}")
            return process
            
        except Exception as e:
            logger.error(f"Failed to run node '{package_name}/{executable_name}': {e}")
            raise
    
    @keyword
    def run_node_with_remap(self, package_name: str, executable_name: str, remaps: Optional[Dict[str, str]] = None, arguments: Optional[List[str]] = None, timeout: Optional[float] = None) -> subprocess.Popen:
        """
        Run a ROS2 node with topic/service remapping.
        
        Args:
            package_name: Name of the ROS2 package containing the node
            executable_name: Name of the executable/node
            remaps: Dictionary of remappings (old_topic -> new_topic)
            arguments: List of additional command-line arguments
            timeout: Override default timeout for this operation
            
        Returns:
            Popen process object for the running node
            
        Example:
            | ${process}= | Run Node With Remap | demo_nodes_cpp | talker | remaps={'/chatter': '/my_chatter'} |
        """
        command = ['run', package_name, executable_name]
        
        # Add remaps if provided
        if remaps:
            for old_topic, new_topic in remaps.items():
                command.extend(['--remap', f'{old_topic}:={new_topic}'])
        
        # Add additional arguments if provided
        if arguments:
            command.extend(arguments)
        
        # Run the node in the background
        full_command = [self._ros2_executable] + command
        timeout_value = timeout or self.timeout
        
        logger.info(f"Running ROS2 node with remaps: {' '.join(full_command)}")
        
        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            logger.info(f"Started node process with PID: {process.pid}")
            return process
            
        except Exception as e:
            logger.error(f"Failed to run node '{package_name}/{executable_name}' with remaps: {e}")
            raise
    
    @keyword
    def find_executables(self, package_name: str, timeout: Optional[float] = None) -> List[str]:
        """
        Find all executables in a ROS2 package.
        
        Args:
            package_name: Name of the ROS2 package
            timeout: Override default timeout for this operation
            
        Returns:
            List of executable names found in the package
            
        Example:
            | ${executables}= | Find Executables | demo_nodes_cpp |
            | Should Contain | ${executables} | talker |
        """
        result = self._run_ros2_command(['pkg', 'executables', package_name], timeout=timeout)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to find executables in package '{package_name}': {result.stderr}")
        
        executables = []
        lines = result.stdout.strip().split('\n')
        
        for line in lines:
            line = line.strip()
            if line and ' ' in line:
                # Format is usually "executable_name package_name"
                parts = line.split()
                if len(parts) >= 2 and parts[1] == package_name:
                    executables.append(parts[0])
        
        logger.info(f"Found {len(executables)} executables in package '{package_name}': {executables}")
        return executables
    
    @keyword
    def wait_for_node_completion(self, process: subprocess.Popen, timeout: float = 30.0) -> bool:
        """
        Wait for a running node to complete.
        
        Args:
            process: Popen process object returned by run_node
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if node completed within timeout, False otherwise
            
        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker |
            | ${completed}= | Wait For Node Completion | ${process} | timeout=60.0 |
        """
        try:
            process.wait(timeout=timeout)
            logger.info(f"Node process completed with return code: {process.returncode}")
            return True
        except subprocess.TimeoutExpired:
            logger.warn(f"Node process did not complete within {timeout}s")
            return False
        except Exception as e:
            logger.error(f"Error waiting for node completion: {e}")
            return False
    
    @keyword
    def terminate_node_process(self, process: subprocess.Popen, force: bool = False) -> bool:
        """
        Terminate a running node process.
        
        Args:
            process: Popen process object to terminate
            force: If True, use SIGKILL instead of SIGTERM
            
        Returns:
            True if process was terminated successfully
            
        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker |
            | ${terminated}= | Terminate Node Process | ${process} |
            | Should Be True | ${terminated} |
        """
        try:
            if force:
                process.kill()
                logger.info(f"Force killed node process with PID: {process.pid}")
            else:
                process.terminate()
                logger.info(f"Terminated node process with PID: {process.pid}")
            
            # Wait a bit for graceful termination
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                if not force:
                    process.kill()
                    logger.info(f"Force killed node process after timeout")
            
            return True
        except Exception as e:
            logger.error(f"Failed to terminate node process: {e}")
            return False
    
    @keyword
    def get_process_output(self, process: subprocess.Popen, timeout: float = 1.0) -> Dict[str, str]:
        """
        Get the output from a running process.
        
        Args:
            process: Popen process object
            timeout: Timeout for reading output
            
        Returns:
            Dictionary with 'stdout' and 'stderr' keys containing process output
            
        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker |
            | Sleep | 2s |
            | ${output}= | Get Process Output | ${process} |
            | Log | Stdout: ${output}[stdout] |
        """
        try:
            stdout, stderr = process.communicate(timeout=timeout)
            return {
                'stdout': stdout or '',
                'stderr': stderr or ''
            }
        except subprocess.TimeoutExpired:
            # Process is still running, return empty output
            return {'stdout': '', 'stderr': ''}
        except Exception as e:
            logger.error(f"Error getting process output: {e}")
            return {'stdout': '', 'stderr': ''}
    
    @keyword
    def is_process_running(self, process: subprocess.Popen) -> bool:
        """
        Check if a process is still running.
        
        Args:
            process: Popen process object to check
            
        Returns:
            True if process is still running, False otherwise
            
        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker |
            | ${running}= | Is Process Running | ${process} |
            | Should Be True | ${running} |
        """
        return process.poll() is None

    # ============================================================================
    # GAZEBO OPERATIONS
    # ============================================================================
    
    @keyword
    def shutdown_gazebo(self, force: bool = False) -> bool:
        """
        Shutdown all Gazebo processes.
        
        Args:
            force: If True, use SIGKILL instead of SIGTERM
            
        Returns:
            True if Gazebo processes were terminated successfully
            
        Example:
            | ${shutdown}= | Shutdown Gazebo |
            | Should Be True | ${shutdown} |
        """
        try:
            logger.info("Shutting down Gazebo processes...")
            
            # Find and kill Gazebo processes
            gazebo_processes = []
            
            # Common Gazebo process names
            gazebo_names = [
                'gazebo', 'gz-sim', 'gz-sim-server', 'gz-sim-gui', 
                'ign-gazebo', 'ign-gazebo-server', 'ign-gazebo-gui',
                'gz', 'ign'
            ]
            
            for name in gazebo_names:
                try:
                    # Find processes by name
                    result = subprocess.run(
                        ['pgrep', '-f', name],
                        capture_output=True,
                        text=True,
                        timeout=5.0
                    )
                    
                    if result.returncode == 0:
                        pids = result.stdout.strip().split('\n')
                        for pid in pids:
                            if pid.strip():
                                gazebo_processes.append(int(pid.strip()))
                                
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError, ValueError):
                    continue
            
            if not gazebo_processes:
                logger.info("No Gazebo processes found")
                return True
            
            logger.info(f"Found {len(gazebo_processes)} Gazebo processes: {gazebo_processes}")
            
            # Terminate processes
            signal = 9 if force else 15  # SIGKILL or SIGTERM
            
            for pid in gazebo_processes:
                try:
                    if force:
                        os.kill(pid, signal)
                        logger.info(f"Force killed Gazebo process {pid}")
                    else:
                        os.kill(pid, signal)
                        logger.info(f"Terminated Gazebo process {pid}")
                except ProcessLookupError:
                    logger.info(f"Process {pid} already terminated")
                except PermissionError:
                    logger.warn(f"Permission denied to terminate process {pid}")
                except Exception as e:
                    logger.warn(f"Failed to terminate process {pid}: {e}")
            
            # Wait a bit for processes to terminate
            time.sleep(2.0)
            
            # Check if any processes are still running
            remaining_processes = []
            for pid in gazebo_processes:
                try:
                    os.kill(pid, 0)  # Check if process exists
                    remaining_processes.append(pid)
                except ProcessLookupError:
                    pass  # Process is gone
            
            if remaining_processes and not force:
                logger.warn(f"Some processes still running, force killing: {remaining_processes}")
                for pid in remaining_processes:
                    try:
                        os.kill(pid, 9)
                        logger.info(f"Force killed remaining process {pid}")
                    except ProcessLookupError:
                        pass
                    except Exception as e:
                        logger.warn(f"Failed to force kill process {pid}: {e}")
            
            logger.info("Gazebo shutdown completed")
            return True
            
        except Exception as e:
            logger.error(f"Failed to shutdown Gazebo: {e}")
            return False
    
    @keyword
    def kill_all_ros_processes(self, force: bool = False) -> bool:
        """
        Kill all ROS2 processes (use with caution).
        
        Args:
            force: If True, use SIGKILL instead of SIGTERM
            
        Returns:
            True if ROS processes were terminated successfully
            
        Example:
            | ${killed}= | Kill All ROS Processes |
            | Should Be True | ${killed} |
        """
        try:
            logger.info("Killing all ROS2 processes...")
            
            # Find ROS2 processes
            ros_processes = []
            
            try:
                result = subprocess.run(
                    ['pgrep', '-f', 'ros2'],
                    capture_output=True,
                    text=True,
                    timeout=5.0
                )
                
                if result.returncode == 0:
                    pids = result.stdout.strip().split('\n')
                    for pid in pids:
                        if pid.strip():
                            ros_processes.append(int(pid.strip()))
                            
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, ValueError):
                pass
            
            if not ros_processes:
                logger.info("No ROS2 processes found")
                return True
            
            logger.info(f"Found {len(ros_processes)} ROS2 processes: {ros_processes}")
            
            # Terminate processes
            signal = 9 if force else 15
            
            for pid in ros_processes:
                try:
                    os.kill(pid, signal)
                    logger.info(f"Terminated ROS2 process {pid}")
                except ProcessLookupError:
                    logger.info(f"Process {pid} already terminated")
                except Exception as e:
                    logger.warn(f"Failed to terminate process {pid}: {e}")
            
            time.sleep(1.0)
            logger.info("ROS2 process cleanup completed")
            return True
            
        except Exception as e:
            logger.error(f"Failed to kill ROS2 processes: {e}")
            return False

    @keyword
    def shutdown_process(self, process_name: str, force: bool = False) -> bool:
        """
        Shutdown a process by name.
        
        Args:
            process_name: Name of the process to shutdown
            force: If True, use SIGKILL instead of SIGTERM
            
        Returns:
            True if process was terminated successfully

        Example:
            | ${shutdown}= | Shutdown Process | ros2 |
            | Should Be True | ${shutdown} |
        """
        try:
            logger.info(f"Shutting down process: {process_name}")
            
            # Find processes by name
            result = subprocess.run(
                ['pgrep', '-f', process_name],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid.strip():
                        os.kill(int(pid.strip()), 9 if force else 15)
                        logger.info(f"Terminated process {pid.strip()}")
            
            logger.info(f"Process {process_name} shutdown completed")
            return True
            
        except Exception as e:
            logger.error(f"Failed to shutdown process {process_name}: {e}")
            return False

    