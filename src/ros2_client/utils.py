"""
Common utilities and base classes for ROS2 Robot Framework Library
"""

import subprocess
import time
from typing import List, Dict, Any, Optional, Union
from pathlib import Path
from robot.api.deco import keyword
from robot.api import logger


class ROS2BaseClient:
    """Base class with common utilities for ROS2 operations."""
    
    ROBOT_LIBRARY_SCOPE = 'GLOBAL'
    ROBOT_LIBRARY_DOC_FORMAT = 'ROBOT'
    
    def __init__(self, timeout: float = 10.0):
        """Initialize base client with common settings."""
        self.timeout = timeout
        self._ros2_executable = self._find_ros2_executable()
        self._initialized = False
    
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
                check=False
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


class ROS2CLIUtils(ROS2BaseClient):
    """CLI-based ROS2 operations using subprocess calls."""
    
    def __init__(self, timeout: float = 10.0):
        """Initialize CLI utils."""
        super().__init__(timeout)
        self._initialized = True
    
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
        response_text = result.stdout.strip()
        logger.info(f"Service call response for '{service_name}': {response_text}")
        
        # Try to parse as JSON-like structure
        try:
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

