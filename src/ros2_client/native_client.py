"""
Native ROS2 node operations using rclpy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool, Int32, Float32
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan, Image
import tf2_ros
from tf2_ros import TransformException
import threading
import time
from typing import List, Dict, Any, Optional, Callable, Union
from robot.api.deco import keyword
from robot.api import logger

from .utils import ROS2BaseClient


class ROS2NativeClient(ROS2BaseClient):
    """Native ROS2 node operations using rclpy."""
    
    def __init__(self, timeout: float = 10.0, node_name: str = "robotframework_ros2"):
        """Initialize native client with ROS2 node."""
        super().__init__(timeout)
        self.node_name = node_name
        self.node: Optional[Node] = None
        self._executor = None
        self._executor_thread = None
        self._message_buffer: Dict[str, List[Any]] = {}
        self._subscribers: Dict[str, Any] = {}
        self._publishers: Dict[str, Any] = {}
        self._callback_group = ReentrantCallbackGroup()
        self._initialized = False
        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._tf_listener: Optional[tf2_ros.TransformListener] = None
        
    def _ensure_initialized(self):
        """Ensure ROS2 node is initialized."""
        if not self._initialized:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node(self.node_name)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self.node)
            
            # Initialize tf2 buffer and listener
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)
            
            # Start executor in separate thread
            self._executor_thread = threading.Thread(
                target=self._executor.spin, 
                daemon=True
            )
            self._executor_thread.start()
            
            self._initialized = True
            logger.info(f"Native ROS2 client initialized with node: {self.node_name}")
    
    def _get_message_class(self, message_type: str):
        """Get the actual message class from string type."""
        message_classes = {
            "std_msgs/msg/String": String,
            "std_msgs/msg/Bool": Bool,
            "std_msgs/msg/Int32": Int32,
            "std_msgs/msg/Float32": Float32,
            "geometry_msgs/msg/PoseStamped": PoseStamped,
            "geometry_msgs/msg/Twist": Twist,
            "geometry_msgs/msg/Point": Point,
            "geometry_msgs/msg/Quaternion": Quaternion,
            "sensor_msgs/msg/LaserScan": LaserScan,
            "sensor_msgs/msg/Image": Image,
        }
        
        if message_type in message_classes:
            return message_classes[message_type]
        else:
            raise ValueError(f"Unsupported message type: {message_type}")
    
    def _create_message(self, msg_class, data: Any):
        """Create a message instance from data."""
        if msg_class == String:
            return String(data=str(data))
        elif msg_class == Bool:
            return Bool(data=bool(data))
        elif msg_class == Int32:
            return Int32(data=int(data))
        elif msg_class == Float32:
            return Float32(data=float(data))
        elif msg_class == Twist:
            msg = Twist()
            if isinstance(data, dict):
                if 'linear' in data:
                    msg.linear.x = float(data['linear'].get('x', 0.0))
                    msg.linear.y = float(data['linear'].get('y', 0.0))
                    msg.linear.z = float(data['linear'].get('z', 0.0))
                if 'angular' in data:
                    msg.angular.x = float(data['angular'].get('x', 0.0))
                    msg.angular.y = float(data['angular'].get('y', 0.0))
                    msg.angular.z = float(data['angular'].get('z', 0.0))
            return msg
        elif msg_class == PoseStamped:
            msg = PoseStamped()
            if isinstance(data, dict):
                # Set header
                if 'header' in data:
                    header = data['header']
                    msg.header.frame_id = header.get('frame_id', 'map')
                    if 'stamp' in header:
                        msg.header.stamp.sec = int(header['stamp'].get('sec', 0))
                        msg.header.stamp.nanosec = int(header['stamp'].get('nanosec', 0))
                
                # Set pose
                if 'pose' in data:
                    pose = data['pose']
                    if 'position' in pose:
                        pos = pose['position']
                        msg.pose.position.x = float(pos.get('x', 0.0))
                        msg.pose.position.y = float(pos.get('y', 0.0))
                        msg.pose.position.z = float(pos.get('z', 0.0))
                    if 'orientation' in pose:
                        ori = pose['orientation']
                        msg.pose.orientation.x = float(ori.get('x', 0.0))
                        msg.pose.orientation.y = float(ori.get('y', 0.0))
                        msg.pose.orientation.z = float(ori.get('z', 0.0))
                        msg.pose.orientation.w = float(ori.get('w', 1.0))
            return msg
        else:
            raise ValueError(f"Unsupported message class: {msg_class}")
    
    def _extract_message_data(self, msg) -> Any:
        """Extract data from a message for storage."""
        if hasattr(msg, 'data'):
            return msg.data
        elif hasattr(msg, 'pose'):
            return {
                'header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    }
                },
                'pose': {
                    'position': {
                        'x': msg.pose.position.x,
                        'y': msg.pose.position.y,
                        'z': msg.pose.position.z
                    },
                    'orientation': {
                        'x': msg.pose.orientation.x,
                        'y': msg.pose.orientation.y,
                        'z': msg.pose.orientation.z,
                        'w': msg.pose.orientation.w
                    }
                }
            }
        elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            return {
                'linear': {
                    'x': msg.linear.x,
                    'y': msg.linear.y,
                    'z': msg.linear.z
                },
                'angular': {
                    'x': msg.angular.x,
                    'y': msg.angular.y,
                    'z': msg.angular.z
                }
            }
        else:
            return str(msg)

    # ============================================================================
    # NATIVE TOPIC OPERATIONS
    # ============================================================================
    
    @keyword
    def create_publisher(self, topic_name: str, message_type: str = "std_msgs/msg/String", qos_profile: int = 10) -> str:
        """
        Create a native ROS2 publisher.
        
        Args:
            topic_name: Name of the topic to publish to
            message_type: Type of the message (default: std_msgs/msg/String)
            qos_profile: QoS profile depth (default: 10)
            
        Returns:
            Publisher ID for use with other methods
            
        Example:
            | ${publisher}= | Create Publisher | /chatter | std_msgs/msg/String |
            | Publish Message | ${publisher} | Hello World |
        """
        self._ensure_initialized()
        
        msg_class = self._get_message_class(message_type)
        
        qos = QoSProfile(
            depth=qos_profile,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        publisher = self.node.create_publisher(
            msg_class, 
            topic_name, 
            qos,
            callback_group=self._callback_group
        )
        publisher_id = f"{topic_name}_{len(self._publishers)}"
        self._publishers[publisher_id] = {
            'publisher': publisher,
            'topic_name': topic_name,
            'message_type': message_type,
            'msg_class': msg_class
        }
        
        logger.info(f"Created native publisher for topic '{topic_name}' with ID: {publisher_id}")
        return publisher_id
    
    @keyword
    def publish_message(self, publisher_id: str, data: Any) -> bool:
        """
        Publish a message using a native publisher.
        
        Args:
            publisher_id: ID of the publisher (from create_publisher)
            data: Message data to publish
            
        Returns:
            True if message was published successfully
            
        Example:
            | ${publisher}= | Create Publisher | /chatter |
            | Publish Message | ${publisher} | Hello World |
        """
        if publisher_id not in self._publishers:
            logger.error(f"Publisher ID '{publisher_id}' not found")
            return False
        
        try:
            publisher_info = self._publishers[publisher_id]
            publisher = publisher_info['publisher']
            msg_class = publisher_info['msg_class']
            
            message = self._create_message(msg_class, data)
            publisher.publish(message)
            logger.info(f"Published native message to publisher '{publisher_id}': {data}")
            return True
        except Exception as e:
            logger.error(f"Failed to publish native message: {e}")
            return False
    
    @keyword
    def create_subscriber(self, topic_name: str, message_type: str = "std_msgs/msg/String", buffer_size: int = 100) -> str:
        """
        Create a native ROS2 subscriber.
        
        Args:
            topic_name: Name of the topic to subscribe to
            message_type: Type of the message (default: std_msgs/msg/String)
            buffer_size: Maximum number of messages to buffer
            
        Returns:
            Subscriber ID for use with other methods
            
        Example:
            | ${subscriber}= | Create Subscriber | /chatter | std_msgs/msg/String |
            | ${message}= | Get Latest Message | /chatter |
        """
        self._ensure_initialized()
        
        msg_class = self._get_message_class(message_type)
        
        qos = QoSProfile(
            depth=buffer_size,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        def message_callback(msg):
            if topic_name not in self._message_buffer:
                self._message_buffer[topic_name] = []
            
            # Store message data
            message_data = self._extract_message_data(msg)
            self._message_buffer[topic_name].append({
                'timestamp': time.time(),
                'data': message_data,
                'raw_msg': msg
            })
            
            # Keep only the latest messages
            if len(self._message_buffer[topic_name]) > buffer_size:
                self._message_buffer[topic_name] = self._message_buffer[topic_name][-buffer_size:]
        
        subscriber = self.node.create_subscription(
            msg_class, 
            topic_name, 
            message_callback, 
            qos,
            callback_group=self._callback_group
        )
        subscriber_id = f"{topic_name}_{len(self._subscribers)}"
        self._subscribers[subscriber_id] = {
            'subscriber': subscriber,
            'topic_name': topic_name,
            'message_type': message_type,
            'msg_class': msg_class
        }
        
        logger.info(f"Created native subscriber for topic '{topic_name}' with ID: {subscriber_id}")
        return subscriber_id
    
    @keyword
    def get_latest_message(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Get the latest message from a subscribed topic.
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            Latest message data or None if no messages
            
        Example:
            | ${subscriber}= | Create Subscriber | /chatter |
            | Sleep | 2s |
            | ${message}= | Get Latest Message | /chatter |
            | Should Not Be None | ${message} |
        """
        if topic_name not in self._message_buffer or not self._message_buffer[topic_name]:
            return None
        
        return self._message_buffer[topic_name][-1]
    
    @keyword
    def get_all_messages(self, topic_name: str) -> List[Dict[str, Any]]:
        """
        Get all buffered messages from a subscribed topic.
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            List of all buffered messages
            
        Example:
            | ${subscriber}= | Create Subscriber | /chatter |
            | Sleep | 5s |
            | ${messages}= | Get All Messages | /chatter |
            | Length Should Be Greater Than | ${messages} | 0 |
        """
        return self._message_buffer.get(topic_name, [])
    
    @keyword
    def clear_message_buffer(self, topic_name: str) -> bool:
        """
        Clear the message buffer for a topic.
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            True if buffer was cleared
            
        Example:
            | Clear Message Buffer | /chatter |
        """
        if topic_name in self._message_buffer:
            self._message_buffer[topic_name] = []
            logger.info(f"Cleared message buffer for topic '{topic_name}'")
            return True
        return False
    
    @keyword
    def wait_for_message(self, topic_name: str, timeout: float = 10.0, check_interval: float = 0.1) -> Optional[Dict[str, Any]]:
        """
        Wait for a message to arrive on a subscribed topic.
        
        Args:
            topic_name: Name of the topic to wait for
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            Message data if received within timeout, None otherwise
            
        Example:
            | ${subscriber}= | Create Subscriber | /chatter |
            | ${message}= | Wait For Message | /chatter | timeout=5.0 |
            | Should Not Be None | ${message} |
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if topic_name in self._message_buffer and self._message_buffer[topic_name]:
                message = self._message_buffer[topic_name][-1]
                logger.info(f"Received message on topic '{topic_name}' after {time.time() - start_time:.2f}s")
                return message
            
            time.sleep(check_interval)
        
        logger.warn(f"No message received on topic '{topic_name}' within {timeout}s")
        return None

    # ============================================================================
    # NATIVE PARAMETER OPERATIONS
    # ============================================================================
    
    @keyword
    def get_parameter(self, parameter_name: str, default_value: Any = None) -> Any:
        """
        Get a parameter value using native ROS2 parameter client.
        
        Args:
            parameter_name: Name of the parameter
            default_value: Default value if parameter doesn't exist
            
        Returns:
            Parameter value or default value
            
        Example:
            | ${value}= | Get Parameter Native | my_param | default_value=42 |
        """
        self._ensure_initialized()
        
        try:
            param = self.node.get_parameter(parameter_name)
            value = param.value
            logger.info(f"Got native parameter '{parameter_name}': {value}")
            return value
        except Exception as e:
            if default_value is not None:
                logger.info(f"Parameter '{parameter_name}' not found, using default: {default_value}")
                return default_value
            else:
                logger.error(f"Failed to get native parameter '{parameter_name}': {e}")
                raise
    
    @keyword
    def set_parameter(self, parameter_name: str, value: Any) -> bool:
        """
        Set a parameter value using native ROS2 parameter client.
        
        Args:
            parameter_name: Name of the parameter
            value: Value to set
            
        Returns:
            True if parameter was set successfully
            
        Example:
            | ${success}= | Set Parameter Native | my_param | 42 |
            | Should Be True | ${success} |
        """
        self._ensure_initialized()
        
        try:
            # Determine parameter type based on value type
            if isinstance(value, bool):
                param_type = Parameter.Type.BOOL
            elif isinstance(value, int):
                param_type = Parameter.Type.INTEGER
            elif isinstance(value, float):
                param_type = Parameter.Type.DOUBLE
            elif isinstance(value, str):
                param_type = Parameter.Type.STRING
            elif isinstance(value, list):
                # For arrays, determine type based on first element
                if not value:
                    param_type = Parameter.Type.STRING_ARRAY  # Default to string array for empty lists
                elif isinstance(value[0], bool):
                    param_type = Parameter.Type.BOOL_ARRAY
                elif isinstance(value[0], int):
                    param_type = Parameter.Type.INTEGER_ARRAY
                elif isinstance(value[0], float):
                    param_type = Parameter.Type.DOUBLE_ARRAY
                else:
                    param_type = Parameter.Type.STRING_ARRAY
            else:
                # Default to string for unknown types
                param_type = Parameter.Type.STRING
                value = str(value)
            
            param = Parameter(parameter_name, param_type, value)
            self.node.set_parameters([param])
            logger.info(f"Set native parameter '{parameter_name}' to: {value} (type: {param_type})")
            return True
        except Exception as e:
            logger.error(f"Failed to set native parameter '{parameter_name}': {e}")
            return False
    
    @keyword
    def declare_parameter(self, parameter_name: str, default_value: Any) -> bool:
        """
        Declare a parameter with a default value.
        
        Args:
            parameter_name: Name of the parameter
            default_value: Default value for the parameter
            
        Returns:
            True if parameter was declared successfully
            
        Example:
            | ${success}= | Declare Parameter Native | my_param | 42 |
            | Should Be True | ${success} |
        """
        self._ensure_initialized()
        
        try:
            self.node.declare_parameter(parameter_name, default_value)
            logger.info(f"Declared native parameter '{parameter_name}' with default: {default_value}")
            return True
        except Exception as e:
            logger.error(f"Failed to declare native parameter '{parameter_name}': {e}")
            return False
    
    @keyword
    def list_parameters(self) -> List[str]:
        """
        List all parameters for the native node.
        
        Returns:
            List of parameter names
            
        Example:
            | ${params}= | List Parameters Native |
            | Should Contain | ${params} | my_param |
        """
        self._ensure_initialized()
        
        try:
            param_names = list(self.node._parameters.keys())
            logger.info(f"Found {len(param_names)} native parameters: {param_names}")
            return param_names
        except Exception as e:
            logger.error(f"Failed to list native parameters: {e}")
            return []
    
    @keyword
    def parameter_exists(self, parameter_name: str) -> bool:
        """
        Check if a parameter exists on the native node.
        
        Args:
            parameter_name: Name of the parameter to check
            
        Returns:
            True if parameter exists, False otherwise
            
        Example:
            | ${exists}= | Parameter Exists Native | my_param |
            | Should Be True | ${exists} |
        """
        self._ensure_initialized()
        
        try:
            exists = self.node.has_parameter(parameter_name)
            logger.info(f"Parameter '{parameter_name}' exists on native node: {exists}")
            return exists
        except Exception as e:
            logger.error(f"Error checking if parameter '{parameter_name}' exists on native node: {e}")
            return False
    
    @keyword
    def get_all_parameters(self) -> Dict[str, Any]:
        """
        Get all parameters and their values for the native node.
        
        Returns:
            Dictionary mapping parameter names to their values
            
        Example:
            | ${params}= | Get All Parameters Native |
            | Should Be Equal | ${params}[my_param] | 42 |
        """
        self._ensure_initialized()
        
        try:
            all_params = {}
            for param_name in self.node._parameters.keys():
                try:
                    param = self.node.get_parameter(param_name)
                    all_params[param_name] = param.value
                except Exception as e:
                    logger.warn(f"Failed to get value for native parameter '{param_name}': {e}")
                    all_params[param_name] = None
            
            logger.info(f"Retrieved {len(all_params)} native parameters")
            return all_params
        except Exception as e:
            logger.error(f"Failed to get all native parameters: {e}")
            return {}

    # ============================================================================
    # TF2 OPERATIONS
    # ============================================================================
    
    @keyword
    def get_tf(self, target_frame: str, source_frame: str, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        Get transform between two frames using tf2.
        
        Args:
            target_frame: Target frame name
            source_frame: Source frame name
            timeout: Timeout in seconds for the transform lookup
            
        Returns:
            Transform data as dictionary or None if transform not available
            
        Example:
            | ${transform}= | Get Tf | map | base_link | timeout=2.0 |
            | Should Not Be None | ${transform} |
            | Log | Transform: ${transform} |
        """
        self.cleanup()
        self._ensure_initialized()
        
        if not self._tf_buffer:
            logger.error("TF2 buffer not initialized")
            return None
        
        try:
            # Get the transform
            transform = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout)
            )
            
            # Extract transform data
            transform_data = {
                'header': {
                    'frame_id': transform.header.frame_id,
                    'stamp': {
                        'sec': transform.header.stamp.sec,
                        'nanosec': transform.header.stamp.nanosec
                    }
                },
                'child_frame_id': transform.child_frame_id,
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                }
            }
            
            logger.info(f"Got transform from '{source_frame}' to '{target_frame}': {transform_data}")
            return transform_data
            
        except TransformException as e:
            logger.warn(f"Transform from '{source_frame}' to '{target_frame}' not available: {e}")
            return None
        except Exception as e:
            logger.error(f"Error getting transform from '{source_frame}' to '{target_frame}': {e}")
            return None
    
    @keyword
    def get_tf_at_time(self, target_frame: str, source_frame: str, time_sec: float, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        Get transform between two frames at a specific time using tf2.
        
        Args:
            target_frame: Target frame name
            source_frame: Source frame name
            time_sec: Time in seconds (relative to node start)
            timeout: Timeout in seconds for the transform lookup
            
        Returns:
            Transform data as dictionary or None if transform not available
            
        Example:
            | ${transform}= | Get Tf At Time | map | base_link | 5.0 | timeout=2.0 |
            | Should Not Be None | ${transform} |
        """
        self._ensure_initialized()
        
        if not self._tf_buffer:
            logger.error("TF2 buffer not initialized")
            return None
        
        try:
            # Create time from seconds
            target_time = rclpy.time.Time(seconds=time_sec)
            
            # Get the transform at specific time
            transform = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=target_time,
                timeout=rclpy.duration.Duration(seconds=timeout)
            )
            
            # Extract transform data
            transform_data = {
                'header': {
                    'frame_id': transform.header.frame_id,
                    'stamp': {
                        'sec': transform.header.stamp.sec,
                        'nanosec': transform.header.stamp.nanosec
                    }
                },
                'child_frame_id': transform.child_frame_id,
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                }
            }
            
            logger.info(f"Got transform from '{source_frame}' to '{target_frame}' at time {time_sec}s: {transform_data}")
            return transform_data
            
        except TransformException as e:
            logger.warn(f"Transform from '{source_frame}' to '{target_frame}' at time {time_sec}s not available: {e}")
            return None
        except Exception as e:
            logger.error(f"Error getting transform from '{source_frame}' to '{target_frame}' at time {time_sec}s: {e}")
            return None
    
    @keyword
    def can_transform(self, target_frame: str, source_frame: str, timeout: float = 1.0) -> bool:
        """
        Check if transform between two frames is available.
        
        Args:
            target_frame: Target frame name
            source_frame: Source frame name
            timeout: Timeout in seconds for the check
            
        Returns:
            True if transform is available, False otherwise
            
        Example:
            | ${available}= | Can Transform | map | base_link | timeout=1.0 |
            | Should Be True | ${available} |
        """
        self._ensure_initialized()
        
        if not self._tf_buffer:
            logger.error("TF2 buffer not initialized")
            return False
        
        try:
            can_transform = self._tf_buffer.can_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout)
            )
            
            logger.info(f"Transform from '{source_frame}' to '{target_frame}' available: {can_transform}")
            return can_transform
            
        except Exception as e:
            logger.error(f"Error checking transform availability from '{source_frame}' to '{target_frame}': {e}")
            return False

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================
    
    @keyword
    def cleanup(self):
        """Clean up resources."""
        if self._initialized and self.node:
            # Clean up tf2 resources
            if self._tf_listener:
                self._tf_listener = None
            if self._tf_buffer:
                self._tf_buffer = None
            
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self._initialized = False
            logger.info("Native ROS2 client cleaned up")
    
    @keyword
    def get_client_info(self) -> Dict[str, Any]:
        """Get information about the native client."""
        self._ensure_initialized()
        return {
            'initialized': self._initialized,
            'node_name': self.node_name,
            'publishers': list(self._publishers.keys()),
            'subscribers': list(self._subscribers.keys()),
            'buffered_topics': list(self._message_buffer.keys()),
            'total_messages': sum(len(msgs) for msgs in self._message_buffer.values()),
            'tf2_available': self._tf_buffer is not None and self._tf_listener is not None
        }
