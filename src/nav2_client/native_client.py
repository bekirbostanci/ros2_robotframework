"""
Native Navigation2 operations using rclpy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
import threading
import time
import math
from typing import List, Dict, Any, Optional, Union, Tuple
from robot.api.deco import keyword
from robot.api import logger

# Navigation2 message imports
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
# Note: Navigation2 services are typically actions, not services
# We'll use actions for navigation and basic services for costmap operations

from .utils import Nav2BaseClient, Pose, NavigationResult


class Nav2NativeClient(Nav2BaseClient):
    """Native Navigation2 operations using rclpy."""
    
    def __init__(self, timeout: float = 30.0, action_timeout: float = 60.0, node_name: str = "robotframework_nav2"):
        """Initialize native client with ROS2 node."""
        super().__init__(timeout, action_timeout)
        self.node_name = node_name
        self.node: Optional[Node] = None
        self._executor = None
        self._executor_thread = None
        self._callback_group = ReentrantCallbackGroup()
        self._initialized = False
        
        # Action clients
        self._navigate_to_pose_action_client: Optional[ActionClient] = None
        self._navigate_through_poses_action_client: Optional[ActionClient] = None
        
        # Service clients (only for costmap operations)
        self._clear_global_costmap_client: Optional[Any] = None
        self._clear_local_costmap_client: Optional[Any] = None
        
        # Publishers
        self._initial_pose_publisher: Optional[Any] = None
        self._cmd_vel_publisher: Optional[Any] = None
        
        # Subscribers
        self._amcl_pose_subscriber: Optional[Any] = None
        self._global_costmap_subscriber: Optional[Any] = None
        self._local_costmap_subscriber: Optional[Any] = None
        
        # Message buffers
        self._message_buffer: Dict[str, List[Any]] = {}
        
    def _ensure_initialized(self):
        """Ensure ROS2 node is initialized."""
        if not self._initialized:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node(self.node_name)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self.node)
            
            # Initialize action clients
            self._navigate_to_pose_action_client = ActionClient(
                self.node, 
                NavigateToPose, 
                'navigate_to_pose',
                callback_group=self._callback_group
            )
            self._navigate_through_poses_action_client = ActionClient(
                self.node, 
                NavigateThroughPoses, 
                'navigate_through_poses',
                callback_group=self._callback_group
            )
            
            # Initialize service clients (only for costmap operations)
            self._clear_global_costmap_client = self.node.create_client(
                Empty, 
                'global_costmap/clear_entirely_global_costmap',
                callback_group=self._callback_group
            )
            self._clear_local_costmap_client = self.node.create_client(
                Empty, 
                'local_costmap/clear_entirely_local_costmap',
                callback_group=self._callback_group
            )
            
            # Initialize publishers
            self._initial_pose_publisher = self.node.create_publisher(
                PoseWithCovarianceStamped, 
                'initialpose', 
                10,
                callback_group=self._callback_group
            )
            self._cmd_vel_publisher = self.node.create_publisher(
                Twist, 
                'cmd_vel', 
                10,
                callback_group=self._callback_group
            )
            
            # Initialize subscribers
            self._amcl_pose_subscriber = self.node.create_subscription(
                PoseWithCovarianceStamped, 
                'amcl_pose', 
                self._amcl_pose_callback, 
                10,
                callback_group=self._callback_group
            )
            self._global_costmap_subscriber = self.node.create_subscription(
                LaserScan,  # Simplified - costmap is actually nav2_msgs/msg/Costmap
                'global_costmap/costmap', 
                self._global_costmap_callback, 
                10,
                callback_group=self._callback_group
            )
            self._local_costmap_subscriber = self.node.create_subscription(
                LaserScan,  # Simplified - costmap is actually nav2_msgs/msg/Costmap
                'local_costmap/costmap', 
                self._local_costmap_callback, 
                10,
                callback_group=self._callback_group
            )
            
            # Start executor in separate thread
            self._executor_thread = threading.Thread(
                target=self._executor.spin, 
                daemon=True
            )
            self._executor_thread.start()
            
            self._initialized = True
            logger.info(f"Native Navigation2 client initialized with node: {self.node_name}")
    
    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback for AMCL pose messages."""
        if 'amcl_pose' not in self._message_buffer:
            self._message_buffer['amcl_pose'] = []
        
        pose_data = {
            'header': {
                'frame_id': msg.header.frame_id,
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            },
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            },
            'covariance': list(msg.pose.covariance)
        }
        
        self._message_buffer['amcl_pose'].append({
            'timestamp': time.time(),
            'data': pose_data,
            'raw_msg': msg
        })
        
        # Keep only the latest messages
        if len(self._message_buffer['amcl_pose']) > 100:
            self._message_buffer['amcl_pose'] = self._message_buffer['amcl_pose'][-100:]
    
    def _global_costmap_callback(self, msg: LaserScan):
        """Callback for global costmap messages."""
        if 'global_costmap' not in self._message_buffer:
            self._message_buffer['global_costmap'] = []
        
        # Simplified costmap data extraction
        costmap_data = {
            'timestamp': time.time(),
            'data': {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            },
            'raw_msg': msg
        }
        
        self._message_buffer['global_costmap'].append(costmap_data)
        
        # Keep only the latest messages
        if len(self._message_buffer['global_costmap']) > 10:
            self._message_buffer['global_costmap'] = self._message_buffer['global_costmap'][-10:]
    
    def _local_costmap_callback(self, msg: LaserScan):
        """Callback for local costmap messages."""
        if 'local_costmap' not in self._message_buffer:
            self._message_buffer['local_costmap'] = []
        
        # Simplified costmap data extraction
        costmap_data = {
            'timestamp': time.time(),
            'data': {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            },
            'raw_msg': msg
        }
        
        self._message_buffer['local_costmap'].append(costmap_data)
        
        # Keep only the latest messages
        if len(self._message_buffer['local_costmap']) > 10:
            self._message_buffer['local_costmap'] = self._message_buffer['local_costmap'][-10:]

    def _create_pose_stamped(self, x: float, y: float, theta: float, frame_id: str = "map") -> PoseStamped:
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose

    def _create_pose_with_covariance_stamped(self, x: float, y: float, theta: float, frame_id: str = "map") -> PoseWithCovarianceStamped:
        """Create a PoseWithCovarianceStamped message."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Set default covariance (identity matrix)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.25  # x variance
        pose_msg.pose.covariance[7] = 0.25  # y variance
        pose_msg.pose.covariance[35] = 0.06853891945200942  # yaw variance
        
        return pose_msg

    # ============================================================================
    # NATIVE NAVIGATION OPERATIONS
    # ============================================================================
    
    @keyword
    def navigate_to_pose_native(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> NavigationResult:
        """
        Navigate to a specific pose using native Navigation2 action client.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default action timeout
            
        Returns:
            NavigationResult object with success status and details
            
        Example:
            | ${result}= | Navigate To Pose Native | 2.0 | 1.0 | 1.57 |
            | Should Be True | ${result.success} |
        """
        self._ensure_initialized()
        
        goal_pose = Pose(x, y, theta)
        self._goal_pose = goal_pose
        self._navigation_active = True
        
        timeout_value = timeout or self.action_timeout
        
        logger.info(f"Native navigation to pose: x={x}, y={y}, theta={theta} (frame: {frame_id})")
        
        try:
            # Wait for action server
            if not self._navigate_to_pose_action_client.wait_for_server(timeout_sec=10.0):
                logger.error("NavigateToPose action server not available")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message="NavigateToPose action server not available"
                )
            
            # Create goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._create_pose_stamped(x, y, theta, frame_id)
            
            # Send goal
            future = self._navigate_to_pose_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.error("Navigation goal was rejected")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message="Navigation goal was rejected"
                )
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_value)
            
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                logger.info(f"Successfully navigated to pose: {goal_pose}")
                self._current_pose = goal_pose
                self._navigation_active = False
                return NavigationResult(
                    success=True,
                    message="Navigation completed successfully",
                    final_pose=goal_pose
                )
            else:
                logger.warn(f"Navigation failed with status: {result.status}")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message=f"Navigation failed with status: {result.status}"
                )
                
        except Exception as e:
            logger.error(f"Native navigation error: {e}")
            self._navigation_active = False
            return NavigationResult(
                success=False,
                message=f"Native navigation error: {e}"
            )
    
    def send_navigation_goal_only(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> bool:
        """
        Send navigation goal without waiting for completion.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            
        Returns:
            True if goal was sent successfully
        """
        self._ensure_initialized()
        
        goal_pose = Pose(x, y, theta)
        self._goal_pose = goal_pose
        self._navigation_active = True
        
        logger.info(f"Sending navigation goal to pose: x={x}, y={y}, theta={theta} (frame: {frame_id})")
        
        try:
            # Wait for action server
            if not self._navigate_to_pose_action_client.wait_for_server(timeout_sec=5.0):
                logger.error("NavigateToPose action server not available")
                self._navigation_active = False
                return False
            
            # Create goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._create_pose_stamped(x, y, theta, frame_id)
            
            # Send goal without waiting for acceptance or completion
            future = self._navigate_to_pose_action_client.send_goal_async(goal_msg)
            
            # Just return True - the goal is sent, we don't wait for anything
            logger.info(f"Navigation goal sent to ({x}, {y}, {theta}) in frame {frame_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send navigation goal: {e}")
            self._navigation_active = False
            return False

    @keyword
    def navigate_through_poses_native(
        self, 
        poses: List[Dict[str, float]], 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> NavigationResult:
        """
        Navigate through a sequence of poses using native Navigation2 action client.
        
        Args:
            poses: List of pose dictionaries with 'x', 'y', 'theta' keys
            frame_id: Reference frame (default: "map")
            timeout: Override default action timeout
            
        Returns:
            NavigationResult object with success status and details
            
        Example:
            | @{poses}= | Create List | ${{'x': 1.0, 'y': 0.0, 'theta': 0.0}} | ${{'x': 2.0, 'y': 1.0, 'theta': 1.57}} |
            | ${result}= | Navigate Through Poses Native | ${poses} |
            | Should Be True | ${result.success} |
        """
        self._ensure_initialized()
        
        if not poses:
            return NavigationResult(
                success=False,
                message="No poses provided for navigation"
            )
        
        timeout_value = timeout or self.action_timeout
        self._navigation_active = True
        
        logger.info(f"Native navigation through {len(poses)} poses")
        
        try:
            # Wait for action server
            if not self._navigate_through_poses_action_client.wait_for_server(timeout_sec=10.0):
                logger.error("NavigateThroughPoses action server not available")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message="NavigateThroughPoses action server not available"
                )
            
            # Create goal
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = []
            
            for pose_dict in poses:
                pose = Pose.from_dict(pose_dict)
                pose_stamped = self._create_pose_stamped(pose.x, pose.y, pose.theta, frame_id)
                goal_msg.poses.append(pose_stamped)
            
            # Send goal
            future = self._navigate_through_poses_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.error("Navigation through poses goal was rejected")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message="Navigation through poses goal was rejected"
                )
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_value)
            
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                final_pose = Pose.from_dict(poses[-1]) if poses else None
                logger.info(f"Successfully navigated through {len(poses)} poses")
                self._current_pose = final_pose
                self._navigation_active = False
                return NavigationResult(
                    success=True,
                    message=f"Navigation through {len(poses)} poses completed successfully",
                    final_pose=final_pose
                )
            else:
                logger.warn(f"Navigation through poses failed with status: {result.status}")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message=f"Navigation through poses failed with status: {result.status}"
                )
                
        except Exception as e:
            logger.error(f"Native navigation through poses error: {e}")
            self._navigation_active = False
            return NavigationResult(
                success=False,
                message=f"Native navigation through poses error: {e}"
            )
    
    @keyword
    def cancel_navigation_native(self, timeout: Optional[float] = None) -> bool:
        """
        Cancel the current navigation operation using native action client.
        
        Args:
            timeout: Override default timeout
            
        Returns:
            True if cancellation was successful
            
        Example:
            | ${cancelled}= | Cancel Navigation Native |
            | Should Be True | ${cancelled} |
        """
        self._ensure_initialized()
        
        logger.info("Cancelling current navigation (native)...")
        
        try:
            # Cancel all active goals
            if self._navigate_to_pose_action_client:
                self._navigate_to_pose_action_client.cancel_all_goals()
            if self._navigate_through_poses_action_client:
                self._navigate_through_poses_action_client.cancel_all_goals()
            
            self._navigation_active = False
            logger.info("Navigation cancelled successfully (native)")
            return True
                
        except Exception as e:
            logger.error(f"Error cancelling navigation (native): {e}")
            return False
    
    # ============================================================================
    # NATIVE POSE AND LOCALIZATION OPERATIONS
    # ============================================================================
    
    @keyword
    def get_current_pose_native(self, timeout: Optional[float] = None) -> Optional[Pose]:
        """
        Get the current robot pose from the localization system using native subscriber.
        
        Args:
            timeout: Override default timeout
            
        Returns:
            Current pose as Pose object, or None if unavailable
            
        Example:
            | ${pose}= | Get Current Pose Native |
            | Should Not Be None | ${pose} |
            | Log | Current position: x=${pose.x}, y=${pose.y} |
        """
        self._ensure_initialized()
        
        try:
            if 'amcl_pose' in self._message_buffer and self._message_buffer['amcl_pose']:
                latest_msg = self._message_buffer['amcl_pose'][-1]
                pose_data = latest_msg['data']['pose']
                
                # Convert quaternion to euler angle (yaw)
                qx = pose_data['orientation']['x']
                qy = pose_data['orientation']['y']
                qz = pose_data['orientation']['z']
                qw = pose_data['orientation']['w']
                
                # Convert quaternion to euler angle (yaw)
                theta = math.atan2(2 * (qw * qz), 1 - 2 * (qz * qz))
                
                pose = Pose(
                    x=pose_data['position']['x'],
                    y=pose_data['position']['y'],
                    theta=theta
                )
                
                self._current_pose = pose
                logger.info(f"Current pose (native): x={pose.x}, y={pose.y}, theta={pose.theta}")
                return pose
            
            logger.warn("Could not retrieve current pose (native)")
            return None
            
        except Exception as e:
            logger.error(f"Error getting current pose (native): {e}")
            return None
    
    @keyword
    def set_initial_pose_native(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> bool:
        """
        Set the initial pose for the robot using native publisher.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            
        Returns:
            True if initial pose was set successfully
            
        Example:
            | ${success}= | Set Initial Pose Native | 0.0 | 0.0 | 0.0 |
            | Should Be True | ${success} |
        """
        self._ensure_initialized()
        
        logger.info(f"Setting initial pose (native): x={x}, y={y}, theta={theta}")
        
        try:
            pose_msg = self._create_pose_with_covariance_stamped(x, y, theta, frame_id)
            self._initial_pose_publisher.publish(pose_msg)
            
            self._current_pose = Pose(x, y, theta)
            logger.info("Initial pose set successfully (native)")
            return True
                
        except Exception as e:
            logger.error(f"Error setting initial pose (native): {e}")
            return False
    
    @keyword
    def wait_for_localization_native(self, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """
        Wait for the robot to be localized using native subscriber.
        
        Args:
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            True if localization converged within timeout
            
        Example:
            | ${localized}= | Wait For Localization Native | timeout=60.0 |
            | Should Be True | ${localized} |
        """
        self._ensure_initialized()
        
        logger.info(f"Waiting for localization to converge (native, timeout: {timeout}s)")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                if 'amcl_pose' in self._message_buffer and self._message_buffer['amcl_pose']:
                    latest_msg = self._message_buffer['amcl_pose'][-1]
                    pose_data = latest_msg['data']['pose']
                    
                    # Simple check: if we can get a pose, assume localization is working
                    if pose_data['position']['x'] is not None and pose_data['position']['y'] is not None:
                        logger.info(f"Localization converged (native) after {time.time() - start_time:.2f}s")
                        return True
                
                time.sleep(check_interval)
                
            except Exception as e:
                logger.debug(f"Localization check error (native): {e}")
                time.sleep(check_interval)
        
        logger.warn(f"Localization did not converge (native) within {timeout}s")
        return False
    
    # ============================================================================
    # NATIVE PATH PLANNING OPERATIONS
    # ============================================================================
    
    # Note: compute_path_native removed - Navigation2 doesn't provide this as a service
    # Path planning is typically handled internally by the navigation stack
    
    # ============================================================================
    # NATIVE COSTMAP OPERATIONS
    # ============================================================================
    
    @keyword
    def get_costmap_info_native(self, costmap_type: str = "global", timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get information about the costmap using native subscriber.
        
        Args:
            costmap_type: Type of costmap ("global" or "local")
            timeout: Override default timeout
            
        Returns:
            Dictionary containing costmap information
            
        Example:
            | ${info}= | Get Costmap Info Native | global |
            | Should Contain | ${info} | ranges |
        """
        self._ensure_initialized()
        
        topic_name = f"{costmap_type}_costmap"
        
        try:
            if topic_name in self._message_buffer and self._message_buffer[topic_name]:
                latest_msg = self._message_buffer[topic_name][-1]
                info = latest_msg['data']
                logger.info(f"Retrieved {costmap_type} costmap info (native)")
                return info
            else:
                logger.warn(f"Could not retrieve {costmap_type} costmap info (native)")
                return {}
                
        except Exception as e:
            logger.error(f"Error getting costmap info (native): {e}")
            return {}
    
    @keyword
    def clear_costmap_native(self, costmap_type: str = "global", timeout: Optional[float] = None) -> bool:
        """
        Clear the specified costmap using native service client.
        
        Args:
            costmap_type: Type of costmap to clear ("global" or "local")
            timeout: Override default timeout
            
        Returns:
            True if costmap was cleared successfully
            
        Example:
            | ${cleared}= | Clear Costmap Native | global |
            | Should Be True | ${cleared} |
        """
        self._ensure_initialized()
        
        logger.info(f"Clearing {costmap_type} costmap (native)...")
        
        try:
            if costmap_type == "global":
                client = self._clear_global_costmap_client
            elif costmap_type == "local":
                client = self._clear_local_costmap_client
            else:
                logger.error(f"Invalid costmap type: {costmap_type}")
                return False
            
            if not client.wait_for_service(timeout_sec=10.0):
                logger.error(f"Clear {costmap_type} costmap service not available")
                return False
            
            # Create request
            request = Empty.Request()
            
            # Call service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout or 10.0)
            
            response = future.result()
            if response is not None:
                logger.info(f"{costmap_type.capitalize()} costmap cleared successfully (native)")
                return True
            else:
                logger.error(f"Failed to clear {costmap_type} costmap (native)")
                return False
                
        except Exception as e:
            logger.error(f"Error clearing costmap (native): {e}")
            return False
    
    # ============================================================================
    # NATIVE STATUS OPERATIONS
    # ============================================================================
    
    @keyword
    def wait_for_nav2_ready_native(self, timeout: float = 60.0, check_interval: float = 2.0) -> bool:
        """
        Wait for Navigation2 stack to be ready using native clients.
        
        Args:
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            True if Navigation2 is ready within timeout
            
        Example:
            | ${ready}= | Wait For Nav2 Ready Native | timeout=120.0 |
            | Should Be True | ${ready} |
        """
        self._ensure_initialized()
        
        logger.info(f"Waiting for Navigation2 to be ready (native, timeout: {timeout}s)")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Check if action servers are available
                navigate_to_pose_ready = self._navigate_to_pose_action_client.wait_for_server(timeout_sec=1.0)
                navigate_through_poses_ready = self._navigate_through_poses_action_client.wait_for_server(timeout_sec=1.0)
                
                # Check if service clients are available
                clear_global_ready = self._clear_global_costmap_client.wait_for_service(timeout_sec=1.0)
                clear_local_ready = self._clear_local_costmap_client.wait_for_service(timeout_sec=1.0)
                
                if (navigate_to_pose_ready and navigate_through_poses_ready and 
                    clear_global_ready and clear_local_ready):
                    logger.info(f"Navigation2 is ready (native) after {time.time() - start_time:.2f}s")
                    return True
                
                time.sleep(check_interval)
                
            except Exception as e:
                logger.debug(f"Navigation2 readiness check error (native): {e}")
                time.sleep(check_interval)
        
        logger.warn(f"Navigation2 did not become ready (native) within {timeout}s")
        return False
    
    @keyword
    def get_navigation_status_native(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get the current navigation status using native clients.
        
        Args:
            timeout: Override default timeout
            
        Returns:
            Dictionary containing navigation status information
            
        Example:
            | ${status}= | Get Navigation Status Native |
            | Log | Navigation active: ${status}[navigation_active] |
        """
        self._ensure_initialized()
        
        status = {
            'navigation_active': self._navigation_active,
            'current_pose': self._current_pose.to_dict() if self._current_pose else None,
            'goal_pose': self._goal_pose.to_dict() if self._goal_pose else None,
            'action_servers_ready': {
                'navigate_to_pose': self._navigate_to_pose_action_client.server_is_ready() if self._navigate_to_pose_action_client else False,
                'navigate_through_poses': self._navigate_through_poses_action_client.server_is_ready() if self._navigate_through_poses_action_client else False
            },
            'services_ready': {
                'clear_global_costmap': self._clear_global_costmap_client.service_is_ready() if self._clear_global_costmap_client else False,
                'clear_local_costmap': self._clear_local_costmap_client.service_is_ready() if self._clear_local_costmap_client else False
            }
        }
        
        # Try to get additional status from message buffers
        try:
            if 'amcl_pose' in self._message_buffer and self._message_buffer['amcl_pose']:
                status['localization_available'] = True
                status['last_pose_update'] = self._message_buffer['amcl_pose'][-1]['timestamp']
            else:
                status['localization_available'] = False
                
        except Exception:
            status['localization_available'] = False
        
        logger.info(f"Navigation status (native): {status}")
        return status

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================
    
    @keyword
    def cleanup(self):
        """Clean up resources."""
        if self._initialized and self.node:
            # Clean up action clients
            if self._navigate_to_pose_action_client:
                self._navigate_to_pose_action_client.destroy()
            if self._navigate_through_poses_action_client:
                self._navigate_through_poses_action_client.destroy()
            
            # Clean up service clients
            if self._clear_global_costmap_client:
                self._clear_global_costmap_client.destroy()
            if self._clear_local_costmap_client:
                self._clear_local_costmap_client.destroy()
            
            # Clean up publishers
            if self._initial_pose_publisher:
                self._initial_pose_publisher.destroy()
            if self._cmd_vel_publisher:
                self._cmd_vel_publisher.destroy()
            
            # Clean up subscribers
            if self._amcl_pose_subscriber:
                self._amcl_pose_subscriber.destroy()
            if self._global_costmap_subscriber:
                self._global_costmap_subscriber.destroy()
            if self._local_costmap_subscriber:
                self._local_costmap_subscriber.destroy()
            
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self._initialized = False
            logger.info("Native Navigation2 client cleaned up")
    
    @keyword
    def get_client_info(self) -> Dict[str, Any]:
        """Get information about the native client."""
        return {
            'initialized': self._initialized,
            'node_name': self.node_name,
            'navigation_active': self._navigation_active,
            'current_pose': self._current_pose.to_dict() if self._current_pose else None,
            'goal_pose': self._goal_pose.to_dict() if self._goal_pose else None,
            'buffered_topics': list(self._message_buffer.keys()),
            'total_messages': sum(len(msgs) for msgs in self._message_buffer.values()),
            'action_servers_available': {
                'navigate_to_pose': self._navigate_to_pose_action_client.server_is_ready() if self._navigate_to_pose_action_client else False,
                'navigate_through_poses': self._navigate_through_poses_action_client.server_is_ready() if self._navigate_through_poses_action_client else False
            },
            'services_available': {
                'clear_global_costmap': self._clear_global_costmap_client.service_is_ready() if self._clear_global_costmap_client else False,
                'clear_local_costmap': self._clear_local_costmap_client.service_is_ready() if self._clear_local_costmap_client else False
            }
        }
