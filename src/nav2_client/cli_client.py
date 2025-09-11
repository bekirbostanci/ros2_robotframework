"""
CLI-based Navigation2 operations using subprocess calls
"""

import subprocess
import time
import math
from typing import List, Dict, Any, Optional, Union, Tuple
from robot.api.deco import keyword
from robot.api import logger

from .utils import Nav2BaseClient, Pose, NavigationResult


class Nav2CLIClient(Nav2BaseClient):
    """CLI-based Navigation2 operations using subprocess calls."""
    
    def __init__(self, timeout: float = 30.0, action_timeout: float = 60.0):
        """Initialize CLI client."""
        super().__init__(timeout, action_timeout)
        self._initialized = True
        logger.info("Navigation2 CLI client initialized")

    # ============================================================================
    # NAVIGATION2 SERVICE OPERATIONS
    # ============================================================================
    
    @keyword
    def navigate_to_pose(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> NavigationResult:
        """
        Navigate to a specific pose using Navigation2.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default action timeout
            
        Returns:
            NavigationResult object with success status and details
            
        Example:
            | ${result}= | Navigate To Pose | 2.0 | 1.0 | 1.57 |
            | Should Be True | ${result.success} |
        """
        goal_pose = Pose(x, y, theta)
        self._goal_pose = goal_pose
        self._navigation_active = True
        
        timeout_value = timeout or self.action_timeout
        
        logger.info(f"Navigating to pose: x={x}, y={y}, theta={theta} (frame: {frame_id})")
        
        try:
            # Use proper YAML format for Navigation2 service call
            z_quat = math.sin(theta/2)
            w_quat = math.cos(theta/2)
            
            # Create proper YAML format for the service request
            request_data = f"pose:\n  header:\n    frame_id: '{frame_id}'\n    stamp:\n      sec: 0\n      nanosec: 0\n  pose:\n    position:\n      x: {x}\n      y: {y}\n      z: 0.0\n    orientation:\n      x: 0.0\n      y: 0.0\n      z: {z_quat}\n      w: {w_quat}"
            
            result = self._run_ros2_command(
                ['service', 'call', '/navigate_to_pose', 'nav2_msgs/srv/NavigateToPose', request_data],
                timeout=timeout_value
            )
            
            if result.returncode == 0:
                # Parse the response - Navigation2 returns result codes
                response_text = result.stdout.strip()
                logger.info(f"Navigation service response: {response_text}")
                
                # Check for success (result: 4 means SUCCESS in nav2_msgs)
                success = "result: 4" in response_text or "result:4" in response_text
                
                if success:
                    logger.info(f"Successfully navigated to pose: {goal_pose}")
                    self._current_pose = goal_pose
                    self._navigation_active = False
                    return NavigationResult(
                        success=True,
                        message="Navigation completed successfully",
                        final_pose=goal_pose
                    )
                else:
                    logger.warn(f"Navigation failed: {response_text}")
                    self._navigation_active = False
                    return NavigationResult(
                        success=False,
                        message=f"Navigation failed: {response_text}"
                    )
            else:
                logger.error(f"Navigation service call failed: {result.stderr}")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message=f"Service call failed: {result.stderr}"
                )
                
        except Exception as e:
            logger.error(f"Navigation error: {e}")
            self._navigation_active = False
            return NavigationResult(
                success=False,
                message=f"Navigation error: {e}"
            )
    
    @keyword
    def navigate_to_pose_simple(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> bool:
        """
        Simple navigation to a pose using Navigation2 action server.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            
        Returns:
            True if navigation command was sent successfully
            
        Example:
            | ${success}= | Navigate To Pose Simple | 2.0 | 1.0 | 1.57 |
            | Should Be True | ${success} |
        """
        logger.info(f"Simple navigation to pose: x={x}, y={y}, theta={theta} (frame: {frame_id})")
        
        # Validate input parameters
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)) or not isinstance(theta, (int, float)):
            logger.error("Invalid pose parameters: x, y, and theta must be numbers")
            return False
        
        if not isinstance(frame_id, str) or not frame_id.strip():
            logger.error("Invalid frame_id: must be a non-empty string")
            return False
        
        try:
            # Use Navigation2 action server for simple navigation
            z_quat = math.sin(theta/2)
            w_quat = math.cos(theta/2)
            
            # Get current timestamp
            current_time = int(time.time())
            
            # Proper YAML format for Navigation2 action goal
            goal_data = f"pose:\n  header:\n    frame_id: '{frame_id}'\n    stamp:\n      sec: {current_time}\n      nanosec: 0\n  pose:\n    position:\n      x: {x}\n      y: {y}\n      z: 0.0\n    orientation:\n      x: 0.0\n      y: 0.0\n      z: {z_quat}\n      w: {w_quat}"
            
            logger.debug(f"Sending navigation goal data: {goal_data}")
            
            # Use action send_goal instead of topic pub
            result = self._run_ros2_command(
                ['action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', goal_data],
                timeout=timeout
            )
            
            if result.returncode == 0:
                logger.info("Navigation goal sent successfully via action server")
                self._navigation_active = True
                self._goal_pose = Pose(x, y, theta)
                return True
            else:
                logger.error(f"Failed to send navigation goal: {result.stderr}")
                if result.stdout:
                    logger.debug(f"Command stdout: {result.stdout}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("Navigation goal command timed out")
            return False
        except Exception as e:
            logger.error(f"Error sending navigation goal: {e}")
            return False
    
    @keyword
    def navigate_through_poses(
        self, 
        poses: List[Dict[str, float]], 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> NavigationResult:
        """
        Navigate through a sequence of poses using Navigation2.
        
        Args:
            poses: List of pose dictionaries with 'x', 'y', 'theta' keys
            frame_id: Reference frame (default: "map")
            timeout: Override default action timeout
            
        Returns:
            NavigationResult object with success status and details
            
        Example:
            | @{poses}= | Create List | ${{'x': 1.0, 'y': 0.0, 'theta': 0.0}} | ${{'x': 2.0, 'y': 1.0, 'theta': 1.57}} |
            | ${result}= | Navigate Through Poses | ${poses} |
            | Should Be True | ${result.success} |
        """
        if not poses:
            return NavigationResult(
                success=False,
                message="No poses provided for navigation"
            )
        
        timeout_value = timeout or self.action_timeout
        self._navigation_active = True
        
        logger.info(f"Navigating through {len(poses)} poses")
        
        try:
            # Build the poses array for the service call
            poses_data = []
            for i, pose_dict in enumerate(poses):
                pose = Pose.from_dict(pose_dict)
                z_quat = math.sin(pose.theta/2)
                w_quat = math.cos(pose.theta/2)
                pose_str = "poses[%d]: {header: {frame_id: '%s'}, pose: {position: {x: %f, y: %f, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}}" % (i, frame_id, pose.x, pose.y, z_quat, w_quat)
                poses_data.append(pose_str)
            
            request_data = ", ".join(poses_data)
            
            result = self._run_ros2_command(
                ['service', 'call', '/navigate_through_poses', 'nav2_msgs/srv/NavigateThroughPoses', request_data],
                timeout=timeout_value
            )
            
            if result.returncode == 0:
                response_text = result.stdout.strip()
                success = "result: 4" in response_text
                
                if success:
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
                    logger.warn(f"Navigation through poses failed: {response_text}")
                    self._navigation_active = False
                    return NavigationResult(
                        success=False,
                        message=f"Navigation failed: {response_text}"
                    )
            else:
                logger.error(f"Navigation service call failed: {result.stderr}")
                self._navigation_active = False
                return NavigationResult(
                    success=False,
                    message=f"Service call failed: {result.stderr}"
                )
                
        except Exception as e:
            logger.error(f"Navigation error: {e}")
            self._navigation_active = False
            return NavigationResult(
                success=False,
                message=f"Navigation error: {e}"
            )
    
    @keyword
    def cancel_navigation(self, timeout: Optional[float] = None) -> bool:
        """
        Cancel the current navigation operation.
        
        Args:
            timeout: Override default timeout
            
        Returns:
            True if cancellation was successful
            
        Example:
            | ${cancelled}= | Cancel Navigation |
            | Should Be True | ${cancelled} |
        """
        logger.info("Cancelling current navigation...")
        
        try:
            result = self._run_ros2_command(
                ['service', 'call', '/navigate_to_pose', 'std_srvs/srv/Empty', '{}'],
                timeout=timeout
            )
            
            if result.returncode == 0:
                self._navigation_active = False
                logger.info("Navigation cancelled successfully")
                return True
            else:
                logger.error(f"Failed to cancel navigation: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error cancelling navigation: {e}")
            return False
    
    @keyword
    def is_navigation_active(self) -> bool:
        """
        Check if navigation is currently active.
        
        Returns:
            True if navigation is active, False otherwise
            
        Example:
            | ${active}= | Is Navigation Active |
            | Should Be False | ${active} |
        """
        return self._navigation_active
    
    # ============================================================================
    # POSE AND LOCALIZATION OPERATIONS
    # ============================================================================
    
    @keyword
    def get_current_pose(self, timeout: Optional[float] = None) -> Optional[Pose]:
        """
        Get the current robot pose from the localization system.
        
        Args:
            timeout: Override default timeout
            
        Returns:
            Current pose as Pose object, or None if unavailable
            
        Example:
            | ${pose}= | Get Current Pose |
            | Should Not Be None | ${pose} |
            | Log | Current position: x=${pose.x}, y=${pose.y} |
        """
        try:
            # Try to get pose from /amcl_pose topic
            result = self._run_ros2_command(
                ['topic', 'echo', '/amcl_pose', '--once'],
                timeout=timeout
            )
            
            if result.returncode == 0 and result.stdout.strip():
                # Parse the pose from the topic output
                pose_data = self._parse_pose_from_topic(result.stdout)
                if pose_data:
                    self._current_pose = pose_data
                    logger.info(f"Current pose: x={pose_data.x}, y={pose_data.y}, theta={pose_data.theta}")
                    return pose_data
            
            logger.warn("Could not retrieve current pose")
            return None
            
        except Exception as e:
            logger.error(f"Error getting current pose: {e}")
            return None
    
    @keyword
    def set_initial_pose(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> bool:
        """
        Set the initial pose for the robot (for localization).
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            
        Returns:
            True if initial pose was set successfully
            
        Example:
            | ${success}= | Set Initial Pose | 0.0 | 0.0 | 0.0 |
            | Should Be True | ${success} |
        """
        logger.info(f"Setting initial pose: x={x}, y={y}, theta={theta}")
        
        try:
            # Publish initial pose to /initialpose topic using a simpler format
            z_quat = math.sin(theta/2)
            w_quat = math.cos(theta/2)
            
            # Use a more compatible YAML format
            pose_data = f"header:\n  frame_id: '{frame_id}'\n  stamp:\n    sec: 0\n    nanosec: 0\npose:\n  pose:\n    position:\n      x: {x}\n      y: {y}\n      z: 0.0\n    orientation:\n      x: 0.0\n      y: 0.0\n      z: {z_quat}\n      w: {w_quat}\n  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
            
            result = self._run_ros2_command(
                ['topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', pose_data],
                timeout=timeout
            )
            
            if result.returncode == 0:
                self._current_pose = Pose(x, y, theta)
                logger.info("Initial pose set successfully")
                return True
            else:
                logger.error(f"Failed to set initial pose: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error setting initial pose: {e}")
            return False
    
    @keyword
    def set_initial_pose_simple(
        self, 
        x: float, 
        y: float, 
        theta: float, 
        timeout: Optional[float] = None
    ) -> bool:
        """
        Set the initial pose for the robot using a simpler approach.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            timeout: Override default timeout
            
        Returns:
            True if initial pose was set successfully
            
        Example:
            | ${success}= | Set Initial Pose Simple | 0.0 | 0.0 | 0.0 |
            | Should Be True | ${success} |
        """
        logger.info(f"Setting initial pose (simple): x={x}, y={y}, theta={theta}")
        
        try:
            # Use a very simple YAML format that should work
            pose_data = f"header:\n  frame_id: 'map'\n  stamp:\n    sec: 0\n    nanosec: 0\npose:\n  pose:\n    position:\n      x: {x}\n      y: {y}\n      z: 0.0\n    orientation:\n      x: 0.0\n      y: 0.0\n      z: {math.sin(theta/2)}\n      w: {math.cos(theta/2)}"
            
            result = self._run_ros2_command(
                ['topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', pose_data],
                timeout=timeout
            )
            
            if result.returncode == 0:
                self._current_pose = Pose(x, y, theta)
                logger.info("Initial pose set successfully (simple method)")
                return True
            else:
                logger.error(f"Failed to set initial pose (simple): {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error setting initial pose (simple): {e}")
            return False
    
    @keyword
    def wait_for_localization(self, timeout: float = 30.0, check_interval: float = 1.0) -> bool:
        """
        Wait for the robot to be localized (AMCL to converge).
        
        Args:
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            True if localization converged within timeout
            
        Example:
            | ${localized}= | Wait For Localization | timeout=60.0 |
            | Should Be True | ${localized} |
        """
        logger.info(f"Waiting for localization to converge (timeout: {timeout}s)")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Check if AMCL pose is available and has reasonable covariance
                result = self._run_ros2_command(
                    ['topic', 'echo', '/amcl_pose', '--once'],
                    timeout=check_interval
                )
                
                if result.returncode == 0 and result.stdout.strip():
                    # Simple check: if we can get a pose, assume localization is working
                    pose = self._parse_pose_from_topic(result.stdout)
                    if pose:
                        logger.info(f"Localization converged after {time.time() - start_time:.2f}s")
                        self._current_pose = pose
                        return True
                
                time.sleep(check_interval)
                
            except Exception as e:
                logger.debug(f"Localization check error: {e}")
                time.sleep(check_interval)
        
        logger.warn(f"Localization did not converge within {timeout}s")
        return False
    
    # ============================================================================
    # PATH PLANNING OPERATIONS
    # ============================================================================
    
    @keyword
    def compute_path(
        self, 
        start_x: float, 
        start_y: float, 
        start_theta: float,
        goal_x: float, 
        goal_y: float, 
        goal_theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None
    ) -> Optional[List[Dict[str, float]]]:
        """
        Compute a path from start to goal pose.
        
        Args:
            start_x: Start X coordinate in meters
            start_y: Start Y coordinate in meters
            start_theta: Start orientation in radians
            goal_x: Goal X coordinate in meters
            goal_y: Goal Y coordinate in meters
            goal_theta: Goal orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            
        Returns:
            List of waypoint dictionaries, or None if path planning failed
            
        Example:
            | ${path}= | Compute Path | 0.0 | 0.0 | 0.0 | 2.0 | 1.0 | 1.57 |
            | Should Not Be None | ${path} |
            | Length Should Be Greater Than | ${path} | 0 |
        """
        logger.info(f"Computing path from ({start_x}, {start_y}) to ({goal_x}, {goal_y})")
        
        try:
            # Use the compute_path_to_pose service
            start_z_quat = math.sin(start_theta/2)
            start_w_quat = math.cos(start_theta/2)
            goal_z_quat = math.sin(goal_theta/2)
            goal_w_quat = math.cos(goal_theta/2)
            request_data = "start: {header: {frame_id: '%s'}, pose: {position: {x: %f, y: %f, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}, goal: {header: {frame_id: '%s'}, pose: {position: {x: %f, y: %f, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}}" % (frame_id, start_x, start_y, start_z_quat, start_w_quat, frame_id, goal_x, goal_y, goal_z_quat, goal_w_quat)
            
            result = self._run_ros2_command(
                ['service', 'call', '/compute_path_to_pose', 'nav2_msgs/srv/ComputePathToPose', request_data],
                timeout=timeout
            )
            
            if result.returncode == 0:
                # Parse the path from the response
                path = self._parse_path_from_response(result.stdout)
                if path:
                    logger.info(f"Path computed successfully with {len(path)} waypoints")
                    return path
                else:
                    logger.warn("Path computation returned empty path")
                    return None
            else:
                logger.error(f"Path computation failed: {result.stderr}")
                return None
                
        except Exception as e:
            logger.error(f"Error computing path: {e}")
            return None
    
    # ============================================================================
    # COSTMAP OPERATIONS
    # ============================================================================
    
    @keyword
    def get_costmap_info(self, costmap_type: str = "global", timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get information about the costmap.
        
        Args:
            costmap_type: Type of costmap ("global" or "local")
            timeout: Override default timeout
            
        Returns:
            Dictionary containing costmap information
            
        Example:
            | ${info}= | Get Costmap Info | global |
            | Should Contain | ${info} | resolution |
        """
        topic_name = f"/{costmap_type}_costmap/costmap" if costmap_type in ["global", "local"] else "/global_costmap/costmap"
        
        try:
            result = self._run_ros2_command(
                ['topic', 'echo', topic_name, '--once'],
                timeout=timeout
            )
            
            if result.returncode == 0 and result.stdout.strip():
                info = self._parse_costmap_info(result.stdout)
                logger.info(f"Retrieved {costmap_type} costmap info")
                return info
            else:
                logger.warn(f"Could not retrieve {costmap_type} costmap info")
                return {}
                
        except Exception as e:
            logger.error(f"Error getting costmap info: {e}")
            return {}
    
    @keyword
    def clear_costmap(self, costmap_type: str = "global", timeout: Optional[float] = None) -> bool:
        """
        Clear the specified costmap.
        
        Args:
            costmap_type: Type of costmap to clear ("global" or "local")
            timeout: Override default timeout
            
        Returns:
            True if costmap was cleared successfully
            
        Example:
            | ${cleared}= | Clear Costmap | global |
            | Should Be True | ${cleared} |
        """
        service_name = f"/{costmap_type}_costmap/clear_entirely_{costmap_type}_costmap" if costmap_type in ["global", "local"] else "/global_costmap/clear_entirely_global_costmap"
        
        logger.info(f"Clearing {costmap_type} costmap...")
        
        try:
            result = self._run_ros2_command(
                ['service', 'call', service_name, 'std_srvs/srv/Empty', '{}'],
                timeout=timeout
            )
            
            if result.returncode == 0:
                logger.info(f"{costmap_type.capitalize()} costmap cleared successfully")
                return True
            else:
                logger.error(f"Failed to clear {costmap_type} costmap: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error clearing costmap: {e}")
            return False
    
    # ============================================================================
    # NAVIGATION2 STATUS OPERATIONS
    # ============================================================================
    
    @keyword
    def wait_for_nav2_ready(self, timeout: float = 60.0, check_interval: float = 2.0) -> bool:
        """
        Wait for Navigation2 stack to be ready.
        
        Args:
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            
        Returns:
            True if Navigation2 is ready within timeout
            
        Example:
            | ${ready}= | Wait For Nav2 Ready | timeout=120.0 |
            | Should Be True | ${ready} |
        """
        logger.info(f"Waiting for Navigation2 to be ready (timeout: {timeout}s)")
        
        required_services = [
            '/navigate_to_pose',
            '/compute_path_to_pose',
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap'
        ]
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            all_ready = True
            
            for service in required_services:
                try:
                    result = self._run_ros2_command(
                        ['service', 'list'],
                        timeout=check_interval
                    )
                    
                    if result.returncode == 0 and service in result.stdout:
                        logger.debug(f"Service {service} is available")
                    else:
                        logger.debug(f"Service {service} not yet available")
                        all_ready = False
                        break
                        
                except Exception as e:
                    logger.debug(f"Error checking service {service}: {e}")
                    all_ready = False
                    break
            
            if all_ready:
                logger.info(f"Navigation2 is ready after {time.time() - start_time:.2f}s")
                return True
            
            time.sleep(check_interval)
        
        logger.warn(f"Navigation2 did not become ready within {timeout}s")
        return False
    
    @keyword
    def get_navigation_status(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get the current navigation status.
        
        Args:
            timeout: Override default timeout
            
        Returns:
            Dictionary containing navigation status information
            
        Example:
            | ${status}= | Get Navigation Status |
            | Log | Navigation active: ${status}[navigation_active] |
        """
        status = {
            'navigation_active': self._navigation_active,
            'current_pose': self._current_pose.to_dict() if self._current_pose else None,
            'goal_pose': self._goal_pose.to_dict() if self._goal_pose else None
        }
        
        # Try to get additional status from topics
        try:
            # Check if robot is moving
            result = self._run_ros2_command(
                ['topic', 'echo', '/cmd_vel', '--once'],
                timeout=timeout or 1.0
            )
            
            if result.returncode == 0 and result.stdout.strip():
                status['robot_moving'] = True
            else:
                status['robot_moving'] = False
                
        except Exception:
            status['robot_moving'] = False
        
        logger.info(f"Navigation status: {status}")
        return status
