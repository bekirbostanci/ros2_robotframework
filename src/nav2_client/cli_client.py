"""
CLI-based Navigation2 operations using subprocess calls
"""

import time
import math
from typing import List, Dict, Any, Optional
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
        timeout: Optional[float] = None,
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

        logger.info(
            f"Navigating to pose: x={x}, y={y}, theta={theta} (frame: {frame_id})"
        )

        try:
            # Use proper YAML format for Navigation2 service call
            z_quat = math.sin(theta / 2)
            w_quat = math.cos(theta / 2)

            # Create proper YAML format for the service request
            request_data = f"pose:\n  header:\n    frame_id: '{frame_id}'\n    stamp:\n      sec: 0\n      nanosec: 0\n  pose:\n    position:\n      x: {x}\n      y: {y}\n      z: 0.0\n    orientation:\n      x: 0.0\n      y: 0.0\n      z: {z_quat}\n      w: {w_quat}"

            result = self._run_ros2_command(
                [
                    "service",
                    "call",
                    "/navigate_to_pose",
                    "nav2_msgs/srv/NavigateToPose",
                    request_data,
                ],
                timeout=timeout_value,
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
                        final_pose=goal_pose,
                    )
                else:
                    logger.warn(f"Navigation failed: {response_text}")
                    self._navigation_active = False
                    return NavigationResult(
                        success=False, message=f"Navigation failed: {response_text}"
                    )
            else:
                logger.error(f"Navigation service call failed: {result.stderr}")
                self._navigation_active = False
                return NavigationResult(
                    success=False, message=f"Service call failed: {result.stderr}"
                )

        except Exception as e:
            logger.error(f"Navigation error: {e}")
            self._navigation_active = False
            return NavigationResult(success=False, message=f"Navigation error: {e}")

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
                [
                    "service",
                    "call",
                    "/navigate_to_pose/_action/cancel_goal",
                    "action_msgs/srv/CancelGoal",
                    "{}",
                ],
                timeout=timeout,
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
        timeout: Optional[float] = None,
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
        logger.info(
            f"Computing path from ({start_x}, {start_y}) to ({goal_x}, {goal_y})"
        )

        try:
            # Use the compute_path_to_pose service
            start_z_quat = math.sin(start_theta / 2)
            start_w_quat = math.cos(start_theta / 2)
            goal_z_quat = math.sin(goal_theta / 2)
            goal_w_quat = math.cos(goal_theta / 2)
            request_data = (
                "start: {header: {frame_id: '%s'}, pose: {position: {x: %f, y: %f, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}, goal: {header: {frame_id: '%s'}, pose: {position: {x: %f, y: %f, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}}"
                % (
                    frame_id,
                    start_x,
                    start_y,
                    start_z_quat,
                    start_w_quat,
                    frame_id,
                    goal_x,
                    goal_y,
                    goal_z_quat,
                    goal_w_quat,
                )
            )

            result = self._run_ros2_command(
                [
                    "service",
                    "call",
                    "/compute_path_to_pose",
                    "nav2_msgs/srv/ComputePathToPose",
                    request_data,
                ],
                timeout=timeout,
            )

            if result.returncode == 0:
                # Parse the path from the response
                path = self._parse_path_from_response(result.stdout)
                if path:
                    logger.info(
                        f"Path computed successfully with {len(path)} waypoints"
                    )
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
    # NAVIGATION2 STATUS OPERATIONS
    # ============================================================================

    @keyword
    def wait_for_nav2_ready(
        self, timeout: float = 60.0, check_interval: float = 2.0
    ) -> bool:
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
            "/navigate_to_pose",
            "/compute_path_to_pose",
            "/global_costmap/clear_entirely_global_costmap",
            "/local_costmap/clear_entirely_local_costmap",
        ]

        start_time = time.time()

        while time.time() - start_time < timeout:
            all_ready = True

            for service in required_services:
                try:
                    result = self._run_ros2_command(
                        ["service", "list"], timeout=check_interval
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
                logger.info(
                    f"Navigation2 is ready after {time.time() - start_time:.2f}s"
                )
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
            "navigation_active": self._navigation_active,
            "current_pose": self._current_pose.to_dict()
            if self._current_pose
            else None,
            "goal_pose": self._goal_pose.to_dict() if self._goal_pose else None,
        }

        # Try to get additional status from topics
        try:
            # Check if robot is moving
            result = self._run_ros2_command(
                ["topic", "echo", "/cmd_vel", "--once"], timeout=timeout or 1.0
            )

            if result.returncode == 0 and result.stdout.strip():
                status["robot_moving"] = True
            else:
                status["robot_moving"] = False

        except Exception:
            status["robot_moving"] = False

        logger.info(f"Navigation status: {status}")
        return status

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request_data: str,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Call a service.

        Args:
            service_name: Name of the service
            service_type: Type of the service
            request_data: Request data for the service
            timeout: Override default timeout

        Returns:
            Dictionary containing the service response

        Example:
            | ${response}= | Call Service | /add_two_ints | example_interfaces/srv/AddTwoInts | "a: 5, b: 3" |
            | Should Equal | ${response}[sum] | 8 |
        """
        result = self._run_ros2_command(
            ["service", "call", service_name, service_type, request_data],
            timeout=timeout,
        )

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to call service '{service_name}': {result.stderr}"
            )

        # Parse the response - this is a simplified parser
        response_text = result.stdout.strip()
        logger.info(f"Service call response for '{service_name}': {response_text}")
        return response_text
