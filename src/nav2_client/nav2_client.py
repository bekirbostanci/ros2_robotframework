"""
Main Navigation2 client that combines CLI and native operations
"""

import asyncio
from typing import List, Dict, Any, Optional
from robot.api.deco import keyword
from robot.api import logger

from .utils import Nav2BaseClient, Pose, NavigationResult
from .cli_client import Nav2CLIClient
from .native_client import Nav2NativeClient


class Nav2ClientLibrary(Nav2BaseClient):
    """
    Main Navigation2 client that automatically chooses between CLI and native operations.

    This is the primary client that users should use. It provides a unified interface
    that automatically uses the most appropriate method (CLI or native) for each operation.
    """

    def __init__(
        self,
        timeout: float = 30.0,
        action_timeout: float = 60.0,
        use_native: bool = True,
        node_name: str = "robotframework_nav2",
    ):
        """
        Initialize main client.

        Args:
            timeout: Default timeout for operations
            action_timeout: Default timeout for navigation actions
            use_native: Whether to use native operations when available
            node_name: Name for the native ROS2 node
        """
        super().__init__(timeout, action_timeout)
        self.use_native = use_native

        # Initialize both clients
        self.cli_client = Nav2CLIClient(timeout, action_timeout)

        if use_native:
            try:
                self.native_client = Nav2NativeClient(
                    timeout, action_timeout, node_name
                )
                logger.info(
                    "Main Navigation2 client initialized with both CLI and native support"
                )
            except Exception as e:
                logger.warn(
                    f"Failed to initialize native client, falling back to CLI only: {e}"
                )
                self.native_client = None
                self.use_native = False
        else:
            self.native_client = None
            logger.info("Main Navigation2 client initialized with CLI support only")

    # ============================================================================
    # NAVIGATION OPERATIONS (Smart Selection)
    # ============================================================================
    @keyword
    async def navigate_to_pose_simple(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None,
    ) -> bool:
        """
        Simple navigation to a pose using Navigation2 action server.

        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            True if navigation command was sent successfully

        Example:
            | ${success}= | Navigate To Pose Simple | 2.0 | 1.0 | 1.57 |
            | Should Be True | ${success} |
        """
        # For native client, we use the full navigation method
        result = self.native_client.navigate_to_pose_native(
            x, y, theta, frame_id, timeout
        )
        return result.success

    async def async_navigate_to_pose_simple(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None,
    ):
        # Don't wait for the result, just send the goal and return immediately
        asyncio.create_task(
            self._send_navigation_goal_async(x, y, theta, frame_id, timeout)
        )

    async def _send_navigation_goal_async(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None,
    ):
        """Send navigation goal without waiting for completion"""
        try:
            # Use the new non-blocking method
            success = self.native_client.send_navigation_goal_only(
                x, y, theta, frame_id, timeout
            )
            if not success:
                logger.error("Failed to send navigation goal")
        except Exception as e:
            logger.error(f"Failed to send navigation goal: {e}")

    @keyword
    def navigate_through_poses(
        self,
        poses: List[Dict[str, float]],
        frame_id: str = "map",
        timeout: Optional[float] = None,
        use_native: Optional[bool] = None,
    ) -> NavigationResult:
        """
        Navigate through a sequence of poses using Navigation2.

        Args:
            poses: List of pose dictionaries with 'x', 'y', 'theta' keys
            frame_id: Reference frame (default: "map")
            timeout: Override default action timeout
            use_native: Override default native preference

        Returns:
            NavigationResult object with success status and details

        Example:
            | @{poses}= | Create List | ${{'x': 1.0, 'y': 0.0, 'theta': 0.0}} | ${{'x': 2.0, 'y': 1.0, 'theta': 1.57}} |
            | ${result}= | Navigate Through Poses | ${poses} |
            | Should Be True | ${result.success} |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.navigate_through_poses_native(
                poses, frame_id, timeout
            )
        else:
            return self.cli_client.navigate_through_poses(poses, frame_id, timeout)

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
        return self.cli_client.cancel_navigation(timeout)

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
    # POSE AND LOCALIZATION OPERATIONS (Smart Selection)
    # ============================================================================

    @keyword
    def get_current_pose(
        self, timeout: Optional[float] = None, use_native: Optional[bool] = None
    ) -> Optional[Pose]:
        """
        Get the current robot pose from the localization system.

        Args:
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            Current pose as Pose object, or None if unavailable

        Example:
            | ${pose}= | Get Current Pose |
            | Should Not Be None | ${pose} |
            | Log | Current position: x=${pose.x}, y=${pose.y} |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.get_current_pose_native(timeout)
        else:
            return self.cli_client.get_current_pose(timeout)

    @keyword
    def set_initial_pose(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None,
        use_native: Optional[bool] = None,
    ) -> bool:
        """
        Set the initial pose for the robot (for localization).

        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            frame_id: Reference frame (default: "map")
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            True if initial pose was set successfully

        Example:
            | ${success}= | Set Initial Pose | 0.0 | 0.0 | 0.0 |
            | Should Be True | ${success} |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.set_initial_pose_native(
                x, y, theta, frame_id, timeout
            )
        else:
            return self.cli_client.set_initial_pose(x, y, theta, frame_id, timeout)

    @keyword
    def set_initial_pose_simple(
        self,
        x: float,
        y: float,
        theta: float,
        timeout: Optional[float] = None,
        use_native: Optional[bool] = None,
    ) -> bool:
        """
        Set the initial pose for the robot using a simpler approach.

        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            True if initial pose was set successfully

        Example:
            | ${success}= | Set Initial Pose Simple | 0.0 | 0.0 | 0.0 |
            | Should Be True | ${success} |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.set_initial_pose_native(
                x, y, theta, "map", timeout
            )
        else:
            return self.cli_client.set_initial_pose_simple(x, y, theta, timeout)

    @keyword
    def wait_for_localization(
        self,
        timeout: float = 30.0,
        check_interval: float = 1.0,
        use_native: Optional[bool] = None,
    ) -> bool:
        """
        Wait for the robot to be localized (AMCL to converge).

        Args:
            timeout: Maximum time to wait in seconds
            check_interval: Time between checks in seconds
            use_native: Override default native preference

        Returns:
            True if localization converged within timeout

        Example:
            | ${localized}= | Wait For Localization | timeout=60.0 |
            | Should Be True | ${localized} |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.wait_for_localization_native(
                timeout, check_interval
            )
        else:
            return self.cli_client.wait_for_localization(timeout, check_interval)

    # ============================================================================
    # PATH PLANNING OPERATIONS (Smart Selection)
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
        Compute a path from start to goal pose using Navigation2.

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

        return self.cli_client.compute_path(
            start_x,
            start_y,
            start_theta,
            goal_x,
            goal_y,
            goal_theta,
            frame_id,
            timeout,
        )

    # ============================================================================
    # COSTMAP OPERATIONS (Smart Selection)
    # ============================================================================

    @keyword
    def get_costmap_info(
        self,
        costmap_type: str = "global",
        timeout: Optional[float] = None,
        use_native: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """
        Get information about the costmap.

        Args:
            costmap_type: Type of costmap ("global" or "local")
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            Dictionary containing costmap information

        Example:
            | ${info}= | Get Costmap Info | global |
            | Should Contain | ${info} | resolution |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.get_costmap_info_native(costmap_type, timeout)
        else:
            return self.cli_client.get_costmap_info(costmap_type, timeout)

    @keyword
    def clear_costmap(
        self,
        costmap_type: str = "global",
        timeout: Optional[float] = None,
        use_native: Optional[bool] = None,
    ) -> bool:
        """
        Clear the specified costmap.

        Args:
            costmap_type: Type of costmap to clear ("global" or "local")
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            True if costmap was cleared successfully

        Example:
            | ${cleared}= | Clear Costmap | global |
            | Should Be True | ${cleared} |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.clear_costmap_native(costmap_type, timeout)
        else:
            return self.cli_client.clear_costmap(costmap_type, timeout)

    # ============================================================================
    # NAVIGATION2 STATUS OPERATIONS (Smart Selection)
    # ============================================================================

    @keyword
    def get_navigation_status(
        self, timeout: Optional[float] = None, use_native: Optional[bool] = None
    ) -> Dict[str, Any]:
        """
        Get the current navigation status.

        Args:
            timeout: Override default timeout
            use_native: Override default native preference

        Returns:
            Dictionary containing navigation status information

        Example:
            | ${status}= | Get Navigation Status |
            | Log | Navigation active: ${status}[navigation_active] |
        """
        use_native = use_native if use_native is not None else self.use_native

        if use_native and self.native_client:
            return self.native_client.get_navigation_status_native(timeout)
        else:
            return self.cli_client.get_navigation_status(timeout)

    # ============================================================================
    # NATIVE-SPECIFIC OPERATIONS
    # ============================================================================

    @keyword
    def navigate_to_pose_native(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None,
    ) -> NavigationResult:
        """Navigate to a specific pose using native Navigation2 action client (native only)."""
        if self.native_client:
            return self.native_client.navigate_to_pose_native(
                x, y, theta, frame_id, timeout
            )
        else:
            logger.warn("Native client not available, cannot navigate natively")
            return NavigationResult(
                success=False, message="Native client not available"
            )

    @keyword
    def navigate_through_poses_native(
        self,
        poses: List[Dict[str, float]],
        frame_id: str = "map",
        timeout: Optional[float] = None,
    ) -> NavigationResult:
        """Navigate through a sequence of poses using native Navigation2 action client (native only)."""
        if self.native_client:
            return self.native_client.navigate_through_poses_native(
                poses, frame_id, timeout
            )
        else:
            logger.warn(
                "Native client not available, cannot navigate through poses natively"
            )
            return NavigationResult(
                success=False, message="Native client not available"
            )

    @keyword
    def cancel_navigation_native(self, timeout: Optional[float] = None) -> bool:
        """Cancel the current navigation operation using native action client (native only)."""
        if self.native_client:
            return self.native_client.cancel_navigation_native(timeout)
        else:
            logger.warn(
                "Native client not available, cannot cancel navigation natively"
            )
            return False

    @keyword
    def get_current_pose_native(
        self, timeout: Optional[float] = None
    ) -> Optional[Pose]:
        """Get the current robot pose using native subscriber (native only)."""
        if self.native_client:
            return self.native_client.get_current_pose_native(timeout)
        else:
            logger.warn("Native client not available, cannot get current pose natively")
            return None

    @keyword
    def set_initial_pose_native(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = "map",
        timeout: Optional[float] = None,
    ) -> bool:
        """Set the initial pose using native publisher (native only)."""
        if self.native_client:
            return self.native_client.set_initial_pose_native(
                x, y, theta, frame_id, timeout
            )
        else:
            logger.warn("Native client not available, cannot set initial pose natively")
            return False

    @keyword
    def wait_for_localization_native(
        self, timeout: float = 30.0, check_interval: float = 1.0
    ) -> bool:
        """Wait for localization using native subscriber (native only)."""
        if self.native_client:
            return self.native_client.wait_for_localization_native(
                timeout, check_interval
            )
        else:
            logger.warn(
                "Native client not available, cannot wait for localization natively"
            )
            return False

    # Note: compute_path_native removed - Navigation2 doesn't provide this as a service

    @keyword
    def get_costmap_info_native(
        self, costmap_type: str = "global", timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """Get costmap information using native subscriber (native only)."""
        if self.native_client:
            return self.native_client.get_costmap_info_native(costmap_type, timeout)
        else:
            logger.warn("Native client not available, cannot get costmap info natively")
            return {}

    @keyword
    def clear_costmap_native(
        self, costmap_type: str = "global", timeout: Optional[float] = None
    ) -> bool:
        """Clear costmap using native service client (native only)."""
        if self.native_client:
            return self.native_client.clear_costmap_native(costmap_type, timeout)
        else:
            logger.warn("Native client not available, cannot clear costmap natively")
            return False

    @keyword
    def wait_for_nav2_ready(
        self, timeout: float = 60.0, check_interval: float = 2.0
    ) -> bool:
        """Wait for Navigation2 to be ready using native clients (native only)."""
        if self.native_client:
            return self.native_client.wait_for_nav2_ready(timeout, check_interval)
        else:
            logger.warn(
                "Native client not available, cannot wait for Nav2 ready natively"
            )
            return False

    @keyword
    def get_navigation_status_native(
        self, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """Get navigation status using native clients (native only)."""
        if self.native_client:
            return self.native_client.get_navigation_status_native(timeout)
        else:
            logger.warn(
                "Native client not available, cannot get navigation status natively"
            )
            return {}

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================

    @keyword
    def cleanup(self):
        """Clean up all resources."""
        if self.native_client:
            self.native_client.cleanup()
        logger.info("Main Navigation2 client cleanup completed")

    @keyword
    def get_client_info(self) -> Dict[str, Any]:
        """Get information about the current client configuration."""
        info = {
            "use_native": self.use_native,
            "native_available": self.native_client is not None,
            "timeout": self.timeout,
            "action_timeout": self.action_timeout,
            "ros2_executable": self._ros2_executable,
            "navigation_active": self._navigation_active,
            "current_pose": self._current_pose.to_dict()
            if self._current_pose
            else None,
            "goal_pose": self._goal_pose.to_dict() if self._goal_pose else None,
        }

        if self.native_client:
            info["native_info"] = self.native_client.get_client_info()

        return info

    @keyword
    def switch_to_cli_mode(self):
        """Switch to CLI-only mode."""
        if self.native_client:
            self.native_client.cleanup()
            self.native_client = None
        self.use_native = False
        logger.info("Switched to CLI-only mode")

    @keyword
    def switch_to_native_mode(self, node_name: str = "robotframework_nav2"):
        """Switch to native mode (if available)."""
        if not self.native_client:
            try:
                self.native_client = Nav2NativeClient(
                    self.timeout, self.action_timeout, node_name
                )
                self.use_native = True
                logger.info("Switched to native mode")
            except Exception as e:
                logger.error(f"Failed to switch to native mode: {e}")
                self.use_native = False
        else:
            logger.info("Already in native mode")
