"""
Common utilities and base classes for Navigation2 Robot Framework Library
"""

import subprocess
import math
from typing import List, Dict, Any, Optional, Union, Tuple
from pathlib import Path
from dataclasses import dataclass
from robot.api.deco import keyword
from robot.api import logger


@dataclass
class Pose:
    """Represents a 2D pose with position and orientation."""

    x: float
    y: float
    theta: float  # Orientation in radians

    def to_dict(self) -> Dict[str, float]:
        """Convert pose to dictionary format."""
        return {"x": self.x, "y": self.y, "theta": self.theta}

    @classmethod
    def from_dict(cls, data: Dict[str, float]) -> "Pose":
        """Create pose from dictionary."""
        return cls(
            x=data.get("x", 0.0), y=data.get("y", 0.0), theta=data.get("theta", 0.0)
        )


@dataclass
class NavigationResult:
    """Represents the result of a navigation operation."""

    success: bool
    message: str
    final_pose: Optional[Pose] = None
    path_length: Optional[float] = None
    execution_time: Optional[float] = None


class Nav2BaseClient:
    """Base class with common utilities for Navigation2 operations."""

    ROBOT_LIBRARY_SCOPE = "GLOBAL"
    ROBOT_LIBRARY_DOC_FORMAT = "ROBOT"

    def __init__(self, timeout: float = 30.0, action_timeout: float = 60.0):
        """Initialize base client with common settings."""
        self.timeout = timeout
        self.action_timeout = action_timeout
        self._ros2_executable = self._find_ros2_executable()
        self._current_pose: Optional[Pose] = None
        self._goal_pose: Optional[Pose] = None
        self._navigation_active = False
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

        except subprocess.TimeoutExpired as e:
            logger.error(
                f"ROS2 command timed out after {timeout_value}s: {' '.join(full_command)}"
            )
            raise
        except Exception as e:
            logger.error(f"Failed to run ROS2 command: {e}")
            raise

    def _parse_pose_from_topic(self, topic_output: str) -> Optional[Pose]:
        """Parse pose data from topic output."""
        try:
            lines = topic_output.strip().split("\n")
            pose_data = {}

            for line in lines:
                line = line.strip()
                if "x:" in line:
                    pose_data["x"] = float(line.split("x:")[1].strip())
                elif "y:" in line:
                    pose_data["y"] = float(line.split("y:")[1].strip())
                elif "z:" in line and "w:" in line:
                    # Extract quaternion and convert to theta
                    z_part = line.split("z:")[1].split(",")[0].strip()
                    w_part = line.split("w:")[1].strip()
                    z = float(z_part)
                    w = float(w_part)
                    # Convert quaternion to euler angle (yaw)
                    pose_data["theta"] = math.atan2(2 * (w * z), 1 - 2 * (z * z))

            if len(pose_data) >= 3:
                return Pose(pose_data["x"], pose_data["y"], pose_data["theta"])

            return None

        except Exception as e:
            logger.error(f"Error parsing pose from topic: {e}")
            return None

    def _parse_path_from_response(
        self, response_text: str
    ) -> Optional[List[Dict[str, float]]]:
        """Parse path waypoints from service response."""
        try:
            # This is a simplified parser - in practice, you might want more robust parsing
            waypoints = []
            lines = response_text.strip().split("\n")

            in_poses_section = False
            for line in lines:
                line = line.strip()
                if "poses:" in line:
                    in_poses_section = True
                    continue
                elif in_poses_section and line.startswith("-"):
                    # Parse individual pose
                    if "x:" in line and "y:" in line:
                        try:
                            x_part = line.split("x:")[1].split(",")[0].strip()
                            y_part = line.split("y:")[1].split(",")[0].strip()
                            x = float(x_part)
                            y = float(y_part)
                            waypoints.append(
                                {"x": x, "y": y, "theta": 0.0}
                            )  # Simplified
                        except (ValueError, IndexError):
                            continue
                elif (
                    in_poses_section
                    and not line.startswith(" ")
                    and not line.startswith("-")
                ):
                    break

            return waypoints if waypoints else None

        except Exception as e:
            logger.error(f"Error parsing path: {e}")
            return None

    def _parse_costmap_info(self, costmap_output: str) -> Dict[str, Any]:
        """Parse costmap information from topic output."""
        info = {}
        lines = costmap_output.strip().split("\n")

        for line in lines:
            line = line.strip()
            if "resolution:" in line:
                info["resolution"] = float(line.split("resolution:")[1].strip())
            elif "width:" in line:
                info["width"] = int(line.split("width:")[1].strip())
            elif "height:" in line:
                info["height"] = int(line.split("height:")[1].strip())
            elif "origin:" in line:
                # Parse origin coordinates
                origin_data = line.split("origin:")[1].strip()
                if "x:" in origin_data and "y:" in origin_data:
                    x_part = origin_data.split("x:")[1].split(",")[0].strip()
                    y_part = origin_data.split("y:")[1].split(",")[0].strip()
                    info["origin_x"] = float(x_part)
                    info["origin_y"] = float(y_part)

        return info

    # ============================================================================
    # UTILITY OPERATIONS
    # ============================================================================

    @keyword
    def calculate_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Calculate the Euclidean distance between two points.

        Args:
            x1: X coordinate of first point
            y1: Y coordinate of first point
            x2: X coordinate of second point
            y2: Y coordinate of second point

        Returns:
            Distance in meters

        Example:
            | ${distance}= | Calculate Distance | 0.0 | 0.0 | 3.0 | 4.0 |
            | Should Be Equal | ${distance} | 5.0 |
        """
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        logger.info(f"Distance between ({x1}, {y1}) and ({x2}, {y2}): {distance:.3f}m")
        return distance

    @keyword
    def calculate_angle(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Calculate the angle from point 1 to point 2.

        Args:
            x1: X coordinate of first point
            y1: Y coordinate of first point
            x2: X coordinate of second point
            y2: Y coordinate of second point

        Returns:
            Angle in radians

        Example:
            | ${angle}= | Calculate Angle | 0.0 | 0.0 | 1.0 | 1.0 |
            | Should Be Equal | ${angle} | 0.785 |
        """
        angle = math.atan2(y2 - y1, x2 - x1)
        logger.info(
            f"Angle from ({x1}, {y1}) to ({x2}, {y2}): {angle:.3f} rad ({math.degrees(angle):.1f}°)"
        )
        return angle

    @keyword
    def normalize_angle(self, angle: float) -> float:
        """
        Normalize an angle to the range [-π, π].

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle in radians

        Example:
            | ${normalized}= | Normalize Angle | 3.14159 |
            | Should Be Equal | ${normalized} | 3.14159 |
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @keyword
    def degrees_to_radians(self, degrees: float) -> float:
        """
        Convert degrees to radians.

        Args:
            degrees: Angle in degrees

        Returns:
            Angle in radians

        Example:
            | ${radians}= | Degrees To Radians | 90.0 |
            | Should Be Equal | ${radians} | 1.571 |
        """
        radians = math.radians(degrees)
        logger.info(f"{degrees}° = {radians:.3f} rad")
        return radians

    @keyword
    def radians_to_degrees(self, radians: float) -> float:
        """
        Convert radians to degrees.

        Args:
            radians: Angle in radians

        Returns:
            Angle in degrees

        Example:
            | ${degrees}= | Radians To Degrees | 1.571 |
            | Should Be Equal | ${degrees} | 90.0 |
        """
        degrees = math.degrees(radians)
        logger.info(f"{radians:.3f} rad = {degrees:.1f}°")
        return degrees

    @keyword
    def get_path_length(self, path: List[Dict[str, float]]) -> float:
        """
        Calculate the total length of a path.

        Args:
            path: List of waypoint dictionaries with 'x', 'y' keys

        Returns:
            Total path length in meters

        Example:
            | ${path}= | Compute Path | 0.0 | 0.0 | 0.0 | 2.0 | 1.0 | 1.57 |
            | ${length}= | Get Path Length | ${path} |
            | Should Be Greater Than | ${length} | 0.0 |
        """
        if not path or len(path) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(path)):
            prev_point = path[i - 1]
            curr_point = path[i]
            dx = curr_point["x"] - prev_point["x"]
            dy = curr_point["y"] - prev_point["y"]
            segment_length = math.sqrt(dx * dx + dy * dy)
            total_length += segment_length

        logger.info(f"Path length: {total_length:.3f} meters")
        return total_length
