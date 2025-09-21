"""
CLI-based ROS2 operations using subprocess calls
"""

import subprocess
import os
import time
from typing import List, Dict, Any, Optional, Union
from pathlib import Path
from robot.api.deco import keyword
from robot.api import logger

from .utils import ROS2CLIUtils


class ROS2CLIClient(ROS2CLIUtils):
    """CLI-based ROS2 operations using subprocess calls."""

    def __init__(self, timeout: float = 10.0):
        """Initialize CLI client."""
        super().__init__(timeout)
        logger.info("ROS2 CLI client initialized")

    # ============================================================================
    # PARAMETER OPERATIONS
    # ============================================================================

    @keyword
    def list_parameters(
        self, node_name: str, timeout: Optional[float] = None
    ) -> List[str]:
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
        result = self._run_ros2_command(["param", "list", node_name], timeout=timeout)

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to list parameters for node '{node_name}': {result.stderr}"
            )

        params = [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]
        logger.info(f"Found {len(params)} parameters for node '{node_name}': {params}")
        return params

    @keyword
    def get_parameter(
        self, node_name: str, parameter_name: str, timeout: Optional[float] = None
    ) -> Any:
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
        result = self._run_ros2_command(
            ["param", "get", node_name, parameter_name], timeout=timeout
        )

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to get parameter '{parameter_name}' from node '{node_name}': {result.stderr}"
            )

        value_text = result.stdout.strip()
        logger.info(
            f"Parameter '{parameter_name}' from node '{node_name}': {value_text}"
        )

        # Try to parse the value
        try:
            # Handle different parameter types
            if value_text.lower() in ["true", "false"]:
                return value_text.lower() == "true"
            elif value_text.isdigit():
                return int(value_text)
            elif value_text.replace(".", "").replace("-", "").isdigit():
                return float(value_text)
            elif value_text.startswith("[") and value_text.endswith("]"):
                # Simple list parsing - this could be enhanced
                return value_text
            else:
                return value_text
        except Exception:
            return value_text

    @keyword
    def set_parameter(
        self,
        node_name: str,
        parameter_name: str,
        value: Union[str, int, float, bool],
        timeout: Optional[float] = None,
    ) -> bool:
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
            ["param", "set", node_name, parameter_name, value_str], timeout=timeout
        )

        if result.returncode != 0:
            logger.error(
                f"Failed to set parameter '{parameter_name}' on node '{node_name}': {result.stderr}"
            )
            return False

        logger.info(
            f"Successfully set parameter '{parameter_name}' on node '{node_name}' to: {value_str}"
        )
        return True

    @keyword
    def parameter_exists(
        self, node_name: str, parameter_name: str, timeout: Optional[float] = None
    ) -> bool:
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
            logger.info(
                f"Parameter '{parameter_name}' exists on node '{node_name}': {exists}"
            )
            return exists
        except Exception as e:
            logger.error(
                f"Error checking if parameter '{parameter_name}' exists on node '{node_name}': {e}"
            )
            return False

    @keyword
    def get_all_parameters(
        self, node_name: str, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
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

    def get_action_list(self, timeout: Optional[float] = None) -> List[str]:
        """
        Get list of available actions.

        Returns:
            List of action names
        """
        result = self._run_ros2_command(["action", "list"], timeout=timeout)
        return [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]

    @keyword
    def send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_data: str,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Send a goal to a ROS2 action server.

        Args:
            action_name: Name of the action (e.g., '/execute_action')
            action_type: Type of the action (e.g., 'pyrobosim_msgs/action/ExecuteTaskAction')
            goal_data: Goal data as a JSON string
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing the result of the action

        Example:
            | ${result}= | Send Action Goal | /execute_action | pyrobosim_msgs/action/ExecuteTaskAction | '{"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}' |
        """
        try:
            # Use ros2 action send_goal command
            command = ["action", "send_goal", action_name, action_type, goal_data]
            result = self._run_ros2_command(command, timeout=timeout)

            if result.returncode != 0:
                logger.error(f"Failed to send action goal to '{action_name}': {result.stderr}")
                return {
                    "success": False,
                    "error": result.stderr,
                    "stdout": result.stdout
                }

            # Parse the output to extract useful information
            output_lines = result.stdout.strip().split("\n")
            action_result = {
                "success": True,
                "action_name": action_name,
                "action_type": action_type,
                "goal_data": goal_data,
                "raw_output": result.stdout,
                "return_code": result.returncode
            }

            # Try to extract goal ID and status from output
            for line in output_lines:
                if "Goal accepted" in line or "goal_id" in line.lower():
                    action_result["goal_accepted"] = True
                elif "Goal rejected" in line or "rejected" in line.lower():
                    action_result["goal_accepted"] = False
                elif "Result:" in line:
                    action_result["result"] = line.replace("Result:", "").strip()

            logger.info(f"Successfully sent action goal to '{action_name}'")
            return action_result

        except Exception as e:
            logger.error(f"Exception while sending action goal to '{action_name}': {e}")
            return {
                "success": False,
                "error": str(e),
                "action_name": action_name,
                "action_type": action_type,
                "goal_data": goal_data
            }

    @keyword
    def get_action_info(self, action_name: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Get detailed information about an action.

        Args:
            action_name: Name of the action
            timeout: Override default timeout for this operation

        Returns:
            Dictionary containing action information

        Example:
            | ${info}= | Get Action Info | /execute_action |
        """
        try:
            result = self._run_ros2_command(["action", "info", action_name], timeout=timeout)
            
            if result.returncode != 0:
                logger.error(f"Failed to get action info for '{action_name}': {result.stderr}")
                return {
                    "success": False,
                    "error": result.stderr,
                    "action_name": action_name
                }

            # Parse the output to extract useful information
            info = {"success": True, "action_name": action_name}
            lines = result.stdout.strip().split("\n")
            
            for line in lines:
                if "Action clients:" in line:
                    info["clients"] = line.replace("Action clients:", "").strip()
                elif "Action servers:" in line:
                    info["servers"] = line.replace("Action servers:", "").strip()
                elif "Action type:" in line:
                    info["action_type"] = line.replace("Action type:", "").strip()

            logger.info(f"Successfully retrieved action info for '{action_name}'")
            return info

        except Exception as e:
            logger.error(f"Exception while getting action info for '{action_name}': {e}")
            return {
                "success": False,
                "error": str(e),
                "action_name": action_name
            }

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
        try:
            actions = self.get_action_list(timeout)
            exists = action_name in actions
            logger.info(f"Action '{action_name}' exists: {exists}")
            return exists
        except Exception as e:
            logger.error(f"Exception while checking if action '{action_name}' exists: {e}")
            return False

    # ============================================================================
    # LAUNCH OPERATIONS
    # ============================================================================

    @keyword
    def launch_file(
        self,
        launch_file_path: str,
        arguments: Optional[Dict[str, str]] = None,
    ) -> subprocess.Popen:
        """
        Launch a ROS2 launch file.

        Args:
            launch_file_path: Path to the launch file (can be relative or absolute)
            arguments: Dictionary of launch arguments (key=value pairs)

        Returns:
            Popen process object for the launched process

        Example:
            | ${process}= | Launch File | /path/to/my_launch.launch.py |
            | ${process}= | Launch File | my_package launch/my_launch.launch.py | arguments={'arg1': 'value1'} |
        """
        command = ["launch", launch_file_path]

        # Add arguments if provided
        if arguments:
            for key, value in arguments.items():
                command.append(f"{key}:={value}")

        # Run the launch command in the background
        full_command = [self._ros2_executable] + command

        logger.info(f"Launching ROS2 file: {' '.join(full_command)}")

        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=None
                if os.name == "nt"
                else os.setsid,  # Create new process group
            )

            logger.info(f"Launched process with PID: {process.pid}")

            # Give the process a moment to start
            time.sleep(0.5)

            # Check if process is still running
            if process.poll() is not None:
                # Process has already terminated, get the output
                stdout, stderr = process.communicate()
                logger.error(
                    f"Launch process terminated immediately. Return code: {process.returncode}"
                )
                logger.error(f"STDOUT: {stdout}")
                logger.error(f"STDERR: {stderr}")
                raise RuntimeError(f"Launch process failed to start: {stderr}")

            return process

        except Exception as e:
            logger.error(f"Failed to launch file '{launch_file_path}': {e}")
            raise

    @keyword
    def launch_package(
        self,
        package_name: str,
        launch_file_name: str,
        arguments: Optional[Dict[str, str]] = None,
    ) -> subprocess.Popen:
        """
        Launch a ROS2 launch file from a package.

        Args:
            package_name: Name of the ROS2 package
            launch_file_name: Name of the launch file within the package
            arguments: Dictionary of launch arguments (key=value pairs)

        Returns:
            Popen process object for the launched process

        Example:
            | ${process}= | Launch Package | my_package | my_launch.launch.py |
            | ${process}= | Launch Package | nav2_bringup | tb3_simulation_launch.py | arguments={'use_sim_time': 'True'} |
        """
        # For package launch, we need to pass package and file separately
        command = ["launch", package_name, launch_file_name]

        # Add arguments if provided
        if arguments:
            for key, value in arguments.items():
                command.append(f"{key}:={value}")

        # Run the launch command in the background
        full_command = [self._ros2_executable] + command

        logger.info(f"Launching ROS2 package: {' '.join(full_command)}")

        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=None
                if os.name == "nt"
                else os.setsid,  # Create new process group
            )

            logger.info(f"Launched process with PID: {process.pid}")

            # Give the process a moment to start
            time.sleep(0.5)

            # Check if process is still running
            if process.poll() is not None:
                # Process has already terminated, get the output
                stdout, stderr = process.communicate()
                logger.error(
                    f"Launch process terminated immediately. Return code: {process.returncode}"
                )
                logger.error(f"STDOUT: {stdout}")
                logger.error(f"STDERR: {stderr}")
                raise RuntimeError(f"Launch process failed to start: {stderr}")

            return process

        except Exception as e:
            logger.error(
                f"Failed to launch package '{package_name}' file '{launch_file_name}': {e}"
            )
            raise

    @keyword
    def find_launch_files(
        self, package_name: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Find all launch files in a ROS2 package.

        Args:
            package_name: Name of the ROS2 package
            timeout: Timeout for the operation

        Returns:
            List of launch file names found in the package

        Example:
            | ${launch_files}= | Find Launch Files | nav2_bringup |
            | Should Contain | ${launch_files} | tb3_simulation_launch.py |
        """
        result = self._run_ros2_command(["pkg", "list", "--packages"], timeout=timeout)

        if result.returncode != 0:
            raise RuntimeError(f"Failed to list packages: {result.stderr}")

        # Check if package exists
        packages = [
            line.strip() for line in result.stdout.strip().split("\n") if line.strip()
        ]
        if package_name not in packages:
            raise RuntimeError(f"Package '{package_name}' not found")

        # Find launch files using find command
        try:
            find_result = subprocess.run(
                [
                    "find",
                    f"/opt/ros/*/share/{package_name}/launch",
                    "-name",
                    "*.launch.py",
                    "-o",
                    "-name",
                    "*.launch",
                ],
                capture_output=True,
                text=True,
                timeout=timeout,
            )

            if find_result.returncode == 0:
                launch_files = []
                for line in find_result.stdout.strip().split("\n"):
                    if line.strip():
                        # Extract just the filename
                        filename = Path(line.strip()).name
                        launch_files.append(filename)

                logger.info(
                    f"Found {len(launch_files)} launch files in package '{package_name}': {launch_files}"
                )
                return launch_files
            else:
                logger.warn(f"Could not find launch files for package '{package_name}'")
                return []

        except Exception as e:
            logger.warn(f"Error finding launch files for package '{package_name}': {e}")
            return []

    @keyword
    def wait_for_launch_completion(
        self, process: subprocess.Popen, timeout: float = 30.0
    ) -> bool:
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
            logger.info(
                f"Launch process completed with return code: {process.returncode}"
            )
            return True
        except subprocess.TimeoutExpired:
            logger.warn(f"Launch process did not complete within {timeout}s")
            return False
        except Exception as e:
            logger.error(f"Error waiting for launch completion: {e}")
            return False

    @keyword
    def terminate_launch_process(
        self, process: subprocess.Popen, force: bool = False
    ) -> bool:
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
                logger.info(
                    f"Launch process with PID {process.pid} has already terminated"
                )
                return True

            if force:
                if os.name == "nt":
                    process.kill()
                else:
                    # Kill the entire process group
                    os.killpg(os.getpgid(process.pid), 9)
                logger.info(f"Force killed launch process with PID: {process.pid}")
            else:
                if os.name == "nt":
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
                    if os.name == "nt":
                        process.kill()
                    else:
                        os.killpg(os.getpgid(process.pid), 9)
                    logger.info("Force killed launch process after timeout")

            return True
        except Exception as e:
            logger.error(f"Failed to terminate launch process: {e}")
            return False

    # ============================================================================
    # RUN OPERATIONS
    # ============================================================================

    @keyword
    def run_node(
        self,
        package_name: str,
        executable_name: str,
        arguments: Optional[List[str]] = None,
        setup_script: Optional[str] = None,
    ) -> subprocess.Popen:
        """
        Run a ROS2 node directly.

        Args:
            package_name: Name of the ROS2 package containing the node
            executable_name: Name of the executable/node
            arguments: List of command-line arguments for the node
            setup_script: Optional path to a setup script to source before running the node

        Returns:
            Popen process object for the running node

        Example:
            | ${process}= | Run Node | demo_nodes_cpp | talker |
            | ${process}= | Run Node | nav2_controller | controller_server | arguments=['--ros-args', '-p', 'use_sim_time:=True'] |
            | ${process}= | Run Node | pyrobosim_ros | demo.py | setup_script=/path/to/setup.bash |
        """
        command = ["run", package_name, executable_name]

        # Add arguments if provided
        if arguments:
            command.extend(arguments)

        # Run the node in the background
        full_command = [self._ros2_executable] + command

        logger.info(f"Running ROS2 node: {' '.join(full_command)}")

        try:
            # Build shell command with optional setup script
            if setup_script:
                shell_command = f"source {setup_script} && {' '.join(full_command)}"
                logger.info(f"Using setup script: {setup_script}")
            else:
                shell_command = ' '.join(full_command)
                logger.info("Running without setup script")
            
            process = subprocess.Popen(
                shell_command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                text=True,
                shell=True,
                executable='/bin/bash'
            )
            logger.info(f"Started node process with PID: {process.pid}")
            return process

        except Exception as e:
            logger.error(f"Failed to run node '{package_name}/{executable_name}': {e}")
            raise

    @keyword
    def run_node_with_remap(
        self,
        package_name: str,
        executable_name: str,
        remaps: Optional[Dict[str, str]] = None,
        arguments: Optional[List[str]] = None,
        setup_script: Optional[str] = None,
    ) -> subprocess.Popen:
        """
        Run a ROS2 node with topic/service remapping.

        Args:
            package_name: Name of the ROS2 package containing the node
            executable_name: Name of the executable/node
            remaps: Dictionary of remappings (old_topic -> new_topic)
            arguments: List of additional command-line arguments
            setup_script: Optional path to a setup script to source before running the node

        Returns:
            Popen process object for the running node

        Example:
            | ${process}= | Run Node With Remap | demo_nodes_cpp | talker | remaps={'/chatter': '/my_chatter'} |
            | ${process}= | Run Node With Remap | pyrobosim_ros | demo.py | setup_script=/path/to/setup.bash |
        """
        command = ["run", package_name, executable_name]

        # Add remaps if provided
        if remaps:
            for old_topic, new_topic in remaps.items():
                command.extend(["--remap", f"{old_topic}:={new_topic}"])

        # Add additional arguments if provided
        if arguments:
            command.extend(arguments)

        # Run the node in the background
        full_command = [self._ros2_executable] + command

        logger.info(f"Running ROS2 node with remaps: {' '.join(full_command)}")

        try:
            # Build shell command with optional setup script
            if setup_script:
                shell_command = f"source {setup_script} && {' '.join(full_command)}"
                logger.info(f"Using setup script: {setup_script}")
                process = subprocess.Popen(
                    shell_command, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True,
                    shell=True,
                    executable='/bin/bash'
                )
            else:
                logger.info("Running without setup script")
                process = subprocess.Popen(
                    full_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
                )

            logger.info(f"Started node process with PID: {process.pid}")
            return process

        except Exception as e:
            logger.error(
                f"Failed to run node '{package_name}/{executable_name}' with remaps: {e}"
            )
            raise

    @keyword
    def find_executables(
        self, package_name: str, timeout: Optional[float] = None
    ) -> List[str]:
        """
        Find all executables in a ROS2 package.

        Args:
            package_name: Name of the ROS2 package
            timeout: Timeout for the operation

        Returns:
            List of executable names found in the package

        Example:
            | ${executables}= | Find Executables | demo_nodes_cpp |
            | Should Contain | ${executables} | talker |
        """
        result = self._run_ros2_command(
            ["pkg", "executables", package_name], timeout=timeout
        )

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to find executables in package '{package_name}': {result.stderr}"
            )

        executables = []
        lines = result.stdout.strip().split("\n")

        for line in lines:
            line = line.strip()
            if line and " " in line:
                # Format is usually "executable_name package_name"
                parts = line.split()
                if len(parts) >= 2 and parts[1] == package_name:
                    executables.append(parts[0])

        logger.info(
            f"Found {len(executables)} executables in package '{package_name}': {executables}"
        )
        return executables

    @keyword
    def wait_for_node_completion(
        self, process: subprocess.Popen, timeout: float = 30.0
    ) -> bool:
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
            logger.info(
                f"Node process completed with return code: {process.returncode}"
            )
            return True
        except subprocess.TimeoutExpired:
            logger.warn(f"Node process did not complete within {timeout}s")
            return False
        except Exception as e:
            logger.error(f"Error waiting for node completion: {e}")
            return False

    @keyword
    def terminate_node_process(
        self, process: subprocess.Popen, force: bool = False
    ) -> bool:
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
                    logger.info("Force killed node process after timeout")

            return True
        except Exception as e:
            logger.error(f"Failed to terminate node process: {e}")
            return False

    @keyword
    def get_process_output(
        self, process: subprocess.Popen, timeout: float = 1.0
    ) -> Dict[str, str]:
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
            return {"stdout": stdout or "", "stderr": stderr or ""}
        except subprocess.TimeoutExpired:
            # Process is still running, return empty output
            return {"stdout": "", "stderr": ""}
        except Exception as e:
            logger.error(f"Error getting process output: {e}")
            return {"stdout": "", "stderr": ""}

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
                ["pgrep", "-f", process_name],
                capture_output=True,
                text=True,
                timeout=5.0,
            )

            if result.returncode == 0:
                pids = result.stdout.strip().split("\n")
                for pid in pids:
                    if pid.strip():
                        os.kill(int(pid.strip()), 9 if force else 15)
                        logger.info(f"Terminated process {pid.strip()}")

            logger.info(f"Process {process_name} shutdown completed")
            return True

        except Exception as e:
            logger.error(f"Failed to shutdown process {process_name}: {e}")
            return False

    @keyword
    def pkill_process(self, process_name: str, force: bool = False) -> bool:
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
                ["pkill", "-9", "-f", process_name],
                capture_output=True,
                text=True,
                timeout=5.0,
            )

            if result.returncode == 0:
                pids = result.stdout.strip().split("\n")
                for pid in pids:
                    if pid.strip():
                        os.kill(int(pid.strip()), 9 if force else 15)
                        logger.info(f"Terminated process {pid.strip()}")

            logger.info(f"Process {process_name} shutdown completed")
            return True

        except Exception as e:
            logger.error(f"Failed to shutdown process {process_name}: {e}")
            return False

    def has_running_nodes(self, timeout: float) -> bool:
        """
        Check if there are any nodes running.
        """
        while timeout > 0:
            try:
                nodes = self.list_nodes(timeout=2)
                if not nodes or set(nodes) <= {
                    "/robotframework_nav2",
                    "/robotframework_ros2",
                }:
                    return False
            except Exception:
                pass
            time.sleep(1)
            timeout -= 1
        return True
