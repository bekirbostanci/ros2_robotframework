Nav2ClientLibrary
=================

The main Navigation2 client that combines CLI and native operations for RobotFramework testing.

Overview
--------

The `Nav2ClientLibrary` is the primary client for Navigation2 operations. It provides a unified interface that automatically uses the most appropriate method (CLI or native) for each operation.

Initialization
--------------

.. code-block:: robot

   *** Settings ***
   Library    Nav2ClientLibrary    timeout=30.0    action_timeout=60.0    use_native=True    node_name=robotframework_nav2

Parameters
~~~~~~~~~~

- **timeout** (float, optional): Default timeout for operations in seconds (default: 30.0)
- **action_timeout** (float, optional): Default timeout for navigation actions in seconds (default: 60.0)
- **use_native** (bool, optional): Whether to use native operations when available (default: True)
- **node_name** (str, optional): Name for the native ROS2 node (default: "robotframework_nav2")

Navigation Operations
=====================

.. py:method:: navigate_to_pose_simple(x, y, theta, frame_id="map", timeout=None)

   Simple navigation to a pose using Navigation2 action server.

   **Parameters:**
   - **x** (float): X coordinate in meters
   - **y** (float): Y coordinate in meters
   - **theta** (float): Orientation in radians
   - **frame_id** (str): Reference frame (default: "map")
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if navigation command was sent successfully

   **Example:**
   .. code-block:: robot

      ${success}=    Navigate To Pose Simple    2.0    1.0    1.57
      Should Be True    ${success}

.. py:method:: cancel_navigation(timeout=None)

   Cancel the current navigation operation.

   **Parameters:**
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if cancellation was successful

   **Example:**
   .. code-block:: robot

      ${cancelled}=    Cancel Navigation
      Should Be True    ${cancelled}

.. py:method:: is_navigation_active()

   Check if navigation is currently active.

   **Returns:** True if navigation is active, False otherwise

   **Example:**
   .. code-block:: robot

      ${active}=    Is Navigation Active
      Should Be False    ${active}

Pose and Localization Operations
=================================

.. py:method:: get_current_pose(timeout=None)

   Get the current robot pose from the localization system.

   **Parameters:**
   - **timeout** (float, optional): Override default timeout

   **Returns:** Current pose as Pose object, or None if unavailable

   **Example:**
   .. code-block:: robot

      ${pose}=    Get Current Pose
      Should Not Be None    ${pose}
      Log    Current position: x=${pose.x}, y=${pose.y}

.. py:method:: set_initial_pose(x, y, theta, frame_id="map", timeout=None)

   Set the initial pose for the robot (for localization).

   **Parameters:**
   - **x** (float): X coordinate in meters
   - **y** (float): Y coordinate in meters
   - **theta** (float): Orientation in radians
   - **frame_id** (str): Reference frame (default: "map")
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if initial pose was set successfully

   **Example:**
   .. code-block:: robot

      ${success}=    Set Initial Pose    0.0    0.0    0.0
      Should Be True    ${success}

.. py:method:: set_initial_pose_simple(x, y, theta, timeout=None)

   Set the initial pose for the robot using a simpler approach.

   **Parameters:**
   - **x** (float): X coordinate in meters
   - **y** (float): Y coordinate in meters
   - **theta** (float): Orientation in radians
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if initial pose was set successfully

   **Example:**
   .. code-block:: robot

      ${success}=    Set Initial Pose Simple    0.0    0.0    0.0
      Should Be True    ${success}

.. py:method:: wait_for_localization(timeout=30.0, check_interval=1.0)

   Wait for the robot to be localized (AMCL to converge).

   **Parameters:**
   - **timeout** (float): Maximum time to wait in seconds (default: 30.0)
   - **check_interval** (float): Time between checks in seconds (default: 1.0)

   **Returns:** True if localization converged within timeout

   **Example:**
   .. code-block:: robot

      ${localized}=    Wait For Localization    timeout=60.0
      Should Be True    ${localized}

Path Planning Operations
========================

.. py:method:: compute_path(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, frame_id="map", timeout=None)

   Compute a path from start to goal pose using Navigation2.

   **Parameters:**
   - **start_x** (float): Start X coordinate in meters
   - **start_y** (float): Start Y coordinate in meters
   - **start_theta** (float): Start orientation in radians
   - **goal_x** (float): Goal X coordinate in meters
   - **goal_y** (float): Goal Y coordinate in meters
   - **goal_theta** (float): Goal orientation in radians
   - **frame_id** (str): Reference frame (default: "map")
   - **timeout** (float, optional): Override default timeout

   **Returns:** List of waypoint dictionaries, or None if path planning failed

   **Example:**
   .. code-block:: robot

      ${path}=    Compute Path    0.0    0.0    0.0    2.0    1.0    1.57
      Should Not Be None    ${path}
      Length Should Be Greater Than    ${path}    0

Costmap Operations
==================

.. py:method:: get_costmap_info(costmap_type="global", timeout=None)

   Get information about the costmap.

   **Parameters:**
   - **costmap_type** (str): Type of costmap ("global" or "local") (default: "global")
   - **timeout** (float, optional): Override default timeout

   **Returns:** Dictionary containing costmap information

   **Example:**
   .. code-block:: robot

      ${info}=    Get Costmap Info    global
      Should Contain    ${info}    resolution

.. py:method:: clear_costmap(costmap_type="global", timeout=None)

   Clear the specified costmap.

   **Parameters:**
   - **costmap_type** (str): Type of costmap to clear ("global" or "local") (default: "global")
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if costmap was cleared successfully

   **Example:**
   .. code-block:: robot

      ${cleared}=    Clear Costmap    global
      Should Be True    ${cleared}

Navigation2 Status Operations
=============================

.. py:method:: get_navigation_status(timeout=None)

   Get the current navigation status.

   **Parameters:**
   - **timeout** (float, optional): Override default timeout

   **Returns:** Dictionary containing navigation status information

   **Example:**
   .. code-block:: robot

      ${status}=    Get Navigation Status
      Log    Navigation active: ${status}[navigation_active]

Native-Specific Operations
==========================

.. py:method:: navigate_to_pose_native(x, y, theta, frame_id="map", timeout=None)

   Navigate to a specific pose using native Navigation2 action client (native only).

   **Parameters:**
   - **x** (float): X coordinate in meters
   - **y** (float): Y coordinate in meters
   - **theta** (float): Orientation in radians
   - **frame_id** (str): Reference frame (default: "map")
   - **timeout** (float, optional): Override default timeout

   **Returns:** NavigationResult object

   **Example:**
   .. code-block:: robot

      ${result}=    Navigate To Pose Native    2.0    1.0    1.57
      Should Be True    ${result.success}

.. py:method:: navigate_through_poses(poses, frame_id="map", timeout=None)

   Navigate through a sequence of poses using native Navigation2 action client (native only).

   **Parameters:**
   - **poses** (List[Dict[str, float]]): List of pose dictionaries
   - **frame_id** (str): Reference frame (default: "map")
   - **timeout** (float, optional): Override default timeout

   **Returns:** NavigationResult object

   **Example:**
   .. code-block:: robot

      @{poses}=    Create List
      ${pose1}=    Create Dictionary    x=1.0    y=0.0    theta=0.0
      ${pose2}=    Create Dictionary    x=2.0    y=1.0    theta=1.57
      Append To List    ${poses}    ${pose1}
      Append To List    ${poses}    ${pose2}
      ${result}=    Navigate Through Poses    ${poses}
      Should Be True    ${result.success}

.. py:method:: get_current_pose_native(timeout=None)

   Get the current robot pose using native subscriber (native only).

   **Parameters:**
   - **timeout** (float, optional): Override default timeout

   **Returns:** Current pose as Pose object, or None if unavailable

   **Example:**
   .. code-block:: robot

      ${pose}=    Get Current Pose Native
      Should Not Be None    ${pose}

.. py:method:: set_initial_pose_native(x, y, theta, frame_id="map", timeout=None)

   Set the initial pose using native publisher (native only).

   **Parameters:**
   - **x** (float): X coordinate in meters
   - **y** (float): Y coordinate in meters
   - **theta** (float): Orientation in radians
   - **frame_id** (str): Reference frame (default: "map")
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if initial pose was set successfully

   **Example:**
   .. code-block:: robot

      ${success}=    Set Initial Pose Native    0.0    0.0    0.0
      Should Be True    ${success}

.. py:method:: wait_for_localization_native(timeout=30.0, check_interval=1.0)

   Wait for localization using native subscriber (native only).

   **Parameters:**
   - **timeout** (float): Maximum time to wait in seconds (default: 30.0)
   - **check_interval** (float): Time between checks in seconds (default: 1.0)

   **Returns:** True if localization converged within timeout

   **Example:**
   .. code-block:: robot

      ${localized}=    Wait For Localization Native    timeout=60.0
      Should Be True    ${localized}

.. py:method:: get_costmap_info_native(costmap_type="global", timeout=None)

   Get costmap information using native subscriber (native only).

   **Parameters:**
   - **costmap_type** (str): Type of costmap ("global" or "local") (default: "global")
   - **timeout** (float, optional): Override default timeout

   **Returns:** Dictionary containing costmap information

   **Example:**
   .. code-block:: robot

      ${info}=    Get Costmap Info Native    global
      Should Contain    ${info}    resolution

.. py:method:: clear_costmap_native(costmap_type="global", timeout=None)

   Clear costmap using native service client (native only).

   **Parameters:**
   - **costmap_type** (str): Type of costmap to clear ("global" or "local") (default: "global")
   - **timeout** (float, optional): Override default timeout

   **Returns:** True if costmap was cleared successfully

   **Example:**
   .. code-block:: robot

      ${cleared}=    Clear Costmap Native    global
      Should Be True    ${cleared}

.. py:method:: wait_for_nav2_ready(timeout=60.0, check_interval=2.0)

   Wait for Navigation2 to be ready using native clients (native only).

   **Parameters:**
   - **timeout** (float): Maximum time to wait in seconds (default: 60.0)
   - **check_interval** (float): Time between checks in seconds (default: 2.0)

   **Returns:** True if Navigation2 is ready within timeout

   **Example:**
   .. code-block:: robot

      ${ready}=    Wait For Nav2 Ready    timeout=120.0
      Should Be True    ${ready}

.. py:method:: get_navigation_status_native(timeout=None)

   Get navigation status using native clients (native only).

   **Parameters:**
   - **timeout** (float, optional): Override default timeout

   **Returns:** Dictionary containing navigation status information

   **Example:**
   .. code-block:: robot

      ${status}=    Get Navigation Status Native
      Log    Navigation active: ${status}[navigation_active]

Utility Methods
===============

.. py:method:: cleanup()

   Clean up all resources.

   **Example:**
   .. code-block:: robot

      Cleanup

.. py:method:: get_client_info()

   Get information about the current client configuration.

   **Returns:** Dictionary containing client configuration information

   **Example:**
   .. code-block:: robot

      ${info}=    Get Client Info
      Log    Native available: ${info['native_available']}
      Log    Timeout: ${info['timeout']}
      Log    Action timeout: ${info['action_timeout']}
