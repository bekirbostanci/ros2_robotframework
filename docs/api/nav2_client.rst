Navigation2 Client Library
=========================

The Nav2ClientLibrary provides comprehensive Navigation2 operations for Robot Framework testing.

Overview
--------

The Nav2ClientLibrary is the main library for Navigation2 operations. It combines CLI and native operations to provide the best performance and reliability for navigation tasks.

Key Features:
- Navigation operations (navigate to pose, navigate through poses)
- Path planning operations (global and local planners)
- Localization operations (AMCL, initial pose)
- Costmap operations (global and local costmaps)
- Recovery operations (recovery behaviors)
- Monitoring operations (navigation status, pose tracking)
- Utility operations (cleanup, info, validation)

Usage
-----

.. code-block:: robot

   *** Settings ***
   Library    Nav2ClientLibrary    timeout=30.0    action_timeout=60.0    node_name=test_nav2

   *** Test Cases ***
   Test Navigation
       Set Initial Pose    x=0.0    y=0.0    yaw=0.0
       ${result}=    Navigate To Pose    x=1.0    y=2.0    yaw=0.0
       Should Be True    ${result.success}

Initialization
--------------

Nav2ClientLibrary
~~~~~~~~~~~~~~~~~

.. autoclass:: nav2_client.nav2_client.Nav2ClientLibrary
   :members:
   :undoc-members:
   :show-inheritance:

The main Navigation2 client that combines CLI and native operations for Robot Framework testing.

**Keyword: Initialize Nav2 Client**

.. code-block:: robot

   *** Settings ***
   Library    Nav2ClientLibrary    timeout=30.0    action_timeout=60.0    node_name=test_nav2

Navigation Operations
--------------------

**Keyword: Navigate To Pose**

Navigates the robot to a specific pose.

.. code-block:: robot

   ${result}=    Navigate To Pose    x=1.0    y=2.0    yaw=0.0

**Keyword: Navigate Through Poses**

Navigates the robot through a sequence of poses.

.. code-block:: robot

   ${poses}=    Create List    ${pose1}    ${pose2}    ${pose3}
   ${result}=    Navigate Through Poses    ${poses}

**Keyword: Cancel Navigation**

Cancels the current navigation operation.

.. code-block:: robot

   Cancel Navigation

**Keyword: Is Navigation Active**

Checks if navigation is currently active.

.. code-block:: robot

   ${active}=    Is Navigation Active

**Keyword: Get Navigation Result**

Gets the result of the last navigation operation.

.. code-block:: robot

   ${result}=    Get Navigation Result

Path Planning Operations
------------------------

**Keyword: Plan Path**

Plans a path from start to goal pose.

.. code-block:: robot

   ${path}=    Plan Path    start_x=0.0    start_y=0.0    goal_x=2.0    goal_y=2.0

**Keyword: Plan Global Path**

Plans a global path using the global planner.

.. code-block:: robot

   ${path}=    Plan Global Path    start_x=0.0    start_y=0.0    goal_x=2.0    goal_y=2.0

**Keyword: Plan Local Path**

Plans a local path using the local planner.

.. code-block:: robot

   ${path}=    Plan Local Path    start_x=0.0    start_y=0.0    goal_x=2.0    goal_y=2.0

**Keyword: Get Path Length**

Gets the length of a planned path.

.. code-block:: robot

   ${length}=    Get Path Length    ${path}

Localization Operations
-----------------------

**Keyword: Set Initial Pose**

Sets the initial pose of the robot for localization.

.. code-block:: robot

   Set Initial Pose    x=0.0    y=0.0    yaw=0.0

**Keyword: Get Current Pose**

Gets the current pose of the robot.

.. code-block:: robot

   ${pose}=    Get Current Pose

**Keyword: Wait For Localization**

Waits for the robot to be localized.

.. code-block:: robot

   ${localized}=    Wait For Localization    timeout=10.0

**Keyword: Is Localized**

Checks if the robot is localized.

.. code-block:: robot

   ${localized}=    Is Localized

Costmap Operations
------------------

**Keyword: Clear Global Costmap**

Clears the global costmap.

.. code-block:: robot

   Clear Global Costmap

**Keyword: Clear Local Costmap**

Clears the local costmap.

.. code-block:: robot

   Clear Local Costmap

**Keyword: Get Global Costmap**

Gets the global costmap data.

.. code-block:: robot

   ${costmap}=    Get Global Costmap

**Keyword: Get Local Costmap**

Gets the local costmap data.

.. code-block:: robot

   ${costmap}=    Get Local Costmap

**Keyword: Costmap Exists**

Checks if a costmap exists.

.. code-block:: robot

   ${exists}=    Costmap Exists    global

Recovery Operations
-------------------

**Keyword: Start Recovery**

Starts recovery behaviors.

.. code-block:: robot

   Start Recovery

**Keyword: Stop Recovery**

Stops recovery behaviors.

.. code-block:: robot

   Stop Recovery

**Keyword: Is Recovery Active**

Checks if recovery is active.

.. code-block:: robot

   ${active}=    Is Recovery Active

**Keyword: Get Recovery Result**

Gets the result of recovery operations.

.. code-block:: robot

   ${result}=    Get Recovery Result

Monitoring Operations
--------------------

**Keyword: Monitor Navigation Status**

Monitors the navigation status.

.. code-block:: robot

   ${status}=    Monitor Navigation Status

**Keyword: Monitor Pose**

Monitors the robot's pose changes.

.. code-block:: robot

   ${pose}=    Monitor Pose

**Keyword: Monitor Costmap**

Monitors costmap changes.

.. code-block:: robot

   ${costmap}=    Monitor Costmap    global

**Keyword: Get Navigation Info**

Gets comprehensive navigation information.

.. code-block:: robot

   ${info}=    Get Navigation Info

Utility Operations
------------------

**Keyword: Cleanup**

Cleans up resources and connections.

.. code-block:: robot

   Cleanup

**Keyword: Get System Info**

Gets system information.

.. code-block:: robot

   ${info}=    Get System Info

**Keyword: Validate Navigation Setup**

Validates the navigation setup.

.. code-block:: robot

   ${valid}=    Validate Navigation Setup

Data Classes
------------

Pose
~~~~

Data class representing a 2D pose with position and orientation.

.. code-block:: robot

   ${pose}=    Create Pose    x=1.0    y=2.0    yaw=0.0

NavigationResult
~~~~~~~~~~~~~~~~

Data class representing the result of a navigation operation.

.. code-block:: robot

   ${result}=    Create Navigation Result    success=True    message=Success

Navigation Message Types
------------------------

The Navigation2 client library works with standard Navigation2 message types:

- **geometry_msgs/msg/PoseStamped** - Pose with timestamp and frame
- **geometry_msgs/msg/PoseWithCovarianceStamped** - Pose with covariance
- **nav2_msgs/action/NavigateToPose** - Navigation action goals
- **nav2_msgs/action/NavigateThroughPoses** - Multi-pose navigation
- **geometry_msgs/msg/Twist** - Velocity commands for navigation

Service Types
-------------

Common service types used with Navigation2:

- **std_srvs/srv/Empty** - Empty service requests (costmap clearing)
- **nav2_msgs/srv/GetCostmap** - Costmap retrieval
- **nav2_msgs/srv/SetInitialPose** - Initial pose setting

Action Types
------------

Navigation2 action types for navigation and control:

- **nav2_msgs/action/NavigateToPose** - Single pose navigation
- **nav2_msgs/action/NavigateThroughPoses** - Multi-pose navigation
- **nav2_msgs/action/ComputePathToPose** - Path planning actions

Example Usage
-------------

Basic Navigation Test
~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   *** Settings ***
   Library    Nav2ClientLibrary    timeout=30.0    action_timeout=60.0    node_name=test_nav2

   *** Test Cases ***
   Test Basic Navigation
       Set Initial Pose    x=0.0    y=0.0    yaw=0.0
       ${result}=    Navigate To Pose    x=1.0    y=2.0    yaw=0.0
       Should Be True    ${result.success}

   Test Pose Operations
       ${pose}=    Get Current Pose
       Should Not Be Empty    ${pose}
       ${localized}=    Is Localized
       Should Be True    ${localized}

   Test Costmap Operations
       Clear Global Costmap
       Clear Local Costmap
       ${costmap}=    Get Global Costmap
       Should Not Be Empty    ${costmap}

Advanced Navigation Test
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   *** Test Cases ***
   Test Multi-Pose Navigation
       ${pose1}=    Create Pose    x=1.0    y=1.0    yaw=0.0
       ${pose2}=    Create Pose    x=2.0    y=2.0    yaw=1.57
       ${poses}=    Create List    ${pose1}    ${pose2}
       ${result}=    Navigate Through Poses    ${poses}
       Should Be True    ${result.success}

   Test Path Planning
       ${path}=    Plan Path    start_x=0.0    start_y=0.0    goal_x=3.0    goal_y=3.0
       Should Not Be Empty    ${path}
       ${length}=    Get Path Length    ${path}
       Should Be True    ${length} > 0

   Test Recovery Operations
       Start Recovery
       ${active}=    Is Recovery Active
       Should Be True    ${active}
       Stop Recovery

   Test Navigation Monitoring
       ${status}=    Monitor Navigation Status
       Should Not Be Empty    ${status}
       ${info}=    Get Navigation Info
       Should Not Be Empty    ${info}

Complex Navigation Scenario
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   *** Test Cases ***
   Test Complete Navigation Workflow
       # Setup
       Set Initial Pose    x=0.0    y=0.0    yaw=0.0
       ${localized}=    Wait For Localization    timeout=10.0
       Should Be True    ${localized}

       # Navigation
       ${result}=    Navigate To Pose    x=1.0    y=1.0    yaw=0.0
       Should Be True    ${result.success}

       # Verify position
       ${pose}=    Get Current Pose
       Should Not Be Empty    ${pose}

       # Test recovery if needed
       ${active}=    Is Recovery Active
       IF    ${active}
           Stop Recovery
       END

       # Cleanup
       Cleanup
