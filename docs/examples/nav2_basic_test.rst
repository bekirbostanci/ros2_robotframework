Navigation2 Basic Test
=====================

This example demonstrates Navigation2 operations using the Nav2ClientLibrary.

Overview
--------

The Navigation2 basic test covers:

- **Navigation2 Library Import**: Verify library initialization
- **Basic Navigation Operations**: Distance and angle calculations
- **Pose Management**: Initial pose setting and localization
- **Navigation Status**: Real-time navigation status monitoring
- **Costmap Operations**: Global and local costmap management
- **Path Planning**: Path computation and validation
- **Navigation Through Poses**: Multi-waypoint navigation
- **Navigation Cancellation**: Navigation cancellation and status verification

Test Structure
--------------

.. code-block:: robot

   *** Settings ***
   Documentation    Basic Navigation2 test using the Nav2ClientLibrary
   Library          ros2_client.ROS2ClientLibrary
   Library          nav2_client.Nav2ClientLibrary
   Library          Collections
   Library          OperatingSystem
   Library          Process

   *** Variables ***
   ${NAV2_TIMEOUT}    60.0
   ${POSE_TIMEOUT}    30.0
   ${WAIT_TIME}       5s
   ${TOLERANCE}       0.5

Key Test Cases
--------------

Navigation2 Library Import
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Navigation2 Library Import
       [Documentation]    Verify that the Navigation2 library can be imported and initialized
       [Tags]    smoke
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       Log    Navigation2 library imported successfully

Basic Navigation Operations
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Basic Navigation Operations
       [Documentation]    Test basic navigation operations with simulation
       [Tags]    navigation
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Test utility functions
       ${distance}=    Calculate Distance    0.0    0.0    3.0    4.0
       Should Be Equal As Numbers    ${distance}    5.0    0.1
       
       ${angle}=    Calculate Angle    0.0    0.0    1.0    1.0
       Should Be Equal As Numbers    ${angle}    0.785398    0.001
       
       ${degrees}=    Radians To Degrees    1.570796
       Should Be Equal As Numbers    ${degrees}    90.0    precision=0
       
       ${radians}=    Degrees To Radians    90.0
       Should Be Equal As Numbers    ${radians}    1.570796    0.1

Pose Management
~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Pose Management
       [Documentation]    Test pose management and transformations
       [Tags]    pose
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Test setting initial pose
       ${success}=    Set Initial Pose    0.0    0.0    0.0
       Should Be True    ${success}
       
       # Test getting current pose (may return None if no robot)
       ${current_pose}=    Get Current Pose    timeout=${POSE_TIMEOUT}
       Log    Current pose: ${current_pose}
       
       # Test waiting for localization
       ${localized}=    Wait For Localization    timeout=10.0
       Log    Localization status: ${localized}

Navigation Status
~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Navigation Status
       [Documentation]    Test navigation status operations
       [Tags]    status
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Check if navigation is active
       ${active}=    Is Navigation Active
       Should Be Equal    ${active}    ${False}
       
       # Get navigation status
       ${status}=    Get Navigation Status
       Should Not Be Empty    ${status}
       Should Contain    ${status}    navigation_active
       Should Contain    ${status}    current_pose
       Should Contain    ${status}    goal_pose

Costmap Operations
~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Costmap Operations
       [Documentation]    Test costmap information retrieval
       [Tags]    costmap
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Get global costmap info
       ${global_info}=    Get Costmap Info    global
       Log    Global costmap info: ${global_info}
       
       # Get local costmap info
       ${local_info}=    Get Costmap Info    local
       Log    Local costmap info: ${local_info}
       
       # Test clearing costmaps (may fail if no robot running)
       ${cleared_global}=    Clear Costmap    global
       Log    Global costmap cleared: ${cleared_global}
       
       ${cleared_local}=    Clear Costmap    local
       Log    Local costmap cleared: ${cleared_local}

Path Planning
~~~~~~~~~~~~~

.. code-block:: robot

   Test Path Planning
       [Documentation]    Test path planning operations
       [Tags]    path_planning
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Test path computation (may fail if no robot/map)
       ${path}=    Compute Path    0.0    0.0    0.0    2.0    1.0    1.57
       Log    Computed path: ${path}
       
       # If path was computed, test path length calculation
       IF    ${path} is not None
           ${path_length}=    Get Path Length    ${path}
           Should Be True    ${path_length} > 0.0
           Log    Path length: ${path_length} meters
       END

Navigation Through Poses
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Navigation Through Poses
       [Documentation]    Test navigation through multiple poses
       [Tags]    navigation    poses
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Create a list of poses
       @{poses}=    Create List
       
       # Add poses to the list
       ${pose1}=    Create Dictionary    x=-2.0    y=0.0    theta=0.0
       ${pose2}=    Create Dictionary    x=-2.0    y=1.5    theta=1.57
       ${pose3}=    Create Dictionary    x=-2.0    y=1.0    theta=3.14
       
       Append To List    ${poses}    ${pose1}
       Append To List    ${poses}    ${pose2}
       Append To List    ${poses}    ${pose3}
       
       Log    Created ${poses} poses for navigation
       
       # Test navigation through poses (may fail if no robot)
       ${result}=    Navigate Through Poses    ${poses}    timeout=${NAV2_TIMEOUT}
       Log    Navigation result: ${result}
       
       # Check if navigation was successful
       IF    $result is not None
           Should Be True    $result.success
           Log    Navigation completed successfully
       ELSE
           Log    Navigation result is None
       END

Single Pose Navigation
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Single Pose Navigation
       [Documentation]    Test navigation to a single pose
       [Tags]    navigation    single_pose
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Test navigation to a single pose (may fail if no robot)
       ${result}=    Navigate To Pose Simple    1.0    1.0    0.785    timeout=${NAV2_TIMEOUT}
       Log    Navigation result: ${result}
       
       # Check if navigation was successful
       IF    ${result} is not None
           Should Be True    ${result}
           Log    Navigation completed successfully
       ELSE
           Log    Navigation result is None
       END

Navigation Cancellation
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Navigation Cancellation
       [Documentation]    Test navigation cancellation
       [Tags]    navigation    cancellation
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Test cancelling navigation
       ${cancelled}=    Cancel Navigation
       Log    Navigation cancelled: ${cancelled}
       
       # Verify navigation is not active after cancellation
       ${active}=    Is Navigation Active
       Should Be Equal    ${active}    ${False}

Setup and Teardown
------------------

The test includes comprehensive setup and teardown procedures:

.. code-block:: robot

   Setup Navigation2 Simulation
       [Documentation]    Setup Navigation2 simulation
       ${running}=    Has Running Nodes
       Should Be Equal    ${running}    ${False}
       # Set environment variables for the test
       Set Environment Variable    TURTLEBOT3_MODEL      waffle
       
       # Clean up any existing simulation
       Clean Up Navigation2 Simulation
       
       # Launch the Navigation2 simulation
       Log    Starting Navigation2 simulation launch...
       ${process}=    Launch Package    turtlebot3    simulation.launch.py
       Should Not Be Equal    ${process}    ${None}
       Log    Launch process started with PID: ${process.pid}
       
       ${ready}=    Wait For Nav2 Ready
       Should Be True    ${ready}
       
       # Wait for the launch to initialize
       Sleep    ${WAIT_TIME}
       RETURN    ${process}
   
   Clean Up Navigation2 Simulation
       [Documentation]    Clean up Navigation2 simulation
       ${shutdown}=    Shutdown Process    ign gazebo
       Should Be True    ${shutdown}
       Log    Navigation2 simulation cleanup completed
       
       ${shutdown}=    Shutdown Process    ros_gz_bridge
       Should Be True    ${shutdown}
       
       ${shutdown}=    Shutdown Process    rviz2
       Should Be True    ${shutdown}
       
       ${shutdown}=    Kill Process By Name    ros
       Should Be True    ${shutdown}
       
       ${running}=    Has Running Nodes
       Should Be Equal    ${running}    ${False}

Running the Example
-------------------

To run this example:

.. code-block:: bash

   # Run the Navigation2 basic test
   robot examples/basics/nav2_basic_test.robot
   
   # Run with verbose output
   robot -v examples/basics/nav2_basic_test.robot
   
   # Run specific test cases
   robot -t "Test Basic Navigation Operations" examples/basics/nav2_basic_test.robot

Expected Output
---------------

The test will:

1. **Verify Library Import**: Ensure Navigation2 library is properly imported
2. **Test Basic Operations**: Verify distance and angle calculations
3. **Test Pose Management**: Set initial pose and check localization
4. **Test Navigation Status**: Monitor navigation status and activity
5. **Test Costmap Operations**: Retrieve and clear costmap information
6. **Test Path Planning**: Compute paths and calculate path lengths
7. **Test Navigation**: Navigate through multiple poses and single poses
8. **Test Cancellation**: Cancel navigation and verify status

Key Learning Points
-------------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **Navigation2 Operations**
      
      Learn Navigation2-specific operations

   .. panel::
      :body:

      **Pose Management**
      
      Understand pose setting and localization

   .. panel::
      :body:

      **Costmap Operations**
      
      Learn costmap information and clearing

   .. panel::
      :body:

      **Path Planning**
      
      Understand path computation and validation

   .. panel::
      :body:

      **Navigation Control**
      
      Learn navigation start, stop, and cancellation

   .. panel::
      :body:

      **Status Monitoring**
      
      Understand navigation status monitoring

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

1. **Simulation not available**: Ensure TurtleBot3 simulation is installed
2. **Navigation failures**: Check if robot is properly localized
3. **Costmap errors**: Verify costmap configuration
4. **Path planning failures**: Check map and robot configuration

Debug Tips
~~~~~~~~~~

1. Check Navigation2 status before navigation
2. Verify robot localization
3. Test individual operations manually
4. Check simulation environment setup

Next Steps
----------

After running this example, you can:

- Explore the :doc:`nav2_simple_monitoring` example for advanced monitoring
- Check the :doc:`nav2_turtlebot` example for complete TurtleBot3 testing
- Read the :doc:`../user_guide/nav2_operations` for detailed Navigation2 usage
- Review the :doc:`../api/nav2_client` for complete API reference
