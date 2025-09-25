Navigation2 TurtleBot3
======================

This example demonstrates complete TurtleBot3 Navigation2 testing using the Nav2ClientLibrary.

Overview
--------

The Navigation2 TurtleBot3 test covers:

- **TurtleBot3 Simulation Setup**: Launch TurtleBot3 Gazebo simulation
- **Navigation2 Integration**: Start Navigation2 with TurtleBot3
- **Initial Pose Setting**: Set robot initial pose for localization
- **Navigation Testing**: Test navigation to specific poses
- **Position Verification**: Check robot position using transforms
- **Tolerance Checking**: Verify robot arrival within specified tolerance

Test Structure
--------------

.. code-block:: robot

   *** Settings ***
   Documentation    Simple Navigation2 test: launch simulator, wait 5s, send vehicle to another place
   Library          ros2_client.ROS2ClientLibrary
   Library          nav2_client.Nav2ClientLibrary
   Library          Collections
   Library          OperatingSystem
   Library          Process

   *** Variables ***
   ${WAIT_TIME}             5s
   ${NAVIGATION_TIMEOUT}    30
   ${GOAL_X}               -1.7
   ${GOAL_Y}                0.5
   ${GOAL_THETA}            1.57
   ${TOLERANCE}             0.5

Key Test Cases
--------------

Navigation2 Simple Movement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Navigation2 Simple Movement
       [Documentation]    Launch Navigation2 simulation, wait 5 seconds, send vehicle to another place
       [Tags]    nav2    simple    movement
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
       # Send vehicle to another place
       Log    Sending vehicle to position (${GOAL_X}, ${GOAL_Y}, ${GOAL_THETA})...
       ${nav_success}=    Navigate To Pose Simple    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}    timeout=${NAVIGATION_TIMEOUT}
       Log    Navigation command sent: ${nav_success}
       
       # Get robot pose
       ${final_pose}=   Get Transform    map    base_link
       Log    Final position: ${final_pose}
       Should Not Be Empty    ${final_pose}

       # Check if robot is within tolerance
       IF    ${final_pose} is not None
           ${arrived}=    Is Within Tolerance    ${final_pose}    tolerance=${TOLERANCE}    target_x=${GOAL_X}    target_y=${GOAL_Y}
           Should Be True    ${arrived}
       ELSE
           Log    Final position is None, skipping tolerance check
       END

Setup and Teardown
------------------

The test includes comprehensive setup and teardown procedures:

.. code-block:: robot

   Setup Navigation2 Simulation
       [Documentation]    Setup Navigation2 simulation
       # Clean up any existing simulation
       Clean Up Navigation2 Simulation

       # Set environment variables for the test
       Set Environment Variable    TURTLEBOT3_MODEL      waffle
       
       # Launch the TurtleBot3 Gazebo simulation
       Log    Starting TurtleBot3 Gazebo simulation launch...
       ${sim_process}=    Launch Package    turtlebot3_gazebo    turtlebot3_world.launch.py
       Log    Simulation process started with PID: ${sim_process.pid}
       
       # Wait for simulation to initialize
       Sleep    3s
       
       # Launch Navigation2
       Log    Starting Navigation2 launch...
       ${nav_process}=    Launch Package    turtlebot3_navigation2    navigation2.launch.py    arguments=use_sim_time:=True

       Sleep    5s

       # Use CLI service call directly (to avoid JSON conversion issues)
       ${result}=    Run Process    ros2    service    call    /set_initial_pose    nav2_msgs/srv/SetInitialPose    {pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}
       Should Be Equal As Integers    ${result.rc}    0    Service call should succeed
       Log    Service call output: ${result.stdout}
       Log    Service call stderr: ${result.stderr}

       Sleep    2s

       Log    Navigation2 process started with PID: ${nav_process.pid}

       ${ready}=    Wait For Nav2 Ready
       Should Be True    ${ready}

       # Wait for the launch to initialize
       Sleep    ${WAIT_TIME}
       RETURN    ${sim_process}    ${nav_process}
   
   Clean Up Navigation2 Simulation
       [Documentation]    Clean up Navigation2 simulation
       # Shutdown Navigation2 processes
       ${shutdown}=    Shutdown Process    navigation2
       Should Be True    ${shutdown}
       Log    Navigation2 cleanup completed

       # Shutdown Gazebo simulation
       ${shutdown}=    Shutdown Process    gazebo
       Should Be True    ${shutdown}
       Log    Gazebo simulation cleanup completed

Key Features
------------

TurtleBot3 Integration
~~~~~~~~~~~~~~~~~~~~~~

This example demonstrates complete TurtleBot3 integration:

- **Gazebo Simulation**: Launch TurtleBot3 in Gazebo world
- **Navigation2 Stack**: Start Navigation2 with TurtleBot3 configuration
- **Initial Pose Setting**: Set robot initial pose for proper localization
- **Navigation Testing**: Test navigation to specific poses
- **Position Verification**: Check robot position using transforms

Initial Pose Setting
~~~~~~~~~~~~~~~~~~~~

The example shows how to set the initial pose using direct service calls:

.. code-block:: robot

   # Use CLI service call directly (to avoid JSON conversion issues)
   ${result}=    Run Process    ros2    service    call    /set_initial_pose    nav2_msgs/srv/SetInitialPose    {pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}
   Should Be Equal As Integers    ${result.rc}    0    Service call should succeed
   Log    Service call output: ${result.stdout}
   Log    Service call stderr: ${result.stderr}

Navigation Testing
~~~~~~~~~~~~~~~~~~

The example demonstrates navigation testing:

.. code-block:: robot

   # Send vehicle to another place
   Log    Sending vehicle to position (${GOAL_X}, ${GOAL_Y}, ${GOAL_THETA})...
   ${nav_success}=    Navigate To Pose Simple    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}    timeout=${NAVIGATION_TIMEOUT}
   Log    Navigation command sent: ${nav_success}
   
   # Get robot pose
   ${final_pose}=   Get Transform    map    base_link
   Log    Final position: ${final_pose}
   Should Not Be Empty    ${final_pose}

   # Check if robot is within tolerance
   IF    ${final_pose} is not None
       ${arrived}=    Is Within Tolerance    ${final_pose}    tolerance=${TOLERANCE}    target_x=${GOAL_X}    target_y=${GOAL_Y}
       Should Be True    ${arrived}
   ELSE
       Log    Final position is None, skipping tolerance check
   END

Running the Example
-------------------

To run this example:

.. code-block:: bash

   # Run the Navigation2 TurtleBot3 test
   robot examples/medium/nav2_turtlebot.robot
   
   # Run with verbose output
   robot -v examples/medium/nav2_turtlebot.robot
   
   # Run specific test cases
   robot -t "Test Navigation2 Simple Movement" examples/medium/nav2_turtlebot.robot

Expected Output
---------------

The test will:

1. **Setup TurtleBot3 Simulation**: Launch TurtleBot3 in Gazebo world
2. **Start Navigation2**: Launch Navigation2 with TurtleBot3 configuration
3. **Set Initial Pose**: Set robot initial pose for proper localization
4. **Test Navigation**: Navigate to a specific pose and verify arrival
5. **Verify Position**: Check robot position using transforms
6. **Check Tolerance**: Verify robot arrival within specified tolerance

Key Learning Points
-------------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **TurtleBot3 Integration**
      
      Learn to integrate TurtleBot3 with Navigation2

   .. panel::
      :body:

      **Gazebo Simulation**
      
      Understand Gazebo simulation setup

   .. panel::
      :body:

      **Initial Pose Setting**
      
      Learn to set robot initial pose

   .. panel::
      :body:

      **Navigation Testing**
      
      Understand navigation testing procedures

   .. panel::
      :body:

      **Position Verification**
      
      Learn to verify robot position

   .. panel::
      :body:

      **Tolerance Checking**
      
      Understand position tolerance verification

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

1. **TurtleBot3 not available**: Ensure TurtleBot3 packages are installed
2. **Gazebo simulation issues**: Check Gazebo installation and configuration
3. **Navigation2 failures**: Verify Navigation2 configuration
4. **Initial pose issues**: Check initial pose setting

Debug Tips
~~~~~~~~~~

1. Check TurtleBot3 environment variables
2. Verify Gazebo simulation startup
3. Test Navigation2 configuration
4. Check initial pose setting

Next Steps
----------

After running this example, you can:

- Explore the :doc:`nav2_simple_monitoring` example for advanced monitoring
- Check the :doc:`pyrobo_example` example for PyRoboSim integration
- Read the :doc:`../user_guide/nav2_operations` for detailed Navigation2 usage
- Review the :doc:`../api/nav2_client` for complete API reference
