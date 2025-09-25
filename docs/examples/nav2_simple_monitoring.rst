Navigation2 Simple Monitoring
============================

This example demonstrates Navigation2 monitoring and control operations using the Nav2ClientLibrary.

Overview
--------

The Navigation2 simple monitoring test covers:

- **Navigation2 Simulation Setup**: Launch and manage Navigation2 simulation
- **Simple Movement**: Navigate to a specific pose and verify arrival
- **Navigation Cancellation**: Test navigation cancellation and status monitoring
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

Navigation Cancellation
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Navigation2 Cancel Navigation
       [Documentation]    Test Navigation2 cancel navigation
       [Tags]    nav2    cancel    navigation
       [Setup]    Setup Navigation2 Simulation
       [Teardown]    Clean Up Navigation2 Simulation
       
        # Send vehicle to another place
       Log    Sending vehicle to position (${GOAL_X}, ${GOAL_Y}, ${GOAL_THETA})...
       Async Navigate To Pose Simple    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}    timeout=${NAVIGATION_TIMEOUT}

       Sleep    1s    
       # Call Service    /navigate_to_pose/_action/cancel_goal    action_msgs/srv/CancelGoal   {}
       Cancel Navigation
       # Check if navigation is active
       ${active}=    Is Navigation Active
       Should Be Equal    ${active}    ${False}
       
       # Get navigation status
       ${status}=    Get Navigation Status
       Should Not Be Empty    ${status}
       Should Contain    ${status}    navigation_active
       Should Contain    ${status}    current_pose
       Should Contain    ${status}    goal_pose

       ${final_pose}=    Get Transform    map    base_link
       Should Not Be Empty    ${final_pose}

       IF    ${final_pose} is not None
           ${arrived}=    Is Within Tolerance    ${final_pose}    tolerance=${TOLERANCE}    target_x=${GOAL_X}    target_y=${GOAL_Y}
           Should Be Equal    ${arrived}    ${False}
       ELSE
           Log    Final position is None, skipping tolerance check
       END

Setup and Teardown
------------------

The test includes comprehensive setup and teardown procedures:

.. code-block:: robot

   Setup Navigation2 Simulation
       [Documentation]    Setup Navigation2 simulation
       ${running}=    Has Running Nodes
       # Clean up any existing simulation
       Clean Up Navigation2 Simulation

       Should Be Equal    ${running}    ${False}
       # Set environment variables for the test
       Set Environment Variable    TURTLEBOT3_MODEL      waffle
       
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

   # Run the Navigation2 simple monitoring test
   robot examples/medium/nav2_simple_monitoring_test.robot
   
   # Run with verbose output
   robot -v examples/medium/nav2_simple_monitoring_test.robot
   
   # Run specific test cases
   robot -t "Test Navigation2 Simple Movement" examples/medium/nav2_simple_monitoring_test.robot

Expected Output
---------------

The test will:

1. **Setup Navigation2 Simulation**: Launch TurtleBot3 simulation with Navigation2
2. **Test Simple Movement**: Navigate to a specific pose and verify arrival
3. **Test Navigation Cancellation**: Start navigation, cancel it, and verify status
4. **Verify Position**: Check robot position using transforms
5. **Check Tolerance**: Verify robot arrival within specified tolerance

Key Learning Points
-------------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **Navigation2 Setup**
      
      Learn to launch and manage Navigation2 simulation

   .. panel::
      :body:

      **Navigation Control**
      
      Understand navigation start, stop, and cancellation

   .. panel::
      :body:

      **Position Verification**
      
      Learn to check robot position using transforms

   .. panel::
      :body:

      **Tolerance Checking**
      
      Understand position tolerance verification

   .. panel::
      :body:

      **Status Monitoring**
      
      Learn to monitor navigation status

   .. panel::
      :body:

      **Async Operations**
      
      Understand asynchronous navigation operations

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

1. **Simulation not available**: Ensure TurtleBot3 simulation is installed
2. **Navigation failures**: Check if robot is properly localized
3. **Transform errors**: Verify transform availability between frames
4. **Timeout errors**: Increase timeout values for slow systems

Debug Tips
~~~~~~~~~~

1. Check Navigation2 status before navigation
2. Verify robot localization
3. Test individual operations manually
4. Check simulation environment setup

Next Steps
----------

After running this example, you can:

- Explore the :doc:`nav2_turtlebot` example for complete TurtleBot3 testing
- Check the :doc:`pyrobo_example` example for PyRoboSim integration
- Read the :doc:`../user_guide/nav2_operations` for detailed Navigation2 usage
- Review the :doc:`../api/nav2_client` for complete API reference
