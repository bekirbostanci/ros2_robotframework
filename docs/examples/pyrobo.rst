PyRoboSim
=========

This example demonstrates PyRoboSim integration using the ROS2ClientLibrary.

Overview
--------

The PyRoboSim example covers:

- **PyRoboSim Setup**: Launch and manage PyRoboSim simulation
- **Action Goal Testing**: Send action goals to PyRoboSim
- **Navigation Testing**: Test robot navigation between locations
- **World State Monitoring**: Monitor robot state and position
- **Multi-robot Testing**: Test multiple robots simultaneously
- **Service Integration**: Use native service clients for better performance

Test Structure
--------------

.. code-block:: robot

   *** Settings ***
   Documentation    Simple Navigation2 test: launch simulator, wait 5s, send vehicle to another place
   Library          ros2_client.ROS2ClientLibrary
   Library          Collections

   *** Variables ***
   ${WAIT_TIME}             5s
   ${PYROBOSIM_SETUP}       /home/bekir/ros2_ws/src/pyrobosim/setup/source_pyrobosim.bash

Key Test Cases
--------------

PyRoboSim Action Goal Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test PyRobo Simulation Send Command
       [Documentation]    Test Pyrobo simulation send command
       [Tags]    pyrobosim    send    command
       [Setup]    Setup PyRobo Simulation
       [Teardown]    Clean Up PyRobo Simulation
       
       # Test the new action send_goal functionality
       ${result}=    Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}  timeout=15.0
       Log    Action result: ${result}
       Should Be True    ${result}[success]    Action should be sent successfully

       ${topics}=    List Topics
       Log    Topics: ${topics}
       Sleep    1s

Navigation Testing
~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test PyRobo Action Navigation
       [Documentation]    Test Pyrobo action navigation with exact command from user
       [Tags]    pyrobosim    action    navigation
       [Setup]    Setup PyRobo Simulation
       [Teardown]    Clean Up PyRobo Simulation
       
       # Wait for the simulation to be ready
       Sleep    2s
       
       # Send the exact action command as requested by user
       ${result}=    Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "table"}, "realtime_factor": 1.0}
       Log    Action navigation result: ${result}
       Should Be True    ${result}[success]    Action navigation should be sent successfully
       
       # Log the raw output for debugging
       Log    Raw action output: ${result}[raw_output]
       
       # Wait for navigation to complete
       Sleep    1s
       
       # Create service client for world state request (using native service calls for better performance)
       ${world_state_client}=    Create Service Client    /request_world_state    pyrobosim_msgs/srv/RequestWorldState
       
       # Check if service is available
       ${service_available}=    Service Available    /request_world_state    timeout=5.0
       Should Be True    ${service_available}    World state service should be available
       
       # Get robot position after navigation using native service call
       ${world_state}=    Call Service    /request_world_state    timeout=10.0
       Log    World state response: ${world_state}
       
       # Extract robot position from the response (now properly formatted)
       ${robot_position}=    Set Variable    ${world_state}[state][robots][0][pose][position]
       Log    Robot position after navigation: X=${robot_position}[x], Y=${robot_position}[y], Z=${robot_position}[z]
       
       # Extract robot orientation
       ${robot_orientation}=    Set Variable    ${world_state}[state][robots][0][pose][orientation]
       Log    Robot orientation after navigation: X=${robot_orientation}[x], Y=${robot_orientation}[y], Z=${robot_orientation}[z], W=${robot_orientation}[w]
       
       # Log additional robot info
       ${robot_info}=    Set Variable    ${world_state}[state][robots][0]
       Log    Robot name: ${robot_info}[name]
       Log    Robot last visited location: ${robot_info}[last_visited_location]
       Log    Robot executing action: ${robot_info}[executing_action]
       Log    Robot battery level: ${robot_info}[battery_level]
       
       # Log some world state info
       Log    Number of locations: ${world_state}[state][locations].__len__()
       Log    Number of objects: ${world_state}[state][objects].__len__()
       
       # Get service info for debugging
       ${service_info}=    Get Service Info
       Log    Service clients created: ${service_info}[clients]
       Log    Service client for /request_world_state: ${service_info}[clients][/request_world_state]

Multi-robot Testing
~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test PyRobo Multiple Robot
       [Documentation]    Test Pyrobo multiple robot
       [Tags]    pyrobosim    multiple    robot
       [Setup]    Setup PyRobo Multiple Robot Simulation
       [Teardown]    Clean Up PyRobo Simulation
       
       # Test the new async action send_goal functionality - fire and forget (parallel execution)
       # Both robots will start their navigation tasks simultaneously without waiting for completion
       Async Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot0", "type": "navigate", "source_location": "kitchen", "target_location": "table"}, "realtime_factor": 1.0}
       
       Async Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot1", "type": "navigate", "source_location": "kitchen", "target_location": "my_desk"}, "realtime_factor": 1.0}
       

       FOR    ${i}    IN RANGE    20
           Log    Waiting for 1 second...
           Sleep    1s

           ${completed}=    Set Variable    True
           ${robot0_location}=    Check Robot Last Visited Location    robot0
           ${robot0_location_compare}=    Run Keyword And Return Status    Should Be Equal As Strings    ${robot0_location}    table0_tabletop
           ${robot1_location}=    Check Robot Last Visited Location    robot1
           ${robot1_location_compare}=    Run Keyword And Return Status    Should Be Equal As Strings    ${robot1_location}    my_desk_desktop


           IF    ${robot0_location_compare} and ${robot1_location_compare}
               Log    Both robots have reached their destinations!
               Exit For Loop
           END
       END

Setup and Teardown
------------------

The test includes comprehensive setup and teardown procedures:

.. code-block:: robot

   Setup PyRobo Simulation
       [Documentation]    Setup Pyrobo simulation

       # Launch the Pyrobo simulations
       Log    Starting Pyrobo simulation launch...

       # Test with setup script
       ${process}=    Run Node     pyrobosim_ros    demo.py    setup_script=${PYROBOSIM_SETUP}
       
       # Get process output for debugging
       ${output}=    Get Process Output    ${process}
       Log    Process stdout: ${output}[stdout]
       Log    Process stderr: ${output}[stderr]

       # Check if we can see the node
       ${nodes}=    List Nodes
       Log    Available nodes: ${nodes}
       
       RETURN    ${process}
   
   Clean Up Pyrobo Simulation
       [Documentation]    Clean up Navigation2 simulation

       # Clean up service clients and other ROS2 resources
       Cleanup
       
       ${shutdown}=    Kill Process By Name    demo.py
       ${shutdown}=    Kill Process By Name    pyrobosim_ros demo.py
       Shutdown Process    pyrobosim_ros

       Log    Pyrobo simulation cleanup completed

Key Features
------------

Action Goal Testing
~~~~~~~~~~~~~~~~~~~

This example demonstrates action goal testing:

- **Action Goal Sending**: Send action goals to PyRoboSim
- **Navigation Commands**: Test robot navigation between locations
- **Result Verification**: Verify action goal success
- **Timeout Handling**: Handle action goal timeouts

Service Integration
~~~~~~~~~~~~~~~~~~~

The example shows native service client usage:

.. code-block:: robot

   # Create service client for world state request (using native service calls for better performance)
   ${world_state_client}=    Create Service Client    /request_world_state    pyrobosim_msgs/srv/RequestWorldState
   
   # Check if service is available
   ${service_available}=    Service Available    /request_world_state    timeout=5.0
   Should Be True    ${service_available}    World state service should be available
   
   # Get robot position after navigation using native service call
   ${world_state}=    Call Service    /request_world_state    timeout=10.0
   Log    World state response: ${world_state}

Multi-robot Testing
~~~~~~~~~~~~~~~~~~~

The example demonstrates multi-robot testing:

.. code-block:: robot

   # Test the new async action send_goal functionality - fire and forget (parallel execution)
   # Both robots will start their navigation tasks simultaneously without waiting for completion
   Async Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot0", "type": "navigate", "source_location": "kitchen", "target_location": "table"}, "realtime_factor": 1.0}
   
   Async Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot1", "type": "navigate", "source_location": "kitchen", "target_location": "my_desk"}, "realtime_factor": 1.0}

World State Monitoring
~~~~~~~~~~~~~~~~~~~~~~

The example shows world state monitoring:

.. code-block:: robot

   # Extract robot position from the response (now properly formatted)
   ${robot_position}=    Set Variable    ${world_state}[state][robots][0][pose][position]
   Log    Robot position after navigation: X=${robot_position}[x], Y=${robot_position}[y], Z=${robot_position}[z]
   
   # Extract robot orientation
   ${robot_orientation}=    Set Variable    ${world_state}[state][robots][0][pose][orientation]
   Log    Robot orientation after navigation: X=${robot_orientation}[x], Y=${robot_orientation}[y], Z=${robot_orientation}[z], W=${robot_orientation}[w]
   
   # Log additional robot info
   ${robot_info}=    Set Variable    ${world_state}[state][robots][0]
   Log    Robot name: ${robot_info}[name]
   Log    Robot last visited location: ${robot_info}[last_visited_location]
   Log    Robot executing action: ${robot_info}[executing_action]
   Log    Robot battery level: ${robot_info}[battery_level]

Running the Example
-------------------

To run this example:

.. code-block:: bash

   # Run the PyRoboSim example
   robot examples/medium/pyrobo_example.robot
   
   # Run with verbose output
   robot -v examples/medium/pyrobo_example.robot
   
   # Run specific test cases
   robot -t "Test PyRobo Simulation Send Command" examples/medium/pyrobo_example.robot

Expected Output
---------------

The test will:

1. **Setup PyRoboSim**: Launch PyRoboSim simulation
2. **Test Action Goals**: Send action goals and verify success
3. **Test Navigation**: Test robot navigation between locations
4. **Monitor World State**: Monitor robot state and position
5. **Test Multi-robot**: Test multiple robots simultaneously
6. **Verify Results**: Verify robot arrival at destinations

Key Learning Points
-------------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **PyRoboSim Integration**
      
      Learn to integrate PyRoboSim with ROS2

   .. panel::
      :body:

      **Action Goal Testing**
      
      Understand action goal sending and verification

   .. panel::
      :body:

      **Service Integration**
      
      Learn to use native service clients

   .. panel::
      :body:

      **Multi-robot Testing**
      
      Understand multi-robot testing procedures

   .. panel::
      :body:

      **World State Monitoring**
      
      Learn to monitor robot state and position

   .. panel::
      :body:

      **Async Operations**
      
      Understand asynchronous action goal sending

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

1. **PyRoboSim not available**: Ensure PyRoboSim is installed and configured
2. **Action goal failures**: Check action goal format and parameters
3. **Service availability**: Verify service availability and configuration
4. **Multi-robot issues**: Check multi-robot simulation setup

Debug Tips
~~~~~~~~~~

1. Check PyRoboSim environment setup
2. Verify action goal format
3. Test service availability
4. Check multi-robot configuration

Next Steps
----------

After running this example, you can:

- Explore the :doc:`nav2_simple_monitoring` example for Navigation2 testing
- Check the :doc:`nav2_turtlebot` example for TurtleBot3 integration
- Read the :doc:`../user_guide/overview` for detailed usage information
- Review the :doc:`../api/ros2_client` for complete API reference
