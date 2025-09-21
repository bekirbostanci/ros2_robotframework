*** Settings ***
Documentation    Simple Navigation2 test: launch simulator, wait 5s, send vehicle to another place
Library          ros2_client.ROS2ClientLibrary
Library          Collections

*** Variables ***
${WAIT_TIME}             5s
${PYROBOSIM_SETUP}       /home/bekir/ros2_ws/src/pyrobosim/setup/source_pyrobosim.bash

*** Test Cases ***
Test PyRobo Simulation Send Command
    [Documentation]    Test Pyrobo simulation send command
    [Tags]    pyrobosim    send    command
    [Setup]    Setup PyRobo Simulation
    [Teardown]    Clean Up PyRobo Simulation
    
    # Test the new action send_goal functionality
    ${result}=    Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}  timeout=15.0
    Log    Action result: ${result}
    Should Be True    ${result}[success]    Action should be sent successfully
    Sleep    1s

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

Test PyRobo Multiple Robot
    [Documentation]    Test Pyrobo multiple robot
    [Tags]    pyrobosim    multiple    robot
    [Setup]    Setup PyRobo Multiple Robot Simulation
    [Teardown]    Clean Up PyRobo Simulation
    
    # Test the new async action send_goal functionality
    Async Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot0", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}
    
    Async Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot1", "type": "navigate", "source_location": "kitchen", "target_location": "my_desk"}, "realtime_factor": 1.0}
    Sleep    2s
    
*** Keywords ***
Setup PyRobo Multiple Robot Simulation
    [Documentation]    Setup Pyrobo simulation

    # Launch the Pyrobo simulations
    Log    Starting Pyrobo simulation launch...

    # Test with setup script
    ${process}=    Run Node     pyrobosim_ros   demo.py --ros-args -p world_file:=test_world_multirobot.yaml    setup_script=${PYROBOSIM_SETUP}
    
    # Get process output for debugging
    ${output}=    Get Process Output    ${process}
    Log    Process stdout: ${output}[stdout]
    Log    Process stderr: ${output}[stderr]

    # Check if we can see the node
    ${nodes}=    List Nodes
    Log    Available nodes: ${nodes}
    
    RETURN    ${process} 

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