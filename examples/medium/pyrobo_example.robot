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
    ${result}=    Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}
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
    ${result}=    Send Action Goal    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    {"action": {"robot": "robot", "type": "navigate", "source_location": "kitchen", "target_location": "desk"}, "realtime_factor": 1.0}
    Log    Action navigation result: ${result}
    Should Be True    ${result}[success]    Action navigation should be sent successfully
    
    # Log the raw output for debugging
    Log    Raw action output: ${result}[raw_output]
    
    Sleep    3s

*** Keywords ***
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

    ${shutdown}=    Kill Process By Name    demo.py
    ${shutdown}=    Kill Process By Name    pyrobosim_ros demo.py
    Shutdown Process    pyrobosim_ros

    Log    Pyrobo simulation cleanup completed