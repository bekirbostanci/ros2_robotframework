*** Settings ***
Documentation    Simple Navigation2 test: launch simulator, wait 5s, send vehicle to another place
Library          ros2_client.ROS2ClientLibrary
Library          Collections

*** Variables ***
${WAIT_TIME}             5s
${PYROBOSIM_SETUP}       /home/bekir/ros2_ws/src/pyrobosim/setup/source_pyrobosim.bash

*** Test Cases ***
Test Shutdown PyRobo Simulation
    [Documentation]    Test shutdown Pyrobo simulation
    [Tags]    pyrobosim    shutdown
    [Setup]    Setup PyRobo Simulation
    [Teardown]    Clean Up Pyrobo Simulation
    
    Sleep    1s    

Test PyRobo Simulation Send Command
    [Documentation]    Test Pyrobo simulation send command
    [Tags]    pyrobosim    send    command
    [Setup]    Setup PyRobo Simulation
    [Teardown]    Clean Up PyRobo Simulation
    
    Call Action Service    /execute_action    pyrobosim_msgs/action/ExecuteTaskAction    "forward 1.0"
    Sleep    1s

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