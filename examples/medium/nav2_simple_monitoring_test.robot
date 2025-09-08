*** Settings ***
Documentation    Simple Navigation2 test: launch simulator, wait 5s, send vehicle to another place
Library          ros2_client.ROS2ClientLibrary
Library          nav2_client.Nav2ClientLibrary
Library          Collections
Library          OperatingSystem

*** Variables ***
${LAUNCH_TIMEOUT}    10.0
${WAIT_TIME}         5s
${WAIT_TIME_FOR_FINAL_POSE}    30
${GOAL_X}            2.0
${GOAL_Y}            1.0
${GOAL_THETA}        1.57

*** Test Cases ***
Test Navigation2 Simple Movement
    [Documentation]    Launch Navigation2 simulation, wait 5 seconds, send vehicle to another place
    [Tags]    nav2    simple    movement
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Set environment variables for the test
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Launch the Navigation2 simulation
    Log    Starting Navigation2 simulation launch...
    ${process}=    Launch Package    turtlebot3    simulation.launch.py
    Should Not Be Equal    ${process}    ${None}
    Log    Launch process started with PID: ${process.pid}
    
    # Wait for the launch to initialize
    Sleep    ${WAIT_TIME}
    
    # Send vehicle to another place
    Log    Sending vehicle to position (${GOAL_X}, ${GOAL_Y}, ${GOAL_THETA})...
    ${nav_success}=    Navigate To Pose Simple    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}    timeout=5.0
    Log    Navigation command sent: ${nav_success}

    ${final_pose}=   Get Transform    map    base_link
    Log    Final position: ${final_pose}

   

*** Keywords ***
Clean Up Navigation2 Simulation
    [Documentation]    Clean up Navigation2 simulation
    ${shutdown}=    Shutdown Process    ign gazebo
    Should Be True    ${shutdown}
    Log    Navigation2 simulation cleanup completed

    ${shutdown}=    Shutdown Process    ros_gz_bridge
    Should Be True    ${shutdown}