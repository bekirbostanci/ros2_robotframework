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

*** Test Cases ***
Test Shutdown Navigation2 Simulation
    [Documentation]    Test shutdown Navigation2 simulation
    [Tags]    nav2    shutdown
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    Sleep    1s    

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

*** Keywords ***
Setup Navigation2 Simulation
    [Documentation]    Setup Navigation2 simulation
    Check Empty Nodes
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

    Check Empty Nodes
