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
    Log    Shutdown Navigation2 Simulation
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

*** Keywords ***
Setup Navigation2 Simulation
    [Documentation]    Setup Navigation2 simulation
    # Clean up any existing simulation
    Clean Up Navigation2 Simulation

    # Set environment variables for the test
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    

    # Launch the TurtleBot3 Gazebo simulation
    Log    Starting TurtleBot3 Gazebo simulation launch...
    ${sim_process}=    Launch Package    turtlebot3_gazebo    turtlebot3_world.launch.py
    # Should Not Be Equal    ${sim_process}    ${None}
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

    # Should Not Be Equal    ${nav_process}    ${None}
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

