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
    ${nav_success}=    Navigate To Pose Simple    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}
    Log    Navigation command sent: ${nav_success}
    
    # Get final position
    FOR    ${i}    IN RANGE    ${WAIT_TIME_FOR_FINAL_POSE}
        ${final_pose}=    Get Current Pose
        IF    ${final_pose} is not None
            Log    Final position: x=${final_pose.x}, y=${final_pose.y}, theta=${final_pose.theta}
            Exit For Loop
        ELSE
            Log    Could not get final position, retrying in 1 second...
            Sleep    1.0
        END
    END
    
    # Stop the launch process
    Log    Stopping Navigation2 simulation launch...
    ${terminated}=    Terminate Launch Process    ${process}
    Should Be True    ${terminated}    Launch process should be terminated successfully
    
    # Verify the process is no longer running
    ${stopped}=    Is Process Running    ${process}
    Should Not Be True    ${stopped}    Launch process should be stopped
    
    # Shutdown Gazebo to clean up any remaining processes
    Log    Shutting down Gazebo...
    ${gazebo_shutdown}=    Shutdown Gazebo
    Should Be True    ${gazebo_shutdown}
    Log    Gazebo shutdown completed
    
    Log    Navigation2 simple movement test completed successfully
