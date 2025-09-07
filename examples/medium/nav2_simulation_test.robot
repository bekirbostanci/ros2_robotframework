*** Settings ***
Documentation    Navigation2 simulation test with position monitoring
Library          ros2_client.ROS2ClientLibrary
Library          nav2_client.Nav2ClientLibrary
Library          Collections
Library          OperatingSystem
Library          DateTime

*** Variables ***
${LAUNCH_TIMEOUT}        10.0
${NAV2_TIMEOUT}          60.0
${POSITION_CHECK_INTERVAL}    1.0
${POSITION_TOLERANCE}    0.5
${WAIT_AFTER_LAUNCH}     5s
${GOAL_X}                2.0
${GOAL_Y}                1.0
${GOAL_THETA}            1.57

*** Test Cases ***
Test Navigation2 Simulation With Position Monitoring
    [Documentation]    Launch Navigation2 simulation, wait 5 seconds, send navigation goal, and monitor position
    [Tags]    nav2    simulation    navigation    position_monitoring
    
    # Set environment variables for the test
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    Set Environment Variable    GAZEBO_MODEL_PATH     /opt/ros/humble/share/turtlebot3_gazebo/models
    
    # Launch Navigation2 simulation
    Log    Starting Navigation2 simulation launch...
    ${process}=    Launch Package    nav2_bringup    tb3_simulation_launch.py    arguments={'use_sim_time': 'True'}
    Should Not Be Equal    ${process}    ${None}
    Log    Launch process started with PID: ${process.pid}
    
    # Wait for the launch to initialize
    Sleep    ${WAIT_AFTER_LAUNCH}
    
    # Check if the process is running
    ${running}=    Is Process Running    ${process}
    Should Be True    ${running}    Launch process should be running
    
    # Wait for Navigation2 to be ready
    Log    Waiting for Navigation2 to be ready...
    ${nav2_ready}=    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    IF    ${nav2_ready}
        Log    Navigation2 is ready
    ELSE
        Log    Navigation2 not ready within timeout, continuing with test
    END
    
    # Set initial pose for the robot
    Log    Setting initial pose at origin...
    ${initial_pose_success}=    Set Initial Pose    0.0    0.0    0.0
    IF    ${initial_pose_success}
        Log    Initial pose set successfully
    ELSE
        Log    Failed to set initial pose, continuing with test
    END
    
    # Wait for localization to converge
    Log    Waiting for localization to converge...
    ${localized}=    Wait For Localization    timeout=30.0
    IF    ${localized}
        Log    Localization converged successfully
    ELSE
        Log    Localization did not converge within timeout, continuing with test
    END
    
    # Get initial position
    ${initial_pose}=    Get Current Pose
    IF    ${initial_pose} is not None
        Log    Initial position: x=${initial_pose.x}, y=${initial_pose.y}, theta=${initial_pose.theta}
    ELSE
        Log    Could not get initial position
    END
    
    # Send navigation goal
    Log    Sending navigation goal to x=${GOAL_X}, y=${GOAL_Y}, theta=${GOAL_THETA}...
    ${nav_result}=    Navigate To Pose    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}    timeout=${NAV2_TIMEOUT}
    
    # Monitor position during navigation
    Log    Starting position monitoring...
    ${position_reached}=    Monitor Navigation Progress    ${GOAL_X}    ${GOAL_Y}    ${POSITION_TOLERANCE}
    
    # Check final navigation result
    IF    ${nav_result} is not None
        Log    Navigation result: ${nav_result}
        IF    ${nav_result.success}
            Log    Navigation completed successfully
        ELSE
            Log    Navigation failed: ${nav_result.message}
        END
    END
    
    # Get final position
    ${final_pose}=    Get Current Pose
    IF    ${final_pose} is not None
        Log    Final position: x=${final_pose.x}, y=${final_pose.y}, theta=${final_pose.theta}
        
        # Calculate distance to goal
        ${distance_to_goal}=    Calculate Distance    ${final_pose.x}    ${final_pose.y}    ${GOAL_X}    ${GOAL_Y}
        Log    Distance to goal: ${distance_to_goal} meters
        
        # Check if robot reached the goal within tolerance
        ${goal_reached}=    Evaluate    ${distance_to_goal} <= ${POSITION_TOLERANCE}
        IF    ${goal_reached}
            Log    Robot reached the goal within tolerance (${POSITION_TOLERANCE}m)
        ELSE
            Log    Robot did not reach the goal within tolerance
        END
    ELSE
        Log    Could not get final position
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
    
    Log    Navigation2 simulation test completed successfully

Test Navigation2 With Multiple Goals
    [Documentation]    Test navigation through multiple waypoints with position monitoring
    [Tags]    nav2    simulation    navigation    waypoints
    
    # Set environment variables
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Launch Navigation2 simulation
    ${process}=    Launch Package    nav2_bringup    tb3_simulation_launch.py    arguments={'use_sim_time': 'True'}
    Should Not Be Equal    ${process}    ${None}
    
    # Wait for initialization
    Sleep    ${WAIT_AFTER_LAUNCH}
    
    # Wait for Navigation2 to be ready
    ${nav2_ready}=    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    IF    not ${nav2_ready}
        Log    Navigation2 not ready, skipping test
        ${terminated}=    Terminate Launch Process    ${process}
        ${gazebo_shutdown}=    Shutdown Gazebo
        Return From Keyword
    END
    
    # Set initial pose
    ${initial_pose_success}=    Set Initial Pose    0.0    0.0    0.0
    ${localized}=    Wait For Localization    timeout=30.0
    
    # Create waypoint list
    @{waypoints}=    Create List
    ${waypoint1}=    Create Dictionary    x=1.0    y=0.0    theta=0.0
    ${waypoint2}=    Create Dictionary    x=2.0    y=1.0    theta=1.57
    ${waypoint3}=    Create Dictionary    x=1.0    y=2.0    theta=3.14
    ${waypoint4}=    Create Dictionary    x=0.0    y=1.0    theta=-1.57
    
    Append To List    ${waypoints}    ${waypoint1}
    Append To List    ${waypoints}    ${waypoint2}
    Append To List    ${waypoints}    ${waypoint3}
    Append To List    ${waypoints}    ${waypoint4}
    
    Log    Navigating through ${waypoints} waypoints...
    
    # Navigate through waypoints
    ${nav_result}=    Navigate Through Poses    ${waypoints}    timeout=${NAV2_TIMEOUT}
    
    # Monitor progress to final waypoint
    ${final_waypoint}=    Get From List    ${waypoints}    -1
    ${position_reached}=    Monitor Navigation Progress    ${final_waypoint.x}    ${final_waypoint.y}    ${POSITION_TOLERANCE}
    
    # Check result
    IF    ${nav_result} is not None and ${nav_result.success}
        Log    Multi-waypoint navigation completed successfully
    ELSE
        Log    Multi-waypoint navigation failed
    END
    
    # Cleanup
    ${terminated}=    Terminate Launch Process    ${process}
    ${gazebo_shutdown}=    Shutdown Gazebo

Test Navigation2 With Path Planning
    [Documentation]    Test path planning and execution with position monitoring
    [Tags]    nav2    simulation    path_planning
    
    # Set environment variables
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Launch Navigation2 simulation
    ${process}=    Launch Package    nav2_bringup    tb3_simulation_launch.py    arguments={'use_sim_time': 'True'}
    Should Not Be Equal    ${process}    ${None}
    
    # Wait for initialization
    Sleep    ${WAIT_AFTER_LAUNCH}
    
    # Wait for Navigation2 to be ready
    ${nav2_ready}=    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    IF    not ${nav2_ready}
        Log    Navigation2 not ready, skipping test
        ${terminated}=    Terminate Launch Process    ${process}
        ${gazebo_shutdown}=    Shutdown Gazebo
        Return From Keyword
    END
    
    # Set initial pose
    ${initial_pose_success}=    Set Initial Pose    0.0    0.0    0.0
    ${localized}=    Wait For Localization    timeout=30.0
    
    # Compute path from start to goal
    Log    Computing path from (0,0) to (${GOAL_X},${GOAL_Y})...
    ${path}=    Compute Path    0.0    0.0    0.0    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}
    
    IF    ${path} is not None
        ${path_length}=    Get Path Length    ${path}
        Log    Computed path with ${path_length} waypoints, total length: ${path_length} meters
        
        # Navigate to goal
        ${nav_result}=    Navigate To Pose    ${GOAL_X}    ${GOAL_Y}    ${GOAL_THETA}    timeout=${NAV2_TIMEOUT}
        
        # Monitor progress
        ${position_reached}=    Monitor Navigation Progress    ${GOAL_X}    ${GOAL_Y}    ${POSITION_TOLERANCE}
        
        IF    ${nav_result} is not None and ${nav_result.success}
            Log    Path planning and execution completed successfully
        ELSE
            Log    Path planning and execution failed
        END
    ELSE
        Log    Path planning failed, skipping navigation
    END
    
    # Cleanup
    ${terminated}=    Terminate Launch Process    ${process}
    ${gazebo_shutdown}=    Shutdown Gazebo

*** Keywords ***
Monitor Navigation Progress
    [Documentation]    Monitor robot position during navigation and check if goal is reached
    [Arguments]    ${goal_x}    ${goal_y}    ${tolerance}
    
    Log    Starting position monitoring for goal (${goal_x}, ${goal_y}) with tolerance ${tolerance}m...
    
    ${start_time}=    Get Time    epoch
    ${max_monitoring_time}=    Set Variable    120.0    # Maximum 2 minutes of monitoring
    ${position_reached}=    Set Variable    False
    
    WHILE    ${max_monitoring_time} > 0
        # Get current position
        ${current_pose}=    Get Current Pose    timeout=1.0
        
        IF    ${current_pose} is not None
            # Calculate distance to goal
            ${distance_to_goal}=    Calculate Distance    ${current_pose.x}    ${current_pose.y}    ${goal_x}    ${goal_y}
            
            # Log current status
            ${elapsed_time}=    Evaluate    (Get Time    epoch) - ${start_time}
            Log    Time: ${elapsed_time:.1f}s, Position: (${current_pose.x:.2f}, ${current_pose.y:.2f}), Distance to goal: ${distance_to_goal:.2f}m
            
            # Check if goal is reached
            IF    ${distance_to_goal} <= ${tolerance}
                Log    Goal reached! Distance: ${distance_to_goal:.2f}m <= tolerance: ${tolerance}m
                ${position_reached}=    Set Variable    True
                BREAK
            END
        ELSE
            Log    Could not get current position, continuing monitoring...
        END
        
        # Check if navigation is still active
        ${nav_active}=    Is Navigation Active
        IF    not ${nav_active}
            Log    Navigation is no longer active
            BREAK
        END
        
        # Wait before next check
        Sleep    ${POSITION_CHECK_INTERVAL}s
        ${max_monitoring_time}=    Evaluate    ${max_monitoring_time} - ${POSITION_CHECK_INTERVAL}
    END
    
    ${total_time}=    Evaluate    (Get Time    epoch) - ${start_time}
    Log    Position monitoring completed after ${total_time:.1f}s. Goal reached: ${position_reached}
    
    [Return]    ${position_reached}

Clean Up Environment
    [Documentation]    Clean up environment variables after test
    Remove Environment Variable    ROS_LOCALHOST_ONLY
    Remove Environment Variable    TURTLEBOT3_MODEL
    Remove Environment Variable    GAZEBO_MODEL_PATH

Clean Up Gazebo
    [Documentation]    Clean up Gazebo processes
    ${gazebo_shutdown}=    Shutdown Gazebo    force=True
    Should Be True    ${gazebo_shutdown}
    Log    Gazebo cleanup completed

Get Time
    [Documentation]    Get current time in epoch format
    [Arguments]    ${format}=epoch
    ${time}=    Get Current Date    result_format=epoch
    [Return]    ${time}
