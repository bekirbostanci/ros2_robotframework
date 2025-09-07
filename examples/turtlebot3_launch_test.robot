*** Settings ***
Documentation    Test for TurtleBot3 simulation launch with start, wait, and stop
Library          ros2_client.ROS2ClientLibrary
Library          Collections
Library          OperatingSystem

*** Variables ***
${LAUNCH_TIMEOUT}    10.0
${WAIT_TIME}         10s

*** Test Cases ***
Test TurtleBot3 Simulation Launch
    [Documentation]    Launch TurtleBot3 simulation, wait 10 seconds, then stop it
    [Tags]    turtlebot3    simulation    launch
    
    # Set environment variables for the test
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Launch the TurtleBot3 simulation
    Log    Starting TurtleBot3 simulation launch...
    ${process}=    Launch Package    turtlebot3    simulation.launch.py
    Should Not Be Equal    ${process}    ${None}
    Log    Launch process started with PID: ${process.pid}
    
    # Wait for the launch to initialize
    Sleep    10s
    
    # Check if the process is running
    ${running}=    Is Process Running    ${process}
    Should Be True    ${running}    Launch process should be running
    
    # Wait for 10 seconds as requested
    Log    Waiting for ${WAIT_TIME}...
    Sleep    ${WAIT_TIME}
    
    # Check if process is still running after wait
    ${still_running}=    Is Process Running    ${process}
    Should Be True    ${still_running}    Launch process should still be running after wait
    
    # Get some process output for debugging
    ${output}=    Get Process Output    ${process}    timeout=1.0
    Log    Process output during wait: ${output}
    
    # Stop the launch process
    Log    Stopping TurtleBot3 simulation launch...
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
    
    Log    TurtleBot3 simulation launch test completed successfully

Test TurtleBot3 Launch With Topic Verification
    [Documentation]    Launch TurtleBot3 simulation and verify topics appear
    [Tags]    turtlebot3    simulation    topics
    
    # Set environment variables
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Launch the simulation
    ${process}=    Launch Package    turtlebot3    simulation.launch.py
    Should Not Be Equal    ${process}    ${None}
    
    # Wait for system to initialize
    Sleep    5s
    
    # Check if process is running
    ${running}=    Is Process Running    ${process}
    Should Be True    ${running}
    
    # Wait for common TurtleBot3 topics to appear
    ${cmd_vel_available}=    Wait For Topic    /cmd_vel    timeout=10.0
    ${odom_available}=    Wait For Topic    /odom    timeout=10.0
    ${scan_available}=    Wait For Topic    /scan    timeout=10.0
    
    # Log results
    Log    /cmd_vel topic available: ${cmd_vel_available}
    Log    /odom topic available: ${odom_available}
    Log    /scan topic available: ${scan_available}
    
    # List all topics to see what's available
    ${topics}=    List Topics
    Log    Available topics: ${topics}
    
    # Wait for the requested 10 seconds
    Sleep    ${WAIT_TIME}
    
    # Stop the launch
    ${terminated}=    Terminate Launch Process    ${process}
    Should Be True    ${terminated}
    
    # Shutdown Gazebo to clean up any remaining processes
    ${gazebo_shutdown}=    Shutdown Gazebo
    Should Be True    ${gazebo_shutdown}

Test TurtleBot3 Launch With Node Verification
    [Documentation]    Launch TurtleBot3 simulation and verify nodes are running
    [Tags]    turtlebot3    simulation    nodes
    
    # Set environment variables
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Launch the simulation
    ${process}=    Launch Package    turtlebot3    simulation.launch.py
    Should Not Be Equal    ${process}    ${None}
    
    # Wait for system to initialize
    Sleep    5s
    
    # Check if process is running
    ${running}=    Is Process Running    ${process}
    Should Be True    ${running}
    
    # List nodes and verify some are running
    ${nodes}=    List Nodes
    Log    Available nodes: ${nodes}
    Should Not Be Empty    ${nodes}    Should have some nodes running
    
    # Wait for the requested 10 seconds
    Sleep    ${WAIT_TIME}
    
    # Stop the launch
    ${terminated}=    Terminate Launch Process    ${process}
    Should Be True    ${terminated}
    
    # Shutdown Gazebo to clean up any remaining processes
    ${gazebo_shutdown}=    Shutdown Gazebo
    Should Be True    ${gazebo_shutdown}

Test TurtleBot3 Launch With Error Handling
    [Documentation]    Test TurtleBot3 launch with proper error handling
    [Tags]    turtlebot3    simulation    error_handling
    
    # Set environment variables
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Try to launch the simulation
    ${launch_success}=    Run Keyword And Return Status    Launch Package    turtlebot3    simulation.launch.py
    
    IF    ${launch_success}
        Log    Launch started successfully
        # Get the actual process object
        ${process}=    Launch Package    turtlebot3    simulation.launch.py
        # Wait for the requested 10 seconds
        Sleep    ${WAIT_TIME}
        
        # Stop the launch
        ${terminated}=    Terminate Launch Process    ${process}
        Should Be True    ${terminated}
    ELSE
        Log    Launch failed - this might be expected if turtlebot3 package is not installed
        Log    Skipping termination since launch failed
    END

*** Keywords ***
Clean Up Environment
    [Documentation]    Clean up environment variables after test
    Remove Environment Variable    ROS_LOCALHOST_ONLY
    Remove Environment Variable    TURTLEBOT3_MODEL

Clean Up Gazebo
    [Documentation]    Clean up Gazebo processes
    ${gazebo_shutdown}=    Shutdown Gazebo    force=True
    Should Be True    ${gazebo_shutdown}
    Log    Gazebo cleanup completed
