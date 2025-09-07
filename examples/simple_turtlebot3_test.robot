*** Settings ***
Documentation    Simple test: Start TurtleBot3 launch, wait 10 seconds, stop it
Library          ros2_client.ROS2ClientLibrary
Library          OperatingSystem

*** Test Cases ***
Start Launch Wait 10 Seconds Stop
    [Documentation]    Start TurtleBot3 simulation launch, wait 10 seconds, then stop it
    [Tags]    simple    turtlebot3
    
    # Set required environment variables
    Set Environment Variable    ROS_LOCALHOST_ONLY    1
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Start the launch
    Log    Starting TurtleBot3 simulation launch...
    ${process}=    Launch Package    turtlebot3    simulation.launch.py
    Should Not Be Equal    ${process}    ${None}
    Log    Launch started with PID: ${process.pid}
    
    # Wait 10 seconds
    Log    Waiting 10 seconds...
    Sleep    10s
    
    # Stop the launch
    Log    Stopping launch...
    ${terminated}=    Terminate Launch Process    ${process}
    Should Be True    ${terminated}
    Log    Launch stopped successfully
    
    # Shutdown Gazebo to clean up any remaining processes
    Log    Shutting down Gazebo...
    ${gazebo_shutdown}=    Shutdown Gazebo
    Should Be True    ${gazebo_shutdown}
    Log    Gazebo shutdown completed
