*** Settings ***
Documentation    Basic Navigation2 test using the Nav2ClientLibrary
Library          ros2_client.ROS2ClientLibrary
Library          nav2_client.Nav2ClientLibrary
Library          Collections
Library          OperatingSystem
Library          Process

*** Variables ***
${NAV2_TIMEOUT}    60.0
${POSE_TIMEOUT}    30.0
${WAIT_TIME}       5s
${TOLERANCE}       0.5

*** Test Cases ***
Test Navigation2 Library Import
    [Documentation]    Verify that the Navigation2 library can be imported and initialized
    [Tags]    smoke
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    Log    Navigation2 library imported successfully

Test Basic Navigation Operations
    [Documentation]    Test basic navigation operations with simulation
    [Tags]    navigation
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Test utility functions
    ${distance}=    Calculate Distance    0.0    0.0    3.0    4.0
    Should Be Equal As Numbers    ${distance}    5.0    0.1
    
    ${angle}=    Calculate Angle    0.0    0.0    1.0    1.0
    Should Be Equal As Numbers    ${angle}    0.785398    0.001
    
    ${degrees}=    Radians To Degrees    1.570796
    Should Be Equal As Numbers    ${degrees}    90.0    precision=0
    
    ${radians}=    Degrees To Radians    90.0
    Should Be Equal As Numbers    ${radians}    1.570796    0.1

Test Pose Management
    [Documentation]    Test pose management and transformations
    [Tags]    pose
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Test setting initial pose
    ${success}=    Set Initial Pose    0.0    0.0    0.0
    Should Be True    ${success}
    
    # Test getting current pose (may return None if no robot)
    ${current_pose}=    Get Current Pose    timeout=${POSE_TIMEOUT}
    Log    Current pose: ${current_pose}
    
    # Test waiting for localization
    ${localized}=    Wait For Localization    timeout=10.0
    Log    Localization status: ${localized}

Test Navigation Status
    [Documentation]    Test navigation status operations
    [Tags]    status
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Check if navigation is active
    ${active}=    Is Navigation Active
    Should Be Equal    ${active}    ${False}
    
    # Get navigation status
    ${status}=    Get Navigation Status
    Should Not Be Empty    ${status}
    Should Contain    ${status}    navigation_active
    Should Contain    ${status}    current_pose
    Should Contain    ${status}    goal_pose

Test Costmap Operations
    [Documentation]    Test costmap information retrieval
    [Tags]    costmap
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Get global costmap info
    ${global_info}=    Get Costmap Info    global
    Log    Global costmap info: ${global_info}
    
    # Get local costmap info
    ${local_info}=    Get Costmap Info    local
    Log    Local costmap info: ${local_info}
    
    # Test clearing costmaps (may fail if no robot running)
    ${cleared_global}=    Clear Costmap    global
    Log    Global costmap cleared: ${cleared_global}
    
    ${cleared_local}=    Clear Costmap    local
    Log    Local costmap cleared: ${cleared_local}

Test Path Planning
    [Documentation]    Test path planning operations
    [Tags]    path_planning
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Test path computation (may fail if no robot/map)
    ${path}=    Compute Path    0.0    0.0    0.0    2.0    1.0    1.57
    Log    Computed path: ${path}
    
    # If path was computed, test path length calculation
    IF    ${path} is not None
        ${path_length}=    Get Path Length    ${path}
        Should Be True    ${path_length} > 0.0
        Log    Path length: ${path_length} meters
    END

Test Navigation Through Poses
    [Documentation]    Test navigation through multiple poses
    [Tags]    navigation    poses
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Create a list of poses
    @{poses}=    Create List
    
    # Add poses to the list
    ${pose1}=    Create Dictionary    x=-2.0    y=0.0    theta=0.0
    # ${pose2}=    Create Dictionary    x=0.0    y=1.0    theta=1.57
    ${pose3}=    Create Dictionary    x=-2.0    y=1.0    theta=3.14
    
    Append To List    ${poses}    ${pose1}
    # Append To List    ${poses}    ${pose2}
    Append To List    ${poses}    ${pose3}
    
    Log    Created ${poses} poses for navigation
    
    # Test navigation through poses (may fail if no robot)
    ${result}=    Navigate Through Poses    ${poses}    timeout=${NAV2_TIMEOUT}
    Log    Navigation result: ${result}
    
    # Check if navigation was successful
    IF    $result is not None
        Should Be True    $result.success
        Log    Navigation completed successfully
    ELSE
        Log    Navigation result is None
    END

Test Single Pose Navigation
    [Documentation]    Test navigation to a single pose
    [Tags]    navigation    single_pose
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Test navigation to a single pose (may fail if no robot)
    ${result}=    Navigate To Pose Simple    1.0    1.0    0.785    timeout=${NAV2_TIMEOUT}
    Log    Navigation result: ${result}
    
    # Check if navigation was successful
    IF    ${result} is not None
        Should Be True    ${result.success}
        Log    Navigation completed successfully
    ELSE
        Log    Navigation result is None
    END

Test Navigation Cancellation
    [Documentation]    Test navigation cancellation
    [Tags]    navigation    cancellation
    [Setup]    Setup Navigation2 Simulation
    [Teardown]    Clean Up Navigation2 Simulation
    
    # Test cancelling navigation
    ${cancelled}=    Cancel Navigation
    Log    Navigation cancelled: ${cancelled}
    
    # Verify navigation is not active after cancellation
    ${active}=    Is Navigation Active
    Should Be Equal    ${active}    ${False}

*** Keywords ***
Setup Navigation2 Simulation
    [Documentation]    Setup Navigation2 simulation
    ${running}=    Has Running Nodes
    Should Be Equal    ${running}    ${False}
    # Set environment variables for the test
    Set Environment Variable    TURTLEBOT3_MODEL      waffle
    
    # Clean up any existing simulation
    Clean Up Navigation2 Simulation

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

    ${running}=    Has Running Nodes
    Should Be Equal    ${running}    ${False}
