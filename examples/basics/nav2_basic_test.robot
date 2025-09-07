*** Settings ***
Documentation    Basic Navigation2 test using the Nav2ClientLibrary
Library          nav2_client.Nav2ClientLibrary
Library          Collections

*** Variables ***
${NAV2_TIMEOUT}    60.0
${POSE_TIMEOUT}    30.0

*** Test Cases ***
Test Navigation2 Library Import
    [Documentation]    Verify that the Navigation2 library can be imported and initialized
    [Tags]    smoke
    Log    Navigation2 library imported successfully

Test Basic Navigation Operations
    [Documentation]    Test basic navigation operations (without actual robot)
    [Tags]    navigation
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
    # Test utility functions
    ${distance}=    Calculate Distance    0.0    0.0    3.0    4.0
    Should Be Equal As Numbers    ${distance}    5.0    0.1
    
    ${angle}=    Calculate Angle    0.0    0.0    1.0    1.0
    Should Be Equal As Numbers    ${angle}    0.785    0.1
    
    ${degrees}=    Radians To Degrees    1.571
    Should Be Equal As Numbers    ${degrees}    90.0    0.1
    
    ${radians}=    Degrees To Radians    90.0
    Should Be Equal As Numbers    ${radians}    1.571    0.1

Test Pose Management
    [Documentation]    Test pose management and transformations
    [Tags]    pose
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
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
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
    # Check if navigation is active
    ${active}=    Is Navigation Active
    Should Be False    ${active}
    
    # Get navigation status
    ${status}=    Get Navigation Status
    Should Not Be Empty    ${status}
    Should Contain    ${status}    navigation_active
    Should Contain    ${status}    current_pose
    Should Contain    ${status}    goal_pose

Test Costmap Operations
    [Documentation]    Test costmap information retrieval
    [Tags]    costmap
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
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
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
    # Test path computation (may fail if no robot/map)
    ${path}=    Compute Path    0.0    0.0    0.0    2.0    1.0    1.57
    Log    Computed path: ${path}
    
    # If path was computed, test path length calculation
    IF    ${path} is not None
        ${path_length}=    Get Path Length    ${path}
        Should Be Greater Than    ${path_length}    0.0
        Log    Path length: ${path_length} meters
    END

Test Navigation Through Poses
    [Documentation]    Test navigation through multiple poses
    [Tags]    navigation    poses
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
    # Create a list of poses
    @{poses}=    Create List
    ...    ${EMPTY}    # This will be filled with pose dictionaries
    
    # Add poses to the list
    ${pose1}=    Create Dictionary    x=1.0    y=0.0    theta=0.0
    ${pose2}=    Create Dictionary    x=2.0    y=1.0    theta=1.57
    ${pose3}=    Create Dictionary    x=1.0    y=2.0    theta=3.14
    
    Append To List    ${poses}    ${pose1}
    Append To List    ${poses}    ${pose2}
    Append To List    ${poses}    ${pose3}
    
    Log    Created ${poses} poses for navigation
    
    # Test navigation through poses (may fail if no robot)
    ${result}=    Navigate Through Poses    ${poses}    timeout=${NAV2_TIMEOUT}
    Log    Navigation result: ${result}
    
    # Check if navigation was successful
    IF    ${result} is not None
        Should Contain    ${result}    success
        Should Contain    ${result}    message
    END

Test Single Pose Navigation
    [Documentation]    Test navigation to a single pose
    [Tags]    navigation    single_pose
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
    # Test navigation to a single pose (may fail if no robot)
    ${result}=    Navigate To Pose    1.0    1.0    0.785    timeout=${NAV2_TIMEOUT}
    Log    Navigation result: ${result}
    
    # Check if navigation was successful
    IF    ${result} is not None
        Should Contain    ${result}    success
        Should Contain    ${result}    message
    END

Test Navigation Cancellation
    [Documentation]    Test navigation cancellation
    [Tags]    navigation    cancellation
    [Setup]    Wait For Nav2 Ready    timeout=${NAV2_TIMEOUT}
    
    # Test cancelling navigation
    ${cancelled}=    Cancel Navigation
    Log    Navigation cancelled: ${cancelled}
    
    # Verify navigation is not active after cancellation
    ${active}=    Is Navigation Active
    Should Be Equal    ${active}    False

*** Keywords ***
Wait For Nav2 Ready
    [Arguments]    ${timeout}=60.0
    [Documentation]    Wait for Navigation2 stack to be ready
    ${ready}=    Wait For Nav2 Ready    timeout=${timeout}
    IF    not ${ready}
        Log    Navigation2 not ready within ${timeout}s, continuing with tests
    END
