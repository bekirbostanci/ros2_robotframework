*** Settings ***
Documentation    Simple ROS2 CLI Library Example
Library          ros2_client.ROS2ClientLibrary
Library          Process

*** Variables ***
${TIMEOUT}    10.0

*** Test Cases ***
Launch System And Verify Topics
    [Documentation]    Launch a simple ROS2 system and verify topics exist
    [Tags]    example    ros2
    
    # Start demo talker
    ${talker}=    Start Process    ros2    run    demo_nodes_cpp    talker
    Set Test Variable    ${talker_process}    ${talker}
    
    # Wait for system to be ready
    Sleep    3s
    
    # Verify topics exist
    ${topics}=    List Topics    timeout=${TIMEOUT}
    Should Contain    ${topics}    /chatter
    Log    Available topics: ${topics}
    
    # Get topic information
    ${info}=    Get Topic Info    /chatter    timeout=${TIMEOUT}
    Should Be Equal    ${info}[type]    std_msgs/msg/String
    Log    Topic info: ${info}
    
    # Echo a message
    ${messages}=    Echo Topic    /chatter    count=1    timeout=${TIMEOUT}
    Should Not Be Empty    ${messages}
    Log    Received message: ${messages}[0]
    
    # Clean up
    Terminate Process    ${talker_process}

Test Service Call Example
    [Documentation]    Example of calling a ROS2 service
    [Tags]    example    service
    
    # Start demo service server
    ${server}=    Start Process    ros2    run    demo_nodes_cpp    add_two_ints_server
    Set Test Variable    ${server_process}    ${server}
    
    # Wait for service to be ready
    ${available}=    Wait For Service    /add_two_ints    timeout=10.0
    Should Be True    ${available}
    
    # Call the service
    ${response}=    Call Service    /add_two_ints    example_interfaces/srv/AddTwoInts    "a: 5, b: 3"    timeout=${TIMEOUT}
    Should Be Equal    ${response}[sum]    8
    Log    Service response: ${response}
    
    # Clean up
    Terminate Process    ${server_process}

Test Node Information
    [Documentation]    Example of getting node information
    [Tags]    example    node
    
    # Start demo listener
    ${listener}=    Start Process    ros2    run    demo_nodes_cpp    listener
    Set Test Variable    ${listener_process}    ${listener}
    
    # Wait for node to be ready
    Sleep    2s
    
    # Get node information
    ${nodes}=    List Nodes    timeout=${TIMEOUT}
    Should Contain    ${nodes}    /listener
    Log    Available nodes: ${nodes}
    
    ${node_info}=    Get Node Info    /listener    timeout=${TIMEOUT}
    Should Contain    ${node_info}[subscribers]    /chatter
    Log    Node info: ${node_info}
    
    # Clean up
    Terminate Process    ${listener_process}

*** Keywords ***
Cleanup All Processes
    [Documentation]    Clean up all demo processes
    [Tags]    cleanup
    
    # Kill any remaining demo processes
    Run Process    pkill    -f    demo_nodes_cpp    shell=True
    Sleep    1s
