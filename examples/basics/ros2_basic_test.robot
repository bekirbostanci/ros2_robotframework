*** Settings ***
Documentation    Basic ROS2 CLI Library Test
Library          ros2_client.ROS2ClientLibrary
Library          Process
Library          Collections

*** Variables ***
${TEST_TIMEOUT}    10.0
${TEST_MESSAGE}    Hello from new structure!

*** Test Cases ***
Test ROS2 CLI Library Basic Functionality
    [Documentation]    Test basic ROS2 CLI operations
    [Tags]    basic    ros2
    
    # Test listing topics
    ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
    Should Not Be Empty    ${topics}
    Log    Found topics: ${topics}
    
    # Test listing services
    ${services}=    List Services    timeout=${TEST_TIMEOUT}
    Log    Found services: ${services}
    
    # Test listing nodes
    ${nodes}=    List Nodes    timeout=${TEST_TIMEOUT}
    Log    Found nodes: ${nodes}
    
    # Test topic existence check
    ${has_topics}=    Run Keyword And Return Status    Should Not Be Empty    ${topics}
    IF    ${has_topics}
        ${first_topic}=    Get From List    ${topics}    0
        ${exists}=    Topic Exists    ${first_topic}    timeout=${TEST_TIMEOUT}
        Should Be True    ${exists}
        Log    Topic ${first_topic} exists: ${exists}
    END

Test ROS2 CLI Library With Demo Nodes
    [Documentation]    Test ROS2 CLI operations with demo nodes
    [Tags]    demo    ros2
    [Timeout]    30s
    
    # Start demo talker node
    ${talker_process}=    Start Process    ros2    run    demo_nodes_cpp    talker
    Set Test Variable    ${talker_process}
    
    # Wait for the node to start
    Sleep    3s
    
    # Test node operations
    ${nodes}=    List Nodes    timeout=${TEST_TIMEOUT}
    Should Contain    ${nodes}    /talker
    Log    Found nodes: ${nodes}
    
    # Test topic operations
    ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
    Should Contain    ${topics}    /chatter
    Log    Found topics: ${topics}
    
    # Test topic info
    ${topic_info}=    Get Topic Info    /chatter    timeout=${TEST_TIMEOUT}
    Should Be Equal    ${topic_info}[type]    std_msgs/msg/String
    Log    Topic info: ${topic_info}
    
    # Test node info
    ${node_info}=    Get Node Info    /talker    timeout=${TEST_TIMEOUT}
    Log    Node info: ${node_info}
    
    # Check that publishers are properly parsed
    Should Not Be Empty    ${node_info}[publishers]
    ${publisher_names}=    Create List
    FOR    ${publisher}    IN    @{node_info}[publishers]
        ${name}=    Set Variable    ${publisher}[name]
        Append To List    ${publisher_names}    ${name}
    END
    Should Contain    ${publisher_names}    /chatter
    Log    Publisher names: ${publisher_names}
    
    # Check that subscribers are properly parsed
    Should Not Be Empty    ${node_info}[subscribers]
    ${subscriber_names}=    Create List
    FOR    ${subscriber}    IN    @{node_info}[subscribers]
        ${name}=    Set Variable    ${subscriber}[name]
        Append To List    ${subscriber_names}    ${name}
    END
    Should Contain    ${subscriber_names}    /parameter_events
    Log    Subscriber names: ${subscriber_names}
    
    # Check that service servers are properly parsed
    Should Not Be Empty    ${node_info}[service_servers]
    ${service_names}=    Create List
    FOR    ${service}    IN    @{node_info}[service_servers]
        ${name}=    Set Variable    ${service}[name]
        Append To List    ${service_names}    ${name}
    END
    Should Contain    ${service_names}    /talker/describe_parameters
    Log    Service names: ${service_names}
    
    # Test echo topic (capture one message)
    ${messages}=    Echo Topic    /chatter    count=1    timeout=${TEST_TIMEOUT}
    Should Not Be Empty    ${messages}
    Log    Echoed message: ${messages}[0]
    
    # Clean up
    Terminate Process    ${talker_process}

Test ROS2 CLI Library Launch Operations
    [Documentation]    Test launch operations with demo nodes
    [Tags]    launch    ros2
    [Timeout]    30s
    
    # Launch a simple demo launch file (if available)
    ${launch_success}=    Run Keyword And Return Status    Launch Package    demo_nodes_cpp    talker_listener.launch.py
    IF    ${launch_success}
        Log    Launch started successfully
        ${process}=    Launch Package    demo_nodes_cpp    talker_listener.launch.py
        
        # Wait for system to initialize
        Sleep    3s
        
        
        # Verify topics are available
        ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
        Should Contain    ${topics}    /chatter
        
        # Wait 5 seconds
        Sleep    5s
        
        # Stop the launch
        ${terminated}=    Terminate Launch Process    ${process}
        Should Be True    ${terminated}
        Log    Launch terminated successfully
    ELSE
        Log    Demo launch file not available, skipping launch test
    END

Test ROS2 CLI Library Run Operations
    [Documentation]    Test run operations with demo nodes
    [Tags]    run    ros2
    [Timeout]    20s
    
    # Run a demo node directly
    ${process}=    Run Node    demo_nodes_cpp    talker
    Should Not Be Equal    ${process}    ${None}
    Log    Node started with PID: ${process.pid}
    
    # Wait for node to start
    Sleep    2s
    
    
    # Verify the node is in the node list
    ${nodes}=    List Nodes    timeout=${TEST_TIMEOUT}
    Should Contain    ${nodes}    /talker
    
    # Wait 5 seconds
    Sleep    5s
    
    # Stop the node
    ${terminated}=    Terminate Node Process    ${process}
    Should Be True    ${terminated}
    Log    Node terminated successfully

Test ROS2 CLI Library Error Handling
    [Documentation]    Test error handling for non-existent resources
    [Tags]    error    ros2
    
    # Test non-existent topic
    ${exists}=    Topic Exists    /non_existent_topic    timeout=2.0
    Should Not Be True    ${exists}
    
    # Test non-existent service
    ${exists}=    Service Exists    /non_existent_service    timeout=2.0
    Should Not Be True    ${exists}
    
    # Test non-existent node
    ${exists}=    Node Exists    /non_existent_node    timeout=2.0
    Should Not Be True    ${exists}

Test ROS2 Native Operations
    [Documentation]    Test native ROS2 operations with subscribers and publishers
    [Tags]    native    ros2
    [Timeout]    30s
    
    # Get client information
    ${info}=    Get Client Info
    Log    Using client with native support: ${info}[native_available]
    
    # Start a demo talker node
    ${talker_process}=    Start Process    ros2    run    demo_nodes_cpp    talker
    Set Test Variable    ${talker_process}
    
    # Wait for the node to start
    Sleep    3s
    
    # Create a native subscriber for the chatter topic
    ${subscriber}=    Create Subscriber    /chatter    std_msgs/msg/String
    Log    Created native subscriber: ${subscriber}
    
    # Wait for messages
    Sleep    5s
    
    # Get the latest message using native operations
    ${message}=    Get Latest Message    /chatter
    IF    ${message} is not None
        Log    Received message: ${message}[data]
        Should Contain    ${message}[data]    Hello World
    ELSE
        Log    No message received yet
    END
    
    # Get all buffered messages
    ${all_messages}=    Get All Messages    /chatter
    Log    Total messages received: ${all_messages.__len__()}
    
    # Create a native publisher for testing
    ${publisher}=    Create Publisher    /test_response    std_msgs/msg/String
    Log    Created native publisher: ${publisher}
    
    # Publish a test message
    ${success}=    Publish Message    ${publisher}    ${TEST_MESSAGE}
    Should Be True    ${success}
    Log    Published test message successfully
    
    # Clean up
    Terminate Process    ${talker_process}
    Cleanup


Test ROS2 Mixed Operations
    [Documentation]    Test mixing CLI and native operations
    [Tags]    mixed    ros2
    [Timeout]    25s
    
    # Start a demo talker
    ${talker_process}=    Start Process    ros2    run    demo_nodes_cpp    talker
    Set Test Variable    ${talker_process}
    
    # Wait for the node to start
    Sleep    3s
    
    # Use CLI for discovery
    ${nodes}=    List Nodes
    Should Contain    ${nodes}    /talker
    Log    Found nodes: ${nodes}
    
    # Use native for real-time communication
    ${subscriber}=    Create Subscriber    /chatter    std_msgs/msg/String
    Sleep    3s
    
    # Get messages using native operations
    ${message}=    Get Latest Message    /chatter
    IF    ${message} is not None
        Log    Latest message: ${message}[data]
    END
    
    # Use CLI for one-time operations
    ${topic_info}=    Get Topic Info    /chatter
    Should Be Equal    ${topic_info}[type]    std_msgs/msg/String
    Log    Topic info: ${topic_info}
    
    # Clean up
    Terminate Process    ${talker_process}
    Cleanup

*** Keywords ***
Cleanup Test Processes
    [Documentation]    Clean up any running test processes
    [Tags]    cleanup
    
    # Kill any remaining demo processes
    Run Process    pkill    -f    demo_nodes_cpp    shell=True
    Sleep    1s
