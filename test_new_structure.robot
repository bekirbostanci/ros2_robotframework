*** Settings ***
Documentation    Test the new ROS2 client structure
Library          ros2_client.ROS2ClientLibrary    use_native=True
Library          Collections

*** Variables ***
${TEST_TIMEOUT}    10.0

*** Test Cases ***
Test New Structure Import
    [Documentation]    Test that the new structure can be imported and initialized
    [Tags]    structure    import
    
    # Test that the library is initialized
    ${info}=    Get Client Info
    Should Not Be Empty    ${info}
    Log    Client info: ${info}
    
    # Check that native mode is available
    Should Be True    ${info}[use_native]
    Should Be True    ${info}[native_available]

Test CLI Operations Still Work
    [Documentation]    Test that CLI operations still work with the new structure
    [Tags]    cli    compatibility
    
    # Test listing topics (should use CLI)
    ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
    Should Not Be Empty    ${topics}
    Log    Found topics: ${topics}
    
    # Test listing services (should use CLI)
    ${services}=    List Services    timeout=${TEST_TIMEOUT}
    Log    Found services: ${services}
    
    # Test listing nodes (should use CLI)
    ${nodes}=    List Nodes    timeout=${TEST_TIMEOUT}
    Log    Found nodes: ${nodes}

Test Native Publisher Creation
    [Documentation]    Test creating a native publisher
    [Tags]    native    publisher
    
    # Create a native publisher
    ${publisher}=    Create Publisher    /test_topic    std_msgs/msg/String
    Should Not Be Empty    ${publisher}
    Log    Created publisher: ${publisher}
    
    # Publish a message
    ${success}=    Publish Message    ${publisher}    Hello from new structure!
    Should Be True    ${success}

Test Native Subscriber Creation
    [Documentation]    Test creating a native subscriber
    [Tags]    native    subscriber
    
    # Create a native subscriber
    ${subscriber}=    Create Subscriber    /test_topic    std_msgs/msg/String
    Should Not Be Empty    ${subscriber}
    Log    Created subscriber: ${subscriber}
    
    # Wait a bit for messages
    Sleep    2s
    
    # Try to get latest message
    ${message}=    Get Latest Message    /test_topic
    Log    Latest message: ${message}

Test Mode Switching
    [Documentation]    Test switching between CLI and native modes
    [Tags]    mode    switching
    
    # Switch to CLI mode
    Switch To CLI Mode
    ${info}=    Get Client Info
    Should Not Be True    ${info}[use_native]
    Log    Switched to CLI mode
    
    # Switch back to native mode
    Switch To Native Mode
    ${info}=    Get Client Info
    Should Be True    ${info}[use_native]
    Log    Switched back to native mode

Test Cleanup
    [Documentation]    Test cleanup functionality
    [Tags]    cleanup
    
    # Clean up resources
    Cleanup
    Log    Cleanup completed successfully
