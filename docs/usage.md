# ROS2 Robot Framework Library Usage Guide

## Installation

```bash
# Install the library
pip install -e .

# Or with additional features
pip install -e .[nav2,behaviour-tree]
```

## Basic Usage

### Import the Library

```robot
*** Settings ***
Library    ros2_client.ROS2ClientLibrary
```

### Topic Operations

```robot
*** Test Cases ***
Test Topics
    # List all topics
    ${topics}=    List Topics
    Log    Available topics: ${topics}
    
    # Check if a topic exists
    ${exists}=    Topic Exists    /chatter
    Should Be True    ${exists}
    
    # Get topic information
    ${info}=    Get Topic Info    /chatter
    Should Be Equal    ${info}[type]    std_msgs/msg/String
    
    # Echo messages from a topic
    ${messages}=    Echo Topic    /chatter    count=3
    Should Not Be Empty    ${messages}
    
    # Publish to a topic
    ${success}=    Publish Topic    /chatter    std_msgs/msg/String    "Hello World"
    Should Be True    ${success}
    
    # Wait for a topic to become available
    ${available}=    Wait For Topic    /chatter    timeout=10.0
    Should Be True    ${available}
```

### Service Operations

```robot
*** Test Cases ***
Test Services
    # List all services
    ${services}=    List Services
    Log    Available services: ${services}
    
    # Check if a service exists
    ${exists}=    Service Available    /add_two_ints
    Should Be True    ${exists}
    
    # Get service information
    ${info}=    Get Service Info    /add_two_ints
    Should Be Equal    ${info}[type]    example_interfaces/srv/AddTwoInts
    
    # Call a service
    ${response}=    Call Service    /add_two_ints    example_interfaces/srv/AddTwoInts    "a: 5, b: 3"
    Should Be Equal    ${response}[sum]    8
    
    # Wait for a service to become available
    ${available}=    Wait For Service    /add_two_ints    timeout=10.0
    Should Be True    ${available}
```

### Node Operations

```robot
*** Test Cases ***
Test Nodes
    # List all nodes
    ${nodes}=    List Nodes
    Log    Available nodes: ${nodes}
    
    # Check if a node exists
    ${exists}=    Node Exists    /talker
    Should Be True    ${exists}
    
    # Get node information
    ${info}=    Get Node Info    /talker
    Log    Node info: ${info}
    
    # Access publishers (returns list of dictionaries with 'name' and 'type')
    ${publisher_names}=    Create List
    FOR    ${publisher}    IN    @{info}[publishers]
        ${name}=    Set Variable    ${publisher}[name]
        Append To List    ${publisher_names}    ${name}
    END
    Should Contain    ${publisher_names}    /chatter
    
    # Access subscribers (returns list of dictionaries with 'name' and 'type')
    ${subscriber_names}=    Create List
    FOR    ${subscriber}    IN    @{info}[subscribers]
        ${name}=    Set Variable    ${subscriber}[name]
        Append To List    ${subscriber_names}    ${name}
    END
    Log    Subscriber names: ${subscriber_names}
    
    # Access service servers (returns list of dictionaries with 'name' and 'type')
    ${service_names}=    Create List
    FOR    ${service}    IN    @{info}[service_servers]
        ${name}=    Set Variable    ${service}[name]
        Append To List    ${service_names}    ${name}
    END
    Log    Service names: ${service_names}
    
    # Wait for a node to become available
    ${available}=    Wait For Node    /talker    timeout=10.0
    Should Be True    ${available}
```

### Parameter Operations

```robot
*** Test Cases ***
Test Parameters
    # List parameters for a node
    ${params}=    List Parameters    /my_node
    Log    Available parameters: ${params}
    
    # Check if a parameter exists
    ${exists}=    Parameter Exists    /my_node    my_param
    Should Be True    ${exists}
    
    # Get a parameter value
    ${value}=    Get Parameter    /my_node    my_param
    Should Be Equal    ${value}    42
    
    # Set a parameter value
    ${success}=    Set Parameter    /my_node    my_param    100
    Should Be True    ${success}
    
    # Get all parameters
    ${all_params}=    Get All Parameters    /my_node
    Should Contain    ${all_params}    my_param
```

### Launch Operations

```robot
*** Test Cases ***
Test Launch Operations
    # Launch a launch file from a package
    ${process}=    Launch Package    demo_nodes_cpp    talker_listener.launch.py
    Should Not Be Equal    ${process}    ${None}
    
    # Launch with arguments
    ${arguments}=    Create Dictionary    use_sim_time=True
    ${process}=    Launch Package    nav2_bringup    tb3_simulation_launch.py    arguments=${arguments}
    Should Not Be Equal    ${process}    ${None}
    
    # Find launch files in a package
    ${launch_files}=    Find Launch Files    demo_nodes_cpp
    Log    Available launch files: ${launch_files}
    
    # Wait for launch completion
    ${completed}=    Wait For Launch Completion    ${process}    timeout=30.0
    Should Be True    ${completed}
    
    # Terminate launch process
    ${terminated}=    Terminate Launch Process    ${process}
    Should Be True    ${terminated}
```

### Run Operations

```robot
*** Test Cases ***
Test Run Operations
    # Run a node directly
    ${process}=    Run Node    demo_nodes_cpp    talker
    Should Not Be Equal    ${process}    ${None}
    
    # Run node with arguments
    ${arguments}=    Create List    --ros-args    -p    use_sim_time:=True
    ${process}=    Run Node    nav2_controller    controller_server    arguments=${arguments}
    Should Not Be Equal    ${process}    ${None}
    
    # Run node with topic remapping
    ${remaps}=    Create Dictionary    /chatter=/my_chatter
    ${process}=    Run Node With Remap    demo_nodes_cpp    talker    remaps=${remaps}
    Should Not Be Equal    ${process}    ${None}
    
    # Find executables in a package
    ${executables}=    Find Executables    demo_nodes_cpp
    Should Contain    ${executables}    talker
    Should Contain    ${executables}    listener
    
    # Check if process is running
    ${running}=    Is Process Running    ${process}
    Should Be True    ${running}
    
    # Get process output
    ${output}=    Get Process Output    ${process}    timeout=2.0
    Log    Process output: ${output}
    
    # Wait for node completion
    ${completed}=    Wait For Node Completion    ${process}    timeout=10.0
    Should Be False    ${completed}    # Should timeout for long-running nodes
    
    # Terminate node process
    ${terminated}=    Terminate Node Process    ${process}
    Should Be True    ${terminated}
```

## Advanced Usage

### Custom Timeouts

```robot
*** Test Cases ***
Test With Custom Timeout
    # Use custom timeout for operations
    ${topics}=    List Topics    timeout=5.0
    ${info}=    Get Topic Info    /chatter    timeout=3.0
```

### Error Handling

```robot
*** Test Cases ***
Test Error Handling
    # Check for non-existent resources
    ${exists}=    Topic Exists    /non_existent_topic
    Should Not Be True    ${exists}
    
    # Handle service call failures
    ${response}=    Run Keyword And Expect Error    *    Call Service    /non_existent_service    std_srvs/srv/Empty    ""
```

## Complete Example

```robot
*** Settings ***
Documentation    Complete ROS2 system test using launch and run operations
Library          ros2_client.ROS2ClientLibrary

*** Test Cases ***
Test Complete ROS2 System With Launch
    # Launch a complete system using launch file
    ${launch_process}=    Launch Package    demo_nodes_cpp    talker_listener.launch.py
    
    # Wait for system to be ready
    Sleep    3s
    
    # Verify nodes are running
    ${nodes}=    List Nodes
    Should Contain    ${nodes}    /talker
    Should Contain    ${nodes}    /listener
    
    # Verify topics exist
    ${topics}=    List Topics
    Should Contain    ${topics}    /chatter
    
    # Verify topic communication
    ${messages}=    Echo Topic    /chatter    count=3
    Should Not Be Empty    ${messages}
    
    # Clean up
    ${terminated}=    Terminate Launch Process    ${launch_process}
    Should Be True    ${terminated}

Test Complete ROS2 System With Run
    # Start nodes individually
    ${talker_process}=    Run Node    demo_nodes_cpp    talker
    ${listener_process}=    Run Node    demo_nodes_cpp    listener
    
    # Wait for system to be ready
    Sleep    3s
    
    # Verify nodes are running
    ${nodes}=    List Nodes
    Should Contain    ${nodes}    /talker
    Should Contain    ${nodes}    /listener
    
    # Verify topics exist
    ${topics}=    List Topics
    Should Contain    ${topics}    /chatter
    
    # Verify topic communication
    ${messages}=    Echo Topic    /chatter    count=3
    Should Not Be Empty    ${messages}
    
    # Clean up
    ${talker_terminated}=    Terminate Node Process    ${talker_process}
    ${listener_terminated}=    Terminate Node Process    ${listener_process}
    Should Be True    ${talker_terminated}
    Should Be True    ${listener_terminated}
```

## Best Practices

1. **Always use timeouts** - Set appropriate timeouts for operations
2. **Clean up processes** - Always terminate started processes using the terminate keywords
3. **Check existence first** - Use existence checks before operations
4. **Handle errors gracefully** - Use Robot Framework's error handling keywords
5. **Use wait keywords** - Use wait keywords for dynamic systems
6. **Log important information** - Use Log keyword for debugging
7. **Use launch files for complex systems** - Launch files are better for multi-node systems
8. **Use run operations for individual nodes** - Run operations are better for single node testing
9. **Monitor process status** - Use Is Process Running to check if processes are still active
10. **Get process output for debugging** - Use Get Process Output to debug issues

## Troubleshooting

### Common Issues

1. **ROS2 not found** - Ensure ROS2 is installed and sourced
2. **Permission errors** - Check file permissions and user access
3. **Timeout errors** - Increase timeout values for slow systems
4. **Process not found** - Ensure demo nodes are available

### Debug Tips

1. Enable debug logging in Robot Framework
2. Check ROS2 environment variables
3. Verify ROS2 installation with `ros2 --help`
4. Test individual commands manually
