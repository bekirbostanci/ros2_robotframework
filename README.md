# Robot Framework ROS2 Library

A comprehensive Robot Framework library for testing and automating ROS2 applications.

## Features

### Core ROS2 Operations
- **Topic Operations**: List topics, get topic info, echo messages, publish messages, wait for topics
- **Service Operations**: List services, call services, get service info, wait for services
- **Node Operations**: List nodes, get node info, wait for nodes
- **Parameter Operations**: List, get, set parameters, check parameter existence
- **Launch Operations**: Launch files and packages, find launch files, manage launch processes
- **Run Operations**: Run nodes directly, run with remapping, find executables, manage node processes

### Advanced Features
- **Process Management**: Start, monitor, and terminate ROS2 processes
- **Discovery**: Find launch files and executables in packages
- **Remapping**: Topic and service remapping for node execution
- **Timeout Support**: Configurable timeouts for all operations

## Installation

```bash
pip install -e .
```

For additional features:
```bash
pip install -e .[nav2,behaviour-tree]
```

## Quick Start

### Basic Usage
```robot
*** Settings ***
Library    ROS2ClientLibrary

*** Test Cases ***
Test Basic ROS2 Operations
    # List available topics
    ${topics}=    List Topics
    Log    Available topics: ${topics}
    
    # Check if a specific topic exists
    ${exists}=    Topic Exists    /chatter
    Should Be True    ${exists}
    
    # Get topic information
    ${info}=    Get Topic Info    /chatter
    Log    Topic info: ${info}
```

### Launch and Run Operations
```robot
*** Settings ***
Library    ROS2ClientLibrary

*** Test Cases ***
Test Launch File
    # Launch a ROS2 launch file
    ${process}=    Launch Package    demo_nodes_cpp    talker_listener.launch.py
    Should Not Be Equal    ${process}    ${None}
    
    # Wait for topics to appear
    ${available}=    Wait For Topic    /chatter    timeout=10.0
    Should Be True    ${available}
    
    # Echo some messages
    ${messages}=    Echo Topic    /chatter    count=3
    
    # Clean up
    ${terminated}=    Terminate Launch Process    ${process}
    Should Be True    ${terminated}

Test Run Node
    # Run a node directly
    ${process}=    Run Node    demo_nodes_cpp    talker
    Should Not Be Equal    ${process}    ${None}
    
    # Wait for the node to start
    Sleep    2s
    
    # Check if process is running
    ${running}=    Is Process Running    ${process}
    Should Be True    ${running}
    
    # Terminate the process
    ${terminated}=    Terminate Node Process    ${process}
    Should Be True    ${terminated}
```

### Running Examples
```bash
# Run the basic test
robot examples/simple_ros2_test.robot

# Run the launch and run test
robot examples/launch_and_run_test.robot
```

## Documentation

See the `docs/` directory for comprehensive documentation and examples.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

Apache License 2.0
