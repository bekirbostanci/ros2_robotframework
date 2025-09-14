# Robot Framework ROS2 Library

A comprehensive Robot Framework library for testing and automating ROS2 applications.

![Test Animation](docs/test.gif)

## Features

### Core ROS2 Operations
- **Topic Operations**: List topics, get topic info, echo messages, publish messages, wait for topics
- **Service Operations**: List services, call services, get service info, wait for services
- **Node Operations**: List nodes, get node info, wait for nodes
- **Parameter Operations**: List, get, set parameters, check parameter existence
- **Launch Operations**: Launch files and packages, find launch files, manage launch processes
- **Run Operations**: Run nodes directly, run with remapping, find executables, manage node processes

### Native ROS2 Python Node Operations (NEW!)
- **Native Topic Operations**: Direct publishing/subscribing using ROS2 Python nodes
- **Native Service Operations**: Direct service calls using ROS2 Python service clients
- **Native Parameter Operations**: Direct parameter access using ROS2 Python parameter clients
- **Native TF2 Operations**: Direct transform operations using ROS2 Python TF2
- **Message Storage**: Automatic message buffering and retrieval
- **Real-time Communication**: Low-latency, high-performance ROS2 communication

### Advanced Features
- **Process Management**: Start, monitor, and terminate ROS2 processes
- **Discovery**: Find launch files and executables in packages
- **Remapping**: Topic and service remapping for node execution
- **Timeout Support**: Configurable timeouts for all operations
- **Hybrid Mode**: Automatic fallback from native to CLI operations when needed

## Installation

```bash
pip install -e .
```

## Quick Start

### Basic Usage
```robot
*** Settings ***
Library    ROS2ClientLibrary    use_native_node=True

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

Test Native ROS2 Operations
    # Subscribe to a topic using native ROS2 node
    ${success}=    Native Subscribe Topic    /chatter    std_msgs/msg/String
    Should Be True    ${success}
    
    # Publish a message using native ROS2 node
    ${success}=    Native Publish String    /chatter    "Hello World!"
    Should Be True    ${success}
    
    # Wait for and get the message
    ${message}=    Native Wait For Message    /chatter    timeout=5.0
    Should Not Be None    ${message}
    Log    Received: ${message}[data]
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
robot examples/basics/ros2_basic_test.robot

# Run the Nav2 monitoring test
robot examples/medium/nav2_simple_monitoring_test.robot
```

## Tested with Navigation2

This library has been extensively tested with Navigation2 applications. For testing and validation purposes, we used the following repository:

**Test Repository**: [navigation2_ignition_gazebo_turtlebot3](https://github.com/Onicc/navigation2_ignition_gazebo_turtlebot3)

### Test Coverage
The library has been validated with:
- ✅ **Basic Navigation**: Point-to-point navigation tasks
- ✅ **Obstacle Avoidance**: Dynamic obstacle detection and avoidance
- ✅ **Path Planning**: Global and local path planning algorithms
- ✅ **Recovery Behaviors**: Navigation recovery and error handling
- ✅ **Multi-robot Scenarios**: Testing with multiple robot instances

> **Note**: While extensively tested with this specific repository, the library is designed to work with **any ROS2 project** and can be used with any ROS2-based robotic system, including custom robots, different navigation stacks, and various simulation environments.

## Test Results

Here's an example of the test output and monitoring capabilities:

![Test Report](docs/output_report.png)

## Documentation

See the `docs/` directory for comprehensive documentation and examples.

