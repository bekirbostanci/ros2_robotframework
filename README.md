# ROS2 RobotFramework

A comprehensive Robot Framework library for testing and automating ROS2 applications.

![Test Animation](https://github.com/bekirbostanci/ros2_robotframework/raw/main/docs/images/test.gif)

## Documentation

** [Full Documentation](https://ros2-robotframework.readthedocs.io/main/)** - Complete API reference, examples, and guides

## What is Robot Framework?

**Robot Framework** is an open-source automation framework that uses keyword-driven testing with simple, readable syntax. It's perfect for ROS2 because:

> 📚 **Learn more**: [Robot Framework Documentation](https://robotframework.org/)

- **🔧 Keyword-Driven**: Complex ROS2 operations become simple keywords (`Navigate To Pose`, `Wait For Topic`)
- **📊 Rich Reporting**: Built-in HTML reports with detailed logs for debugging
- **🔄 CI/CD Ready**: Seamlessly integrates with automated testing pipelines
- **👥 Accessible**: Non-programmers can write and understand test cases
- **🧪 System Testing**: Ideal for testing complex multi-node ROS2 systems

## Key Features

### Core ROS2 Operations
- **Topic Operations**: List topics, get topic info, echo messages, publish messages, wait for topics
- **Service Operations**: List services, call services, get service info, wait for services
- **Node Operations**: List nodes, get node info, wait for nodes
- **Parameter Operations**: List, get, set parameters, check parameter existence
- **Launch Operations**: Launch files and packages, find launch files, manage launch processes
- **Run Operations**: Run nodes directly, run with remapping, find executables, manage node processes

### Native ROS2 Python Node Operations
- **Native Topic Operations**: Direct publishing/subscribing using ROS2 Python nodes
- **Native Service Operations**: Direct service calls using ROS2 Python service clients
- **Native Parameter Operations**: Direct parameter access using ROS2 Python parameter clients
- **Native TF2 Operations**: Direct transform operations using ROS2 Python TF2
- **Message Storage**: Automatic message buffering and retrieval
- **Real-time Communication**: Low-latency, high-performance ROS2 communication

### Navigation2 Support
- **Navigation2 Client**: Complete Navigation2 client with native ROS2 operations
- **Navigate to Pose**: Point-to-point navigation with action client
- **Navigate Through Poses**: Multi-waypoint navigation
- **Costmap Operations**: Global and local costmap clearing
- **Pose Management**: Initial pose setting and AMCL pose monitoring
- **Navigation Status**: Real-time navigation status monitoring

## Installation

### From Source
```bash
git clone https://github.com/bekirbostanci/ros2_robotframework.git
cd ros2_robotframework
pip install -e .
```

### From PyPI (when published)
```bash
pip install ros2-robotframework
```

### Dependencies
This library requires ROS2 to be installed and sourced. Make sure you have:
- ROS2 (tested with Jazz and Humble)
- Python 3.8 or higher
- All ROS2 message packages (std_msgs, geometry_msgs, etc.)

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

> **Note**: While extensively tested with this specific repository, the library is designed to work with **any ROS2 project** and can be used with any ROS2-based robotic system, including custom robots, different navigation stacks, and various simulation environments.

## Test Results

Here's an example of the test output and monitoring capabilities:

![Test Report](https://github.com/bekirbostanci/ros2_robotframework/raw/main/docs/images/output_report.png)

## License

This project is licensed under the Apache License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

**📖 For complete documentation, examples, and API reference, visit: [https://ros2-robotframework.readthedocs.io/main/](https://ros2-robotframework.readthedocs.io/main/)**