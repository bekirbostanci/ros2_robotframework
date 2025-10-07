# ROS2 RobotFramework

A comprehensive Robot Framework library for testing and automating ROS2 applications.

Complete API reference, examples, and guides **[Documentation](https://ros2-robotframework.readthedocs.io/main/)**


![Test Animation](https://github.com/bekirbostanci/ros2_robotframework/raw/main/docs/images/test.gif)


## Key Features

- **Core ROS2 Operations**: Topics, services, nodes, parameters, and launch management
- **Native ROS2 Integration**: Direct Python node operations for high-performance testing
- **Navigation2 Support**: Complete navigation testing with pose management and costmap operations
- **Rich Reporting**: HTML reports with detailed logs and test metrics
- **CI/CD Ready**: Seamless integration with automated testing pipelines
- **Accessible**: Keyword-driven testing that non-programmers can understand

## Installation

```bash
# From source
git clone https://github.com/bekirbostanci/ros2_robotframework.git
cd ros2_robotframework
pip install -e .

# From PyPI (when published)
pip install ros2-robotframework
```

**Requirements**: ROS2 (Jazz/Humble), Python 3.8+, ROS2 message packages

## Quick Start

```robot
*** Settings ***
Library    ROS2ClientLibrary    use_native_node=True

*** Test Cases ***
Test ROS2 System
    # Check if a topic exists
    ${exists}=    Topic Exists    /chatter
    Should Be True    ${exists}
    
    # List all topics
    ${topics}=    List Topics
    Log    Available topics: ${topics}
```

**Run Examples:**
```bash
robot examples/basics/ros2_basic_test.robot
robot examples/medium/nav2_simple_monitoring_test.robot
```

## Test Results

![Test Report](https://github.com/bekirbostanci/ros2_robotframework/raw/main/docs/images/output_report.png)

> **Tested with**: Navigation2, TurtleBot3, and various ROS2 applications. Works with any ROS2 project.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

Apache License 2.0 - see [LICENSE](LICENSE) for details.
