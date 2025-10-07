ROS2 RobotFramework
===================

This page explains the fundamental concepts behind the ROS2 RobotFramework Library, helping you understand what this tool provides for comprehensive robotic system testing.

Definitions
------------

Before using this library, it's helpful to understand the underlying technologies:

**ROS2** is a middleware framework that provides communication tools, message passing, and distributed computing capabilities for building robotic systems. It enables modular robot development through nodes, topics, services, and actions that can run across multiple machines. **Robot Framework** is a keyword-driven testing framework that allows you to write tests in plain English using reusable keywords for automation and validation. It generates comprehensive HTML reports and integrates seamlessly with CI/CD pipelines for continuous testing. Together, these technologies enable powerful robotic system testing through simple, readable test cases.

- **ROS2**: `Robot Operating System 2 <https://docs.ros.org/en/rolling/>`_ - A collection of software libraries and tools for building robot applications
- **Robot Framework**: `Robot Framework Documentation <https://robotframework.org/>`_ - A generic open-source automation framework for acceptance testing

What is ROS2 RobotFramework Library?
---------------------------------------

**ROS2 RobotFramework Library** is a specialized Robot Framework library that provides keywords for testing ROS2 applications. It enables comprehensive testing of robotic systems through simple, readable keywords.

.. image:: https://github.com/bekirbostanci/ros2_robotframework/raw/main/docs/images/test.gif
   :alt: ROS2 RobotFramework Library in action
   :align: center

.. mermaid::
   :align: center

   graph TD
       A[Robot Framework Test] --> B[ROS2 RobotFramework Library]
       B --> C[ROS2 Topics]
       B --> D[ROS2 Services]
       B --> E[ROS2 Actions]
       B --> F[ROS2 Parameters]
       B --> G[ROS2 Nodes]
       C --> H[ROS2 System]
       D --> H
       E --> H
       F --> H
       G --> H

**What This Tool Provides:**

.. container:: row

   .. container:: col-md-6

      **🔍 System Testing**
         - Test entire ROS2 systems
         - Verify node communication
         - Validate message flow
         - Check system behavior

   .. container:: col-md-6

      **🧪 Integration Testing**
         - Test sensor integration
         - Verify actuator responses
         - Validate navigation systems
         - Check safety systems

**Key Features:**

- **Topic Operations**: Publish, subscribe, and monitor ROS2 topics
- **Service Operations**: Call services and verify responses
- **Action Operations**: Test long-running tasks with feedback
- **Parameter Management**: Get, set, and monitor parameters
- **Node Management**: Launch, monitor, and control nodes
- **Navigation2 Support**: Specialized keywords for navigation testing

**Why Use ROS2 RobotFramework Library?**

.. container:: row

   .. container:: col-md-6

      **🎯 For Testers**
         - Write tests without deep ROS2 knowledge
         - Focus on system behavior, not implementation
         - Create readable, maintainable test suites
         - Generate comprehensive test reports

   .. container:: col-md-6

      **🎯 For Developers**
         - Integrate testing into development workflow
         - Automate regression testing
         - Validate system integration
         - Ensure code quality and reliability

**Real-World Applications:**

- **Autonomous Vehicles**: Test navigation, perception, and control systems
- **Industrial Robots**: Validate pick-and-place operations and safety systems
- **Service Robots**: Test interaction and navigation capabilities
- **Research Platforms**: Validate algorithms and system integration
- **Educational Projects**: Teach robotics concepts through testing

Example Use Cases
-----------------

**Navigation System Testing:**

.. code-block:: robot

   *** Test Cases ***
   Test Navigation System
       # Launch navigation stack
       Launch Navigation Stack
       
       # Set initial pose
       Set Initial Pose    x=0.0    y=0.0    theta=0.0
       
       # Navigate to goal
       Navigate To Pose    x=5.0    y=3.0    theta=1.57
       
       # Verify navigation success
       ${status}=    Get Navigation Status
       Should Be Equal    ${status}    SUCCEEDED

**Sensor Integration Testing:**

.. code-block:: robot

   *** Test Cases ***
   Test Sensor Data Flow
       # Subscribe to sensor topic
       Subscribe To Topic    /scan    sensor_msgs/msg/LaserScan
       
       # Wait for sensor data
       ${scan}=    Wait For Message    /scan    timeout=10.0
       Should Not Be None    ${scan}
       
       # Verify data quality
       ${ranges}=    Get Variable Value    ${scan}[ranges]
       Should Be True    len(${ranges}) > 0

**System Health Monitoring:**

.. code-block:: robot

   *** Test Cases ***
   Test System Health
       # Check all required nodes are running
       ${nodes}=    List Nodes
       Should Contain    ${nodes}    /robot_state_publisher
       Should Contain    ${nodes}    /joint_state_publisher
       
       # Verify topic communication
       ${topics}=    List Topics
       Should Contain    ${topics}    /joint_states
       Should Contain    ${topics}    /tf

Test Results and Monitoring
---------------------------

Here's an example of the test output and monitoring capabilities:

.. image:: https://github.com/bekirbostanci/ros2_robotframework/raw/main/docs/images/output_report.png
   :alt: Test Results and Monitoring Output
   :align: center

Getting Started
---------------

Ready to start testing your ROS2 applications? Here's how to get started:

1. **Install the Library**: See :doc:`installation` for setup instructions
2. **Write Your First Test**: Follow the :doc:`quickstart` guide
3. **Explore Examples**: Check out :doc:`examples/index` for real-world examples
4. **Learn the API**: Browse the :doc:`api/index` for all available keywords

**Next Steps:**

- :doc:`installation` - Set up the library
- :doc:`quickstart` - Write your first test
- :doc:`examples/index` - See real examples
- :doc:`api/index` - Explore all keywords
