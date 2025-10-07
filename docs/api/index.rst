API Reference
==============

Complete API documentation for the ROS2 RobotFramework libraries with detailed keyword explanations and examples.


The ROS2 RobotFramework provides two comprehensive libraries:

ROS2 Client Library
~~~~~~~~~~~~~~~~~~~~

Complete ROS2 functionality including:
- **Topic Operations**: Publish, subscribe, monitor, discover topics
- **Service Operations**: Call services, monitor service status
- **Action Operations**: Send action goals, monitor action results
- **Node Operations**: List nodes, get node information
- **Parameter Operations**: Get, set, and monitor parameters
- **Launch Operations**: Launch files and packages
- **Transform Operations**: TF2 transform operations
- **Utility Operations**: System health checks and cleanup

Nav2 Client Library
~~~~~~~~~~~~~~~~~~~~

Specialized Navigation2 functionality including:
- **Navigation Operations**: Navigate to poses, multi-pose navigation
- **Path Planning**: Global and local path planning
- **Localization**: AMCL integration, pose management
- **Costmap Operations**: Global and local costmap management
- **Recovery Operations**: Recovery behavior control
- **Monitoring**: Navigation status and pose tracking

.. raw:: html

   <h2>Quick Start</h2>

Import the libraries in your Robot Framework tests:

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary    timeout=10.0
   Library    Nav2ClientLibrary    timeout=30.0    action_timeout=60.0

   *** Test Cases ***
   Test ROS2 System
       ${topics}=    List Topics
       Log    Available topics: ${topics}

   Test Navigation
       ${result}=    Navigate To Pose    x=1.0    y=2.0    yaw=0.0
       Should Be True    ${result.success}

.. raw:: html

   <h2>Keyword Documentation</h2>

Each library page contains:

- **Detailed Keyword Descriptions**: What each keyword does and how to use it
- **Usage Examples**: Complete Robot Framework code examples
- **Parameter Information**: Required and optional parameters for each keyword
- **Return Values**: What each keyword returns
- **Data Classes**: Documentation for Pose, NavigationResult, and other data structures
- **Message Types**: Information about ROS2 and Navigation2 message types
- **Complete Examples**: Basic, advanced, and complex usage scenarios

