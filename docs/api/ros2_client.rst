ROS2 Client Library
===================

The ROS2ClientLibrary provides comprehensive ROS2 operations for Robot Framework testing.

Overview
--------

The ROS2ClientLibrary is the main library for ROS2 operations. It combines CLI and native operations to provide the best performance and reliability for each task.

Key Features:
- Topic operations (publish, subscribe, monitor, discover)
- Service operations (call, monitor, discover)
- Action operations (send goals, monitor, discover)
- Node operations (list, info, monitor)
- Parameter operations (get, set, list, monitor)
- Launch operations (launch files, packages)
- Run operations (execute nodes, manage processes)
- Transform operations (tf2 support)
- Utility operations (cleanup, info, validation)

Usage
-----

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary    timeout=10.0    node_name=test_robot

   *** Test Cases ***
   Test ROS2 System
       ${topics}=    List Topics
       ${publisher}=    Create Publisher    /chatter    std_msgs/msg/String
       Publish Message    ${publisher}    Hello World

Initialization
--------------

ROS2ClientLibrary
~~~~~~~~~~~~~~~~~

.. autoclass:: ros2_client.ros2_client.ROS2ClientLibrary
   :members:
   :undoc-members:
   :show-inheritance:

The main ROS2 client that combines CLI and native operations for Robot Framework testing.

**Keyword: Initialize ROS2 Client**

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary    timeout=10.0    node_name=test_robot

Topic Operations
----------------

**Keyword: Create Publisher**

Creates a ROS2 publisher for a specific topic and message type.

.. code-block:: robot

   ${publisher}=    Create Publisher    /chatter    std_msgs/msg/String

**Keyword: Create Subscriber**

Creates a ROS2 subscriber for a specific topic and message type.

.. code-block:: robot

   ${subscriber}=    Create Subscriber    /chatter    std_msgs/msg/String

**Keyword: Publish Message**

Publishes a message to a topic using a publisher.

.. code-block:: robot

   Publish Message    ${publisher}    Hello World

**Keyword: Wait For Message**

Waits for a message on a topic with a timeout.

.. code-block:: robot

   ${message}=    Wait For Message    /chatter    std_msgs/msg/String    timeout=5.0

**Keyword: List Topics**

Lists all available ROS2 topics.

.. code-block:: robot

   ${topics}=    List Topics

**Keyword: Topic Exists**

Checks if a topic exists.

.. code-block:: robot

   ${exists}=    Topic Exists    /chatter

**Keyword: Get Topic Info**

Gets detailed information about a topic.

.. code-block:: robot

   ${info}=    Get Topic Info    /chatter

Service Operations
------------------

**Keyword: Create Service Client**

Creates a service client for a specific service and service type.

.. code-block:: robot

   ${client}=    Create Service Client    /add_two_ints    example_interfaces/srv/AddTwoInts

**Keyword: Call Service**

Calls a service with request parameters.

.. code-block:: robot

   ${result}=    Call Service    ${client}    a=5    b=3

**Keyword: List Services**

Lists all available ROS2 services.

.. code-block:: robot

   ${services}=    List Services

**Keyword: Service Exists**

Checks if a service exists.

.. code-block:: robot

   ${exists}=    Service Exists    /add_two_ints

**Keyword: Get Service Info**

Gets detailed information about a service.

.. code-block:: robot

   ${info}=    Get Service Info    /add_two_ints

Action Operations
-----------------

**Keyword: Create Action Client**

Creates an action client for a specific action and action type.

.. code-block:: robot

   ${client}=    Create Action Client    /navigate_to_pose    nav2_msgs/action/NavigateToPose

**Keyword: Send Action Goal**

Sends a goal to an action server.

.. code-block:: robot

   ${result}=    Send Action Goal    ${client}    x=1.0    y=2.0    yaw=0.0

**Keyword: Wait For Action Result**

Waits for an action result with a timeout.

.. code-block:: robot

   ${result}=    Wait For Action Result    ${client}    timeout=30.0

**Keyword: List Actions**

Lists all available ROS2 actions.

.. code-block:: robot

   ${actions}=    List Actions

**Keyword: Action Exists**

Checks if an action exists.

.. code-block:: robot

   ${exists}=    Action Exists    /navigate_to_pose

**Keyword: Get Action Info**

Gets detailed information about an action.

.. code-block:: robot

   ${info}=    Get Action Info    /navigate_to_pose

Node Operations
---------------

**Keyword: List Nodes**

Lists all running ROS2 nodes.

.. code-block:: robot

   ${nodes}=    List Nodes

**Keyword: Node Exists**

Checks if a node exists.

.. code-block:: robot

   ${exists}=    Node Exists    /talker

**Keyword: Get Node Info**

Gets detailed information about a node.

.. code-block:: robot

   ${info}=    Get Node Info    /talker

**Keyword: Get Node Topics**

Gets topics published/subscribed by a node.

.. code-block:: robot

   ${topics}=    Get Node Topics    /talker

**Keyword: Get Node Services**

Gets services provided by a node.

.. code-block:: robot

   ${services}=    Get Node Services    /talker

Parameter Operations
--------------------

**Keyword: Get Parameter**

Gets a parameter value from a node.

.. code-block:: robot

   ${value}=    Get Parameter    /talker    my_param

**Keyword: Set Parameter**

Sets a parameter value on a node.

.. code-block:: robot

   Set Parameter    /talker    my_param    new_value

**Keyword: List Parameters**

Lists all parameters of a node.

.. code-block:: robot

   ${params}=    List Parameters    /talker

**Keyword: Parameter Exists**

Checks if a parameter exists.

.. code-block:: robot

   ${exists}=    Parameter Exists    /talker    my_param

Launch Operations
-----------------

**Keyword: Launch File**

Launches a ROS2 launch file.

.. code-block:: robot

   ${process}=    Launch File    my_launch_file.launch.py

**Keyword: Launch Package**

Launches a ROS2 package.

.. code-block:: robot

   ${process}=    Launch Package    my_package    my_launch_file.launch.py

**Keyword: List Launch Files**

Lists available launch files in a package.

.. code-block:: robot

   ${files}=    List Launch Files    my_package

**Keyword: List Packages**

Lists all available ROS2 packages.

.. code-block:: robot

   ${packages}=    List Packages

Run Operations
--------------

**Keyword: Run Node**

Runs a ROS2 node.

.. code-block:: robot

   ${process}=    Run Node    my_package    my_node

**Keyword: Stop Node**

Stops a running ROS2 node.

.. code-block:: robot

   Stop Node    ${process}

**Keyword: Is Node Running**

Checks if a node is running.

.. code-block:: robot

   ${running}=    Is Node Running    /talker

Transform Operations
--------------------

**Keyword: Get Transform**

Gets a transform between two frames.

.. code-block:: robot

   ${transform}=    Get Transform    base_link    map

**Keyword: Transform Exists**

Checks if a transform exists.

.. code-block:: robot

   ${exists}=    Transform Exists    base_link    map

**Keyword: Wait For Transform**

Waits for a transform to become available.

.. code-block:: robot

   ${transform}=    Wait For Transform    base_link    map    timeout=5.0

Utility Operations
------------------

**Keyword: ROS Doctor**

Runs ROS2 doctor to check system health.

.. code-block:: robot

   ${output}=    ROS Doctor

**Keyword: Cleanup**

Cleans up resources and connections.

.. code-block:: robot

   Cleanup

**Keyword: Get System Info**

Gets system information.

.. code-block:: robot

   ${info}=    Get System Info

Data Classes
------------

Pose
~~~~

Data class representing a 2D pose with position and orientation.

.. code-block:: robot

   ${pose}=    Create Pose    x=1.0    y=2.0    yaw=0.0

NavigationResult
~~~~~~~~~~~~~~~~

Data class representing the result of a navigation operation.

.. code-block:: robot

   ${result}=    Create Navigation Result    success=True    message=Success

Message Types
-------------

The ROS2 client library works with standard ROS2 message types:

- **std_msgs/msg/String** - String messages
- **std_msgs/msg/Int32** - Integer messages
- **geometry_msgs/msg/Twist** - Velocity commands
- **sensor_msgs/msg/LaserScan** - Laser scan data
- **nav_msgs/msg/Odometry** - Odometry data

Service Types
-------------

Common service types used with the library:

- **std_srvs/srv/Empty** - Empty service requests
- **std_srvs/srv/SetBool** - Boolean service requests
- **example_interfaces/srv/AddTwoInts** - Example service for testing

Action Types
------------

Common action types for navigation and control:

- **nav2_msgs/action/NavigateToPose** - Navigation actions
- **control_msgs/action/FollowJointTrajectory** - Joint trajectory control

Example Usage
-------------

Basic ROS2 Test
~~~~~~~~~~~~~~

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary    timeout=10.0    node_name=test_robot

   *** Test Cases ***
   Test Topic Communication
       ${publisher}=    Create Publisher    /chatter    std_msgs/msg/String
       Publish Message    ${publisher}    Hello World
       ${message}=    Wait For Message    /chatter    std_msgs/msg/String
       Should Be Equal    ${message}    Hello World

   Test Service Call
       ${client}=    Create Service Client    /add_two_ints    example_interfaces/srv/AddTwoInts
       ${result}=    Call Service    ${client}    a=5    b=3
       Should Be Equal    ${result.sum}    8

   Test Node Operations
       ${nodes}=    List Nodes
       Should Not Be Empty    ${nodes}
       ${exists}=    Node Exists    /talker
       Should Be True    ${exists}

Advanced Usage
~~~~~~~~~~~~~~

.. code-block:: robot

   *** Test Cases ***
   Test Action Goal
       ${client}=    Create Action Client    /navigate_to_pose    nav2_msgs/action/NavigateToPose
       ${result}=    Send Action Goal    ${client}    x=1.0    y=2.0    yaw=0.0
       Should Be True    ${result.success}

   Test Parameter Operations
       Set Parameter    /talker    my_param    test_value
       ${value}=    Get Parameter    /talker    my_param
       Should Be Equal    ${value}    test_value

   Test Transform Operations
       ${transform}=    Get Transform    base_link    map
       Should Not Be Empty    ${transform}
