Quick Start Guide
=================

This guide will help you get started with ROS2 RobotFramework in just a few minutes.

Prerequisites
-------------

Before starting, make sure you have:

- ROS2 installed and sourced
- Python 3.8 or higher
- The library installed (see :doc:`installation`)

Basic Setup
-----------

1. **Source your ROS2 environment**:

   .. code-block:: bash

      source /opt/ros/humble/setup.bash
      # or your specific ROS2 distribution

2. **Create a simple test file**:

   .. code-block:: robot

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

3. **Run your first test**:

   .. code-block:: bash

      robot your_test_file.robot

Your First Test
---------------

Let's create a comprehensive test that demonstrates the library's capabilities:

.. code-block:: robot

   *** Settings ***
   Documentation    Complete ROS2 system test using launch and run operations
   Library          ROS2ClientLibrary

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

Key Concepts
============

Understanding the Library Structure
-----------------------------------

The library provides two main components:

.. panels::
   :container: +full-width text-center
   :column: col-lg-6 col-md-12

   .. panel::
      :body:

      **ROS2ClientLibrary**
      
      Core ROS2 operations (topics, services, nodes, parameters, launch, run)

   .. panel::
      :body:

      **Nav2ClientLibrary**
      
      Navigation2-specific operations (navigation, costmaps, recovery)

Operation Types
~~~~~~~~~~~~~~~

.. tabs::

   .. tab:: Topic Operations

      .. code-block:: robot

         # List all topics
         ${topics}=    List Topics
         
         # Check if topic exists
         ${exists}=    Topic Exists    /chatter
         
         # Get topic information
         ${info}=    Get Topic Info    /chatter
         
         # Echo messages from topic
         ${messages}=    Echo Topic    /chatter    count=3
         
         # Publish to topic
         ${success}=    Publish Topic    /chatter    std_msgs/msg/String    "Hello World"

   .. tab:: Service Operations

      .. code-block:: robot

         # List all services
         ${services}=    List Services
         
         # Check if service exists
         ${exists}=    Service Available    /add_two_ints
         
         # Get service information
         ${info}=    Get Service Info    /add_two_ints
         
         # Call a service
         ${response}=    Call Service    /add_two_ints    example_interfaces/srv/AddTwoInts    "a: 5, b: 3"

   .. tab:: Node Operations

      .. code-block:: robot

         # List all nodes
         ${nodes}=    List Nodes
         
         # Check if node exists
         ${exists}=    Node Exists    /talker
         
         # Get node information
         ${info}=    Get Node Info    /talker
         
         # Wait for node to appear
         ${available}=    Wait For Node    /talker    timeout=10.0

   .. tab:: Launch Operations

      .. code-block:: robot

         # Launch a launch file
         ${process}=    Launch Package    demo_nodes_cpp    talker_listener.launch.py
         
         # Wait for launch completion
         ${completed}=    Wait For Launch Completion    ${process}    timeout=30.0
         
         # Terminate launch process
         ${terminated}=    Terminate Launch Process    ${process}

Native vs CLI Operations
------------------------

The library supports two modes of operation:

.. panels::
   :container: +full-width text-center
   :column: col-lg-6 col-md-12

   .. panel::
      :body:

      **CLI Operations**
      
      Uses ROS2 command-line tools
      
      - More reliable
      - Better error handling
      - Slower execution

   .. panel::
      :body:

      **Native Operations**
      
      Direct ROS2 Python API calls
      
      - Faster execution
      - Real-time communication
      - More features

Example: Native Operations
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary    use_native_node=True

   *** Test Cases ***
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

Running Examples
----------------

The library comes with several example files:

.. code-block:: bash

   # Run basic ROS2 test
   robot examples/basics/ros2_basic_test.robot
   
   # Run Navigation2 monitoring test
   robot examples/medium/nav2_simple_monitoring_test.robot
   
   # Run Nav2 with TurtleBot3
   robot examples/medium/nav2_turtlebot.robot

Best Practices
--------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **Always use timeouts**
      
      Set appropriate timeouts for operations

   .. panel::
      :body:

      **Clean up processes**
      
      Always terminate started processes

   .. panel::
      :body:

      **Check existence first**
      
      Use existence checks before operations

   .. panel::
      :body:

      **Handle errors gracefully**
      
      Use Robot Framework's error handling keywords

   .. panel::
      :body:

      **Use wait keywords**
      
      Use wait keywords for dynamic systems

   .. panel::
      :body:

      **Log important information**
      
      Use Log keyword for debugging

Next Steps
----------

Now that you have the basics, you can:

- Explore the :doc:`examples/index` for more complex examples
- Read the :doc:`user_guide/overview` for detailed usage information
- Check the :doc:`api/ros2_client` for complete API reference
- Learn about :doc:`user_guide/nav2_operations` for Navigation2 features
