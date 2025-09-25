Basic ROS2 Test
==============

This example demonstrates fundamental ROS2 operations using the ROS2ClientLibrary.

Overview
--------

The basic ROS2 test covers:

- **Discovery Operations**: Listing topics, services, and nodes
- **Node Management**: Starting, monitoring, and terminating nodes
- **Topic Operations**: Publishing, subscribing, and echoing messages
- **Launch Operations**: Starting and managing launch files
- **Error Handling**: Testing with non-existent resources

Test Structure
--------------

.. code-block:: robot

   *** Settings ***
   Documentation    Basic ROS2 CLI Library Test
   Library          ros2_client.ROS2ClientLibrary
   Library          Process
   Library          Collections

   *** Variables ***
   ${TEST_TIMEOUT}    10.0
   ${TEST_MESSAGE}    Hello from new structure!

Key Test Cases
--------------

Discovery Operations
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Verify ROS2 Discovery Operations
       [Documentation]    Test basic ROS2 CLI operations
       [Tags]    basic    ros2
       
       # Test listing topics
       ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
       Should Not Be Empty    ${topics}
       Log    Found topics: ${topics}
       
       # Test listing services
       ${services}=    List Services    timeout=${TEST_TIMEOUT}
       Log    Found services: ${services}
       
       # Test listing nodes
       ${nodes}=    List Nodes    timeout=${TEST_TIMEOUT}
       Log    Found nodes: ${nodes}

Node and Topic Operations
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test ROS2 Node And Topic Operations With Demo
       [Documentation]    Test ROS2 CLI operations with demo nodes
       [Tags]    demo    ros2
       [Timeout]    30s
       
       # Start demo talker node
       ${talker_process}=    Start Process    ros2    run    demo_nodes_cpp    talker
       Set Test Variable    ${talker_process}
       
       # Wait for the node to start
       Sleep    3s
       
       # Test node operations
       ${nodes}=    List Nodes    timeout=${TEST_TIMEOUT}
       Should Contain    ${nodes}    /talker
       
       # Test topic operations
       ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
       Should Contain    ${topics}    /chatter
       
       # Test topic info
       ${topic_info}=    Get Topic Info    /chatter    timeout=${TEST_TIMEOUT}
       Should Be Equal    ${topic_info}[type]    std_msgs/msg/String
       
       # Test node info
       ${node_info}=    Get Node Info    /talker    timeout=${TEST_TIMEOUT}
       Log    Node info: ${node_info}
       
       # Clean up
       Terminate Process    ${talker_process}

Launch Operations
~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test ROS2 Launch File Operations
       [Documentation]    Test launch operations with demo nodes
       [Tags]    launch    ros2
       [Timeout]    30s
       
       # Launch a simple demo launch file
       ${launch_success}=    Run Keyword And Return Status    Launch Package    demo_nodes_cpp    talker_listener.launch.py
       IF    ${launch_success}
           Log    Launch started successfully
           ${process}=    Launch Package    demo_nodes_cpp    talker_listener.launch.py
           
           # Wait for system to initialize
           Sleep    3s
           
           # Verify topics are available
           ${topics}=    List Topics    timeout=${TEST_TIMEOUT}
           Should Contain    ${topics}    /chatter
           
           # Stop the launch
           ${terminated}=    Terminate Launch Process    ${process}
           Should Be True    ${terminated}
           Log    Launch terminated successfully
       ELSE
           Log    Demo launch file not available, skipping launch test
       END

Native Operations
~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test ROS2 Native Publisher And Subscriber Operations
       [Documentation]    Test native ROS2 operations with subscribers and publishers
       [Tags]    native    ros2
       [Timeout]    30s
       
       # Get client information
       ${info}=    Get Client Info
       Log    Using client with native support: ${info}[native_available]
       
       # Start a demo talker node
       ${talker_process}=    Run Node    demo_nodes_cpp    talker
       Set Test Variable    ${talker_process}
       
       # Wait for the node to start
       Sleep    3s
       
       # Create a native subscriber for the chatter topic
       ${subscriber}=    Create Subscriber    /chatter    std_msgs/msg/String
       Log    Created native subscriber: ${subscriber}
       
       # Wait for messages
       Sleep    5s
       
       # Get the latest message using native operations
       ${message}=    Get Latest Message    /chatter
       IF    ${message} is not None
           Log    Received message: ${message}[data]
           Should Contain    ${message}[data]    Hello World
       ELSE
           Log    No message received yet
       END
       
       # Clean up
       Shutdown Process    ${talker_process}
       Cleanup

Running the Example
-------------------

To run this example:

.. code-block:: bash

   # Run the basic ROS2 test
   robot examples/basics/ros2_basic_test.robot
   
   # Run with verbose output
   robot -v examples/basics/ros2_basic_test.robot
   
   # Run specific test cases
   robot -t "Verify ROS2 Discovery Operations" examples/basics/ros2_basic_test.robot

Expected Output
---------------

The test will:

1. **Verify ROS2 Discovery**: List and verify topics, services, and nodes
2. **Test Node Operations**: Start a demo talker node and verify it's running
3. **Test Topic Communication**: Verify topic information and message flow
4. **Test Launch Operations**: Start and stop launch files
5. **Test Native Operations**: Use native ROS2 Python API for real-time communication
6. **Test Error Handling**: Verify proper handling of non-existent resources

Key Learning Points
-------------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **Discovery Operations**
      
      Learn how to list and verify ROS2 resources

   .. panel::
      :body:

      **Process Management**
      
      Understand how to start and stop ROS2 processes

   .. panel::
      :body:

      **Topic Communication**
      
      See how to publish, subscribe, and echo messages

   .. panel::
      :body:

      **Launch File Management**
      
      Learn to start and stop complex ROS2 systems

   .. panel::
      :body:

      **Native Operations**
      
      Understand direct ROS2 Python API usage

   .. panel::
      :body:

      **Error Handling**
      
      Learn proper error handling and validation

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

1. **Demo nodes not available**: Ensure `demo_nodes_cpp` package is installed
2. **Permission errors**: Check file permissions and user access
3. **Timeout errors**: Increase timeout values for slow systems
4. **Process not found**: Ensure demo nodes are available

Debug Tips
~~~~~~~~~~

1. Enable debug logging in Robot Framework
2. Check ROS2 environment variables
3. Verify ROS2 installation with `ros2 --help`
4. Test individual commands manually

Next Steps
----------

After running this example, you can:

- Explore the :doc:`ros2_native_functions` example for advanced native operations
- Check the :doc:`nav2_basic_test` example for Navigation2 features
- Read the :doc:`../user_guide/overview` for detailed usage information
