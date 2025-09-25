ROS2 Native Functions
======================

This example demonstrates advanced native ROS2 operations using the ROS2NativeClient.

Overview
--------

The native functions test covers:

- **Native Topic Operations**: Direct publishing/subscribing using ROS2 Python nodes
- **Message Type Support**: String, Bool, Int32, Twist, PoseStamped messages
- **Message Buffering**: Automatic message storage and retrieval
- **Parameter Operations**: Native parameter management
- **TF2 Operations**: Transform operations using ROS2 Python TF2
- **Error Handling**: Comprehensive error handling and validation

Test Structure
--------------

.. code-block:: robot

   *** Settings ***
   Documentation    Comprehensive test suite for ROS2 Native Client functionality
   Library          ros2_client.native_client.ROS2NativeClient
   Library          Collections
   Library          BuiltIn
   Library          String

   *** Variables ***
   ${TEST_TOPIC_1}    /test_topic_1
   ${TEST_TOPIC_2}    /test_topic_2
   ${TEST_TOPIC_3}    /test_topic_3
   ${TEST_SERVICE}    /test_service
   ${TEST_PARAM}      test_param
   ${TEST_PARAM_VALUE}    42
   ${TEST_STRING_MSG}     Hello World
   ${TEST_BOOL_MSG}       ${True}
   ${TEST_INT_MSG}        123
   ${TEST_FLOAT_MSG}      3.14
   ${TEST_TIMEOUT}        5.0

Key Test Cases
--------------

Native Topic Operations
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Native Topic Operations
       [Documentation]    Test that native topic operations work correctly
       
       # Test topic discovery
       ${topics}=    List Topics
       Log    Found topics: ${topics}
       Should Not Be Empty    ${topics}
       
       # Test topic existence check
       ${exists}=    Topic Exists    /rosout
       Should Be True    ${exists}
       
       # Test topic info
       ${info}=    Get Topic Info    /rosout
       Log    Topic info: ${info}
       Should Not Be Empty    ${info}
       
       # Test topic type
       ${type}=    Get Topic Type    /rosout
       Log    Topic type: ${type}
       Should Not Be Empty    ${type}
       
       # Test find topics by type
       ${string_topics}=    Find Topics By Type    rcl_interfaces/msg/Log
       Log    String topics: ${string_topics}

String Publisher and Subscriber
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test String Publisher and Subscriber
       [Documentation]    Test creating string publisher and subscriber, publishing and receiving messages
       [Tags]    topics    string
       
       # Create publisher
       ${publisher_id}=    Create Publisher    ${TEST_TOPIC_1}    std_msgs/msg/String
       Should Not Be Empty    ${publisher_id}
       
       # Create subscriber
       ${subscriber_id}=    Create Subscriber    ${TEST_TOPIC_1}    std_msgs/msg/String
       Should Not Be Empty    ${subscriber_id}
       
       # Wait a bit for subscription to be established
       Sleep    1s
       
       # Publish message
       ${publish_success}=    Publish Message    ${publisher_id}    ${TEST_STRING_MSG}
       Should Be True    ${publish_success}
       
       # Wait for message
       ${message}=    Wait For Message    ${TEST_TOPIC_1}    timeout=${TEST_TIMEOUT}
       Should Not Be Equal    ${message}    ${None}
       Should Be Equal    ${message}[data]    ${TEST_STRING_MSG}
       
       # Get latest message
       ${latest_message}=    Get Latest Message    ${TEST_TOPIC_1}
       Should Not Be Equal    ${latest_message}    ${None}
       Should Be Equal    ${latest_message}[data]    ${TEST_STRING_MSG}

Multiple Message Types
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Multiple Message Types
       [Documentation]    Test publishing and subscribing to different message types
       [Tags]    topics    message_types
       
       # Test Bool message
       ${bool_publisher}=    Create Publisher    ${TEST_TOPIC_2}    std_msgs/msg/Bool
       ${bool_subscriber}=    Create Subscriber    ${TEST_TOPIC_2}    std_msgs/msg/Bool
       Sleep    1s
       ${bool_success}=    Publish Message    ${bool_publisher}    ${TEST_BOOL_MSG}
       Should Be True    ${bool_success}
       ${bool_message}=    Wait For Message    ${TEST_TOPIC_2}    timeout=${TEST_TIMEOUT}
       Should Not Be Equal    ${bool_message}    ${None}
       Should Be Equal    ${bool_message}[data]    ${TEST_BOOL_MSG}
       
       # Test Int32 message
       ${int_publisher}=    Create Publisher    ${TEST_TOPIC_3}    std_msgs/msg/Int32
       ${int_subscriber}=    Create Subscriber    ${TEST_TOPIC_3}    std_msgs/msg/Int32
       Sleep    1s
       ${int_success}=    Publish Message    ${int_publisher}    ${TEST_INT_MSG}
       Should Be True    ${int_success}
       ${int_message}=    Wait For Message    ${TEST_TOPIC_3}    timeout=${TEST_TIMEOUT}
       Should Not Be Equal    ${int_message}    ${None}
       Should Be Equal As Numbers    ${int_message}[data]    ${TEST_INT_MSG}

Twist Message
~~~~~~~~~~~~~

.. code-block:: robot

   Test Twist Message
       [Documentation]    Test publishing and subscribing to Twist messages
       [Tags]    topics    twist
       
       ${twist_publisher}=    Create Publisher    /cmd_vel    geometry_msgs/msg/Twist
       ${twist_subscriber}=    Create Subscriber    /cmd_vel    geometry_msgs/msg/Twist
       Sleep    1s
       
       # Create twist data
       ${linear_dict}=    Create Dictionary    x=1.0    y=0.0    z=0.0
       ${angular_dict}=    Create Dictionary    x=0.0    y=0.0    z=0.5
       ${twist_data}=    Create Dictionary    linear=${linear_dict}    angular=${angular_dict}
       
       ${twist_success}=    Publish Message    ${twist_publisher}    ${twist_data}
       Should Be True    ${twist_success}
       
       ${twist_message}=    Wait For Message    /cmd_vel    timeout=${TEST_TIMEOUT}
       Should Not Be Equal    ${twist_message}    ${None}
       Should Be Equal As Numbers    ${twist_message}[data][linear][x]    1.0
       Should Be Equal As Numbers    ${twist_message}[data][angular][z]    0.5

PoseStamped Message
~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test PoseStamped Message
       [Documentation]    Test publishing and subscribing to PoseStamped messages
       [Tags]    topics    pose
       
       ${pose_publisher}=    Create Publisher    /test_pose    geometry_msgs/msg/PoseStamped
       ${pose_subscriber}=    Create Subscriber    /test_pose    geometry_msgs/msg/PoseStamped
       Sleep    1s
       
       # Create pose data
       ${header_dict}=    Create Dictionary    frame_id=map
       ${position_dict}=    Create Dictionary    x=1.0    y=2.0    z=0.0
       ${orientation_dict}=    Create Dictionary    x=0.0    y=0.0    z=0.0    w=1.0
       ${pose_dict}=    Create Dictionary    position=${position_dict}    orientation=${orientation_dict}
       ${pose_data}=    Create Dictionary    header=${header_dict}    pose=${pose_dict}
       
       ${pose_success}=    Publish Message    ${pose_publisher}    ${pose_data}
       Should Be True    ${pose_success}
       
       ${pose_message}=    Wait For Message    /test_pose    timeout=${TEST_TIMEOUT}
       Should Not Be Equal    ${pose_message}    ${None}
       Should Be Equal    ${pose_message}[data][header][frame_id]    map
       Should Be Equal As Numbers    ${pose_message}[data][pose][position][x]    1.0
       Should Be Equal As Numbers    ${pose_message}[data][pose][position][y]    2.0

Message Buffer Operations
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Message Buffer Operations
       [Documentation]    Test message buffer operations like getting all messages and clearing buffer
       [Tags]    topics    buffer
       
       ${publisher}=    Create Publisher    /buffer_test    std_msgs/msg/String
       ${subscriber}=    Create Subscriber    /buffer_test    std_msgs/msg/String
       Sleep    1s
       
       # Publish multiple messages
       Publish Message    ${publisher}    Message 1
       Sleep    0.1s
       Publish Message    ${publisher}    Message 2
       Sleep    0.1s
       Publish Message    ${publisher}    Message 3
       Sleep    0.5s
       
       # Get all messages
       ${all_messages}=    Get All Messages    /buffer_test
       ${length}=    Get Length    ${all_messages}
       Should Be True    ${length} > 2
       
       # Get latest message
       ${latest}=    Get Latest Message    /buffer_test
       Should Be Equal    ${latest}[data]    Message 3
       
       # Clear buffer
       ${clear_success}=    Clear Message Buffer    /buffer_test
       Should Be True    ${clear_success}
       
       # Verify buffer is cleared
       ${cleared_messages}=    Get All Messages    /buffer_test
       Should Be Empty    ${cleared_messages}

Parameter Operations
~~~~~~~~~~~~~~~~~~~~

.. code-block:: robot

   Test Parameter Operations
       [Documentation]    Test native parameter operations
       [Tags]    parameters
       
       # Declare parameter
       ${declare_success}=    Declare Parameter    ${TEST_PARAM}    ${TEST_PARAM_VALUE}
       Should Be True    ${declare_success}
       
       # Check parameter exists
       ${exists}=    Parameter Exists    ${TEST_PARAM}
       Should Be True    ${exists}
       
       # Get parameter
       ${param_value}=    Get Parameter    ${TEST_PARAM}
       Should Be Equal As Numbers    ${param_value}    ${TEST_PARAM_VALUE}
       
       # Set parameter
       ${new_value}=    Set Variable    100
       ${set_success}=    Set Parameter    ${TEST_PARAM}    ${new_value}
       Should Be True    ${set_success}
       
       # Verify new value
       ${updated_value}=    Get Parameter    ${TEST_PARAM}
       Should Be Equal As Numbers    ${updated_value}    ${new_value}
       
       # List parameters
       ${param_list}=    List Parameters
       Should Contain    ${param_list}    ${TEST_PARAM}
       
       # Get all parameters
       ${all_params}=    Get All Parameters
       Should Be Equal As Numbers    ${all_params}[${TEST_PARAM}]    ${new_value}

TF2 Operations
~~~~~~~~~~~~~~~

.. code-block:: robot

   Test TF2 Operations
       [Documentation]    Test TF2 transform operations
       [Tags]    tf2    transforms
       
       # Test can_transform (will likely return False in test environment)
       ${can_transform}=    Can Transform    map    base_link    timeout=1.0
       # This might be False in test environment, which is expected
       
       # Test get_tf (will likely return None in test environment)
       ${transform}=    Get Tf    map    base_link    timeout=1.0
       # This might be None in test environment, which is expected
       
       # Test get_tf_at_time
       ${transform_at_time}=    Get Tf At Time    map    base_link    0.0    timeout=1.0
       # This might be None in test environment, which is expected

Running the Example
-------------------

To run this example:

.. code-block:: bash

   # Run the native functions test
   robot examples/basics/ros2_native_functions.robot
   
   # Run with verbose output
   robot -v examples/basics/ros2_native_functions.robot
   
   # Run specific test cases
   robot -t "Test String Publisher and Subscriber" examples/basics/ros2_native_functions.robot

Expected Output
---------------

The test will:

1. **Test Native Topic Operations**: Verify topic discovery and information retrieval
2. **Test String Communication**: Create publishers/subscribers and exchange string messages
3. **Test Multiple Message Types**: Verify support for Bool, Int32, Twist, and PoseStamped messages
4. **Test Message Buffering**: Demonstrate message storage and retrieval capabilities
5. **Test Parameter Operations**: Verify native parameter management
6. **Test TF2 Operations**: Test transform operations (may return None in test environment)
7. **Test Error Handling**: Verify proper error handling for invalid operations

Key Learning Points
-------------------

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **Native Operations**
      
      Learn direct ROS2 Python API usage

   .. panel::
      :body:

      **Message Types**
      
      Understand different ROS2 message types

   .. panel::
      :body:

      **Message Buffering**
      
      Learn automatic message storage and retrieval

   .. panel::
      :body:

      **Parameter Management**
      
      Understand native parameter operations

   .. panel::
      :body:

      **TF2 Operations**
      
      Learn transform operations

   .. panel::
      :body:

      **Error Handling**
      
      Understand comprehensive error handling

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

1. **Message type errors**: Ensure correct message type imports
2. **Buffer issues**: Check message buffering configuration
3. **Parameter errors**: Verify parameter declaration and access
4. **TF2 errors**: TF2 operations may return None in test environment

Debug Tips
~~~~~~~~~~

1. Check message type compatibility
2. Verify parameter declarations
3. Test individual operations manually
4. Check ROS2 environment setup

Next Steps
----------

After running this example, you can:

- Explore the :doc:`nav2_basic_test` example for Navigation2 features
- Check the :doc:`../user_guide/overview` for detailed usage information
- Read the :doc:`../api/ros2_client` for complete API reference
