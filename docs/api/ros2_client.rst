ROS2ClientLibrary
=================

The main ROS2 client that combines CLI and native operations for RobotFramework testing.

Overview
--------

The `ROS2ClientLibrary` is the primary client that users should use. It provides a unified interface that intelligently combines:

- **Native operations** for all topic operations, real-time communication, and performance-critical tasks
- **CLI operations** for system management, launch operations, and process management

The client automatically selects the best approach for each operation:

- **Native operations** for all topic operations, publishers, subscribers, and service clients (better performance)
- **CLI operations** for system management, launch operations, and process management (more reliable)

Initialization
--------------

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary    timeout=10.0    node_name=test_robot

Parameters
~~~~~~~~~~

- **timeout** (float, optional): Default timeout for operations in seconds (default: 10.0)
- **node_name** (str, optional): Name for the native ROS2 node (must be unique) (default: "robotframework_ros2")

Topic Operations
===============

Discovery and Information
-------------------------

.. py:method:: list_topics(timeout=None)

   List all available ROS2 topics.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of topic names

   **Example:**
   .. code-block:: robot

      ${topics}=    List Topics
      Should Contain    ${topics}    /chatter

.. py:method:: get_topic_info(topic_name, timeout=None)

   Get detailed information about a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Dictionary containing topic information (type, publishers, subscribers)

   **Example:**
   .. code-block:: robot

      ${info}=    Get Topic Info    /chatter
      Log    Topic type: ${info['type']}

.. py:method:: get_topic_type(topic_name, timeout=None)

   Get the message type for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Message type string (e.g., 'std_msgs/msg/String')

   **Example:**
   .. code-block:: robot

      ${type}=    Get Topic Type    /chatter
      Should Be Equal    ${type}    std_msgs/msg/String

.. py:method:: find_topics_by_type(message_type, timeout=None)

   Find topics that use a specific message type.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **message_type** (str): Message type to search for (e.g., 'std_msgs/msg/String')
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of topic names using the specified message type

   **Example:**
   .. code-block:: robot

      ${string_topics}=    Find Topics By Type    std_msgs/msg/String
      Should Contain    ${string_topics}    /chatter

.. py:method:: topic_exists(topic_name, timeout=None)

   Check if a topic exists.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic to check
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** True if topic exists, False otherwise

   **Example:**
   .. code-block:: robot

      ${exists}=    Topic Exists    /chatter
      Should Be True    ${exists}

.. py:method:: wait_for_topic(topic_name, timeout=30.0, check_interval=1.0)

   Wait for a topic to become available.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic to wait for
   - **timeout** (float): Maximum time to wait in seconds (default: 30.0)
   - **check_interval** (float): Time between checks in seconds (default: 1.0)

   **Returns:** True if topic becomes available, False if timeout

   **Example:**
   .. code-block:: robot

      ${available}=    Wait For Topic    /chatter    timeout=10.0
      Should Be True    ${available}

Publishing and Subscribing
--------------------------

.. py:method:: create_publisher(topic_name, message_type)

   Create a publisher for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **message_type** (str): Type of the message (e.g., 'std_msgs/msg/String')

   **Returns:** Publisher ID for use with publish_message

   **Example:**
   .. code-block:: robot

      ${publisher}=    Create Publisher    /chatter    std_msgs/msg/String
      Publish Message    ${publisher}    Hello World

.. py:method:: publish_message(publisher_id, data)

   Publish a message using a created publisher.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **publisher_id** (str): Publisher ID from create_publisher
   - **data** (Any): Message data to publish

   **Returns:** True if successful

   **Example:**
   .. code-block:: robot

      ${publisher}=    Create Publisher    /chatter    std_msgs/msg/String
      ${success}=    Publish Message    ${publisher}    Hello World
      Should Be True    ${success}

.. py:method:: create_subscriber(topic_name, message_type)

   Create a subscriber for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **message_type** (str): Type of the message (e.g., 'std_msgs/msg/String')

   **Returns:** Subscriber ID for use with message retrieval

   **Example:**
   .. code-block:: robot

      ${subscriber}=    Create Subscriber    /chatter    std_msgs/msg/String
      ${message}=    Get Latest Message    /chatter

.. py:method:: get_latest_message(topic_name)

   Get the latest message from a subscribed topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic

   **Returns:** Latest message data or None if no message available

   **Example:**
   .. code-block:: robot

      ${subscriber}=    Create Subscriber    /chatter    std_msgs/msg/String
      ${message}=    Get Latest Message    /chatter
      Should Not Be None    ${message}

Monitoring and Analysis
-----------------------

.. py:method:: echo_topic(topic_name, count=1, timeout=None)

   Echo messages from a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **count** (int): Number of messages to capture (default: 1)
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of message strings

   **Example:**
   .. code-block:: robot

      ${messages}=    Echo Topic    /chatter    count=5
      Length Should Be    ${messages}    5

.. py:method:: get_topic_frequency(topic_name, timeout=10.0)

   Get the message frequency for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **timeout** (float): Time to measure frequency in seconds (default: 10.0)

   **Returns:** Average message frequency in Hz

   **Example:**
   .. code-block:: robot

      ${freq}=    Get Topic Frequency    /chatter    timeout=5.0
      Should Be True    ${freq} > 0

.. py:method:: get_topic_bandwidth(topic_name, timeout=10.0)

   Get the bandwidth usage for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **timeout** (float): Time to measure bandwidth in seconds (default: 10.0)

   **Returns:** Average bandwidth in bytes per second

   **Example:**
   .. code-block:: robot

      ${bw}=    Get Topic Bandwidth    /chatter    timeout=5.0
      Log    Bandwidth: ${bw} bytes/sec

.. py:method:: get_topic_delay(topic_name, timeout=10.0)

   Get the message delay for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **timeout** (float): Time to measure delay in seconds (default: 10.0)

   **Returns:** Average message delay in seconds

   **Example:**
   .. code-block:: robot

      ${delay}=    Get Topic Delay    /chatter    timeout=5.0
      Should Be True    ${delay} >= 0

Service Operations
==================

Discovery and Information
-------------------------

.. py:method:: list_services(timeout=None)

   List all available ROS2 services.

   Uses CLI command: ros2 service list

   **Parameters:**
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of service names

   **Example:**
   .. code-block:: robot

      ${services}=    List Services
      Should Contain    ${services}    /add_two_ints

.. py:method:: get_service_info_cli(service_name, timeout=None)

   Get detailed information about a service via CLI.

   Uses CLI command: ros2 service info

   **Parameters:**
   - **service_name** (str): Name of the service
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Dictionary containing service information (type, nodes)

   **Example:**
   .. code-block:: robot

      ${info}=    Get Service Info CLI    /add_two_ints
      Log    Service type: ${info['type']}

.. py:method:: get_service_type(service_name, timeout=None)

   Get the service type for a service.

   Uses CLI command: ros2 service type

   **Parameters:**
   - **service_name** (str): Name of the service
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Service type string (e.g., 'example_interfaces/srv/AddTwoInts')

   **Example:**
   .. code-block:: robot

      ${type}=    Get Service Type    /add_two_ints
      Should Be Equal    ${type}    example_interfaces/srv/AddTwoInts

.. py:method:: find_services_by_type(service_type, timeout=None)

   Find services that use a specific service type.

   Uses CLI command: ros2 service find

   **Parameters:**
   - **service_type** (str): Service type to search for (e.g., 'example_interfaces/srv/AddTwoInts')
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of service names using the specified service type

   **Example:**
   .. code-block:: robot

      ${add_services}=    Find Services By Type    example_interfaces/srv/AddTwoInts
      Should Contain    ${add_services}    /add_two_ints

Service Communication
---------------------

.. py:method:: create_service_client(service_name, service_type)

   Create a native ROS2 service client.

   Uses native ROS2 operations for better performance and reliability.

   **Parameters:**
   - **service_name** (str): Name of the service (e.g., '/add_two_ints')
   - **service_type** (str): Type of the service (e.g., 'example_interfaces/srv/AddTwoInts')

   **Returns:** Client ID that can be used with call_service and service_available

   **Example:**
   .. code-block:: robot

      ${client_id}=    Create Service Client    /add_two_ints    example_interfaces/srv/AddTwoInts
      ${result}=    Call Service    ${client_id}    {"a": 5, "b": 3}

.. py:method:: call_service(client_id, request_data=None, timeout=10.0)

   Call a service using a native service client.

   Uses native ROS2 operations for better performance and reliability.

   **Parameters:**
   - **client_id** (str): ID of the service client (from create_service_client)
   - **request_data** (Any): Request data to send to the service
   - **timeout** (float): Timeout for the service call (default: 10.0)

   **Returns:** Service response data or None if failed

   **Example:**
   .. code-block:: robot

      ${client_id}=    Create Service Client    /add_two_ints    example_interfaces/srv/AddTwoInts
      ${result}=    Call Service    ${client_id}    {"a": 5, "b": 3}
      Should Be Equal    ${result['sum']}    8

.. py:method:: service_available(client_id, timeout=1.0)

   Check if a service is available using native client.

   **Parameters:**
   - **client_id** (str): ID of the service client (from create_service_client)
   - **timeout** (float): Timeout for checking availability (default: 1.0)

   **Returns:** True if service is available, False otherwise

   **Example:**
   .. code-block:: robot

      ${client_id}=    Create Service Client    /add_two_ints    example_interfaces/srv/AddTwoInts
      ${available}=    Service Available    ${client_id}
      Should Be True    ${available}

.. py:method:: create_service_server(service_name, service_type, callback_function=None)

   Create a native ROS2 service server.

   Uses native ROS2 operations for better performance and reliability.

   **Parameters:**
   - **service_name** (str): Name of the service (e.g., '/add_two_ints')
   - **service_type** (str): Type of the service (e.g., 'example_interfaces/srv/AddTwoInts')
   - **callback_function** (Any, optional): Optional callback function to handle service requests

   **Returns:** Server ID for managing the service server

   **Example:**
   .. code-block:: robot

      ${server_id}=    Create Service Server    /add_two_ints    example_interfaces/srv/AddTwoInts

.. py:method:: get_service_info()

   Get information about native service clients and servers.

   **Returns:** Dictionary containing information about all created service clients and servers

   **Example:**
   .. code-block:: robot

      ${info}=    Get Service Info
      Log    Service clients: ${info['clients']}

Node Operations
===============

.. py:method:: list_nodes(timeout=None)

   List all running ROS2 nodes.

   Uses CLI command: ros2 node list

   **Parameters:**
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of node names

   **Example:**
   .. code-block:: robot

      ${nodes}=    List Nodes
      Should Contain    ${nodes}    /talker

.. py:method:: get_node_info(node_name, timeout=None)

   Get detailed information about a node.

   Uses CLI command: ros2 node info

   **Parameters:**
   - **node_name** (str): Name of the node
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Dictionary containing node information (topics, services, actions)

   **Example:**
   .. code-block:: robot

      ${info}=    Get Node Info    /talker
      Log    Node topics: ${info['topics']}

.. py:method:: node_exists(node_name, timeout=None)

   Check if a node exists.

   **Parameters:**
   - **node_name** (str): Name of the node to check
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** True if node exists, False otherwise

   **Example:**
   .. code-block:: robot

      ${exists}=    Node Exists    /talker
      Should Be True    ${exists}

.. py:method:: wait_for_node(node_name, timeout=30.0, check_interval=1.0)

   Wait for a node to become available.

   **Parameters:**
   - **node_name** (str): Name of the node to wait for
   - **timeout** (float): Maximum time to wait in seconds (default: 30.0)
   - **check_interval** (float): Time between checks in seconds (default: 1.0)

   **Returns:** True if node becomes available, False if timeout

   **Example:**
   .. code-block:: robot

      ${available}=    Wait For Node    /talker    timeout=10.0
      Should Be True    ${available}

.. py:method:: has_running_nodes(timeout=30.0)

   Check if there are any nodes running.

   **Parameters:**
   - **timeout** (float): Timeout for checking node availability (default: 30.0)

   **Returns:** True if any nodes are running, False otherwise

   **Example:**
   .. code-block:: robot

      ${running}=    Has Running Nodes    timeout=5.0
      Should Be True    ${running}

Parameter Operations
====================

Native Node Parameters
---------------------

.. py:method:: list_parameters()

   List all parameters for the native test node.

   Uses native ROS2 operations for the test node only.

   **Returns:** List of parameter names for the native node

   **Example:**
   .. code-block:: robot

      ${params}=    List Parameters
      Log    Native node parameters: ${params}

.. py:method:: get_parameter(parameter_name, default_value=None)

   Get a parameter from the native test node.

   Uses native ROS2 operations for the test node only.

   **Parameters:**
   - **parameter_name** (str): Name of the parameter
   - **default_value** (Any): Default value if parameter doesn't exist

   **Returns:** Parameter value or default value

   **Example:**
   .. code-block:: robot

      ${value}=    Get Parameter    my_param    default=10
      Should Be Equal    ${value}    10

.. py:method:: set_parameter(parameter_name, value)

   Set a parameter on the native test node.

   Uses native ROS2 operations for the test node only.

   **Parameters:**
   - **parameter_name** (str): Name of the parameter
   - **value** (Union[str, int, float, bool]): Value to set

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Set Parameter    my_param    42
      Should Be True    ${success}

.. py:method:: parameter_exists(parameter_name)

   Check if a parameter exists on the native test node.

   Uses native ROS2 operations for the test node only.

   **Parameters:**
   - **parameter_name** (str): Name of the parameter to check

   **Returns:** True if parameter exists, False otherwise

   **Example:**
   .. code-block:: robot

      ${exists}=    Parameter Exists    my_param
      Should Be True    ${exists}

.. py:method:: get_all_parameters()

   Get all parameters from the native test node.

   Uses native ROS2 operations for the test node only.

   **Returns:** Dictionary of all parameter names and values

   **Example:**
   .. code-block:: robot

      ${all_params}=    Get All Parameters
      Log    All parameters: ${all_params}

Any Node Parameters
-------------------

.. py:method:: list_node_parameters(node_name, timeout=None)

   List parameters for any node via CLI.

   Uses CLI command: ros2 param list

   **Parameters:**
   - **node_name** (str): Name of the node
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of parameter names for the specified node

   **Example:**
   .. code-block:: robot

      ${params}=    List Node Parameters    /my_node
      Should Contain    ${params}    use_sim_time

.. py:method:: get_node_parameter(node_name, param_name, timeout=None)

   Get a parameter from any node via CLI.

   Uses CLI command: ros2 param get

   **Parameters:**
   - **node_name** (str): Name of the node
   - **param_name** (str): Name of the parameter
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Parameter value

   **Example:**
   .. code-block:: robot

      ${value}=    Get Node Parameter    /my_node    use_sim_time
      Should Be Equal    ${value}    False

.. py:method:: set_node_parameter(node_name, param_name, value, timeout=None)

   Set a parameter on any node via CLI.

   Uses CLI command: ros2 param set

   **Parameters:**
   - **node_name** (str): Name of the node
   - **param_name** (str): Name of the parameter
   - **value** (Any): Value to set
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Set Node Parameter    /my_node    use_sim_time    True
      Should Be True    ${success}

.. py:method:: dump_node_parameters(node_name, output_file=None, timeout=None)

   Dump all parameters from a node to YAML format.

   Uses CLI command: ros2 param dump

   **Parameters:**
   - **node_name** (str): Name of the node
   - **output_file** (str, optional): Optional output file path (if None, returns YAML string)
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** YAML string of parameters or file path if output_file specified

   **Example:**
   .. code-block:: robot

      ${yaml}=    Dump Node Parameters    /my_node
      Log    Parameters YAML: ${yaml}

Launch Operations
=================

.. py:method:: launch_file(launch_file_path, arguments=None, timeout=None)

   Launch a ROS2 launch file.

   Uses CLI command: ros2 launch

   **Parameters:**
   - **launch_file_path** (str): Path to the launch file
   - **arguments** (Dict[str, str], optional): Optional launch arguments as key-value pairs
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Launch process object

   **Example:**
   .. code-block:: robot

      ${process}=    Launch File    /path/to/launch_file.launch.py    arguments={"use_sim_time": "true"}

.. py:method:: launch_package(package_name, launch_file_name, arguments=None)

   Launch a ROS2 package launch file.

   Uses CLI command: ros2 launch

   **Parameters:**
   - **package_name** (str): Name of the ROS2 package
   - **launch_file_name** (str): Name of the launch file within the package
   - **arguments** (str, optional): Optional launch arguments as key-value pairs

   **Returns:** Launch process object

   **Example:**
   .. code-block:: robot

      ${process}=    Launch Package    nav2_bringup    tb3_simulation_launch.py    arguments={"use_sim_time": "true"}

.. py:method:: find_launch_files(package_name, timeout=None)

   Find launch files in a ROS2 package.

   **Parameters:**
   - **package_name** (str): Name of the ROS2 package
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of launch file names in the package

   **Example:**
   .. code-block:: robot

      ${files}=    Find Launch Files    nav2_bringup
      Should Contain    ${files}    tb3_simulation_launch.py

.. py:method:: wait_for_launch_completion(process, timeout=30.0)

   Wait for a launch process to complete.

   **Parameters:**
   - **process**: Launch process object
   - **timeout** (float): Maximum time to wait in seconds (default: 30.0)

   **Returns:** True if process completed, False if timeout

   **Example:**
   .. code-block:: robot

      ${completed}=    Wait For Launch Completion    ${process}    timeout=60.0
      Should Be True    ${completed}

.. py:method:: terminate_launch_process(process, force=False)

   Terminate a launch process.

   **Parameters:**
   - **process**: Launch process object
   - **force** (bool): Whether to force termination (default: False)

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Terminate Launch Process    ${process}    force=True
      Should Be True    ${success}

Run Operations
==============

.. py:method:: run_node(package_name, executable_name, arguments=None, setup_script=None)

   Run a ROS2 node.

   Uses CLI command: ros2 run

   **Parameters:**
   - **package_name** (str): Name of the ROS2 package
   - **executable_name** (str): Name of the executable
   - **arguments** (List[str], optional): Optional command line arguments
   - **setup_script** (str, optional): Optional setup script to source before running

   **Returns:** Process object

   **Example:**
   .. code-block:: robot

      ${process}=    Run Node    demo_nodes_cpp    talker    arguments=["--ros-args", "-p", "use_sim_time:=true"]

.. py:method:: run_node_with_remap(package_name, executable_name, remaps=None, arguments=None, setup_script=None, timeout=None)

   Run a ROS2 node with topic remapping.

   Uses CLI command: ros2 run with remapping

   **Parameters:**
   - **package_name** (str): Name of the ROS2 package
   - **executable_name** (str): Name of the executable
   - **remaps** (Dict[str, str], optional): Dictionary of topic remappings (old_topic -> new_topic)
   - **arguments** (List[str], optional): Optional command line arguments
   - **setup_script** (str, optional): Optional setup script to source before running
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** Process object

   **Example:**
   .. code-block:: robot

      ${process}=    Run Node With Remap    demo_nodes_cpp    talker    remaps={"chatter": "my_chatter"}

.. py:method:: find_executables(package_name, timeout=None)

   Find executables in a ROS2 package.

   **Parameters:**
   - **package_name** (str): Name of the ROS2 package
   - **timeout** (float, optional): Override default timeout for this operation

   **Returns:** List of executable names in the package

   **Example:**
   .. code-block:: robot

      ${executables}=    Find Executables    demo_nodes_cpp
      Should Contain    ${executables}    talker

Process Management
------------------

.. py:method:: wait_for_node_completion(process, timeout=30.0)

   Wait for a node process to complete.

   **Parameters:**
   - **process**: Process object
   - **timeout** (float): Maximum time to wait in seconds (default: 30.0)

   **Returns:** True if process completed, False if timeout

   **Example:**
   .. code-block:: robot

      ${completed}=    Wait For Node Completion    ${process}    timeout=60.0
      Should Be True    ${completed}

.. py:method:: terminate_node_process(process, force=False)

   Terminate a node process.

   **Parameters:**
   - **process**: Process object
   - **force** (bool): Whether to force termination (default: False)

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Terminate Node Process    ${process}    force=True
      Should Be True    ${success}

.. py:method:: get_process_output(process, timeout=1.0)

   Get output from a process.

   **Parameters:**
   - **process**: Process object
   - **timeout** (float): Timeout for reading output (default: 1.0)

   **Returns:** Dictionary with 'stdout' and 'stderr' keys

   **Example:**
   .. code-block:: robot

      ${output}=    Get Process Output    ${process}
      Log    stdout: ${output['stdout']}

.. py:method:: is_process_running(process)

   Check if a process is running.

   **Parameters:**
   - **process**: Process object

   **Returns:** True if process is running, False otherwise

   **Example:**
   .. code-block:: robot

      ${running}=    Is Process Running    ${process}
      Should Be True    ${running}

.. py:method:: shutdown_process(process_name, force=False)

   Shutdown a process by name.

   **Parameters:**
   - **process_name** (str): Name of the process to shutdown
   - **force** (bool): Whether to force shutdown (default: False)

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Shutdown Process    talker    force=True
      Should Be True    ${success}

.. py:method:: kill_process_by_name(process_name)

   Kill a process by name.

   **Parameters:**
   - **process_name** (str): Name of the process to kill

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Kill Process By Name    talker
      Should Be True    ${success}

Native-Specific Operations
==========================

Advanced Message Operations
---------------------------

.. py:method:: get_all_messages(topic_name)

   Get all buffered messages from a subscribed topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic

   **Returns:** List of all buffered messages

   **Example:**
   .. code-block:: robot

      ${messages}=    Get All Messages    /chatter
      Length Should Be    ${messages}    5

.. py:method:: clear_message_buffer(topic_name)

   Clear message buffer for a topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Clear Message Buffer    /chatter
      Should Be True    ${success}

.. py:method:: wait_for_message(topic_name, timeout=10.0, check_interval=0.1)

   Wait for a message on a subscribed topic.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **topic_name** (str): Name of the topic
   - **timeout** (float): Maximum time to wait in seconds (default: 10.0)
   - **check_interval** (float): Time between checks in seconds (default: 0.1)

   **Returns:** Message data or None if timeout

   **Example:**
   .. code-block:: robot

      ${message}=    Wait For Message    /chatter    timeout=5.0
      Should Not Be None    ${message}

Advanced Parameter Operations
----------------------------

.. py:method:: declare_parameter(parameter_name, default_value)

   Declare a parameter with default value on the native node.

   Uses native ROS2 operations for better performance.

   **Parameters:**
   - **parameter_name** (str): Name of the parameter
   - **default_value** (Any): Default value for the parameter

   **Returns:** True if successful, False otherwise

   **Example:**
   .. code-block:: robot

      ${success}=    Declare Parameter    my_param    42
      Should Be True    ${success}

Transform Operations
====================

.. py:method:: get_transform(target_frame, source_frame, timeout=5.0)

   Get transform between two frames using tf2.

   Uses native ROS2 operations for real-time transform queries.

   **Parameters:**
   - **target_frame** (str): Target frame name
   - **source_frame** (str): Source frame name
   - **timeout** (float): Maximum time to wait for transform in seconds (default: 5.0)

   **Returns:** Transform data dictionary with translation and rotation, or None if failed

   **Example:**
   .. code-block:: robot

      ${transform}=    Get Transform    map    base_link    timeout=1.0
      Should Not Be None    ${transform}
      Log    Position: ${transform['translation']}

.. py:method:: get_transform_at_time(target_frame, source_frame, time_stamp, timeout=5.0)

   Get transform between two frames at a specific time using tf2.

   Uses native ROS2 operations for real-time transform queries.

   **Parameters:**
   - **target_frame** (str): Target frame name
   - **source_frame** (str): Source frame name
   - **time_stamp** (float): Time stamp for the transform
   - **timeout** (float): Maximum time to wait for transform in seconds (default: 5.0)

   **Returns:** Transform data dictionary with translation and rotation, or None if failed

   **Example:**
   .. code-block:: robot

      ${transform}=    Get Transform At Time    map    base_link    1234567890.0    timeout=1.0
      Should Not Be None    ${transform}

.. py:method:: can_transform(target_frame, source_frame, timeout=5.0)

   Check if a transform is available between two frames.

   Uses native ROS2 operations for real-time transform queries.

   **Parameters:**
   - **target_frame** (str): Target frame name
   - **source_frame** (str): Source frame name
   - **timeout** (float): Maximum time to wait for transform in seconds (default: 5.0)

   **Returns:** True if transform is available, False otherwise

   **Example:**
   .. code-block:: robot

      ${available}=    Can Transform    map    base_link    timeout=1.0
      Should Be True    ${available}

Utility Methods
===============

.. py:method:: cleanup()

   Clean up all resources and connections.

   This method should be called at the end of test suites to properly clean up ROS2 connections and resources.

   **Example:**
   .. code-block:: robot

      Cleanup

.. py:method:: get_client_info()

   Get information about the current client configuration.

   **Returns:** Dictionary containing client configuration information

   **Example:**
   .. code-block:: robot

      ${info}=    Get Client Info
      Log    Native available: ${info['native_available']}
      Log    Timeout: ${info['timeout']}

.. py:method:: is_within_tolerance(transform_data, tolerance, target_x, target_y, target_z=0.0)

   Check if a position (from transform data) is within a given tolerance of a target point.

   This is a utility method for navigation and positioning tests.

   **Parameters:**
   - **transform_data** (dict): Transform data dictionary from get_transform method
   - **tolerance** (float): Maximum allowed distance from target in meters
   - **target_x** (float): Target X coordinate
   - **target_y** (float): Target Y coordinate
   - **target_z** (float): Target Z coordinate (default: 0.0)

   **Returns:** True if position is within tolerance of target, False otherwise

   **Example:**
   .. code-block:: robot

      ${transform}=    Get Transform    map    base_link    timeout=1.0
      ${within}=    Is Within Tolerance    ${transform}    2.0    3.0    0.0    0.5
      Should Be True    ${within}
