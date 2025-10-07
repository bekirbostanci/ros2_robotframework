Quick Start Guide
=================

Get started with ROS2 RobotFramework in a few simple steps.

Prerequisites
-------------

- ROS2 installed and sourced
- Python 3.8 or higher
- Library installed (see :doc:`installation`)

Your First Test
---------------

1. **Create a test file** (e.g., `my_test.robot`):

   .. code-block:: robot

      *** Settings ***
      Library    ROS2ClientLibrary

      *** Test Cases ***
      Test Basic ROS2 Operations
          # List available topics
          ${topics}=    List Topics
          Log    Available topics: ${topics}
          
          # Check if a topic exists
          ${exists}=    Topic Exists    /chatter
          Should Be True    ${exists}

2. **Run the test**:

   .. code-block:: bash

      robot my_test.robot

Basic Operations
----------------

**Topic Operations**
   - List topics: ``${topics}=    List Topics``
   - Check topic exists: ``${exists}=    Topic Exists    /chatter``
   - Get topic info: ``${info}=    Get Topic Info    /chatter``

**Node Operations**
   - List nodes: ``${nodes}=    List Nodes``
   - Check node exists: ``${exists}=    Node Exists    /talker``
   - Wait for node: ``${available}=    Wait For Node    /talker    timeout=10.0``

**Service Operations**
   - List services: ``${services}=    List Services``
   - Check service available: ``${exists}=    Service Available    /add_two_ints``
   - Call service: ``${response}=    Call Service    /add_two_ints    example_interfaces/srv/AddTwoInts    "a: 5, b: 3"``

Running Examples
----------------

Try the included examples:

.. code-block:: bash

   # Run basic ROS2 test
   robot examples/basics/ros2_basic_test.robot
   
   # Run Navigation2 test
   robot examples/medium/nav2_simple_monitoring_test.robot

Next Steps
----------

- Explore more examples in :doc:`examples/index`
- Learn about advanced features in :doc:`about`
