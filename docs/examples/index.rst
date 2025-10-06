Examples
========

Practical examples for using ROS2 RobotFramework.

Basic Examples
--------------

Simple examples to get started:

.. toctree::
   :maxdepth: 1

   basic_ros2_test
   ros2_native_functions
   nav2_basic_test

Advanced Examples
-----------------

More complex scenarios:

.. toctree::
   :maxdepth: 1

   nav2_simple_monitoring
   nav2_turtlebot
   pyrobo_example

Running Examples
----------------

Run any example with:

.. code-block:: bash

   # Run basic ROS2 test
   robot examples/basics/ros2_basic_test.robot
   
   # Run with verbose output
   robot -v examples/basics/ros2_basic_test.robot
   
   # Generate detailed report
   robot --outputdir results examples/basics/ros2_basic_test.robot

Example Structure
-----------------

All examples follow this structure:

.. code-block:: robot

   *** Settings ***
   Documentation    Example description
   Library          ROS2ClientLibrary
   
   *** Test Cases ***
   Test Example Name
       # Setup
       # Test steps
       # Verification
       # Cleanup
