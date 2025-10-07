Examples
========

Practical examples for using ROS2 RobotFramework.

.. toctree::
   :maxdepth: 2

   ros2/index
   nav2/index
   others/index

.. raw:: html

   <h2>Running Examples</h2>

Run any example with:

.. code-block:: bash

   # Run basic ROS2 test
   robot examples/basics/ros2_basic_test.robot
   
   # Run with verbose output
   robot -v examples/basics/ros2_basic_test.robot
   
   # Generate detailed report
   robot --outputdir results examples/basics/ros2_basic_test.robot

.. raw:: html

   <h2>Example Structure</h2>

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
