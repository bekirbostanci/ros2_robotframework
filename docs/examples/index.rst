Examples
========

This section contains practical examples demonstrating how to use the ROS2 RobotFramework library.

Basic Examples
--------------

These examples cover fundamental ROS2 operations and are perfect for getting started.

.. toctree::
   :maxdepth: 1

   basic_ros2_test
   ros2_native_functions
   nav2_basic_test

Medium Examples
---------------

These examples demonstrate more complex scenarios and real-world applications.

.. toctree::
   :maxdepth: 1

   nav2_simple_monitoring
   nav2_turtlebot
   pyrobo_example

Example Categories
==================

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **Basic ROS2 Operations**
      
      Topics, services, nodes, parameters

   .. panel::
      :body:

      **Launch and Run Operations**
      
      Starting and managing ROS2 processes

   .. panel::
      :body:

      **Navigation2 Operations**
      
      Navigation, costmaps, recovery behaviors

   .. panel::
      :body:

      **Native ROS2 Operations**
      
      Direct Python API usage

   .. panel::
      :body:

      **Multi-robot Scenarios**
      
      Testing with multiple robots

   .. panel::
      :body:

      **Real-world Applications**
      
      Complete system testing

Running Examples
================

To run any example:

.. code-block:: bash

   # Run a specific example
   robot examples/basics/ros2_basic_test.robot
   
   # Run with verbose output
   robot -v examples/basics/ros2_basic_test.robot
   
   # Run and generate detailed report
   robot --outputdir results examples/basics/ros2_basic_test.robot

Example Structure
=================

Each example follows a consistent structure:

.. code-block:: robot

   *** Settings ***
   Documentation    Example description
   Library          ROS2ClientLibrary    # or Nav2ClientLibrary
   
   *** Test Cases ***
   Test Example Name
       # Setup
       # Test steps
       # Verification
       # Cleanup

Best Practices in Examples
=========================

.. panels::
   :container: +full-width text-center
   :column: col-lg-6 col-md-12

   .. panel::
      :body:

      **Always Clean Up**
      
      Examples demonstrate proper cleanup of processes and resources

   .. panel::
      :body:

      **Use Timeouts**
      
      All operations include appropriate timeout values

   .. panel::
      :body:

      **Handle Errors**
      
      Examples show proper error handling and validation

   .. panel::
      :body:

      **Document Steps**
      
      Each step is clearly documented with comments

Contributing Examples
=====================

If you have an interesting example that demonstrates the library's capabilities, we'd love to include it! Please:

1. Follow the existing example structure
2. Include comprehensive documentation
3. Test your example thoroughly
4. Submit a pull request

For more information, see the :doc:`../development/contributing` guide.
