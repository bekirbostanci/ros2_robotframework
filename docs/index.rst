Welcome to ROS2 RobotFramework
==============================

A simple Robot Framework library for testing ROS2 applications.

What is ROS2 RobotFramework?
-----------------------------

ROS2 RobotFramework is a Robot Framework library that makes it easy to test ROS2 applications. It provides simple keywords for common ROS2 operations like:

- Testing topics and services
- Checking nodes and parameters  
- Running navigation tests
- Launching ROS2 processes

Quick Start
-----------

1. Install the library (see :doc:`installation`)
2. Write your first test (see :doc:`quickstart`)
3. Run examples (see :doc:`examples/index`)

Simple Example
--------------

.. code-block:: robot

   *** Settings ***
   Library    ROS2ClientLibrary

   *** Test Cases ***
   Test ROS2 System
       # Check if a topic exists
       ${exists}=    Topic Exists    /chatter
       Should Be True    ${exists}
       
       # List all topics
       ${topics}=    List Topics
       Log    Available topics: ${topics}

Documentation
-------------

.. toctree::
   :maxdepth: 3

   installation
   quickstart
   examples/index
   api/index

