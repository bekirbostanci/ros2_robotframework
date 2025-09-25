Installation
============

This page provides detailed installation instructions for the ROS2 RobotFramework library.

Prerequisites
-------------

Before installing the library, ensure you have the following prerequisites:

.. panels::
   :container: +full-width text-center
   :column: col-lg-4 col-md-6 col-sm-12

   .. panel::
      :body:

      **ROS2 Installation**
      
      ROS2 (tested with Jazz and Humble)

   .. panel::
      :body:

      **Python Version**
      
      Python 3.8 or higher

   .. panel::
      :body:

      **Message Packages**
      
      All ROS2 message packages (std_msgs, geometry_msgs, etc.)

Installation Methods
--------------------

.. tabs::

   .. tab:: From Source (Recommended)

      .. code-block:: bash

         # Clone the repository
         git clone https://github.com/bekirbostanci/ros2_robotframework.git
         cd ros2_robotframework
         
         # Install in development mode
         pip install -e .

   .. tab:: From PyPI (when published)

      .. code-block:: bash

         # Install from PyPI
         pip install ros2-robotframework

   .. tab:: With Additional Features

      .. code-block:: bash

         # Install with Navigation2 support
         pip install -e .[nav2]
         
         # Install with all optional dependencies
         pip install -e .[nav2,dev,test,docs]

Dependencies
-----------

The library has the following dependencies:

Core Dependencies
~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Package
     - Description
   * - robotframework
     - Robot Framework core library
   * - rclpy
     - ROS2 Python client library
   * - std-msgs
     - Standard ROS2 message types
   * - geometry-msgs
     - Geometry message types
   * - sensor-msgs
     - Sensor message types
   * - nav-msgs
     - Navigation message types
   * - tf2-msgs
     - TF2 message types
   * - tf2-geometry-msgs
     - TF2 geometry message types
   * - action-msgs
     - Action message types
   * - lifecycle-msgs
     - Lifecycle message types
   * - numpy
     - Numerical computing library
   * - typing-extensions
     - Type hints extensions
   * - PyYAML
     - YAML parsing library

Optional Dependencies
~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Package
     - Description
   * - nav2-msgs
     - Navigation2 message types (for Navigation2 support)
   * - nav2-common
     - Navigation2 common utilities
   * - pytest
     - Testing framework (for development)
   * - black
     - Code formatter (for development)
   * - flake8
     - Linter (for development)
   * - mypy
     - Type checker (for development)
   * - sphinx
     - Documentation generator (for documentation)

Verification
-----------

After installation, verify that the library is working correctly:

.. code-block:: bash

   # Check if the library is installed
   python -c "import ros2_client; print('ROS2 Client Library installed successfully')"
   
   # Check if Navigation2 client is available
   python -c "import nav2_client; print('Navigation2 Client Library installed successfully')"
   
   # Run a simple test
   robot examples/basics/ros2_basic_test.robot

Troubleshooting
---------------

Common Installation Issues
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. panels::
   :container: +full-width text-center
   :column: col-lg-6 col-md-12

   .. panel::
      :body:

      **ROS2 not found**
      
      Ensure ROS2 is installed and sourced:
      
      .. code-block:: bash
      
         source /opt/ros/humble/setup.bash
         # or for your specific ROS2 distribution

   .. panel::
      :body:

      **Permission errors**
      
      Check file permissions and user access:
      
      .. code-block:: bash
      
         sudo chown -R $USER:$USER /path/to/your/workspace

   .. panel::
      :body:

      **Missing dependencies**
      
      Install missing ROS2 packages:
      
      .. code-block:: bash
      
         sudo apt install ros-humble-std-msgs ros-humble-geometry-msgs

   .. panel::
      :body:

      **Python version issues**
      
      Ensure you're using Python 3.8 or higher:
      
      .. code-block:: bash
      
         python3 --version
         # Should show 3.8 or higher

Debug Tips
~~~~~~~~~~

1. **Enable debug logging** in Robot Framework
2. **Check ROS2 environment variables** with `echo $ROS_DISTRO`
3. **Verify ROS2 installation** with `ros2 --help`
4. **Test individual commands manually** before using in Robot Framework

Next Steps
----------

After successful installation, you can:

- Read the :doc:`quickstart` guide to get started
- Explore the :doc:`examples/index` for practical examples
- Check the :doc:`user_guide/overview` for detailed usage information
