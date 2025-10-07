Installation
============

Simple installation instructions for ROS2 RobotFramework.

Prerequisites
-------------

- ROS2 installed and sourced
- Python 3.8 or higher
- Basic ROS2 message packages

Installation
------------

**From Source**

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/bekirbostanci/ros2_robotframework.git
   cd ros2_robotframework
   
   # Install in development mode
   pip install -e .

**From PyPI**

.. code-block:: bash

   pip install ros2-robotframework

Verification
-----------

Test your installation:

.. code-block:: bash

   # Check if installed
   python -c "import ros2_client; print('ROS2 Client installed')"
   
   # Run a test
   robot examples/basics/ros2_basic_test.robot

Troubleshooting
---------------

**Common Issues:**

- **ROS2 not found**: Make sure ROS2 is sourced: ``source /opt/ros/humble/setup.bash``
- **Missing packages**: Install ROS2 packages: ``sudo apt install ros-humble-std-msgs``
- **Python version**: Ensure Python 3.8+: ``python3 --version``

Next Steps
----------

- Read the :doc:`quickstart` guide
- Try the :doc:`examples/index`
