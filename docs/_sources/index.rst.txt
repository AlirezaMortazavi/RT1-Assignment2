.. RT1-Assignment2 documentation master file, created by
   sphinx-quickstart on Fri May 24 19:46:11 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to rt1_assignment2's documentation!
==========================================
This documentation provides a comprehensive guide to the `rt_assignment2` package. The package includes multiple nodes implemented in Python, each performing specific tasks essential for the assignment. Below you will find detailed documentation for each node, including their functionality, usage, and integration within the overall system.

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


node_a
===========================
The `node_a` script is a core component of the `rt_assignment2` package. This node handles essential operations that form the backbone of the assignment's functionality. It interacts with the ROS action server to send movement goals and cancel them when required. Additionally, it subscribes to odometry data to update the robot's position and velocity.

.. automodule:: scripts.node_a
  :members:

.. _scripts.node_b:

node_b
===========================
The `node_b` script provides a service that allows for the retrieval of the last desired x and y positions. This functionality is essential for tracking and managing the robot's movement goals. By storing the desired positions as ROS parameters, this node ensures that other components of the system can access the last target positions whenever needed.

.. module:: scripts.node_b

.. automodule:: scripts.node_b
   :members:
   :undoc-members:
   :show-inheritance:
   
.. _scripts.node_c:

node_c
===========================
The `node_c` script logs real-time data of the robot's position, velocity, and other relevant metrics. This node is crucial for monitoring the robot's current state and performance, ensuring that the system has access to up-to-date information at all times. It subscribes to position and velocity topics, calculates distances and average velocities, and provides this information through a ROS service. 

.. module:: scripts.node_c

.. automodule:: scripts.node_c
   :members:
   :undoc-members:
   :show-inheritance: 
