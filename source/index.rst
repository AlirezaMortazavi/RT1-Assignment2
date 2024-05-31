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


node_a!
**************************************
The `node_a` script is a core component of the `rt_assignment2` package. This node handles essential operations that form the backbone of the assignment's functionality.

rt1_assignmrnt2 Module
===========================

.. automodule:: scripts.node_a
  :members:

.. _scripts.node_b:

node_b
===========================
The `node_b` script provides a service that allows for the retrieval of the last desired x and y positions. This functionality is essential for tracking and managing the robot's movement goals.

.. module:: scripts.node_b

.. automodule:: scripts.node_b
   :members:
   :undoc-members:
   :show-inheritance:
   
.. _scripts.node_c:

node_c
===========================
The `node_c` script implements the `info_service` node, which logs real-time data of the robot's position, velocity, and other relevant metrics. This node plays a critical role in ensuring that the system has access to necessary data at all times.  

.. module:: scripts.node_c

.. automodule:: scripts.node_c
   :members:
   :undoc-members:
   :show-inheritance: 
