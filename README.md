# Research Track 1 Assignment 2
# Seyed Alireza Mortazavi (S6136275)
## Overview

This project implements a Robot Operating System (ROS) with three nodes: Node A, Node B, and Node C. These nodes work collaboratively to set and cancel goals, provide information about the last desired positions, and calculate distance and average velocity.

## Nodes

### Node A: Goal Setting and Cancelation

- **File:** `node_a.py`
- **Description:** Node A enables users to set new goals or cancel existing goals for a robot within a ROS (Robot Operating System) environment. It subscribes to odometry data, publishes position and velocity information, and interacts with the 'reaching_goal' action server. Users can input commands to set new goals or cancel the current goal, providing a dynamic way to control the robot's movements and goals.
 
![rt2](https://github.com/AlirezaMortazavi/RT1-Assignment2/assets/69080319/d2c28dd0-7ec1-42e4-84ba-5785fb4c5969)

### Node B: Last Desired Position Service

- **File:** `node_b.py`
- **Description:** Node B provides a service that responds with the last desired positions (x and y) stored in the ROS parameter server. It complements the goal-setting functionality of Node A by allowing other nodes or external components to retrieve information about the last set goals. This service-oriented architecture enhances the modularity and flexibility of the overall ROS control system.

### Node C: Information Calculation

- **File:** `node_c.py`
- **Description:** Node C calculates and provides information about the distance between desired and actual positions, as well as the average velocity. This node offers a service named 'info_service' that provides external entities with information about the calculated distance and average velocity. Node C enhances the overall situational awareness of the robot and can be valuable for decision-making processes.
  
## Installation 

1. Ensure you have a working ROS installation.
2. Clone thIS repository:

    ```bash
    git clone https://github.com/AlirezaMortazavi/RT1-Assignment2.git
    ```
