# RT1-Assignment2

## Overview

This project implements a Robot Operating System (ROS) with three nodes: Node A, Node B, and Node C. These nodes work collaboratively to set and cancel goals, provide information about the last desired positions, and calculate distance and average velocity.

## Nodes

### Node A: Goal Setting and Cancelation

- **File:** `node_a.py`
- **Description:** Node A allows users to set new goals or cancel existing goals for a robot. It subscribes to odometry data, publishes position and velocity information, and interacts with the 'reaching_goal' action server.
 
![rt2](https://github.com/AlirezaMortazavi/RT1-Assignment2/assets/69080319/d2c28dd0-7ec1-42e4-84ba-5785fb4c5969)

### Node B: Last Desired Position Service

- **File:** `node_b.py`
- **Description:** Node B provides a service that responds with the last desired positions (x and y) stored in the ROS parameter server. It complements the goal-setting functionality of Node A.

### Node C: Information Calculation

- **File:** `node_c.py`
- **Description:** Node C calculates and provides information about the distance between desired and actual positions, as well as the average velocity. It offers a service ('info_service') and subscribes to position and velocity data.

## Installation and Dependencies

1. Ensure you have a working ROS installation.
2. Clone the repository:

    ```bash
    git clone https://github.com/AlirezaMortazavi/RT1-Assignment2.git
    cd RT1-Assignment2
    ```
