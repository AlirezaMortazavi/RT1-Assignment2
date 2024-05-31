#!/usr/bin/env python3

"""
.. module:: scripts.node_c
   :platform: Unix
   :synopsis: Python module for the info_service node.
.. moduleauthor:: Seyed Alireza Mortazavi

Subscribers:
    - /pos_vel: Subscribes to the position and velocity information.

Services:
    - info_service: Provides real-time info about distance and velocity.

"""

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

def ros_node_init():
    """
    Initialize the ROS node, service, and subscriber.

    This function initializes a ROS node named 'info_service', which provides a service named 
    'info_service' using the custom service type 'Ave_pos_vel'. It also subscribes to the 
    '/pos_vel' topic to receive messages containing position and velocity data. 

    Side Effects:
        - Initializes a ROS node.
        - Sets up a ROS service named 'info_service'.
        - Sets up a ROS subscriber to the '/pos_vel' topic.
    """
    rospy.init_node('info_service')
    rospy.Service("info_service", Ave_pos_vel, handle_info_request)
    rospy.Subscriber("/pos_vel", Vel, calculate_distance_and_average_velocity)
    rospy.loginfo("Info service node initialized and service and subscriber set up.")

def calculate_distance_and_average_velocity(msg):
    """
    Callback function for the subscriber to calculate distance and average velocity.

    This function is triggered upon receiving a message on the '/pos_vel' topic. It calculates the 
    Euclidean distance between the desired target positions and the actual positions received in the 
    message. Additionally, it calculates the average velocity over a specified window size. These 
    values are updated on the ROS parameter server.

    Args:
        msg (assignment_2_2023.msg.Vel): The received message containing position and velocity data.

    Side Effects:
        - Updates the '/distance' and '/average_vel_x' parameters on the ROS parameter server.
        - Logs the calculated distance and average velocity information.
    """
    # Retrieve desired positions from the parameter server
    des_x = rospy.get_param('/des_pos_x')
    des_y = rospy.get_param('/des_pos_y')
    # Retrieve the size of the window for averaging velocity
    velocity_window_size = rospy.get_param('/window_size')

    # Get actual positions from the message
    actual_x = msg.pos_x
    actual_y = msg.pos_y

    # Calculate the Euclidean distance between desired and actual positions
    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]
    distance = math.dist(des_coordinates, actual_coordinates)

    # Calculate the average velocity over the specified window size
    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]
    average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

    # Update parameters on the ROS parameter server
    rospy.set_param('/distance', distance)
    rospy.set_param('/average_vel_x', average_vel_x)

    # Log the calculated information
    rospy.loginfo(f"Distance: {distance}, Average Velocity X: {average_vel_x}")

def handle_info_request(_):
    """
    Handle service requests to provide distance and average velocity.

    This function is triggered when a service request is made to the 'info_service' service.
    It retrieves the previously calculated distance and average velocity from the ROS parameter
    server and returns these values in the service response.

    Returns:
        assignment_2_2023.srv.Ave_pos_velResponse: The response object containing the distance 
        and average velocity.
    """
    # Retrieve distance and average velocity from the parameter server
    distance = rospy.get_param('/distance')
    average_vel_x = rospy.get_param('/average_vel_x')

    # Return the retrieved values in the service response
    return Ave_pos_velResponse(distance, average_vel_x)

def run_ros_node():
    """
    Keep the ROS node running.

    This function keeps the ROS node active and responsive to incoming service calls and subscriber messages. 
    It effectively maintains the node's operational state until it is manually terminated.

    Side Effects:
        - Keeps the ROS node running and processing callbacks.
    """
    rospy.spin()

if __name__ == "__main__":
    ros_node_init()

    while not rospy.is_shutdown():
        run_ros_node()
