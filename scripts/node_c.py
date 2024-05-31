#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

def ros_node_init():
    """
    Initialize the ROS node, service, and subscriber.

    This function initializes the ROS node named 'info_service'. It sets up a service named 
    'info_service' using the custom service type Ave_pos_vel. It also subscribes to the 
    '/pos_vel' topic to receive position and velocity data.

    Side Effects:
        - Initializes a ROS node.
        - Sets up a ROS service.
        - Sets up a ROS subscriber.
    """
    rospy.init_node('info_service')
    rospy.Service("info_service", Ave_pos_vel, handle_info_request)
    rospy.Subscriber("/pos_vel", Vel, calculate_distance_and_average_velocity)

def calculate_distance_and_average_velocity(msg):
    """
    Callback function for the subscriber to calculate distance and average velocity.

    This function calculates the distance between the desired and actual positions and 
    the average velocity over a specified window size. It updates these values on the 
    ROS parameter server.

    Args:
        msg (assignment_2_2023.msg.Vel): The received message containing position and velocity data.

    Side Effects:
        - Updates parameters on the ROS parameter server.
        - Logs distance and average velocity information.
    """
    des_x = rospy.get_param('/des_pos_x')
    des_y = rospy.get_param('/des_pos_y')
    velocity_window_size = rospy.get_param('/window_size')

    actual_x = msg.pos_x
    actual_y = msg.pos_y

    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]
    distance = math.dist(des_coordinates, actual_coordinates)

    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]

    average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

    rospy.set_param('/distance', distance)
    rospy.set_param('/average_vel_x', average_vel_x)

    rospy.loginfo(f"Distance: {distance}, Average Velocity X: {average_vel_x}")

def handle_info_request(_):
    """
    Callback function for the service to provide distance and average velocity.

    This function retrieves the distance and average velocity from the ROS parameter 
    server and returns them in a service response.

    Returns:
        assignment_2_2023.srv.Ave_pos_velResponse: The response object containing the distance 
        and average velocity.
    """
    distance = rospy.get_param('/distance')
    average_vel_x = rospy.get_param('/average_vel_x')
    return Ave_pos_velResponse(distance, average_vel_x)

def run_ros_node():
    """
    Keep the ROS node running.

    This function keeps the ROS node active and responsive to service calls and subscriber messages.
    """
    rospy.spin()

if __name__ == "__main__":
    ros_node_init()

    while not rospy.is_shutdown():
        run_ros_node()

