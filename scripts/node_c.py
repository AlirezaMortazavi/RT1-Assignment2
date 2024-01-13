#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

# Function to initialize the ROS node and set up services/subscribers
def ros_node_init():
    rospy.init_node('info_service')

    # Service for providing information (distance and average velocity)
    rospy.Service("info_service", Ave_pos_vel, handle_info_request)

    # Subscriber for receiving position and velocity data
    rospy.Subscriber("/pos_vel", Vel, calculate_distance_and_average_velocity)

# Callback function for the subscriber
def calculate_distance_and_average_velocity(msg):
    # Get parameters from the parameter server
    des_x = rospy.get_param('/des_pos_x')
    des_y = rospy.get_param('/des_pos_y')
    velocity_window_size = rospy.get_param('/window_size')

    # Extract actual position from the received message
    actual_x = msg.pos_x
    actual_y = msg.pos_y

    # Calculate the distance between desired and actual positions
    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]
    distance = math.dist(des_coordinates, actual_coordinates)

    # Calculate the average velocity
    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]

    average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

    # Update parameters on the parameter server
    rospy.set_param('/distance', distance)
    rospy.set_param('/average_vel_x', average_vel_x)

    # Log the information
    rospy.loginfo(f"Distance: {distance}, Average Velocity X: {average_vel_x}")

# Callback function for the service
def handle_info_request(_):
    # Retrieve parameters from the parameter server
    distance = rospy.get_param('/distance')
    average_vel_x = rospy.get_param('/average_vel_x')
    return Ave_pos_velResponse(distance, average_vel_x)

# Function to keep the ROS node running
def run_ros_node():
    rospy.spin()

# Main function
if __name__ == "__main__":
    ros_node_init()

    # Loop to keep the ROS node running until shutdown
    while not rospy.is_shutdown():
        run_ros_node()

