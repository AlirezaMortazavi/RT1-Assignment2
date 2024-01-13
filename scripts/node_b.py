#!/usr/bin/env python3

import rospy
from assignment_2_2023.srv import Input, InputResponse

# Function to initialize the ROS node and provide the service
def ros_node_init():
    # Initialize the ROS node with the name 'last_target_service'
    rospy.init_node('last_target_service')
    # Log information that the ROS node has been initialized
    rospy.loginfo("Last target node initialized")

    # Provide a service named 'input', using the custom service type Input
    rospy.Service('input', Input, handle_input_request)


# Function to handle the service callback
def handle_input_request(request):
    # Create a response message
    response = InputResponse()

    # Set the x and y inputs in the response to the last desired positions
    # Obtain last desired x and y positions from the ROS parameter server
    last_desired_x = rospy.get_param('/des_pos_x')
    last_desired_y = rospy.get_param('/des_pos_y')
    # Set the response message with the last desired positions
    response.input_x = last_desired_x
    response.input_y = last_desired_y

    # Return the response message
    return response


# Function to keep the ROS node running
def run_ros_node():
    # Keep the ROS node running
    rospy.spin()


# Main function
if __name__ == "__main__":
    # Initialize the ROS node and provide the service
    ros_node_init()

    # Keep the ROS node running
    run_ros_node()

