#!/usr/bin/env python3

"""
.. module:: scripts.node_b
   :platform: Unix
   :synopsis: Python module for Node B
.. moduleauthor:: Seyed Alireza Mortazavi

This code implements Node B.

Subscriber:
    - /node_b/pose: Subscribes to the pose topic.
"""

import rospy
from assignment_2_2023.srv import Input, InputResponse

def ros_node_init():
    """
    Initialize the ROS node and provide the 'input' service.

    This function sets up the ROS infrastructure for handling service requests regarding the 
    last desired target positions. Specifically, it initializes a ROS node named 'last_target_service'
    and advertises a service named 'input' that uses the custom service type 'Input'.

    The function also logs an informational message indicating that the node has been initialized.

    Side Effects:
        - Initializes a ROS node named 'last_target_service'.
        - Sets up a ROS service named 'input' that calls the `handle_input_request` callback.
    """
    rospy.init_node('last_target_service')
    rospy.loginfo("Last target node initialized")
    rospy.Service('input', Input, handle_input_request)

def handle_input_request(request):
    """
    Handle the service callback to respond with the last desired target positions.

    This callback function is triggered whenever a service request is made to the 'input' service. 
    It retrieves the last desired x and y target positions from the ROS parameter server and 
    constructs a response object containing these positions.

    Args:
        request (assignment_2_2023.srv.InputRequest): The request object from the service call, which is part 
                                                      of the 'Input' service type. The request is not used 
                                                      in this function as the response is generated based on 
                                                      stored parameters.

    Returns:
        assignment_2_2023.srv.InputResponse: The response object containing the last desired x and y positions.
    """
    response = InputResponse()
    last_desired_x = rospy.get_param('/des_pos_x')
    last_desired_y = rospy.get_param('/des_pos_y')
    response.input_x = last_desired_x
    response.input_y = last_desired_y
    return response

def run_ros_node():
   """
    Keep the ROS node running.

    This function enters a loop, keeping the ROS node active and responsive to incoming service calls. 
    It effectively keeps the program alive until it is manually terminated.

    Side Effects:
        - Keeps the ROS node running and processing callbacks.
    """
    rospy.spin()

if __name__ == "__main__":
    ros_node_init()
    run_ros_node()

