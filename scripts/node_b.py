#!/usr/bin/env python3

import rospy
from assignment_2_2023.srv import Input, InputResponse

def ros_node_init():
    """
    Initialize the ROS node and provide the 'input' service.

    This function initializes a ROS node named 'last_target_service' and provides a service
    named 'input' which uses the custom service type 'Input'. It also logs the initialization
    information.

    Side Effects:
        - Initializes a ROS node.
        - Sets up a ROS service.
    """
    rospy.init_node('last_target_service')
    rospy.loginfo("Last target node initialized")
    rospy.Service('input', Input, handle_input_request)

def handle_input_request(request):
    """
    Handle the service callback to respond with the last desired target positions.

    Args:
        request (assignment_2_2023.srv.InputRequest): The request object from the service call.

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

    This function keeps the ROS node active and responsive to service calls.
    """
    rospy.spin()

if __name__ == "__main__":
    ros_node_init()
    run_ros_node()

