#!/usr/bin/env python3

"""
.. module:: rt1_assignment2
   :platform: Unix
   :synopsis: Python module for the node_a
.. author:: Seyed Alireza Mortazavi

Subscriber:
    - /node_a/pose: Subscribes to the pose.
"""

# Import Libraries
import rospy
from nav_msgs.msg import Odometry
import actionlib
from assignment_2_2023.msg import Vel, PlanningAction, PlanningGoal

def ros_components_init():   
    """
    Initialize ROS components including the publisher and action client.

    This function sets up the necessary ROS components for the application:
    - A publisher for publishing the robot's position and velocity.
    - An action client for sending goals to the 'reaching_goal' action server.
    - A flag to indicate whether the current goal has been cancelled.

    Returns:
        tuple: A tuple containing:
            - pub (rospy.Publisher): The ROS publisher for position and velocity.
            - client (actionlib.SimpleActionClient): The ROS action client for the 'reaching_goal' action.
            - goal_cancelled (bool): A flag indicating if the current goal is cancelled.
    """
    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    goal_cancelled = True
    return pub, client, goal_cancelled

def user_input():
    """
    Prompt the user for input to either set a new goal or cancel the current goal.

    This function interacts with the user through the terminal to receive commands for setting or cancelling goals.

    Returns:
        str: The user's input command ('s' for setting a new goal, 'c' for cancelling the current goal).
    """
    return input("Press 's' to set a new goal or 'c' to cancel the current goal: ")

def process_goal_commands(pub, client, goal_cancelled):
    """
    Handle goal commands based on user input, either setting new goals or cancelling existing ones.

    This function continuously listens for user commands to either set a new goal or cancel the current goal. It subscribes
    to the /odom topic to receive the robot's current odometry information, which it uses to update the robot's position and velocity.

    Args:
        pub (rospy.Publisher): The ROS publisher for position and velocity.
        client (actionlib.SimpleActionClient): The ROS action client for the 'reaching_goal' action.
        goal_cancelled (bool): A flag indicating if the current goal is cancelled.
    """
    while not rospy.is_shutdown():
        rospy.Subscriber("/odom", Odometry, publish_position_velocity, pub)
        command = user_input()
        target_pos_x = rospy.get_param('/des_pos_x')
        target_pos_y = rospy.get_param('/des_pos_y')
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_pos_x
        goal.target_pose.pose.position.y = target_pos_y
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_pos_x, target_pos_y)

        if command == 's':
            try:
                input_x = float(input("Enter the x-coordinate for the new goal: "))
                input_y = float(input("Enter the y-coordinate for the new goal: "))
            except ValueError:
                rospy.logwarn("Invalid input. Please enter a valid number.")
                continue

            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y
            client.send_goal(goal)
            goal_cancelled = False

        elif command == 'c':
            if not goal_cancelled:
                goal_cancelled = True
                client.cancel_goal()
                rospy.loginfo("Current goal has been cancelled")
            else:
                rospy.loginfo("No active goal to cancel")
        else:
            rospy.logwarn("Invalid command. Please enter 's' or 'c'.")
        rospy.loginfo("Last received goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

def publish_position_velocity(msg, pub):
    """
    Publish position and velocity based on the received odometry message.

    This function extracts the robot's current position and velocity from the odometry message and publishes it using the provided publisher.

    Args:
        msg (nav_msgs.msg.Odometry): The odometry message containing position and velocity data.
        pub (rospy.Publisher): The ROS publisher for position and velocity.
    """
    current_pos = msg.pose.pose.position
    current_vel_linear = msg.twist.twist.linear
    current_vel_angular = msg.twist.twist.angular

    pos_and_vel = Vel()
    pos_and_vel.pos_x = current_pos.x
    pos_and_vel.pos_y = current_pos.y
    pos_and_vel.vel_x = current_vel_linear.x
    pos_and_vel.vel_z = current_vel_angular.z

    pub.publish(pos_and_vel)

def main():
    """
    Initialize the ROS node and start handling goal commands.

    This is the main entry point of the script. It initializes the ROS node, sets up the necessary components,
    and starts the process of handling user commands for setting and cancelling goals.
    """
    rospy.init_node('set_target_client')
    pub, client, goal_cancelled = ros_components_init()
    process_goal_commands(pub, client, goal_cancelled)

if __name__ == '__main__':
    main()
