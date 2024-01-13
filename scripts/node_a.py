#!/usr/bin/env python3

# Import Libraries
import rospy
from nav_msgs.msg import Odometry
import actionlib
from assignment_2_2023.msg import Vel, PlanningAction, PlanningGoal


def ros_components_init():
    """Initialize ROS components - publisher and action client."""
    # Initialize the ROS publisher for position and velocity
    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)

    # Initialize the ROS action client for the 'reaching_goal' action
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)

    # Wait for the action server to be ready
    client.wait_for_server()

    # Initialize the goal_cancelled flag to True
    goal_cancelled = True
    return pub, client, goal_cancelled


def user_input():
    """Prompt the user for input ('s' to set a new goal, 'c' to cancel the current goal)."""
    return input("Press 's' to set a new goal or 'c' to cancel the current goal: ")


def process_goal_commands(pub, client, goal_cancelled):
    """Handle goal commands - set new goals or cancel existing goals based on user input."""
    while not rospy.is_shutdown():
        # Subscribe to /odom topic and publish position and velocity
        rospy.Subscriber("/odom", Odometry, publish_position_velocity, pub)

        # Get user command
        command = user_input()

        # Get current target position
        target_pos_x = rospy.get_param('/des_pos_x')
        target_pos_y = rospy.get_param('/des_pos_y')

        # Create a new goal with the current target position
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_pos_x
        goal.target_pose.pose.position.y = target_pos_y
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_pos_x, target_pos_y)

        if command == 's':
            try:
                # Get new goal coordinates from user
                input_x = float(input("Enter the x-coordinate for the new goal: "))
                input_y = float(input("Enter the y-coordinate for the new goal: "))
            except ValueError:
                rospy.logwarn("Invalid input. Please enter a valid number.")
                continue

            # Update target position parameters and the goal
            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y

            # Send the new goal to the action server
            client.send_goal(goal)
            goal_cancelled = False

        elif command == 'c':
            if not goal_cancelled:
                # Cancel the current goal if there is one
                goal_cancelled = True
                client.cancel_goal()
                rospy.loginfo("Current goal has been cancelled")
            else:
                rospy.loginfo("No active goal to cancel")
        else:
            rospy.logwarn("Invalid command. Please enter 's' or 'c'.")

        rospy.loginfo("Last received goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x,
                      goal.target_pose.pose.position.y)


def publish_position_velocity(msg, pub):
    """Publish position and velocity based on the received odometry message."""
    # Extract current position and velocity from the Odometry message
    current_pos = msg.pose.pose.position
    current_vel_linear = msg.twist.twist.linear
    current_vel_angular = msg.twist.twist.angular

    # Create a new Vel message with the current position and velocity
    pos_and_vel = Vel()
    pos_and_vel.pos_x = current_pos.x
    pos_and_vel.pos_y = current_pos.y
    pos_and_vel.vel_x = current_vel_linear.x
    pos_and_vel.vel_z = current_vel_angular.z

    # Publish the Vel message
    pub.publish(pos_and_vel)


def main():
    # Initialize the node and start handling goal commands
    rospy.init_node('set_target_client')
    pub, client, goal_cancelled = ros_components_init()
    process_goal_commands(pub, client, goal_cancelled)


if __name__ == '__main__':
    main()

