#!/usr/bin/env python3

import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x_goal, y_goal, theta):
    # Define a client for to send goal requests to the move_base server through a SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait for the action server to come up
    client.wait_for_server()

    goal = MoveBaseGoal()

    # Set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #Convert Euler angles to Quaternion
    roll = 0.0
    pitch = 0.0
    yaw = math.radians(theta)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    # Moving towards the goal
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    if client.get_result():
        rospy.loginfo("Goal execution done!")
    else:
        rospy.logwarn("The base failed to reach the goal for some reason")

if __name__ == '__main__':
    try:
        rospy.init_node('navigate_to_parking', anonymous=False)
        rospy.loginfo("Navigate to Parking Node Started")
        
        # Coordinates of the parking spot (example values)
        x_goal = -6.5
        y_goal = 5.5
        theta = -90
        
        move_to_goal(x_goal, y_goal, theta)

        while True:
            rospy.loginfo("Press 'n' to stop the node.")
            rospy.loginfo("Enter your desired goal: [x y theta]")
            try:
                x_goal, y_goal, theta = map(float, input().split())
                move_to_goal(x_goal, y_goal, theta)
            except ValueError:
                break
        
        rospy.loginfo("Navigate to Parking Node Stopped")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")