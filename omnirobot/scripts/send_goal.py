#!/usr/bin/env python3

import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys

def movebase_client(x, y, theta):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Waits until the action server has started up and started listening for goals
    client.wait_for_server()
    
    # Creates a new goal with the MoveBaseGoal constructor
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
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    
    # Sends the goal to the action server
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action
    wait = client.wait_for_result()
    
    # If the result doesn't arrive, assume the server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient communicate with the move_base server
        rospy.init_node('navigation_goals')
        rospy.loginfo("Navigate to Parking Node Started")
        
        # Parse input arguments
        if len(sys.argv) == 4:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            theta = float(sys.argv[3])
        else:
            x, y, theta = -6.5, 5.5, -90.0
        
        result = movebase_client(x, y, theta)
        if result:
            rospy.loginfo("Robot has arrived to the goal position")
        else:
            rospy.loginfo("The base failed for some reason")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
