#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
import actionlib
class ExplorationControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('exploration_control')

        # Set up subscriber for exploration state
        rospy.Subscriber('/exploration_state', Bool, self.exploration_state_callback)

        # Set up publisher for move_base commands
        self.move_base_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Set flag to indicate whether robot is exploring
        self.is_exploring = True

        rospy.loginfo('Exploration control initialized')

    def exploration_state_callback(self, msg):
        # Check if exploration state has changed
        if msg.data != self.is_exploring:
            self.is_exploring = msg.data
            if self.is_exploring:
                rospy.loginfo('Starting robot exploration')
                # TODO: call code to start robot exploration
                self.restart_robot()
            else:
                rospy.loginfo('Stopping robot exploration')
                # TODO: call code to stop robot exploration
                self.stop_robot()


    def restart_robot(self):
        # Create a new MoveBaseActionGoal message to stop the robot
        stop_msg = MoveBaseActionGoal()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.goal.target_pose.header.stamp = rospy.Time.now()
        stop_msg.goal.target_pose.header.frame_id = "map"
        stop_msg.goal.target_pose.pose.position.x = 0.0
        stop_msg.goal.target_pose.pose.position.y = 0.0
        stop_msg.goal.target_pose.pose.position.z = 0.0
        stop_msg.goal.target_pose.pose.orientation.x = 0.0
        stop_msg.goal.target_pose.pose.orientation.y = 0.0
        stop_msg.goal.target_pose.pose.orientation.z = 0.0
        stop_msg.goal.target_pose.pose.orientation.w = 1.0

        # Publish the stop message to move_base
        self.move_base_pub.publish(stop_msg)
                      

    def stop_robot(self):
        # Stop the robot from exploring
        self.client.cancel_all_goals()

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()