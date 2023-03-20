#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

rospy.init_node('exploration_state_publisher')

pub = rospy.Publisher('/exploration_state', Bool, queue_size=10)

rospy.loginfo('Exploration topic launched')

rospy.spin()