#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    rospy.sleep(1)
    print msg.pose.pose

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)

rospy.spin()
