#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int16
count = 0
def talker():
    global count
    pub= rospy.Publisher('mylimit', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
    	# rospy.loginfo(count)
    	pub.publish(count)
        print(count)
        rate.sleep()
        count+=1
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

