import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class rosact(object):
    def __init__(self):
        rospy.init_node('act')
        self._pub = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)
		rospy.sleep(1)

    def write(self,linear,angular):
        self._pub.publish(Twist(Vector3(linear[0],linear[1],linear[2]),
                   Vector3(angular[0],angular[1],angular[2])))
		rospy.sleep(1)

	def main():
	    act=rosact()
	    act.write([5,0,0],[0,0,0])


if __name__== '__main__':
    main()
