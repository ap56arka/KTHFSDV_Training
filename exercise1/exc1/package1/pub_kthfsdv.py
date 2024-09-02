#!/usr/bin/python3
import rospy
#chosen unsigned Int 64 bit because k is always positive and always increasing
from std_msgs.msg import UInt64


def pub_fun():
	pub  = rospy.Publisher('pal', UInt64, queue_size = 5)
	rospy.init_node('pal_node', anonymous = True)
	rospy.loginfo("Publishing started!")
	rate = rospy.Rate(20);
	n = 4 
	k = 0
	try:
		while not rospy.is_shutdown():
			k+=n
			if k > 2**64 - 1:
				#If run for sufficiently large amount of time, overflow will happen, catching the error here
				raise OverflowError("Value of k exceeded maximum UInt64 value")
				
			pub.publish(k)
			rate.sleep()
	except OverflowError as e:
		rospy.logerr(str(e))
		rospy.signal_shutdown("Node stopped due to overflow")
if __name__ == '__main__':
	try:
		pub_fun()
	except rospy.ROSInterruptException:
		pass
		
		
