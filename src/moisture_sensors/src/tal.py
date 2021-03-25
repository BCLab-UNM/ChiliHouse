#!/usr/bin/env python
# this is a dummy test node. (this acts as a swarmie)
# TODO: need to delete this class
import rospy
from std_msgs.msg import Int64

def talker():
    pub = rospy.Publisher('chatter', Int64, queue_size=10)
    rospy.init_node('tal', anonymous=True)
    rate = rospy.Rate(0.7)
    counter = 1
    while not rospy.is_shutdown():
        pub.publish(counter)
        counter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass