#!/usr/bin/env python
import rospy
import talker
from moisture_sensors.msg import moisture_msg


def updatePotsValue():
    ms = talker.MoistureSensor()
    ms.setPotsValue()
    # print(ms.getPots())
    # print('hello')


# def callback(data):
#     rospy.loginfo(data)
#     updatePotsValue()
                
    
def listener():
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber("moisture", moisture_msg, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    #listener()
    listener()
    counter = 0
    while (counter != 10):
        print(counter)
        counter += 1
        if (counter == 5):
            updatePotsValue()
    
    