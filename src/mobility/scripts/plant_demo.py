#! /usr/bin/env python

import math 
import rospy 
from mobility.swarmie import swarmie

def main():
    rospy.sleep(5)
    swarmie.pump_backward(True)
    rospy.sleep(5)
    swarmie.pump_backward(False)
    prev_heading = swarmie.get_odom_location().get_pose().theta
    for _ in range(0,3):
        swarmie.drive(-.4,ignore=Obstacle.IS_SONAR)
        swarmie.turn(-math.pi/2, ignore=Obstacle.IS_SONAR)
        swarmie.drive(.2,ignore=Obstacle.IS_SONAR)
        swarmie.set_heading(prev_heading, ignore=Obstacle.IS_SONAR)
        swarmie.drive(.4,ignore=Obstacle.IS_SONAR)
        swarmie.pump_forward(True)
        rospy.sleep(5)
        swarmie.pump_forward(False)
    swarmie.drive(-.4,ignore=Obstacle.IS_SONAR)

if __name__ == '__main__' :
    swarmie.start(node_name='plant_demo')
    main()



