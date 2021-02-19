#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import GetModelState

class MoistureSensor:
    moisture_decay_rate = 1
    moisture_arr = []
    
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # Note: it says pose and moisture_level is undefined but should be okay!?
    pots=[{pose:model_coordinates("plant_"+num, "world").pose.position,moisture_level:random.randint(20,500)} for num in range(0,143)]
    
    # pots = {0: {"location": (1, 3), "moisture_level": 66},
	#         1: {"location": (1, 2), "moisture_level": 35},
    #         2: {"location": (2, 2), "moisture_level": 67},
    #         3: {"location": (4, 2), "moisture_level": 54}}


    # gets the moisture level from the pots and 
    # appends to the moisture_arr
    def get_pot_moisture(self):
        for x in range(len(self.pots)):
	        self.moisture_arr.append(self.pots[x].moisture_level)    
        return self.moisture_arr

    # gets moinsture level from moisture_arr and
    # decreases each moisture level by moisture_decay value
    def moisture_decay(self, event=None):
        for x in range(len(self.pots)):
            self.moisture_arr[x] = self.moisture_arr[x] - self.moisture_decay_rate


    # Every 1 second publishes moisture of pot
    def publish_pot_moisture(self, event=None):
        msg = Float64MultiArray()
        msg.data = self.moisture_arr
        self.temperature_publisher.publish(msg)


    def read_moisture_sensor_data(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        self.temperature = get_pot_moisture()


    def __init__(self):
        # Create a ROS publisher
        self.temperature_publisher = rospy.Publisher("/moisture", Float64MultiArray, queue_size=1)

        # Initialize temperature data
        self.temperature = []

    # def read_moisture_sensor_data(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        # self.temperature = 30.0


if __name__ == '__main__':
    rospy.init_node("your_sensor_node")

    # Create an instance of Temperature sensor
    ms = MoistureSensor()

    ms.get_pot_moisture()

    # Create a ROS Timer for reading data
    # rospy.Timer(rospy.Duration(1.0/10.0), ms.read_moisture_sensor_data)
    #ms.read_moisture_sensor_data

    rate = rospy.Rate(0.8)

    # Create another ROS Timer for publishing data
    rospy.Timer(rospy.Duration(1.0), ms.publish_pot_moisture)
    while not rospy.is_shutdown():
        ms.moisture_decay()
        ms.publish_pot_moisture()
        rate.sleep()
    
    # Don't forget this or else the program will exit
    # rospy.spin()
