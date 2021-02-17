#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

class MoistureSensor:

    moisture_decay_rate = 1
    arr = [1, 2, 3, 4, 5]
    moisture_arr = []
    pots = {0: {"location": (1, 3), "moisture_level": 444},
	        1: {"location": (1, 2), "moisture_level": 35},
            2: {"location": (2, 2), "moisture_level": 67},
            3: {"location": (4, 2), "moisture_level": 54}}


    def get_pot_moisture(self):
        pots = self.pots
        moisture_arr = self.moisture_arr
        for x in range(len(pots)):
	    moisture_arr.append(pots[x]["moisture_level"])
	return moisture_arr


    def moisture_decay(self, event=None):
        moisture_decay_rate = self.moisture_decay_rate
        moisture_arr = self.moisture_arr
        pots = self.pots
        for x in range(len(pots)):
            updated_moisture = pots[x]["moisture_level"] - moisture_decay_rate
            moisture_arr[x] = updated_moisture


    # Every 1 second publishes moisture of pot
    def publish_pot_moisture(self, event=None):
        # self.moisture_decay()
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

    def read_moisture_sensor_data(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        self.temperature = 30.0


if __name__ == '__main__':
    rospy.init_node("your_sensor_node")

    # Create an instance of Temperature sensor
    ms = MoistureSensor()

    ms.get_pot_moisture()

    # Create a ROS Timer for reading data
    # rospy.Timer(rospy.Duration(1.0/10.0), ms.read_moisture_sensor_data)
    ms.read_moisture_sensor_data

    rate = rospy.Rate(10)

    # Create another ROS Timer for publishing data
    # rospy.Timer(rospy.Duration(1.0), ms.publish_pot_moisture)
    while not rospy.is_shutdown():
        ms.moisture_decay()
        ms.publish_pot_moisture()
        rate.sleep()
    
    # Don't forget this or else the program will exit
    # rospy.spin()

