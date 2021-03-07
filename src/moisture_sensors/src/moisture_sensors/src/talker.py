#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float64

class MoistureSensor:
    # decay rate for plant
    # need to chagnge this value to real decay rate
    moisture_decay_rate = 1
    moisture_arr = []
    pots=[{ 'z_value_plant':random.randint(50, 100), 
            'z_value_soil': random.randint(0, 45),  
            'z_value_temp':random.randint(0, 10)}
            for num in range(0,10)]


    # gets the moisture level from the pots and 
    # appends to the moisture_arr
    def get_pot_moisture(self):
        pots = self.pots
        for x in range(len(pots)):
            z_value_plant = pots[x]['z_value_plant']
            z_value_soil = pots[x]['z_value_soil']
            # moisture level is the difference between 
            # z value of plant and z value of soil
            moisture_level = abs(z_value_soil - z_value_plant)
            self.moisture_arr.append(moisture_level)
        return self.moisture_arr
        

    # gets moinsture level from moisture_arr and
    # decreases each moisture level by moisture_decay value
    def moisture_decay(self, event=None):
        for x in range(len(self.pots)):
            self.moisture_arr[x] = self.moisture_arr[x] - self.moisture_decay_rate


    # Every 1 second publishes moisture of pot
    def publish_pot_moisture(self, event=None):
        self.moisture_publisher = rospy.Publisher("/moisture", Float64, queue_size=10)
        rate = rospy.Rate(1)
        msg = Float64
        while not rospy.is_shutdown():
            ms.moisture_decay()
            for i in range(len(self.moisture_arr)):
                print(self.moisture_arr[i])
                msg = self.moisture_arr[i]
                self.moisture_publisher.publish(msg)
                i = i+1
                rate.sleep()
            # this value is just for debugging purpose 
            # it indicates starting of updated array
            msg = 111111111
            self.moisture_publisher.publish(msg)
            print('\n')


if __name__ == '__main__':
    rospy.init_node("moistures_sensors_node")

    # Create an instance of Moisture sensor
    ms = MoistureSensor()

    ms.get_pot_moisture()
    
    ms.publish_pot_moisture()
        