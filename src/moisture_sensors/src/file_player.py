#!/usr/bin/env python

import rospy
from moisture_sensors.msg import moisture_msg

import csv
import matplotlib.pyplot as plt 


if __name__ == '__main__':
    rospy.init_node("moistures_sensors_file_player")
    
    
    # @TODO: read file name from ros param server
    with open('95Z_20210302_113755_Multi-PIP.csv', 'rb') as plant_csv:
        plants_data = [plant[0].split(',') for plant in csv.reader(plant_csv, delimiter=' ', quotechar='|')]

    sensor_pair_names = {plant[1] for plant in plants_data}  # get the names like D01SO & C04SO

    # remove any non 95k frequency points
    plants_data = [plant for plant in plants_data if plant[2]=='95000']

    for sensor_pair_name in sensor_pair_names:
        xes = [plant[0] for plant in plants_data if plant[1] == sensor_pair_name]  # get a list of the time times
        yes = [plant[3] for plant in plants_data if plant[1] == sensor_pair_name]  # get a list of the impedance values
        plt.plot(xes, yes, label = sensor_pair_name)
    plt.xlabel('time')
    plt.ylabel('impedance')
    plt.legend()
    plt.show()
    
    
    plant_publisher = rospy.Publisher("/moisture", moisture_msg, queue_size=10)
    while not rospy.is_shutdown():
        plant_publisher.publish(moisture_msg(1,2,3,4))  # args are ('id', 'temp', 'pot_imp', 'plant_imp')
        rospy.sleep(1)

