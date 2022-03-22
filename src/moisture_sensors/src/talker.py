#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Int64
from moisture_sensors.msg import moisture_msg


def simulator_running():
  """Helper Returns True if there is a /gazebo/link_states topic otherwise False"""
  for t in rospy.get_published_topics():
    if t[0] == '/gazebo/link_states':
      # This is the simulator
      return True
  return False


class MoistureSensor:
  # decay rate for plant
  # need to change this value to real decay rate
  moisture_decay_rate = 1
  moisture_arr = []

  if simulator_running():
    pots = [{'z_value_plant': random.randint(30, 70),
             'z_value_soil': random.randint(30, 55),
             'z_value_temp': random.randint(0, 10)}
            for num in range(0, 142)]
  else:
    pots = list(dict())

  # gets the moisture level from the pots and
  # appends to the moisture_arr
  def __init__(self):
    self.moisture_publisher = None

  def get_pot_moisture(self):
    for pot in self.pots:
      z_value_plant = pot['z_value_plant']
      z_value_soil = pot['z_value_soil']
      # moisture level is the difference between
      # z value of plant and z value of soil
      moisture_level = abs(z_value_soil - z_value_plant)
      self.moisture_arr.append(moisture_level)
    return self.moisture_arr

  # returns the pots (list)
  def getPots(self):
    return self.pots

  # sets the value of specific pot
  def setPotsValue(self, potId, z_value_plant, z_value_soil, z_value_temp):
    print('Updating pot value...')
    self.pots[potId] = {'z_value_plant': z_value_plant,
                        'z_value_soil': z_value_soil,
                        'z_value_temp': z_value_temp}

  # gets moisture level from moisture_arr and
  # decreases each moisture level by moisture_decay value
  # TODO: change these decay rate to correct one
  def moisture_decay(self, event=None):
    # for x in range(len(self.pots)):
    #    self.moisture_arr[x] = self.moisture_arr[x] + self.moisture_decay_rate
    for i in range(len(self.pots)):
      self.pots[i]['z_value_temp'] += self.moisture_decay_rate
      self.pots[i]['z_value_soil'] += self.moisture_decay_rate
      self.pots[i]['z_value_plant'] += self.moisture_decay_rate

  # Every 1 second publishes moisture of pot
  def publish_pot_moisture(self, event=None):
    self.moisture_publisher = rospy.Publisher("/moisture", moisture_msg, queue_size=10)
    rate = rospy.Rate(0.7)
    msg = moisture_msg()
    while not rospy.is_shutdown():
      ms.moisture_decay()
      for i in range(len(self.pots)):
        print(self.pots[i])
        msg.id = i
        msg.temp = self.pots[i]['z_value_temp']
        msg.pot_imp = self.pots[i]['z_value_soil']
        msg.plant_imp = self.pots[i]['z_value_plant']
        self.moisture_publisher.publish(msg)
        i = i + 1
        rate.sleep()
      print('\n')

  # gets the data from publisher's node
  def sensor_callback(self, data):
    # print('Watered pot: ', data.data )
    # this pot id is the response from swarmie i.e. data.data
    # assuming data is int type
    self.setPotsValue(data.id, data.plant_imp, data.pot_imp, data.temp)

  def watered_callback(self, data):
    self.setPotsValue(data.data, moisture_msg.DRY_PLANT / 2, moisture_msg.DRY_SOIL / 2, self.pots[data.data]['temp'])

  # listens to the swarmie's node to update the pot values, once it's watered
  def listener(self):
    # TODO: change the chatter topic to relavant one
    rospy.Subscriber("watered", Int64, self.watered_callback)

  def initiate(self):
    self.listener()
    self.get_pot_moisture()
    if simulator_running():
      self.publish_pot_moisture()


if __name__ == '__main__':
  rospy.init_node("moistures_sensors_node")

  # Create an instance of Moisture sensor
  ms = MoistureSensor()
  ms.get_pot_moisture()
  if simulator_running():
    ms.publish_pot_moisture()
    ms.initiate()
  else:
    rospy.spin()
