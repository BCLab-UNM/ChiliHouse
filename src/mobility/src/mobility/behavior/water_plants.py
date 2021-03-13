#! /usr/bin/env python
"""Searcher node."""

from __future__ import print_function

import sys
import rospy
import tf
import math
import random 

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist, Pose2D
from swarmie_msgs.msg import Obstacle

from mobility.planner import Planner
from mobility.swarmie import swarmie, TagException, HomeException, ObstacleException, PathException, AbortException, MoveResult
from moisture_sensors.msg import moisture_msg

def plant_walk(num_moves):
    """Do a plant walk `num_moves` times."""
    for move in range(num_moves):
        if rospy.is_shutdown():
            water_plants_exit(-1)
        rospy.sleep(5)
        # @TODO: get thirstiest plant also weigh in distance
        for plant_id in range(len(swarmie.plants)):
            if swarmie.plants[plant_id]['pot_imp'] > moisture_msg.DRY_SOIL or swarmie.plants[plant_id]['plant_imp'] > moisture_msg.DRY_PLANT:
                try:
                    swarmie.drive_to_plant(plant_id)
                    rospy.loginfo("Watering plant #" + str(plant_id))
                    rospy.sleep(5)
                    swarmie.plants[plant_id]['pot_imp'] = moisture_msg.DRY_SOIL - 1
                    swarmie.plants[plant_id]['plant_imp'] = moisture_msg.DRY_PLANT - 1
                    """
                    while self.model_state_prop("plant_"+str(plant_id)).success:
                        self.delete_model("plant_"+str(plant_id))
                        rospy.sleep(0.5)
                    while not self.model_state_prop("plant_"+str(plant_id)).success:
                        self.spawn_model("plant_"+str(plant_id), self.pot_model, "", Pose(position=self.plants[msg.id]['point'],orientation=Quaternion()),"world")
                        rospy.sleep(0.5)
                    """
                except:
                    pass


def water_plants_exit(code):
    sys.exit(code)


def main(**kwargs):
    global planner, found_tag
    try: 
        planner
    except NameError:
        from mobility.planner import Planner
        planner = Planner(use_rviz_nav_goal=True)
    if not swarmie.planner_publisher:
        swarmie.planner_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
    if not swarmie.plants:
        swarmie.plants_init()
    swarmie.fingers_open()
    swarmie.wrist_middle()
    plant_walk(num_moves=10)
    print ("I'm homesick!")
    water_plants_exit(1)


if __name__ == '__main__' : 
    swarmie.start(node_name='watering plants')
    sys.exit(main())
