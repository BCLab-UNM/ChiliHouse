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


def plant_walk(num_moves):
    """Do a plant walk `num_moves` times."""
    for move in range(num_moves):
        if rospy.is_shutdown():
            water_plants_exit(-1)
        # @TODO: get thirstiest plant also weigh in distance
        swarmie.drive_to_plant(random.randint(0,143))


def water_plants_exit(code):
    sys.exit(code)


def main(**kwargs):
    global planner, found_tag
    try: 
        planner
    except NameError: 
        from mobility.planner import Planner
        planner = Planner(use_rviz_nav_goal=True)
    
    if swarmie.planner_publisher is None:
        swarmie.planner_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
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
