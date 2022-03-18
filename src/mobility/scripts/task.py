#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy 
import StringIO 
import traceback 

from std_msgs.msg import UInt8, String

import threading 
task_lock = threading.Lock()

from mobility import sync

# Behavior programs
import mobility.behavior.init
import mobility.behavior.calibrate_imu
import mobility.behavior.queue
import mobility.behavior.gohome
import mobility.behavior.escape_home
import mobility.behavior.water_plants


from mobility.swarmie import swarmie, AbortException, InsideHomeException

'''Node that coordinates the overall robot task''' 

class Task : 
    
    STATE_IDLE           = 0
    STATE_CAL_IMU        = 1
    STATE_QUEUE          = 2
    STATE_INIT           = 3
    STATE_WATER_PLANTS   = 4
    STATE_PLANT_APPROACH = 5
    STATE_GOHOME         = 6
    STATE_REFILL         = 7
    STATE_ESCAPE_HOME    = 8

    PROG_INIT           = 'init.py'
    PROG_CAL_IMU        = 'calibrate_imu.py'
    PROG_QUEUE          = 'queue.py'
    PROG_SEARCH         = 'water_plants.py'
    PROG_PLANT_APPROACH = 'plant_approach.py'
    PROG_GOHOME         = 'gohome.py'
    PROG_REFILL         = 'refill.py'
    PROG_ESCAPE_HOME    = 'escape_home.py'

    def __init__(self):
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=2, latch=False)
        # Published regularly on a timer.
        self.status_pub = rospy.Publisher('swarmie_status', String, queue_size=1, latch=True)
        # Published once when the status changes.
        self.task_pub = rospy.Publisher('task_state', String, queue_size=1, latch=True)
        self.status_timer = rospy.Timer(rospy.Duration(1), self.publish_status)

        if rospy.has_param('~task_state'):
            self.current_state = rospy.get_param('~task_state')
            self.print_state('<font color="red">Task manager restarted.</font>')
        else:
            self.current_state = Task.STATE_IDLE

        self.prev_state = None
        self.has_block = rospy.get_param('~has_block', False)

        rospy.on_shutdown(self.save_state)

    def save_state(self):
        rospy.set_param('~task_state', self.current_state)
        rospy.set_param('~has_block', self.has_block)

    def publish_status(self, _event):
        self.status_pub.publish(self.get_task())

    def print_state(self, msg):
        s = String()
        s.data = msg 
        self.state_publisher.publish(s)
        print (msg) 
        
    def launch(self, prog):
        try: 
            rval = prog(has_block=self.has_block)
            if rval is None:
                rval = 0
        except SystemExit as e: 
            rval = e.code
        return rval
    
    def get_task(self) :
        if self.current_state == Task.STATE_IDLE : 
            return "idle"
        elif self.current_state == Task.STATE_CAL_IMU :
            return "cal IMU"
        elif self.current_state == Task.STATE_QUEUE :
            return "queue"
        elif self.current_state == Task.STATE_INIT : 
            return "init"
        elif self.current_state == Task.STATE_WATER_PLANTS :
            return "water_plants"
        elif self.current_state == Task.STATE_PLANT_APPROACH : 
            return "plant_approach"
        elif self.current_state == Task.STATE_GOHOME : 
            return "gohome"
        elif self.current_state == Task.STATE_REFILL : 
            return "refill"
        elif self.current_state == Task.STATE_ESCAPE_HOME :
            return "escape_home"
        return "unknown"
        
    @sync(task_lock)
    def run_next(self):
        self.publish_status(None)
        self.task_pub.publish(self.get_task())

        try:
            if self.current_state == Task.STATE_IDLE:
                self.current_state = Task.STATE_CAL_IMU

            elif self.current_state == Task.STATE_CAL_IMU :
                if self.launch(mobility.behavior.calibrate_imu.main) == 0:
                    self.print_state('IMU is calibrated. Entering start queue.')
                    self.current_state = Task.STATE_QUEUE
                else:
                    # TODO: Can the 2D calibration behavior fail? ServiceException's for example?
                    self.print_state('IMU calibration failed!')

            elif self.current_state == Task.STATE_QUEUE:
                if self.launch(mobility.behavior.queue.main) == 0:
                    self.print_state('Finished queuing. Starting init.')
                    self.current_state = Task.STATE_INIT
                else:
                    # TODO: Can the queue behavior fail?
                    self.print_state('Queue failed!')

            elif self.current_state == Task.STATE_INIT:
                if self.launch(mobility.behavior.init.main) == 0:
                    self.print_state('Init succeeded. Starting water plants.')
                    self.current_state = Task.STATE_WATER_PLANTS
                else:
                    # FIXME: What should happen when init fails?
                    self.print_state('Init failed! FIXME!.')

            elif self.current_state == Task.STATE_WATER_PLANTS:
                if self.launch(mobility.behavior.water_plants.main) == 0:
                    self.print_state('water_plants succeeded. Do plant_approach.')
                    self.current_state = Task.STATE_PLANT_APPROACH
                else:
                    self.print_state('water_plants: Out of water! Going back home.')
                    self.current_state = Task.STATE_GOHOME

            elif self.current_state == Task.STATE_PLANT_APPROACH:
                if self.launch(mobility.behavior.plant_approach.main) == 0:
                    self.print_state('Pickup success. Going back home.')
                    self.has_block = True
                    self.current_state = Task.STATE_GOHOME
                else:
                    self.print_state('Pickup failed. Back to watering plants.')
                    self.current_state = Task.STATE_WATER_PLANTS

            elif self.current_state == Task.STATE_GOHOME:
                gohome_status = self.launch(mobility.behavior.gohome.main)
                if gohome_status == 0:
                    if self.has_block:
                        self.print_state('Home found and I have a block. Do drop off.')
                        self.current_state = Task.STATE_REFILL
                    else:
                        self.print_state('Recalibrated home. Back to watering plants.')
                        self.current_state = Task.STATE_WATER_PLANTS
                elif gohome_status == 1:
                    self.print_state('Go Home interrupted, I found a tag. Do plant_approach.')
                    self.current_state = Task.STATE_PLANT_APPROACH

                else:
                    # FIXME: What happens when we don't find home?
                    self.print_state('Home NOT found. Try again.')
                    self.current_state = Task.STATE_GOHOME

            elif self.current_state == Task.STATE_REFILL:
                if self.launch(mobility.behavior.refill.main) == 0:
                    self.print_state('Dropoff complete. Back to watering plants.')
                    self.has_block = False
                    self.current_state = Task.STATE_WATER_PLANTS
                else:
                    self.print_state('Dropoff failed. Back to searching for home.')
                    self.current_state = Task.STATE_GOHOME

            elif self.current_state == Task.STATE_ESCAPE_HOME:
                self.print_state("EMERGENCY: I'm in the home ring, escape!")
                escape_status = self.launch(mobility.behavior.escape_home.main)
                if escape_status == 0 :
                    self.print_state('Escape is complete.')

                    if self.prev_state == Task.STATE_INIT:
                        self.current_state = Task.STATE_INIT
                    elif self.has_block:
                        self.current_state = Task.STATE_GOHOME
                    else:
                        self.current_state = Task.STATE_WATER_PLANTS

                    self.prev_state = None
                else:
                    # FIXME: What should be done here?
                    self.print_state('EMERGENCY: Bad to worse escape reports failure. Searching...')
                    sys.exit(-1)

        except AbortException as e:
            self.print_state('STOP! Entering manual mode.')
            sys.exit(0)

        except InsideHomeException:
            self.prev_state = self.current_state
            self.current_state = Task.STATE_ESCAPE_HOME

        except Exception as e:
            # FIXME: What do we do with bugs in task code?
            print('Task caught unknown exception:\n' + traceback.format_exc())
            sys.exit(-2)

def main() :
    swarmie.start(node_name='task')
    swarmie.plants_init()
    taskman = Task() 
    while not rospy.is_shutdown():
        taskman.run_next() 

if __name__ == '__main__' : 
    main()
