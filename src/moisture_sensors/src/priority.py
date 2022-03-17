import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import pandas as pd

# read the file to get the impadance values
def read_file():
    columns = ["impedance"]
    df = pd.read_csv("impedance.csv", usecols=columns)
    df = df.to_numpy()

    impedance = []
    for i in range(len(df)):
        impedance.append(df[i][0])
    return impedance


class Main:
    refill_tank = False
    MIN_THRESHOLD = 5
    number_of_trips = 0
    priority = []
    impedance_values = []


    def __init__(self):
        # listen for trip status
        rospy.Subscriber("trip_status", Bool, update_trip)
        # listen for moisture sensors impedance values
        rospy.Subscriber("moisture", Float32MultiArray, update_impedance_values)


    # if the number of trips is less than 5,
    # keep watering the plants
    def update_trip(self, data):
        if self.number_of_trips > 5:
            self.number_of_trips = 0
            self.refill_tank = True

        elif data:
            self.number_of_trips = self.number_of_trips + 1        


    # update the impedance values array 
    # read the values and update the priority queue 
    # to water the plants
    def update_impedance_values(self, data):
        self.impedance_values = data
        for impedance_value in self.impedance_values:
            if impedance_value > self.MIN_THRESHOLD:
                # greater the difference -> plant needs more water
                # plant that has greater difference needs to get watered first
                diff = impedance_value - self.MIN_THRESHOLD
                print('need to water this plant\n')
                self.priority.append((diff, data.id))
                self.priority.sort()


    # send the most thristy plant to the robot
    def send_to_robot(self, msg):
        pub = rospy.Publisher("water_me_first", Float32, queue_size=10)
        next_plant_to_water_id = self.priority.pop()[1]
        msg.id = next_plant_to_water_id
        pub.publish(msg)


    # if the trip is more than 5, refill the water tank    
    def refill_water_tank(self):
        if self.refill_tank:
            pub = rospy.Publisher("refill_the_tank", Bool, queue_size=10)
            pub.publish(True)
                       

if __name__ == '__main__':
    main = Main()
    print('hello')
