#!/bin/bash

if [ -f "$(dirname $0)/devel/setup.bash" ]; then
    source "$(dirname $0)/devel/setup.bash"
else
    echo "ERROR: You must compile your code before you run this script."
    echo "ERROR: Run the following command:" 
    echo ""
    echo "  catkin build"
    echo ""
    exit -1
fi

function get_rover()
{
    rovercount=$(rostopic list | grep driveControl | wc -l)
    if [ "$rovercount" -gt 1 ]; then
	echo "WARN: Multiple rovers. Using HOSTNAME if this is a rover"
	echo "ERROR: Use rosrun instead."
	echo ""
	echo 'ROS_NAMESPACE=/<rovername> rosrun mobility <node> <arguments>'
	echo ""
	rover=$HOSTNAME #@TODO check the topics if they don't exist then error out
    elif [ "$rovercount" -eq 0 ]; then
	echo "ERROR: There are no rovers to connect to! You must start the GUI"
	echo "ERROR: and either begin a simulation or deploy to a rover."
	echo ""
	exit -2
    fi
    rover=$(rostopic list | grep driveControl | cut -d/ -f 2)
}

exe=$(basename $0)
if [ "$exe" == "run.sh" ]; then    
    # Delete the rqt cache - can take 24 hours for changes in the UI
    # to show up otherwise
    rm ~/.config/ros.org/rqt_gui.ini
    roslaunch ./launch/rover_gui.launch "$@"
elif [ "$exe" == "dev.sh" ]; then
    get_rover
    echo "Connecting to rover $rover"
    ROS_NAMESPACE="/$rover" rosrun mobility "$@"
elif [ "$exe" == "rdb.sh" ]; then
    get_rover
    echo "Connecting to rover $rover"
    ROS_NAMESPACE="/$rover" rosrun mobility rdb.py
fi
