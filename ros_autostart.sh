#!/bin/bash
# Source ROS setup

sleep 30
source /opt/ros/noetic/setup.bash
source ~/Skyline-Drone/ws_livox/devel/setup.bash
source ~/Skyline-Drone/skyline_ws/devel/setup.bash

# Start All Node
roslaunch skyline drone.launch
sleep 10

# Publish origin once (run once, not loop)
rostopic pub -1 /mavros/global_position/set_gp_origin geographic_msgs/GeoPointStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
position:
  latitude: -6.218625
  longitude: 106.802618
  altitude: 10.0" &

# mkdir -p /mnt/skyline/rosbag_$(date +%Y%m%d_%H%M%S) && \
# rosbag record -a --split --duration=60s --buffsize=10000000 -O /mnt/skyline/rosbag_$(date +%Y%m%d_%H%M%S)/rosbag


wait
