#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

if [ -f /root/autonomous-lidar-drone/ws_livox/devel/setup.bash ]; then
  source /root/autonomous-lidar-drone/ws_livox/devel/setup.bash
fi

if [ -f /root/autonomous-lidar-drone/skyfast_ws/devel/setup.bash ]; then
  source /root/autonomous-lidar-drone/skyfast_ws/devel/setup.bash
fi
