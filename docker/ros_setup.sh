#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

if [ -f /root/Skyfast-Drone/ws_livox/devel/setup.bash ]; then
  source /root/Skyfast-Drone/ws_livox/devel/setup.bash
fi

if [ -f /root/Skyfast-Drone/skyfast_ws/devel/setup.bash ]; then
  source /root/Skyfast-Drone/skyfast_ws/devel/setup.bash
fi
