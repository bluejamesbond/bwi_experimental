#!/bin/bash

CONSOLE_TAG="SingleAgentMapper"

. console.sh

# launch ros
gnome-terminal --tab -e "roslaunch bwi_nav2d nav2d_mapper_krr2014.launch --screen" &
timeout 14 "ROS starting up"

# mapping
log "Starting mapping"
rosservice call /StartMapping 3 > /dev/null 2>&1
timeout 14 "Collecting inital location data"

# exploration
log "Starting exploration"
rosservice call /StartExploration 2 > /dev/null 2>&1

# info
info "Robot is mapping the area currently. Please wait..."

