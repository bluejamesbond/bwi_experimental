#!/bin/bash

CONSOLE_TAG="MultiAgentMapper"

. console.sh

# launch ros
gnome-terminal --tab -e "roslaunch bwi_nav2d nav2d_mapper_krr2014_multi.launch --screen" &
timeout 14 "ROS starting up"

# robot_0 mapping
log "Starting mapping on robot_0"
rosservice call /robot_0/StartMapping 3 > /dev/null 2>&1
timeout 14 "Robot_0 collecting inital location data"

# robot_0 exploration
log "Starting exploration on robot_0"
rosservice call /robot_0/StartExploration 2 > /dev/null 2>&1
timeout 11 "Robot_0 collecting point cloud data"

warn "Do you see a point cloud there? Robot_1 will now use the data to"
warn "localize itself."

# robot_1 localization
log "Starting localization on robot_1 with point cloud data"
rosservice call /robot_1/AutoLocalizeRobot 2 > /dev/null 2>&1
timeout 11 "Robot_1 localizing"

# robot_1 exploration
log "Starting exploration on robot_1"
rosservice call /robot_1/StartExploration 2 > /dev/null 2>&1

# info
info "The two robots should be mapping now. The two maps represent each robot's"
info "view. If you look carefully, you can see that the map is growing from"
info "different sides. The robots are also able to share the map data and explore"
info "frontiers more efficiently."

