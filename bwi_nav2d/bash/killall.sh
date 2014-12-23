#!/bin/bash

CONSOLE_TAG="Killall"

. console.sh

info "Killing roslaunch"
killall roslaunch > /dev/null 2>&1

info "Killing gazebo"
killall gazebo > /dev/null 2>&1

info "Killing rviz"
killall rviz > /dev/null 2>&1
