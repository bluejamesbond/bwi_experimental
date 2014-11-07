cmake_minimum_required(VERSION 2.8.3)
project(bwi_nav2d)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav2d_navigator
  nav2d_operator
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    nav2d_navigator
    nav2d_operator
    roscpp
    sensor_msgs
)

###########
## Build ##
###########
link_directories(${SIGC2_LIBRARY_DIRS} ${GTKMM_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GTKMM_INCLUDE_DIRS} ${SIGC2_LIBRARY_DIRS})

add_executable(emulate_joy src/emulate_joy.cpp)
add_dependencies(emulate_joy ${catkin_EXPORTED_TARGETS})
target_link_libraries(emulate_joy ${catkin_LIBRARIES} ${GTKMM_LIBRARIES} ${SIGC2_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS emulate_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_remote_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
