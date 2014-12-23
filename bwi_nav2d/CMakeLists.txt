cmake_minimum_required(VERSION 2.8.3)
project(bwi_nav2d)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav2d_navigator
  nav2d_operator
  message_generation
  tf
  pluginlib
  actionlib
  geometry_msgs
  actionlib_msgs
  roscpp
  sensor_msgs
)

add_service_files(FILES AutoLocalizeRobot.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    tf
    actionlib
    actionlib_msgs
      geometry_msgs
    nav2d_navigator
    nav2d_operator
    roscpp
    sensor_msgs
)

###########
## Build ##
###########
link_directories(${SIGC2_LIBRARY_DIRS} ${GTKMM_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GTKMM_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${SIGC2_LIBRARY_DIRS})

add_executable(emulate_joy src/emulate_joy.cpp)
add_dependencies(emulate_joy ${catkin_EXPORTED_TARGETS})
target_link_libraries(emulate_joy ${catkin_LIBRARIES} ${GTKMM_LIBRARIES} ${SIGC2_LIBRARIES})

add_executable(windowed_sim_joy src/windowed_sim_joy.cpp)
add_dependencies(windowed_sim_joy ${catkin_EXPORTED_TARGETS})
target_link_libraries(windowed_sim_joy ${catkin_LIBRARIES} ${GTKMM_LIBRARIES} ${Boost_LIBRARIES}${SIGC2_LIBRARIES})

add_executable(auto_localize_robot src/auto_localize_robot.cpp)
add_dependencies(auto_localize_robot bwi_nav2d_generate_messages_cpp)

target_link_libraries(auto_localize_robot ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS emulate_joy emulate_joy
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
