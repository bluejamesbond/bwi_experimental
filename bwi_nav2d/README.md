bwi_nav2d
=================

Experimental ROS packages to support Nav2D. The following code both tests and performs the Nav2D library.

#Installation
In order to take advantage of this code, you must do the following:

##CMakeList
You must install Gtkmm 2.4.

```bash
$ sudo apt-get install build-essential libgtkmm-2.4-dev -y
```

##CMakeList
You must add the following lines to `/catkin_ws/CMakeLists.txt`.

```cmake
# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8.3)
set(CATKIN_TOPLEVEL TRUE)

INCLUDE(FindPkgConfig) # Add

pkg_check_modules(GTKMM gtkmm-2.4) # Add
pkg_check_modules(SIGC2 sigc++-2.0) # Add

#...

```

