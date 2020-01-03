cmake_minimum_required(VERSION 2.8.3)
project(groundstation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Qt5  REQUIRED COMPONENTS  Widgets  )
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/qt_app_node.cpp
  src/mainwindow.cpp
)

set(FORMS
  src/mainwindow.ui
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt_app
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include/groundstation

  ${catkin_INCLUDE_DIRS}
)

add_executable(groundstation ${SOURCES} ${FORMS} )

target_link_libraries(groundstation
   ${catkin_LIBRARIES}
)

target_link_libraries(groundstation  Qt5::Widgets )
