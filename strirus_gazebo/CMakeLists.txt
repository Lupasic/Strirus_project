cmake_minimum_required(VERSION 2.8.3)
project(strirus_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  rospy
  sensor_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plastun_gazebo
#  CATKIN_DEPENDS gazebo_ros gazebo_ros_control rospy sensor_msgs
#  DEPENDS system_lib
)
find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
