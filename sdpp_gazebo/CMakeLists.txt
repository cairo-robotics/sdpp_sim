cmake_minimum_required(VERSION 2.8.3)
project(sdpp_gazebo)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)



find_package(gazebo REQUIRED)
catkin_package(
    CATKIN_DEPENDS
    gazebo_ros
    )
catkin_python_setup()


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

