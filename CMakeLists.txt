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
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(model_walk SHARED src/model_walk.cpp)
target_link_libraries(model_walk ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


