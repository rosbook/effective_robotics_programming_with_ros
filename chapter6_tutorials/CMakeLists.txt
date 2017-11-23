cmake_minimum_required(VERSION 2.8.3)
project(chapter6_tutorials)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  roscpp
  rospy
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sendGoals src/sendGoals.cpp)
target_link_libraries(sendGoals ${catkin_LIBRARIES})
add_dependencies(sendGoals ${catkin_EXPORTED_TARGETS})
