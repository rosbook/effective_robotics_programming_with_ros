cmake_minimum_required(VERSION 2.8.3)
project(chapter10_tutorials)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_create src/pcl_create.cpp)
add_executable(pcl_downsampling src/pcl_downsampling.cpp)
add_executable(pcl_filter src/pcl_filter.cpp)
add_executable(pcl_matching src/pcl_matching.cpp)
add_executable(pcl_model_estimation src/pcl_model_estimation.cpp)
add_executable(pcl_partitioning src/pcl_partitioning.cpp)
add_executable(pcl_planar_segmentation src/pcl_planar_segmentation.cpp)
add_executable(pcl_read src/pcl_read.cpp)
add_executable(pcl_visualize src/pcl_visualize.cpp)
add_executable(pcl_write src/pcl_write.cpp)

target_link_libraries(pcl_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_downsampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_matching ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_model_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_partitioning ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_planar_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_visualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_write ${catkin_LIBRARIES} ${PCL_LIBRARIES})
