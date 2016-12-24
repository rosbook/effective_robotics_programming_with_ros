cmake_minimum_required(VERSION 2.8.3)
project(chapter8_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_msgs
  geometry_msgs
  joy
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
 find_package(PCL REQUIRED)
 include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ executable
add_executable(c8_teleop_joy src/c8_teleop_joy.cpp)
add_executable(c8_odom src/c8_odom.cpp)
add_executable(c8_odom_encoders src/c8_odom_encoders.cpp)
add_executable(c8_odom_with_imu src/c8_odom_imu.cpp)

add_executable(c8_laserscan src/c8_laserscan.cpp)
add_executable(c8_kinect src/c8_kinect.cpp)
add_executable(c8_dynamixel src/c8_dynamixel.cpp)
add_executable(c8_fixtoUTM src/c8_fixtoUTM.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(c8_teleop_joy
   ${catkin_LIBRARIES} )
 target_link_libraries(c8_odom
   ${catkin_LIBRARIES} )
    target_link_libraries(c8_odom_encoders
   ${catkin_LIBRARIES} )
 target_link_libraries(c8_odom_with_imu
   ${catkin_LIBRARIES} )   

 target_link_libraries(c8_laserscan
   ${catkin_LIBRARIES} 
 )

 target_link_libraries(c8_kinect
   ${catkin_LIBRARIES} 
 )


 target_link_libraries(c8_dynamixel
   ${catkin_LIBRARIES} 
 )

 target_link_libraries(c8_fixtoUTM
   ${catkin_LIBRARIES} 
 )