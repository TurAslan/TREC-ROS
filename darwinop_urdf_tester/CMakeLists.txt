cmake_minimum_required(VERSION 2.8.3)
project(darwinop_urdf_tester)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  joint_state_publisher   
  soem
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs tf joint_state_publisher

)
include_directories(
include ${catkin_INCLUDE_DIRS}
${soem_INCLUDE_DIRS}/soem
)

add_executable(ethercat_sender src/ethercat_sender.cpp)
target_link_libraries(ethercat_sender ${catkin_LIBRARIES})

add_executable(ethercat_master src/ethercat_master.cpp)
target_link_libraries(ethercat_master ${catkin_LIBRARIES})

add_executable(ethercat_receiver src/ethercat_receiver.cpp)
target_link_libraries(ethercat_receiver ${catkin_LIBRARIES})

add_executable(joint_moverDarwin src/joint_moverDarwin.cpp)
target_link_libraries(joint_moverDarwin ${catkin_LIBRARIES})



