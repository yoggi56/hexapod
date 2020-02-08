cmake_minimum_required(VERSION 2.8.3)
project(visual_model)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visual_model
#  CATKIN_DEPENDS robot_state_publisher roscpp rospy std_msgs tf urdf xacro
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(visual_model src/leg_state_publisher.cpp)

#target_link_libraries(visual_model
#   ${catkin_LIBRARIES}
# )

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})