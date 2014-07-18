cmake_minimum_required(VERSION 2.8.3)
project(prace_moveit_interface)

find_package(catkin REQUIRED COMPONENTS rospy message_generation actionlib actionlib_msgs std_msgs moveit_msgs  tf moveit_ros_planning_interface  moveit_ros_move_group)
find_package(Boost REQUIRED COMPONENTS thread)

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  FILES
  ExecPlan.action
  JointSpace.action
  Lin.action
  Ptp.action
  SyncMove.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs moveit_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS rospy message_runtime actionlib actionlib_msgs std_msgs moveit_msgs moveit_ros_planning_interface tf moveit_ros_move_group
    DEPENDS # TODO
    INCLUDE_DIRS include # TODO
    LIBRARIES # TODO
)



###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(moveit_interface src/MoveItInterface.cpp)
add_dependencies(moveit_interface ${PROJECT_NAME}_gencpp)

target_link_libraries(moveit_interface ${Boost_LIBRARIES})
target_link_libraries(moveit_interface ${catkin_LIBRARIES})



#############
## Install ##
#############
## Mark executables and/or libraries for installation
#install(TARGETS cob_pick_place_action
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
