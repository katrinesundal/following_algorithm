cmake_minimum_required(VERSION 2.8.3)
project(following_algorithm)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

FIND_PACKAGE(cmake_modules REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

add_definitions(-std=c++11 -Wall -g)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY
   msg
   FILES
   ActuatorMessage.msg
   guiObjectUpdate.msg
   SetShoalParameter.msg
   ShoalCoordinates.msg
   Heartbeat.msg
   ControllerKValues.msg

 )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  get_max_surge_force.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

#Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES following_algorithm
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(thrust_output src/thrust_output.cpp)
target_link_libraries(thrust_output ${catkin_LIBRARIES})
add_dependencies(thrust_output following_algorithm_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(following_algorithm src/following_algorithm.cpp src/var_time_const.cpp)
target_link_libraries(following_algorithm ${catkin_LIBRARIES})
add_dependencies(following_algorithm following_algorithm_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(tf_odom_ned src/tf_odom_ned.cpp)
target_link_libraries(tf_odom_ned ${catkin_LIBRARIES})
add_dependencies(tf_odom_ned following_algorithm_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#add_executable(shoal_of_fish src/shoal_of_fish.cpp)
#target_link_libraries(shoal_of_fish ${catkin_LIBRARIES})
#add_dependencies(shoal_of_fish following_algorithm_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )



