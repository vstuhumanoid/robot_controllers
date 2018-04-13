cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  std_msgs
  message_generation
)


# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   Transition.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
	std_msgs
	actionlib_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_controllers
  CATKIN_DEPENDS actionlib roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(transition_controller src/TransitionController/transition_controller.cpp)

target_link_libraries(transition_controller
        ${catkin_LIBRARIES}
)

add_executable(transition_client src/TransitionController/transition_client.cpp)

target_link_libraries(transition_client
		${catkin_LIBRARIES}
		)

add_executable(wtf_client src/TransitionController/wtf_client.cpp)

target_link_libraries(wtf_client
		${catkin_LIBRARIES}
		)

