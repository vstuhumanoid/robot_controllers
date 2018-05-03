cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
		#actionlib_msgs
		actionlib
		roscpp
		std_msgs
		message_generation
		message_filters
)

add_message_files(
		FILES
		MotionParams.msg
)

## Generate actions in the 'action' folder
add_action_files(
		FILES
		Transition.action
		FrundModel.action
		PlayFile.action
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
  		CATKIN_DEPENDS actionlib roscpp std_msgs  message_runtime #message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src/TransitionController)
add_subdirectory(src/FrundController)
add_subdirectory(src/PlayFileController)
add_subdirectory(src/JointNameConverter)