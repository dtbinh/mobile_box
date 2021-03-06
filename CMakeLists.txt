cmake_minimum_required(VERSION 2.8.3)
project(box)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	Map.msg
	Plan.msg
	Problem.msg
)

# Generate services in the 'srv' folder
add_service_files(
	FILES
	BoxPlan.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
	LIBRARIES box_planner
	CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########


add_library(box_planner
	lib/Closed.cpp
	lib/Map.cpp
	lib/Open.cpp
	lib/Pos.cpp
	lib/Search.cpp
	lib/State.cpp
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(box_plan src/box_plan.cpp)
add_dependencies(box_plan box)
target_link_libraries(box_plan ${catkin_LIBRARIES} box_planner)

install(TARGETS box_planner
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
