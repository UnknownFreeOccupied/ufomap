cmake_minimum_required(VERSION 3.0.2)
project(ufomap_srvs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	ufomap_msgs
)

add_service_files(
  FILES
  ClearVolume.srv
	GetMap.srv
	Reset.srv
	SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  ufomap_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime ufomap_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)