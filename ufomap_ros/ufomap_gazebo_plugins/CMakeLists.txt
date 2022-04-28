cmake_minimum_required(VERSION 3.9)
project(ufomap_gazebo_plugins 
  VERSION 1.0 LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  message_generation
  roscpp
  ufomap_msgs
  ufomap_ros
  ufomap_srvs
)

find_package(ufomap REQUIRED)

find_package(gazebo_dev QUIET)
if (NOT GAZEBO_DEV_FOUND)
  find_package(gazebo REQUIRED)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

add_service_files(
  FILES
	GetMap.srv
)

generate_messages(
  DEPENDENCIES
  ufomap_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ufomap_from_gazebo
  CATKIN_DEPENDS gazebo_plugins message_runtime roscpp ufomap_msgs ufomap_ros ufomap_srvs
  DEPENDS ufomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ufomap_from_gazebo
  src/ufomap_from_gazebo.cpp
)

if(${gazebo_dev_FOUND})
  target_link_libraries(ufomap_from_gazebo
    ${catkin_LIBRARIES}
    UFO::Map
    ${gazebo_dev_LIBRARIES}
  )
else()
  target_link_libraries(ufomap_from_gazebo
    ${catkin_LIBRARIES}
    UFO::Map
    ${GAZEBO_LIBRARIES}
  )
endif()

add_dependencies(ufomap_from_gazebo ${catkin_EXPORTED_TARGETS} ufomap_from_gazebo_generate_messages_cpp)

install(DIRECTORY include/ufomap_from_gazebo/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ufomap_from_gazebo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
