cmake_minimum_required(VERSION 3.8.2)
project(ufomap_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

find_package(ufomap REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
    AABB.msg
    BoundingVolume.msg
#  Capsule.msg
#  Cone.msg
#  Cylinder.msg
#  Ellipsoid.msg
    Frustum.msg
    LineSegment.msg
    OBB.msg
    Plane.msg
    Point.msg
    Ray.msg
    Sphere.msg
#  Triangle.msg
    UFOMap.msg
    UFOMapMetaData.msg
    UFOMapStamped.msg
    Labels.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES}
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC src/conversions.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    ${catkin_LIBRARIES}
    UFO::Map
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)