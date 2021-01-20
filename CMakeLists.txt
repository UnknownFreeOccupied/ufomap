cmake_minimum_required(VERSION 3.9)

project(ufomap 
	VERSION 1.0.0
	DESCRIPTION "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown"
	LANGUAGES CXX
)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Maybe remove this under
# IF (CMAKE_COMPILER_IS_GNUCC)
#   SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-error")
#   SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error -Wpedantic -march=native")
#   SET (CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops -DNDEBUG")
#   SET (CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
#   # Shared object compilation under 64bit (vtable)
#   ADD_DEFINITIONS(-fPIC)
# ENDIF()

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

	# Support folders in IDE's
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)

	# Should this be here?
	# set(MODERN_CMAKE_BUILD_TESTING ON)

	include(CTest)

	find_package(Doxygen)
	if(Doxygen_FOUND)
		add_subdirectory(docs)
	else()
		message(STATUS "Doxygen not found, not building docs")
	endif()
endif()

# To make headers show up in IDEs
set(HEADER_LIST
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/aabb.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/bounding_volume.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/capsule.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/collision_checks.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/cone.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/cylinder.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/ellipsoid.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/frustum.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/line_segment.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/obb.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/plane.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/point.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/ray.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/sphere.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/triangle.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/geometry/types.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/iterator/occupancy_map_nearest.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/iterator/occupancy_map.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/iterator/octree_nearest.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/iterator/octree.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/code.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/color.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/key.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/occupancy_map_base.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/occupancy_map_color.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/occupancy_map_node.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/occupancy_map.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/octree_node.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/octree.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/point_cloud.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/types.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/map/ufomap.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/math/pose6.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/math/quaternion.h"
	"${PROJECT_SOURCE_DIR}/include/ufo/math/vector3.h"
)

set(SRC_LIST
	"${PROJECT_SOURCE_DIR}/src/geometry/bounding_volume.cpp"
	"${PROJECT_SOURCE_DIR}/src/geometry/collision_checks.cpp"
	"${PROJECT_SOURCE_DIR}/src/map/occupancy_map_color.cpp"
	"${PROJECT_SOURCE_DIR}/src/map/occupancy_map.cpp"
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)

add_library(Map SHARED ${SRC_LIST} ${HEADER_LIST})
add_library(UFO::Map ALIAS Map)

set_target_properties(Map 
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_STANDARD_REQUIRED YES
		CXX_EXTENSIONS NO
)

target_include_directories(Map
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(Map
	PUBLIC
		${LZ4_LIBRARIES}
		tbb
)

target_compile_features(Map
	PUBLIC 
		cxx_std_17
)

target_compile_options(Map 
	PUBLIC
    -fPIC
)

set(UFOMAP_BMI2 FALSE CACHE BOOL "Enable/disable BMI2 instructions")
if(DEFINED ENV{UFOMAP_BMI2})
  set(UFOMAP_BMI2 $ENV{UFOMAP_BMI2})
endif(DEFINED ENV{UFOMAP_BMI2})
if(UFOMAP_BMI2)
	message(STATUS "UFOMAP BMI2 instructions enabled")
	target_compile_options(Map 
		PUBLIC
			-mbmi2
	)
else()
	message(STATUS "UFOMAP BMI2 instructions disabled")
endif(UFOMAP_BMI2)

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

install(TARGETS Map 
	EXPORT ufomapTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufomapTargets
	FILE "ufomapTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufomapConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
		"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfig.cmake"
		"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)