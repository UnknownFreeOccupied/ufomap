cmake_minimum_required(VERSION 3.5...3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

project(ufomap
	VERSION 2.0.0
	DESCRIPTION "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown"
	LANGUAGES CXX
)

add_subdirectory(1stparty)

option(UFO_TESTING "Unit testing" OFF)
option(UFO_COVERAGE "Test Coverage" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_CXX_CLANG_TIDY clang-tidy)

option(UFO_DEV_MODE "Set up development helper settings" ON)

add_library(Map INTERFACE)
add_library(UFO::Map ALIAS Map)

set_target_properties(Map PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 17
	CXX_EXTENSIONS OFF
)

set_property(TARGET Map PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(Map INTERFACE 
	# UFO::Compression
	UFO::Compute
	UFO::Container
	UFO::Core
	UFO::Geometry
	UFO::Math
	UFO::Morton
	UFO::PCL
	UFO::Time
	UFO::Utility
	UFO::Vision
)

target_compile_options(Map INTERFACE
	-fPIC
)

target_include_directories(Map INTERFACE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(Map INTERFACE cxx_std_17)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# add_executable(ufomap_render src/render.cpp)

# set_target_properties(ufomap_render PROPERTIES
# 	VERSION ${PROJECT_VERSION}
# 	SOVERSION ${PROJECT_VERSION_MAJOR}
# 	CXX_STANDARD 17
# 	CXX_EXTENSIONS OFF
# 	# COMPILE_WARNING_AS_ERROR ON
# )

# if(UFO_DEV_MODE)
# 	# In dev mode, we load resources from the source tree
# 	target_compile_definitions(ufomap_render PRIVATE
# 		RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
# 	)
# else()
# 	# In release mode, we just load resources relatively to wherever the
# 	# executable is launched from, so that the binary is portable
# 	target_compile_definitions(ufomap_render PRIVATE
# 		RESOURCE_DIR="./resources"
# 	)
# endif()

# set_property(TARGET ufomap_render PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

# # find_package(PCL 1.14 REQUIRED)
# # include_directories(${PCL_INCLUDE_DIRS})
# # link_directories(${PCL_LIBRARY_DIRS})
# # add_definitions(${PCL_DEFINITIONS})

# target_link_libraries(ufomap_render 
# 	PRIVATE 
# 		Map 
# 		${OpenCV_LIBS} 
# 		# ${PCL_LIBRARIES}
# 		# gcov
# )

# target_compile_options(ufomap_render 
# 	PUBLIC 
# 		-march=native
# 		# -fprofile-generate
# 		# -fprofile-use
# 		# -fprofile-correction
# 		# -fprofile-dir=/home/dduberg/ufo/data/pgo 
# 		# -fprofile-generate=/home/dduberg/ufo/data/pgo
# )

# add_executable(ufomap_render2d src/render2d.cpp)

# set_target_properties(ufomap_render2d PROPERTIES
# 	VERSION ${PROJECT_VERSION}
# 	SOVERSION ${PROJECT_VERSION_MAJOR}
# 	CXX_STANDARD 17
# 	CXX_EXTENSIONS OFF
# 	# COMPILE_WARNING_AS_ERROR ON
# )

# if(UFO_DEV_MODE)
# 	# In dev mode, we load resources from the source tree
# 	target_compile_definitions(ufomap_render2d PRIVATE
# 		RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
# 	)
# else()
# 	# In release mode, we just load resources relatively to wherever the
# 	# executable is launched from, so that the binary is portable
# 	target_compile_definitions(ufomap_render2d PRIVATE
# 		RESOURCE_DIR="./resources"
# 	)
# endif()

# set_property(TARGET ufomap_render2d PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

# target_link_libraries(ufomap_render2d 
# 	PRIVATE 
# 		Map 
# 		${OpenCV_LIBS} 
# 		# gcov
# )

# target_compile_options(ufomap_render2d 
# 	PUBLIC 
# 		-march=native
# 		# -fprofile-generate
# 		# -fprofile-use
# 		# -fprofile-correction
# 		# -fprofile-dir=/home/dduberg/ufo/data/pgo 
# 		# -fprofile-generate=/home/dduberg/ufo/data/pgo
# )

# add_executable(ufomap_render4d src/render4d.cpp)

# set_target_properties(ufomap_render4d PROPERTIES
# 	VERSION ${PROJECT_VERSION}
# 	SOVERSION ${PROJECT_VERSION_MAJOR}
# 	CXX_STANDARD 17
# 	CXX_EXTENSIONS OFF
# 	# COMPILE_WARNING_AS_ERROR ON
# )

# if(UFO_DEV_MODE)
# 	# In dev mode, we load resources from the source tree
# 	target_compile_definitions(ufomap_render4d PRIVATE
# 		RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
# 	)
# else()
# 	# In release mode, we just load resources relatively to wherever the
# 	# executable is launched from, so that the binary is portable
# 	target_compile_definitions(ufomap_render4d PRIVATE
# 		RESOURCE_DIR="./resources"
# 	)
# endif()

# set_property(TARGET ufomap_render4d PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

# target_link_libraries(ufomap_render4d 
# 	PRIVATE 
# 		Map 
# 		${OpenCV_LIBS} 
# 		# gcov
# )

# target_compile_options(ufomap_render4d 
# 	PUBLIC 
# 		-march=native
# 		# -fprofile-generate
# 		# -fprofile-use
# 		# -fprofile-correction
# 		# -fprofile-dir=/home/dduberg/ufo/data/pgo 
# 		# -fprofile-generate=/home/dduberg/ufo/data/pgo
# )

include(GNUInstallDirs)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(UFO_TESTING)
  add_subdirectory(tests)
endif()

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